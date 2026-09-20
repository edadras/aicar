import 'dart:math' as math;
import 'dart:typed_data';

import '../ai/interfaces/object_tracker.dart';
import '../camera/camera_calibration.dart';
import '../core/confidence.dart';
import '../core/geometry.dart';
import '../core/kalman.dart';
import '../core/ring_buffer.dart';
import '../perception/detection.dart';
import '../perception/object_class.dart';
import '../sensors/ego_motion.dart';
import 'assignment.dart';
import 'object_track.dart';

/// Tuning constants for [MultiObjectTracker].
class TrackerConfig {
  const TrackerConfig({
    this.maxMissedFrames = 12,
    this.framesToConfirm = 3,
    this.iouGate = 0.10,
    this.metricGateMeters = 6.0,
    this.iouWeight = 0.55,
    this.metricWeight = 0.35,
    this.classWeight = 0.10,
    this.maxCost = 0.82,
    this.historyLength = 48,
    this.processNoisePosition = 0.6,
    this.processNoiseVelocity = 2.5,
    this.measurementNoiseLateral = 0.55,
    this.measurementNoiseLongitudinal = 1.6,
  });

  /// How long a track survives without a detection. At 15 FPS, 12 frames is
  /// ~0.8 s — long enough to ride out a car passing behind a pillar, short
  /// enough that a ghost never lingers into a braking decision.
  final int maxMissedFrames;

  /// Detections must recur before a track is trusted, which is what keeps a
  /// single-frame false positive from triggering a simulated brake.
  final int framesToConfirm;

  final double iouGate;
  final double metricGateMeters;

  final double iouWeight;
  final double metricWeight;
  final double classWeight;

  /// Associations costlier than this are rejected even if the assignment
  /// solver paired them.
  final double maxCost;

  final int historyLength;

  final double processNoisePosition;
  final double processNoiseVelocity;

  /// Lateral position from a monocular camera is far more accurate than
  /// longitudinal distance, and the filter is told so explicitly. Getting this
  /// asymmetry right is what stops distance noise from being interpreted as
  /// violent longitudinal velocity.
  final double measurementNoiseLateral;
  final double measurementNoiseLongitudinal;
}

/// Internal mutable state for one tracked object.
class _TrackState {
  _TrackState({
    required this.id,
    required this.objectClass,
    required this.box,
    required this.position,
    required this.distance,
    required this.distanceConfidence,
    required this.score,
    required this.timestampMicros,
    required int historyLength,
  })  : firstSeenMicros = timestampMicros,
        history = RingBuffer<TrackObservation>(historyLength),
        classBelief = <ObjectClass, double>{objectClass: 1.0} {
    // State = [x, y, vx, vy] in the vehicle frame.
    filter = KalmanFilter(
      stateSize: 4,
      measurementSize: 2,
      initialState: Matrix.fromList(4, 1, <double>[position.x, position.y, 0, 0]),
      // Wide initial velocity covariance: we know nothing about the object's
      // speed from a single frame, and pretending otherwise makes the first
      // few TTC values wildly wrong.
      initialCovariance: Matrix.diagonal(<double>[1.0, 6.0, 25.0, 100.0]),
    );
  }

  final int id;
  ObjectClass objectClass;
  Map<ObjectClass, double> classBelief;
  BoundingBox box;
  Vec2 position;
  double distance;
  Confidence distanceConfidence;
  double score;
  int timestampMicros;
  final int firstSeenMicros;
  final RingBuffer<TrackObservation> history;

  late KalmanFilter filter;
  int age = 1;
  int missedFrames = 0;
  int hits = 1;
  bool confirmed = false;

  Vec2 get filteredPosition => Vec2(filter.state(0), filter.state(1));

  /// The filter's velocity state is the object's velocity **over the ground**:
  /// the prediction step removes the ego vehicle's own translation and
  /// rotation, so what is left is the object's real motion.
  Vec2 get filteredWorldVelocity => Vec2(filter.state(2), filter.state(3));

  /// Velocity confidence from the filter's own covariance plus how long the
  /// track has existed. A one-frame-old track has no usable velocity at all.
  double get velocityConfidence {
    final double sigma =
        math.sqrt(filter.uncertainty(2) * filter.uncertainty(2) +
            filter.uncertainty(3) * filter.uncertainty(3));
    final double fromCovariance = clampDouble(1.0 - sigma / 8.0, 0, 1);
    final double fromAge = clampDouble((hits - 1) / 4.0, 0, 1);
    return clampDouble(fromCovariance * fromAge, 0, 1);
  }
}

/// SORT-style tracker specialised for monocular road scenes.
///
/// Three things separate it from a textbook SORT:
///
///  1. **Ego-motion compensation.** The filter runs in the *vehicle* frame and
///     is corrected for the ego vehicle's own translation and yaw between
///     frames. Without this, every parked car appears to accelerate towards
///     the camera at exactly our own speed, and every TTC is nonsense.
///  2. **Hybrid association cost.** Image-space IoU alone breaks when two
///     vehicles overlap at different distances; metric distance alone breaks
///     when monocular depth is noisy. Combining them (plus a class-consistency
///     term) is much more stable than either.
///  3. **Asymmetric measurement noise.** Lateral position is trustworthy,
///     longitudinal distance is not, and the filter knows it.
class MultiObjectTracker implements ObjectTracker {
  MultiObjectTracker({
    required this.calibration,
    this.config = const TrackerConfig(),
    MotionClassifier? motionClassifier,
  }) : _motionClassifier = motionClassifier ?? const MotionClassifier();

  CameraCalibration calibration;
  final TrackerConfig config;
  final MotionClassifier _motionClassifier;

  final List<_TrackState> _tracks = <_TrackState>[];
  final Map<int, ObjectTrack> _published = <int, ObjectTrack>{};

  int _nextId = 1;
  int _lastTimestampMicros = -1;

  @override
  String get trackerName => 'SORT+ego (Hungarian, hybrid cost)';

  @override
  List<ObjectTrack> get activeTracks =>
      _published.values.where((ObjectTrack t) => t.isConfirmed).toList();

  /// Every track including unconfirmed ones, for the debug overlay.
  List<ObjectTrack> get allTracks => _published.values.toList();

  @override
  List<ObjectTrack> update({
    required DetectionResult detections,
    required EgoMotionState egoMotion,
    required int timestampMicros,
  }) {
    final double dt = _lastTimestampMicros < 0
        ? 0.0
        : (timestampMicros - _lastTimestampMicros) / 1e6;
    _lastTimestampMicros = timestampMicros;

    if (dt > 0 && dt < 2.0) {
      _predict(dt, egoMotion);
    } else if (dt >= 2.0) {
      // A long gap (app backgrounded, camera restarted) invalidates every
      // velocity estimate. Starting clean is safer than carrying stale state.
      reset();
      return const <ObjectTrack>[];
    }

    final List<Detection> usable = detections.detections
        .where((Detection d) =>
            d.score >= ConfidenceThresholds.detectionFloor &&
            !d.objectClass.isInfrastructure)
        .toList();

    final List<Vec2?> measured = <Vec2?>[
      for (final Detection d in usable) _groundPositionFor(d),
    ];

    final List<int> assignment = _associate(usable, measured);

    final Set<int> matchedDetections = <int>{};
    for (int t = 0; t < _tracks.length; t++) {
      final int d = assignment[t];
      if (d < 0) {
        _tracks[t].missedFrames++;
        continue;
      }
      matchedDetections.add(d);
      _updateTrack(_tracks[t], usable[d], measured[d], timestampMicros);
    }

    for (int d = 0; d < usable.length; d++) {
      if (matchedDetections.contains(d)) continue;
      final Vec2? pos = measured[d];
      if (pos == null) continue; // above the horizon: not a road object
      _spawn(usable[d], pos, timestampMicros);
    }

    _tracks.removeWhere((_TrackState t) {
      final bool dead = t.missedFrames > config.maxMissedFrames ||
          (!t.confirmed && t.missedFrames > 1);
      if (dead) _published.remove(t.id);
      return dead;
    });

    return _publish(detections.frameId, timestampMicros, egoMotion);
  }

  // --- Prediction ---------------------------------------------------------

  void _predict(double dt, EgoMotionState ego) {
    // Constant-velocity in the vehicle frame.
    final Matrix f = Matrix.fromList(4, 4, <double>[
      1, 0, dt, 0, //
      0, 1, 0, dt, //
      0, 0, 1, 0, //
      0, 0, 0, 1, //
    ]);
    final double qp = config.processNoisePosition * dt;
    final double qv = config.processNoiseVelocity * dt;
    final Matrix q = Matrix.diagonal(<double>[qp, qp * 2, qv, qv * 2]);

    // Ego motion between frames: the vehicle frame itself rotated by
    // yawRate*dt and translated forward by speed*dt. Objects must be moved
    // into the new frame before the measurement update, otherwise the filter
    // attributes our own motion to them.
    final double dTheta = ego.yawRateRadPerS * dt;
    final double dForward = ego.speedMps * dt;

    for (final _TrackState t in _tracks) {
      t.filter.predict(f, q);

      final Vec2 p = Vec2(t.filter.state(0), t.filter.state(1));
      final Vec2 v = Vec2(t.filter.state(2), t.filter.state(3));

      // Translate, then rotate into the new vehicle frame.
      final Vec2 translated = Vec2(p.x, p.y - dForward);
      final Vec2 rotated = translated.rotated(-dTheta);
      final Vec2 vRotated = v.rotated(-dTheta);

      t.filter.x.set(0, 0, rotated.x);
      t.filter.x.set(1, 0, rotated.y);
      t.filter.x.set(2, 0, vRotated.x);
      t.filter.x.set(3, 0, vRotated.y);
    }
  }

  // --- Association --------------------------------------------------------

  List<int> _associate(List<Detection> detections, List<Vec2?> measured) {
    final int nT = _tracks.length;
    final int nD = detections.length;
    if (nT == 0 || nD == 0) return List<int>.filled(nT, -1);

    final Float64List cost = Float64List(nT * nD);
    for (int t = 0; t < nT; t++) {
      final _TrackState track = _tracks[t];
      final Vec2 predicted = track.filteredPosition;
      for (int d = 0; d < nD; d++) {
        cost[t * nD + d] = _cost(track, predicted, detections[d], measured[d]);
      }
    }

    final List<int> assignment = HungarianAlgorithm.solve(cost, nT, nD);

    // The solver produces a globally optimal pairing even when every pair is
    // implausible, so gate afterwards.
    for (int t = 0; t < nT; t++) {
      final int d = assignment[t];
      if (d >= 0 && cost[t * nD + d] > config.maxCost) {
        assignment[t] = -1;
      }
    }
    return assignment;
  }

  double _cost(
    _TrackState track,
    Vec2 predictedPosition,
    Detection detection,
    Vec2? measuredPosition,
  ) {
    final double iou = track.box.iou(detection.box);

    // Hard gates first: cheap, and they keep the cost matrix meaningful.
    if (iou < config.iouGate) {
      if (measuredPosition == null) return double.infinity;
      final double gap = (measuredPosition - predictedPosition).length;
      // An occluded object can reappear with no box overlap at all, so allow a
      // purely metric match when the geometry agrees closely.
      if (gap > config.metricGateMeters) return double.infinity;
    }

    final double iouCost = 1.0 - iou;

    double metricCost = 0.5;
    if (measuredPosition != null) {
      final double gap = (measuredPosition - predictedPosition).length;
      metricCost = clampDouble(gap / config.metricGateMeters, 0, 1);
    }

    final double classCost = _classCost(track, detection.objectClass);

    return config.iouWeight * iouCost +
        config.metricWeight * metricCost +
        config.classWeight * classCost;
  }

  double _classCost(_TrackState track, ObjectClass candidate) {
    if (track.objectClass == candidate) return 0;
    // Vehicle-to-vehicle confusion is common and harmless for planning;
    // vehicle-to-pedestrian is neither, so it costs much more.
    if (track.objectClass.isVehicle && candidate.isVehicle) return 0.25;
    if (track.objectClass.isVulnerable == candidate.isVulnerable) return 0.6;
    return 1.0;
  }

  // --- Updates ------------------------------------------------------------

  void _updateTrack(
    _TrackState track,
    Detection detection,
    Vec2? measuredPosition,
    int timestampMicros,
  ) {
    track.missedFrames = 0;
    track.hits++;
    track.age++;
    track.timestampMicros = timestampMicros;
    // Smooth the box: raw detector boxes jitter by several pixels frame to
    // frame, and that jitter propagates straight into distance and TTC.
    track.box = track.box.lerp(detection.box, 0.65);
    track.score = track.score * 0.6 + detection.score * 0.4;

    _updateClassBelief(track, detection);

    if (measuredPosition != null) {
      final Matrix h = Matrix.fromList(2, 4, <double>[
        1, 0, 0, 0, //
        0, 1, 0, 0, //
      ]);
      // Longitudinal noise grows with distance: at 60 m a one-pixel error in
      // the contact point is worth several metres.
      final double rangeFactor =
          1.0 + math.pow(measuredPosition.y / 25.0, 2).toDouble();
      final Matrix r = Matrix.diagonal(<double>[
        config.measurementNoiseLateral * math.sqrt(rangeFactor),
        config.measurementNoiseLongitudinal * rangeFactor,
      ]);
      track.filter.update(
        Matrix.fromList(2, 1, <double>[measuredPosition.x, measuredPosition.y]),
        h,
        r,
      );
      track.position = track.filteredPosition;
      track.distance = track.position.length;
      track.distanceConfidence = Confidence(
        calibration.groundDepthConfidenceAtRow(
          detection.box.bottom * calibration.imageHeight,
        ),
        source: 'ground-plane',
      );
    }

    _clampVelocity(track);

    track.history.add(TrackObservation(
      timestampMicros: timestampMicros,
      box: track.box,
      position: track.position,
      distanceMeters: track.distance,
      score: track.score,
    ));

    if (!track.confirmed && track.hits >= config.framesToConfirm) {
      track.confirmed = true;
    }
  }

  void _updateClassBelief(_TrackState track, Detection detection) {
    final Map<ObjectClass, double> belief =
        Map<ObjectClass, double>.from(track.classBelief);
    // Exponential forgetting: recent frames matter more, but one odd frame
    // cannot flip the identity.
    belief.updateAll((ObjectClass k, double v) => v * 0.85);
    belief.update(
      detection.objectClass,
      (double v) => v + detection.score,
      ifAbsent: () => detection.score,
    );
    double total = 0;
    for (final double v in belief.values) {
      total += v;
    }
    if (total > 0) {
      belief.updateAll((ObjectClass k, double v) => v / total);
    }
    track.classBelief = belief;

    ObjectClass best = track.objectClass;
    double bestScore = 0;
    for (final MapEntry<ObjectClass, double> e in belief.entries) {
      if (e.value > bestScore) {
        bestScore = e.value;
        best = e.key;
      }
    }
    // Require a clear majority before renaming a track; flip-flopping labels
    // are worse than a slightly stale one.
    if (bestScore > 0.55) track.objectClass = best;
  }

  /// Reject velocities no object of this class could have. A monocular
  /// distance outlier can otherwise inject a 200 km/h closing speed and
  /// trigger a phantom emergency brake.
  ///
  /// The bound is on the object's **ground speed**, which is what the filter
  /// estimates, so it can be compared directly against a physical limit
  /// without first having to guess how fast we ourselves are going.
  void _clampVelocity(_TrackState track) {
    final double maxSpeed = track.objectClass.maxPlausibleSpeedMps + 5;
    final Vec2 v = track.filteredWorldVelocity;
    if (v.length > maxSpeed) {
      final Vec2 clamped = v.normalized() * maxSpeed;
      track.filter.x.set(2, 0, clamped.x);
      track.filter.x.set(3, 0, clamped.y);
    }
  }

  void _spawn(Detection detection, Vec2 position, int timestampMicros) {
    final _TrackState t = _TrackState(
      id: _nextId++,
      objectClass: detection.objectClass,
      box: detection.box,
      position: position,
      distance: position.length,
      distanceConfidence: Confidence(
        calibration.groundDepthConfidenceAtRow(
          detection.box.bottom * calibration.imageHeight,
        ),
        source: 'ground-plane',
      ),
      score: detection.score,
      timestampMicros: timestampMicros,
      historyLength: config.historyLength,
    );
    t.history.add(TrackObservation(
      timestampMicros: timestampMicros,
      box: detection.box,
      position: position,
      distanceMeters: position.length,
      score: detection.score,
    ));
    _tracks.add(t);
  }

  /// Provisional metric position from the box's ground contact point.
  ///
  /// This is available with no depth model at all — it needs only calibration
  /// — which is why the tracker can run before, and independently of, the
  /// depth stage. [DepthFusion] later refines the distance.
  Vec2? _groundPositionFor(Detection d) {
    final PixelPoint contact = PixelPoint(
      d.box.centerX * calibration.imageWidth,
      d.box.bottom * calibration.imageHeight,
    );
    final Vec2? ground = calibration.projectToGround(contact);
    if (ground != null && ground.y > 0.5 && ground.y < 200) return ground;

    // Contact point above the horizon (occluded base, or a bad box). Fall back
    // to the class size prior, which needs no ground contact.
    final double boxHeightPx = d.box.height * calibration.imageHeight;
    final double? distance = calibration.distanceFromApparentHeight(
      boxHeightPixels: boxHeightPx,
      realHeightMeters: d.objectClass.sizePrior.height,
    );
    if (distance == null || distance <= 0.5 || distance > 200) return null;

    final double lateralAngle =
        (d.box.centerX * calibration.imageWidth - calibration.cx) /
            calibration.fx;
    return Vec2(lateralAngle * distance, distance);
  }

  // --- Publication --------------------------------------------------------

  List<ObjectTrack> _publish(
    int frameId,
    int timestampMicros,
    EgoMotionState ego,
  ) {
    _published.clear();
    for (final _TrackState t in _tracks) {
      final Vec2 worldVelocity = t.filteredWorldVelocity;
      final Vec2 relativeVelocity =
          _toRelativeVelocity(worldVelocity, t.filteredPosition, ego);
      final List<TrackObservation> history = t.history.toList();

      final ObjectTrack published = ObjectTrack(
        id: t.id,
        objectClass: t.objectClass,
        confidence: Confidence(t.score, source: 'track'),
        box: t.box,
        estimatedDistanceMeters: t.distance,
        distanceConfidence: t.distanceConfidence,
        position: t.filteredPosition,
        velocityWorld: worldVelocity,
        relativeVelocity: relativeVelocity,
        velocityConfidence: t.velocityConfidence,
        direction: _motionClassifier.classify(
          relativeVelocity: relativeVelocity,
          worldVelocity: worldVelocity,
          history: history,
          // Path intrusion is decided later, by the collision predictor, which
          // is the stage that knows where the planned path actually goes.
          willEnterEgoPath: false,
        ),
        laneRelation: LaneRelation.unknown,
        laneRelationConfidence: 0,
        collisionRisk: CollisionRisk.low,
        timeToCollisionSeconds: null,
        firstSeenMicros: t.firstSeenMicros,
        lastSeenMicros: t.timestampMicros,
        frameId: frameId,
        age: t.age,
        missedFrames: t.missedFrames,
        isConfirmed: t.confirmed,
        classBelief: t.classBelief,
        history: t.history,
      );
      _published[t.id] = published;
    }
    return activeTracks;
  }

  /// Convert a ground-frame velocity into the rate of change of position as
  /// seen from the moving, rotating vehicle frame.
  ///
  /// Two terms are removed: our own forward translation, and the apparent
  /// motion induced by yaw. The yaw term is not a detail — at 20°/s a car
  /// 40 m ahead appears to slide sideways at 14 m/s, which without this
  /// correction reads as a vehicle cutting across the lane.
  static Vec2 _toRelativeVelocity(
    Vec2 worldVelocity,
    Vec2 position,
    EgoMotionState ego,
  ) {
    final double yaw = ego.yawRateRadPerS;
    return Vec2(
      worldVelocity.x - yaw * position.y,
      worldVelocity.y + yaw * position.x - ego.speedMps,
    );
  }

  /// Feed a refined distance back into the filter.
  ///
  /// Called by [DepthFusion] after it has combined the geometric cue with the
  /// depth network and the size prior. Closing the loop matters: the next
  /// frame's prediction (and therefore the next association) starts from the
  /// better estimate rather than from the raw ground-plane guess.
  void refineDistance(int trackId, double distanceMeters, double confidence) {
    if (confidence <= 0.05) return;
    for (final _TrackState t in _tracks) {
      if (t.id != trackId) continue;

      final Vec2 current = t.filteredPosition;
      final double currentRange = current.length;
      if (currentRange < 0.1) return;

      // Scale the position vector to the refined range, preserving bearing —
      // bearing is a much stronger monocular measurement than range.
      final double scale = distanceMeters / currentRange;
      final Vec2 refined = Vec2(current.x * scale, current.y * scale);

      final Matrix h = Matrix.fromList(2, 4, <double>[
        1, 0, 0, 0, //
        0, 1, 0, 0, //
      ]);
      final double r = clampDouble(3.0 * (1.0 - confidence), 0.15, 8.0);
      t.filter.update(
        Matrix.fromList(2, 1, <double>[refined.x, refined.y]),
        h,
        Matrix.diagonal(<double>[r * 0.4, r]),
      );
      t.position = t.filteredPosition;
      t.distance = t.position.length;
      t.distanceConfidence = Confidence(confidence, source: 'fused');
      _clampVelocity(t);
      return;
    }
  }

  /// Attach the risk assessment computed by the collision predictor.
  void applyRiskAssessment(
    int trackId, {
    required LaneRelation laneRelation,
    required double laneRelationConfidence,
    required CollisionRisk risk,
    double? timeToCollisionSeconds,
    List<Vec2> predictedPath = const <Vec2>[],
    MotionDirection? direction,
  }) {
    final ObjectTrack? existing = _published[trackId];
    if (existing == null) return;
    _published[trackId] = existing.copyWith(
      laneRelation: laneRelation,
      laneRelationConfidence: laneRelationConfidence,
      collisionRisk: risk,
      timeToCollisionSeconds: timeToCollisionSeconds,
      clearTtc: timeToCollisionSeconds == null,
      predictedPath: predictedPath,
      direction: direction,
    );
  }

  @override
  void reset() {
    _tracks.clear();
    _published.clear();
    _lastTimestampMicros = -1;
    // Track ids deliberately keep counting up across resets so that a recorded
    // session never reuses an id for a different object.
  }
}
