import 'dart:math' as math;

import '../core/confidence.dart';
import '../core/geometry.dart';
import '../core/ring_buffer.dart';
import '../perception/object_class.dart';

/// How an object is moving relative to the ego vehicle.
enum MotionDirection {
  approaching('APPROACHING'),
  receding('MOVING AWAY'),
  parallel('PARALLEL'),
  movingLeft('MOVING LEFT'),
  movingRight('MOVING RIGHT'),
  crossingLeftToRight('CROSSING L→R'),
  crossingRightToLeft('CROSSING R→L'),
  enteringPath('ENTERING PATH'),
  stationary('STATIONARY'),
  unknown('UNKNOWN');

  const MotionDirection(this.label);
  final String label;

  bool get isLateral =>
      this == MotionDirection.movingLeft ||
      this == MotionDirection.movingRight ||
      this == MotionDirection.crossingLeftToRight ||
      this == MotionDirection.crossingRightToLeft;
}

/// Where an object sits relative to the ego lane / planned corridor.
enum LaneRelation {
  egoLane('EGO LANE'),
  leftLane('LEFT LANE'),
  rightLane('RIGHT LANE'),
  oncoming('ONCOMING'),
  crossing('CROSSING'),
  offRoad('OFF ROAD'),
  unknown('UNKNOWN');

  const LaneRelation(this.label);
  final String label;

  /// Whether an object in this relation can block the ego vehicle.
  bool get blocksEgoPath =>
      this == LaneRelation.egoLane || this == LaneRelation.crossing;
}

enum CollisionRisk {
  low('LOW', 0),
  medium('MEDIUM', 1),
  high('HIGH', 2),
  critical('CRITICAL', 3);

  const CollisionRisk(this.label, this.severity);
  final String label;
  final int severity;

  bool operator >(CollisionRisk other) => severity > other.severity;
  bool operator >=(CollisionRisk other) => severity >= other.severity;
  bool operator <(CollisionRisk other) => severity < other.severity;
  bool operator <=(CollisionRisk other) => severity <= other.severity;
}

/// One historical observation of a track.
class TrackObservation {
  const TrackObservation({
    required this.timestampMicros,
    required this.box,
    required this.position,
    required this.distanceMeters,
    required this.score,
  });

  final int timestampMicros;
  final BoundingBox box;

  /// Vehicle-frame position (x right, y forward), metres.
  final Vec2 position;
  final double distanceMeters;
  final double score;

  Map<String, dynamic> toJson() => <String, dynamic>{
        'ts': timestampMicros,
        'box': box.toJson(),
        'x': double.parse(position.x.toStringAsFixed(2)),
        'y': double.parse(position.y.toStringAsFixed(2)),
        'd': double.parse(distanceMeters.toStringAsFixed(2)),
        's': double.parse(score.toStringAsFixed(3)),
      };
}

/// A persistent, identified road object.
///
/// This is the central perception product: single-frame [Detection]s are
/// transient, but `MOTORCYCLE #18` keeps its identity across frames, which is
/// what makes velocity, trajectory and time-to-collision meaningful at all.
class ObjectTrack {
  ObjectTrack({
    required this.id,
    required this.objectClass,
    required this.confidence,
    required this.box,
    required this.estimatedDistanceMeters,
    required this.distanceConfidence,
    required this.position,
    required this.velocityWorld,
    required this.relativeVelocity,
    required this.direction,
    required this.laneRelation,
    required this.collisionRisk,
    required this.timeToCollisionSeconds,
    required this.firstSeenMicros,
    required this.lastSeenMicros,
    required this.frameId,
    this.age = 1,
    this.missedFrames = 0,
    this.isConfirmed = false,
    this.velocityConfidence = 0,
    this.laneRelationConfidence = 0,
    this.classBelief = const <ObjectClass, double>{},
    this.predictedPath = const <Vec2>[],
    RingBuffer<TrackObservation>? history,
  }) : history = history ?? RingBuffer<TrackObservation>(48);

  // --- Identity -----------------------------------------------------------

  /// Stable across frames: `CAR #12` stays `#12` for as long as it is
  /// tracked, including through short occlusions.
  final int id;

  final ObjectClass objectClass;
  final Confidence confidence;

  /// Running class belief, so a car does not oscillate into a van and back.
  final Map<ObjectClass, double> classBelief;

  // --- Geometry -----------------------------------------------------------

  final BoundingBox box;

  /// Fused monocular distance, metres. Never exact — see [distanceConfidence].
  final double estimatedDistanceMeters;
  final Confidence distanceConfidence;

  /// Vehicle-frame position, metres (x right of centreline, y forward).
  final Vec2 position;

  // --- Motion -------------------------------------------------------------

  /// The object's own velocity over the ground, m/s, expressed in the
  /// current vehicle frame. This is what the tracking filter actually
  /// estimates, because it is the quantity that stays constant while the ego
  /// vehicle moves and turns.
  final Vec2 velocityWorld;

  /// Velocity **relative to the ego vehicle**: the rate of change of the
  /// object's position in the (moving, rotating) vehicle frame. `y` negative
  /// means the gap is closing. Derived from [velocityWorld] and the ego
  /// motion — this is the quantity time-to-collision is built on.
  final Vec2 relativeVelocity;

  final double velocityConfidence;
  final MotionDirection direction;

  final LaneRelation laneRelation;
  final double laneRelationConfidence;

  // --- Risk ---------------------------------------------------------------

  final CollisionRisk collisionRisk;

  /// Seconds until collision on current relative motion, or `null` when the
  /// object is not closing (TTC is undefined, not "infinite and safe").
  final double? timeToCollisionSeconds;

  // --- Lifecycle ----------------------------------------------------------

  final int firstSeenMicros;
  final int lastSeenMicros;
  final int frameId;

  /// Number of frames in which this track has been updated.
  final int age;

  /// Consecutive frames without a matching detection. Non-zero means the
  /// pose shown is a prediction, not an observation.
  final int missedFrames;

  /// A track only becomes confirmed after several consistent observations,
  /// which is what stops a one-frame false positive triggering a brake.
  final bool isConfirmed;

  final RingBuffer<TrackObservation> history;

  /// Predicted future positions in the vehicle frame, one per planning step.
  final List<Vec2> predictedPath;

  // --- Derived ------------------------------------------------------------

  /// Closing speed, m/s. Positive = the gap is shrinking.
  double get closingSpeedMps => -relativeVelocity.y;

  /// Lateral speed, m/s. Positive = moving right. This is the number that
  /// matters for motorcycles filtering across the lane.
  double get lateralSpeedMps => relativeVelocity.x;

  double get ageSeconds => (lastSeenMicros - firstSeenMicros) / 1e6;

  /// The object's own speed over the ground, m/s.
  double get speedOverGroundMps => velocityWorld.length;

  /// Moving over the ground, as opposed to merely moving relative to us.
  bool get isMoving => velocityWorld.length > 0.7;

  bool get isPredictedOnly => missedFrames > 0;

  /// Line 1 of the on-screen label, e.g. `CAR #12`.
  String get displayLabel => '${objectClass.label.toUpperCase()} #$id';

  /// Compact HUD block:
  /// ```
  /// CAR #12
  /// 18.4m
  /// -4.1m/s
  /// TTC 4.5s
  /// ```
  String get hudText {
    final StringBuffer b = StringBuffer(displayLabel);
    b.write('\n${estimatedDistanceMeters.toStringAsFixed(1)}m');
    if (velocityConfidence > 0.3) {
      final double v = relativeVelocity.y;
      b.write('\n${v >= 0 ? '+' : ''}${v.toStringAsFixed(1)}m/s');
    }
    final double? ttc = timeToCollisionSeconds;
    if (ttc != null && ttc < 15) {
      b.write('\nTTC ${ttc.toStringAsFixed(1)}s');
    }
    return b.toString();
  }

  /// Motorcycle-specific warning used by the HUD. Motorcycles are singled out
  /// because their lateral dynamics are far faster than a car's, so the same
  /// lateral speed is both more likely and more dangerous.
  bool get isMotorcycleCrossing =>
      objectClass == ObjectClass.motorcycle &&
      lateralSpeedMps.abs() > 1.2 &&
      (laneRelation == LaneRelation.crossing ||
          direction == MotionDirection.enteringPath);

  ObjectTrack copyWith({
    ObjectClass? objectClass,
    Confidence? confidence,
    BoundingBox? box,
    double? estimatedDistanceMeters,
    Confidence? distanceConfidence,
    Vec2? position,
    Vec2? velocityWorld,
    Vec2? relativeVelocity,
    double? velocityConfidence,
    MotionDirection? direction,
    LaneRelation? laneRelation,
    double? laneRelationConfidence,
    CollisionRisk? collisionRisk,
    double? timeToCollisionSeconds,
    bool clearTtc = false,
    int? lastSeenMicros,
    int? frameId,
    int? age,
    int? missedFrames,
    bool? isConfirmed,
    Map<ObjectClass, double>? classBelief,
    List<Vec2>? predictedPath,
  }) =>
      ObjectTrack(
        id: id,
        objectClass: objectClass ?? this.objectClass,
        confidence: confidence ?? this.confidence,
        box: box ?? this.box,
        estimatedDistanceMeters:
            estimatedDistanceMeters ?? this.estimatedDistanceMeters,
        distanceConfidence: distanceConfidence ?? this.distanceConfidence,
        position: position ?? this.position,
        velocityWorld: velocityWorld ?? this.velocityWorld,
        relativeVelocity: relativeVelocity ?? this.relativeVelocity,
        velocityConfidence: velocityConfidence ?? this.velocityConfidence,
        direction: direction ?? this.direction,
        laneRelation: laneRelation ?? this.laneRelation,
        laneRelationConfidence:
            laneRelationConfidence ?? this.laneRelationConfidence,
        collisionRisk: collisionRisk ?? this.collisionRisk,
        timeToCollisionSeconds: clearTtc
            ? null
            : (timeToCollisionSeconds ?? this.timeToCollisionSeconds),
        firstSeenMicros: firstSeenMicros,
        lastSeenMicros: lastSeenMicros ?? this.lastSeenMicros,
        frameId: frameId ?? this.frameId,
        age: age ?? this.age,
        missedFrames: missedFrames ?? this.missedFrames,
        isConfirmed: isConfirmed ?? this.isConfirmed,
        classBelief: classBelief ?? this.classBelief,
        predictedPath: predictedPath ?? this.predictedPath,
        history: history,
      );

  Map<String, dynamic> toJson() => <String, dynamic>{
        'id': id,
        'cls': objectClass.name,
        'conf': confidence.toJson(),
        'box': box.toJson(),
        'dist': double.parse(estimatedDistanceMeters.toStringAsFixed(2)),
        'distConf': distanceConfidence.toJson(),
        'x': double.parse(position.x.toStringAsFixed(2)),
        'y': double.parse(position.y.toStringAsFixed(2)),
        'vx': double.parse(relativeVelocity.x.toStringAsFixed(2)),
        'vy': double.parse(relativeVelocity.y.toStringAsFixed(2)),
        'wvx': double.parse(velocityWorld.x.toStringAsFixed(2)),
        'wvy': double.parse(velocityWorld.y.toStringAsFixed(2)),
        'vConf': double.parse(velocityConfidence.toStringAsFixed(3)),
        'dir': direction.name,
        'lane': laneRelation.name,
        'laneConf': double.parse(laneRelationConfidence.toStringAsFixed(3)),
        'risk': collisionRisk.name,
        if (timeToCollisionSeconds != null)
          'ttc': double.parse(timeToCollisionSeconds!.toStringAsFixed(2)),
        'firstSeen': firstSeenMicros,
        'lastSeen': lastSeenMicros,
        'age': age,
        'missed': missedFrames,
        'confirmed': isConfirmed,
      };

  static ObjectTrack fromJson(Map<String, dynamic> j, {required int frameId}) =>
      ObjectTrack(
        id: (j['id'] as num).toInt(),
        objectClass: ObjectClass.fromName(j['cls'] as String),
        confidence: Confidence.fromJson(j['conf'] as Map<String, dynamic>),
        box: BoundingBox.fromJson(j['box'] as Map<String, dynamic>),
        estimatedDistanceMeters: (j['dist'] as num).toDouble(),
        distanceConfidence:
            Confidence.fromJson(j['distConf'] as Map<String, dynamic>),
        position: Vec2((j['x'] as num).toDouble(), (j['y'] as num).toDouble()),
        relativeVelocity:
            Vec2((j['vx'] as num).toDouble(), (j['vy'] as num).toDouble()),
        velocityWorld: Vec2(
          (j['wvx'] as num?)?.toDouble() ?? 0,
          (j['wvy'] as num?)?.toDouble() ?? 0,
        ),
        velocityConfidence: (j['vConf'] as num).toDouble(),
        direction: MotionDirection.values.firstWhere(
          (MotionDirection d) => d.name == j['dir'],
          orElse: () => MotionDirection.unknown,
        ),
        laneRelation: LaneRelation.values.firstWhere(
          (LaneRelation r) => r.name == j['lane'],
          orElse: () => LaneRelation.unknown,
        ),
        laneRelationConfidence: (j['laneConf'] as num).toDouble(),
        collisionRisk: CollisionRisk.values.firstWhere(
          (CollisionRisk r) => r.name == j['risk'],
          orElse: () => CollisionRisk.low,
        ),
        timeToCollisionSeconds: (j['ttc'] as num?)?.toDouble(),
        firstSeenMicros: (j['firstSeen'] as num).toInt(),
        lastSeenMicros: (j['lastSeen'] as num).toInt(),
        frameId: frameId,
        age: (j['age'] as num?)?.toInt() ?? 1,
        missedFrames: (j['missed'] as num?)?.toInt() ?? 0,
        isConfirmed: j['confirmed'] as bool? ?? false,
      );

  @override
  String toString() => '$displayLabel ${estimatedDistanceMeters.toStringAsFixed(1)}m '
      '${direction.label} ${laneRelation.label} risk=${collisionRisk.label}'
      '${timeToCollisionSeconds == null ? '' : ' TTC ${timeToCollisionSeconds!.toStringAsFixed(1)}s'}';
}

/// Classifies an object's motion pattern from its recent trajectory.
///
/// Uses the *relative* velocity in the vehicle frame plus the object's lateral
/// trend over the whole history window, because a single frame-to-frame delta
/// on a noisy monocular distance is far too jittery to name a direction from.
class MotionClassifier {
  const MotionClassifier({
    this.lateralSpeedThreshold = 0.6,
    this.longitudinalSpeedThreshold = 0.8,
    this.stationarySpeedThreshold = 0.5,
  });

  final double lateralSpeedThreshold;
  final double longitudinalSpeedThreshold;
  final double stationarySpeedThreshold;

  MotionDirection classify({
    required Vec2 relativeVelocity,
    required Vec2 worldVelocity,
    required List<TrackObservation> history,
    required bool willEnterEgoPath,
  }) {
    if (willEnterEgoPath) return MotionDirection.enteringPath;

    // "Stationary" is a statement about the world, so it is answered from the
    // ground-frame velocity and answered first: a parked car we are closing
    // on at 14 m/s has a large *relative* closing speed but is not moving,
    // and calling it "approaching" would be actively misleading.
    if (worldVelocity.length < stationarySpeedThreshold) {
      return MotionDirection.stationary;
    }

    // Everything below is reported relative to us, because that is what the
    // driver sees on the HUD and what the risk logic reasons about.
    final double vx = relativeVelocity.x;
    final double vy = relativeVelocity.y;

    final bool lateral = vx.abs() > lateralSpeedThreshold;
    final bool longitudinal = vy.abs() > longitudinalSpeedThreshold;

    if (lateral && !longitudinal) {
      // Pure lateral motion across our path.
      final bool sustained = _lateralTrendSustained(history, vx);
      if (sustained) {
        return vx > 0
            ? MotionDirection.crossingLeftToRight
            : MotionDirection.crossingRightToLeft;
      }
      return vx > 0 ? MotionDirection.movingRight : MotionDirection.movingLeft;
    }

    if (lateral && longitudinal && vx.abs() > vy.abs() * 1.3) {
      // Both, but lateral dominates; the HUD has one line, so report that.
      return vx > 0 ? MotionDirection.movingRight : MotionDirection.movingLeft;
    }

    if (!longitudinal && !lateral) {
      // Moving over the ground at roughly our own velocity: travelling with us.
      return MotionDirection.parallel;
    }

    return vy < 0 ? MotionDirection.approaching : MotionDirection.receding;
  }

  /// Confirm a lateral trend across the history window rather than trusting a
  /// single velocity estimate.
  bool _lateralTrendSustained(List<TrackObservation> history, double vx) {
    if (history.length < 4) return false;
    final int n = math.min(8, history.length);
    final List<TrackObservation> recent =
        history.sublist(history.length - n);
    final double first = recent.first.position.x;
    final double last = recent.last.position.x;
    final double drift = last - first;
    if (drift.abs() < 0.5) return false;
    return drift.sign == vx.sign;
  }
}
