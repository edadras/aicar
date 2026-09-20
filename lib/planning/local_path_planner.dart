import 'dart:math' as math;

import '../ai/interfaces/local_planner.dart';
import '../core/geometry.dart';
import '../navigation/maneuver.dart';
import '../perception/object_class.dart';
import '../road/lane.dart';
import '../tracking/object_track.dart';
import '../world_model/world_state.dart';
import 'planned_path.dart';

/// Tuning for [LocalPathPlanner].
class PlannerConfig {
  const PlannerConfig({
    this.maxRangeMeters = 45,
    this.stepMeters = 2.0,
    this.lateralOffsetRange = 1.6,
    this.lateralOffsetStep = 0.2,
    this.vehicleHalfWidthMeters = 0.95,
    this.minClearanceMeters = 0.45,
    this.comfortLateralAccelMps2 = 1.8,
    this.maxLateralAccelMps2 = 3.2,
    this.comfortDecelMps2 = 2.0,
    this.maxDecelMps2 = 6.5,
    this.continuityWeight = 1.4,
    this.centeringWeight = 1.0,
    this.clearanceWeight = 2.2,
    this.comfortWeight = 0.8,
    this.temporalSmoothing = 0.55,
    this.maxLateralRateMps = 1.5,
  });

  final double maxRangeMeters;
  final double stepMeters;

  /// How far either side of the reference line candidate paths are sampled.
  /// Wider than a lane would let the planner propose leaving it, which is not
  /// this planner's job — lane changes are a separate, explicit simulation.
  final double lateralOffsetRange;
  final double lateralOffsetStep;

  final double vehicleHalfWidthMeters;
  final double minClearanceMeters;

  final double comfortLateralAccelMps2;
  final double maxLateralAccelMps2;
  final double comfortDecelMps2;
  final double maxDecelMps2;

  final double continuityWeight;
  final double centeringWeight;
  final double clearanceWeight;
  final double comfortWeight;
  final double temporalSmoothing;

  /// Ceiling on how fast the planned line may move sideways, m/s.
  ///
  /// Smoothing alone is a *fraction* of the remaining error, so a large step
  /// change in the target still produces a large first move — at 20 FPS a 0.55
  /// smoothing factor still shifts the line 0.36 m in 50 ms, which is 7 m/s of
  /// lateral rate and would be a violent swerve. Limiting the rate directly
  /// bounds the manoeuvre in the units that actually matter.
  final double maxLateralRateMps;
}

/// Sampling-lattice local planner.
///
/// Generates a family of candidate paths as lateral offsets from the road's
/// reference line, scores each against clearance, centring, comfort and
/// continuity with the previous plan, and returns the best.
///
/// A lattice is used rather than a single centre-following line because a
/// single line has no way to express "move over slightly for the cyclist" —
/// it can only either follow the lane or not. Scoring candidates also makes
/// the planner's reasoning inspectable: the debug overlay can show every
/// candidate and why it lost.
///
/// The planner deliberately will not invent geometry. If the world model has
/// no reference line — no usable lanes and no corridor — it returns
/// [PlannedPath.none], and the decision engine goes to `UNCERTAIN`.
class LocalPathPlanner implements LocalPlanner {
  LocalPathPlanner({this.config = const PlannerConfig()});

  final PlannerConfig config;

  PlannedPath? _previous;

  @override
  String get plannerName => 'Sampling lattice (offset candidates)';

  @override
  void reset() => _previous = null;

  @override
  PlannedPath plan(WorldState world, {PlannedPath? previous}) {
    final PlannedPath? last = previous ?? _previous;

    final _Reference? reference = _resolveReference(world);
    if (reference == null) {
      // No road model at all. Holding the previous path for a frame or two is
      // reasonable; inventing a straight line is not.
      final PlannedPath fallback = _holdPrevious(world, last);
      _previous = fallback;
      return fallback;
    }

    final double range = math.min(
      config.maxRangeMeters,
      math.max(10.0, reference.rangeMeters),
    );

    final List<_Candidate> candidates = <_Candidate>[];
    for (double offset = -config.lateralOffsetRange;
        offset <= config.lateralOffsetRange + 1e-9;
        offset += config.lateralOffsetStep) {
      final _Candidate? c =
          _evaluateCandidate(world, reference, offset, range, last);
      if (c != null) candidates.add(c);
    }

    if (candidates.isEmpty) {
      // Every offset is blocked. That is a real and important finding — the
      // road ahead is impassable — not a reason to plan through an obstacle.
      final PlannedPath blocked = _blockedPath(world, reference, range);
      _previous = blocked;
      return blocked;
    }

    candidates.sort((_Candidate a, _Candidate b) => a.cost.compareTo(b.cost));
    final _Candidate best = candidates.first;

    final PlannedPath path = _materialize(world, reference, best, range, last);
    _previous = path;
    return path;
  }

  // --- Reference line -----------------------------------------------------

  _Reference? _resolveReference(WorldState world) {
    if (world.lanes.overallConfidence >= 0.45) {
      final Polynomial? centre = world.lanes.centerline;
      if (centre != null && world.lanes.usableRangeMeters > 8) {
        return _Reference(
          curve: centre,
          halfWidth: world.lanes.laneWidthMeters / 2,
          rangeMeters: world.lanes.usableRangeMeters,
          source: PathSource.laneCenterline,
          confidence: world.lanes.overallConfidence,
        );
      }
    }

    final corridor = world.corridor;
    if (corridor != null && corridor.confidence > 0.2) {
      return _Reference(
        curve: corridor.centerline,
        halfWidth: corridor.halfWidthMeters,
        rangeMeters: corridor.maxRangeMeters,
        source: PathSource.corridorCenterline,
        // NO_LANE_MODE is inherently less certain, and the path says so.
        confidence: corridor.confidence * 0.85,
      );
    }

    // A single lane boundary with a tracked width is still a real reference.
    if (world.lanes.mode == LaneMode.singleBoundary) {
      final Polynomial? centre = world.lanes.centerline;
      if (centre != null) {
        return _Reference(
          curve: centre,
          halfWidth: world.lanes.laneWidthMeters / 2,
          rangeMeters: math.max(12, world.lanes.usableRangeMeters),
          source: PathSource.laneCenterline,
          confidence: world.lanes.overallConfidence * 0.8,
        );
      }
    }

    return null;
  }

  // --- Candidate evaluation ----------------------------------------------

  _Candidate? _evaluateCandidate(
    WorldState world,
    _Reference reference,
    double offset,
    double range,
    PlannedPath? previous,
  ) {
    double clearanceCost = 0;
    double minClearance = double.infinity;
    double? blockedAt;

    final List<Vec2> positions = <Vec2>[];

    for (double d = config.stepMeters; d <= range; d += config.stepMeters) {
      final double lateral = reference.curve.evaluate(d) + offset;
      final Vec2 point = Vec2(lateral, d);
      positions.add(point);

      // Stay inside the drivable corridor when we have one.
      final (double, double)? limits = world.drivableArea.limitsAt(d);
      if (limits != null) {
        final double left = limits.$1 + config.vehicleHalfWidthMeters;
        final double right = limits.$2 - config.vehicleHalfWidthMeters;
        if (right - left < 0.2) {
          blockedAt = d;
          break;
        }
        if (lateral < left - 0.3 || lateral > right + 0.3) {
          // Off the drivable surface: this candidate is not viable at all.
          return null;
        }
      }

      final double clearance = _clearanceAt(world, point);
      if (clearance < minClearance) minClearance = clearance;

      if (clearance < config.minClearanceMeters) {
        blockedAt = d;
        break;
      }
      // Cost grows sharply as clearance shrinks: 2 m of room is fine, 0.6 m
      // is not, and the difference should dominate the other terms.
      final double normalized =
          clampDouble(clearance / 2.0, 0.05, 1.0);
      clearanceCost += (1.0 / normalized - 1.0) * config.stepMeters / range;
    }

    if (positions.length < 3) return null;

    final double plannedRange = blockedAt ?? range;
    // A path that stops 5 m ahead is not a path.
    if (plannedRange < 6 && blockedAt != null) {
      return _Candidate(
        offset: offset,
        positions: positions,
        cost: 1e6, // viable only if literally nothing else is
        minClearance: minClearance,
        blockedAt: blockedAt,
        plannedRange: plannedRange,
      );
    }

    // Centring: prefer the reference line unless there is a reason to move.
    final double centeringCost = (offset * offset) / 2.0;

    // Continuity: jumping laterally between frames looks (and would feel)
    // like swerving, so it is penalised even when the new line scores better.
    double continuityCost = 0;
    if (previous != null && previous.isUsable) {
      final double previousOffset = previous.lateralOffsetFromReference;
      final double jump = (offset - previousOffset).abs();
      continuityCost = jump * jump;
    }

    // Comfort: the lateral acceleration this path would demand.
    final double comfortCost =
        _comfortCost(reference, offset, world.ego.speedMps, range);

    // Navigation bias: nudge towards the side of an upcoming manoeuvre. Tiny
    // by design — navigation hints, it does not steer.
    double navigationCost = 0;
    final ManeuverIntent intent = world.navigationIntent;
    if (intent.lateralBiasSign != 0) {
      final double preferred = intent.lateralBiasSign * 0.4;
      navigationCost = 0.25 * (offset - preferred).abs();
    }

    final double cost = config.clearanceWeight * clearanceCost +
        config.centeringWeight * centeringCost +
        config.continuityWeight * continuityCost +
        config.comfortWeight * comfortCost +
        navigationCost +
        (blockedAt != null ? 4.0 : 0.0);

    return _Candidate(
      offset: offset,
      positions: positions,
      cost: cost,
      minClearance: minClearance,
      blockedAt: blockedAt,
      plannedRange: plannedRange,
    );
  }

  /// Lateral clearance from [point] to the nearest obstacle's swept footprint.
  ///
  /// The footprint is swept from the object's current position to where it is
  /// predicted to be, rather than tested only at the prediction. This matters
  /// most for the case the project cares about most: a motorcycle crossing the
  /// lane. Testing only the predicted point makes the space the motorcycle is
  /// *currently* occupying look free, and the planner cheerfully steers into
  /// it — moving towards a fast-crossing vehicle, which is precisely the wrong
  /// response. Treating the whole swept band as occupied makes it hold its
  /// line and let the decision engine slow down instead.
  double _clearanceAt(WorldState world, Vec2 point) {
    double minimum = double.infinity;
    final Polynomial? laneCentre = world.referenceCenterline;

    for (final ObjectTrack t in world.tracks) {
      if (!t.isConfirmed) continue;
      if (!t.objectClass.isObstacle) continue;

      final PhysicalSizePrior size = t.objectClass.sizePrior;

      // Traffic flowing with us, squarely in our own lane, is a *speed*
      // constraint, not a geometric obstruction: it is handled by the follow
      // controller and by the collision predictor's headway logic. Treating a
      // moving lead vehicle as a lateral blockage would report the road as
      // impassable every time we caught up with slower traffic, and would
      // have the planner swerving around cars it should simply follow.
      //
      // An object that is only *partly* in the lane is kept, because moving
      // over for it is exactly what the lattice is for.
      if (t.velocityWorld.y > 2.0 && t.position.y > 0) {
        final double laneLateral = laneCentre?.evaluate(t.position.y) ?? 0.0;
        final bool squarelyInLane = (t.position.x - laneLateral).abs() <
            size.width / 2 + config.vehicleHalfWidthMeters;
        if (squarelyInLane) continue;
      }

      // Time until we reach this point on the path.
      final double travelTime = world.ego.speedMps > 0.5
          ? point.y / math.max(world.ego.speedMps, 0.5)
          : 0;
      final Vec2 predicted = Vec2(
        t.position.x + t.relativeVelocity.x * travelTime,
        t.position.y + t.relativeVelocity.y * travelTime,
      );

      final double halfLength = size.length / 2;
      final double sweptNear =
          math.min(t.position.y, predicted.y) - halfLength;
      final double sweptFar =
          math.max(t.position.y, predicted.y) + halfLength;
      // Only objects whose swept band covers this part of the path matter.
      if (point.y < sweptNear - 1.0 || point.y > sweptFar + 1.0) continue;

      final double halfWidth =
          size.width / 2 + config.vehicleHalfWidthMeters;
      final double sweptLeft =
          math.min(t.position.x, predicted.x) - halfWidth;
      final double sweptRight =
          math.max(t.position.x, predicted.x) + halfWidth;

      // Distance from the path point to the swept lateral interval.
      double lateralGap;
      if (point.x < sweptLeft) {
        lateralGap = sweptLeft - point.x;
      } else if (point.x > sweptRight) {
        lateralGap = point.x - sweptRight;
      } else {
        lateralGap = -math.min(point.x - sweptLeft, sweptRight - point.x);
      }

      // Passing margin, widened when we are less sure where the object is.
      // This is the *lateral* margin, not the longitudinal stopping margin —
      // see ObjectClass.lateralSafetyMarginMeters.
      final double uncertainty =
          1.0 - clampDouble(t.distanceConfidence.value, 0, 1);
      lateralGap -=
          t.objectClass.lateralSafetyMarginMeters * (0.7 + 0.6 * uncertainty);

      if (lateralGap < minimum) minimum = lateralGap;
    }

    return minimum.isFinite ? minimum : 10.0;
  }

  /// Penalty for the lateral acceleration a candidate demands at the current
  /// speed. Offsetting sideways over a short distance is a sharp manoeuvre.
  double _comfortCost(
    _Reference reference,
    double offset,
    double speedMps,
    double range,
  ) {
    if (speedMps < 1.0) return 0;

    // Curvature of the reference itself, plus the extra needed to reach the
    // offset over the first third of the path.
    final double referenceCurvature = reference.curve.curvatureAt(10);
    final double transitionDistance = math.max(8.0, range / 3);
    final double offsetCurvature =
        2 * offset.abs() / (transitionDistance * transitionDistance);

    final double totalCurvature =
        referenceCurvature.abs() + offsetCurvature;
    final double lateralAccel = totalCurvature * speedMps * speedMps;

    if (lateralAccel <= config.comfortLateralAccelMps2) {
      return lateralAccel / config.comfortLateralAccelMps2 * 0.3;
    }
    // Beyond comfortable, the cost rises steeply.
    final double excess =
        (lateralAccel - config.comfortLateralAccelMps2) /
            math.max(0.1, config.maxLateralAccelMps2 -
                config.comfortLateralAccelMps2);
    return 0.3 + excess * excess * 3.0;
  }

  // --- Materialisation ----------------------------------------------------

  PlannedPath _materialize(
    WorldState world,
    _Reference reference,
    _Candidate best,
    double range,
    PlannedPath? previous,
  ) {
    // Blend the offset with the previous frame's, then bound the resulting
    // lateral rate, so the drawn path and the steering command move smoothly
    // rather than snapping between candidates.
    double offset = best.offset;
    if (previous != null && previous.isUsable) {
      final double dt = clampDouble(
        (world.timestampMicros - previous.timestampMicros) / 1e6,
        0.01,
        0.5,
      );
      final double smoothed = lerpDouble(
        previous.lateralOffsetFromReference,
        best.offset,
        1 - config.temporalSmoothing,
      );
      final double maxStep = config.maxLateralRateMps * dt;
      final double delta = smoothed - previous.lateralOffsetFromReference;
      offset = previous.lateralOffsetFromReference +
          clampDouble(delta, -maxStep, maxStep);
    }

    final Polynomial curve = Polynomial(<double>[
      reference.curve.coefficients[0] + offset,
      ...reference.curve.coefficients.skip(1),
    ]);

    final double plannedRange = best.plannedRange;
    final List<PathPoint> points = <PathPoint>[];
    double arcLength = 0;
    Vec2? lastPosition;

    for (double d = 0; d <= plannedRange; d += config.stepMeters) {
      final double lateral = curve.evaluate(d);
      final Vec2 position = Vec2(lateral, d);
      if (lastPosition != null) {
        arcLength += position.distanceTo(lastPosition);
      }
      lastPosition = position;

      final double curvature = curve.curvatureAt(d);
      final double clearance = _clearanceAt(world, position);

      points.add(PathPoint(
        position: position,
        distanceAlong: arcLength,
        headingRadians: math.atan(curve.derivative(d)),
        curvature: curvature,
        targetSpeedMps: _targetSpeedAt(world, d, curvature, clearance),
        lateralClearance: clearance,
      ));
    }

    final PathSource source = best.offset.abs() > 0.35
        ? PathSource.obstacleAvoidance
        : reference.source;

    // Confidence is the road model's, discounted by how constrained the path
    // was: a path threading a 0.5 m gap is a much weaker proposal than one
    // running down the middle of an empty lane.
    final double clearanceQuality =
        clampDouble(best.minClearance / 1.5, 0.15, 1.0);
    final double rangeQuality = clampDouble(plannedRange / 30.0, 0.2, 1.0);
    final double confidence = clampDouble(
      reference.confidence * (0.4 + 0.35 * clearanceQuality + 0.25 * rangeQuality),
      0,
      1,
    );

    return PlannedPath(
      points: points,
      curve: curve,
      source: source,
      confidence: confidence,
      lateralOffsetFromReference: offset,
      maxRangeMeters: plannedRange,
      frameId: world.frameId,
      timestampMicros: world.timestampMicros,
      isBlocked: best.blockedAt != null,
      blockedAtMeters: best.blockedAt,
      blockReason: best.blockedAt == null
          ? null
          : 'insufficient clearance at '
              '${best.blockedAt!.toStringAsFixed(0)} m',
      corridorHalfWidth: math.min(
        reference.halfWidth,
        config.vehicleHalfWidthMeters + 0.6,
      ),
    );
  }

  /// Speed the path allows here, from curvature, clearance and the rules.
  double _targetSpeedAt(
    WorldState world,
    double distance,
    double curvature,
    double clearance,
  ) {
    // Start from the posted limit, or from the current speed when there is no
    // limit to work with. Never invent a target above the current speed on no
    // evidence — this stack does not propose accelerating into the unknown.
    final int? limit = world.effectiveSpeedLimitKph;
    double target = limit != null
        ? limit / 3.6
        : math.max(world.ego.speedMps, 8.3);

    final double? regulatoryTarget = world.regulatory.targetSpeedMps;
    if (regulatoryTarget != null) {
      target = math.min(target, regulatoryTarget);
    }

    // Curvature limit: v = sqrt(a_lat / k).
    if (curvature.abs() > 1e-5) {
      final double curveSpeed =
          math.sqrt(config.comfortLateralAccelMps2 / curvature.abs());
      target = math.min(target, curveSpeed);
    }

    // Tight clearance means slow down, regardless of what the sign says.
    if (clearance < 1.5) {
      target = math.min(target, 3.0 + clearance * 4.0);
    }

    // Do not plan to travel faster than we can see. At the edge of the road
    // model, the target decays to a speed from which we could stop within the
    // sensed range.
    final double sensedRange = world.roadModelRangeMeters;
    if (sensedRange > 2) {
      final double stoppingSpeed =
          math.sqrt(2 * config.comfortDecelMps2 * sensedRange);
      target = math.min(target, stoppingSpeed);
    }

    return clampDouble(target, 0, 40);
  }

  PlannedPath _blockedPath(
    WorldState world,
    _Reference reference,
    double range,
  ) {
    final List<PathPoint> points = <PathPoint>[];
    for (double d = 0; d <= math.min(range, 8); d += config.stepMeters) {
      final double lateral = reference.curve.evaluate(d);
      points.add(PathPoint(
        position: Vec2(lateral, d),
        distanceAlong: d,
        headingRadians: math.atan(reference.curve.derivative(d)),
        curvature: reference.curve.curvatureAt(d),
        targetSpeedMps: 0,
        lateralClearance: 0,
      ));
    }
    return PlannedPath(
      points: points,
      curve: reference.curve,
      source: reference.source,
      confidence: reference.confidence * 0.4,
      lateralOffsetFromReference: 0,
      maxRangeMeters: math.min(range, 8),
      frameId: world.frameId,
      timestampMicros: world.timestampMicros,
      isBlocked: true,
      blockedAtMeters: 0,
      blockReason: 'no viable path: every candidate blocked',
    );
  }

  /// Reuse the previous path when the road model drops out for a moment.
  ///
  /// Time-limited on purpose: a stale path is useful for the fraction of a
  /// second it takes a detector to recover, and dangerous after that.
  PlannedPath _holdPrevious(WorldState world, PlannedPath? previous) {
    if (previous == null || !previous.isUsable) {
      return PlannedPath.none(
        frameId: world.frameId,
        timestampMicros: world.timestampMicros,
        reason: 'no lane model and no corridor evidence',
      );
    }

    final double ageSeconds =
        (world.timestampMicros - previous.timestampMicros) / 1e6;
    if (ageSeconds > 0.6) {
      return PlannedPath.none(
        frameId: world.frameId,
        timestampMicros: world.timestampMicros,
        reason: 'road model lost for '
            '${ageSeconds.toStringAsFixed(1)} s',
      );
    }

    // Shift the held path backwards by how far we have travelled since.
    final double travelled = world.ego.speedMps * ageSeconds;
    final double remainingRange =
        math.max(0, previous.maxRangeMeters - travelled);

    return PlannedPath(
      points: previous.points
          .where((PathPoint p) => p.position.y > travelled)
          .map((PathPoint p) => PathPoint(
                position: Vec2(p.position.x, p.position.y - travelled),
                distanceAlong: math.max(0, p.distanceAlong - travelled),
                headingRadians: p.headingRadians,
                curvature: p.curvature,
                targetSpeedMps: p.targetSpeedMps,
                lateralClearance: p.lateralClearance,
              ))
          .toList(),
      curve: previous.curve,
      source: PathSource.previousPathHold,
      // Held paths decay quickly in confidence, which is what causes the
      // decision engine to reach UNCERTAIN rather than drive on blind.
      confidence: previous.confidence * clampDouble(1 - ageSeconds / 0.6, 0, 1),
      lateralOffsetFromReference: previous.lateralOffsetFromReference,
      maxRangeMeters: remainingRange,
      frameId: world.frameId,
      timestampMicros: world.timestampMicros,
      corridorHalfWidth: previous.corridorHalfWidth,
    );
  }
}

class _Reference {
  const _Reference({
    required this.curve,
    required this.halfWidth,
    required this.rangeMeters,
    required this.source,
    required this.confidence,
  });

  final Polynomial curve;
  final double halfWidth;
  final double rangeMeters;
  final PathSource source;
  final double confidence;
}

class _Candidate {
  const _Candidate({
    required this.offset,
    required this.positions,
    required this.cost,
    required this.minClearance,
    required this.blockedAt,
    required this.plannedRange,
  });

  final double offset;
  final List<Vec2> positions;
  final double cost;
  final double minClearance;
  final double? blockedAt;
  final double plannedRange;
}
