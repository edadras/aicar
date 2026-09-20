import 'dart:math' as math;

import '../core/geometry.dart';
import '../perception/object_class.dart';
import '../tracking/object_track.dart';
import '../world_model/world_state.dart';
import 'planned_path.dart';

/// Result of assessing one tracked object against the planned path.
class CollisionAssessment {
  const CollisionAssessment({
    required this.trackId,
    required this.laneRelation,
    required this.laneRelationConfidence,
    required this.risk,
    required this.timeToCollisionSeconds,
    required this.predictedPath,
    required this.minimumGapMeters,
    required this.willEnterPath,
    required this.reason,
  });

  final int trackId;
  final LaneRelation laneRelation;
  final double laneRelationConfidence;
  final CollisionRisk risk;

  /// Seconds to collision, or `null` when there is no collision on the current
  /// trajectories. `null` means "not closing", not "safe forever".
  final double? timeToCollisionSeconds;

  final List<Vec2> predictedPath;

  /// Closest the object's footprint comes to the ego footprint, metres.
  /// Negative means the footprints overlap at some point in the horizon.
  final double minimumGapMeters;

  final bool willEnterPath;

  /// Why this risk level was assigned, for the debug overlay and the log.
  final String reason;
}

/// Predicts collisions between the ego vehicle following its planned path and
/// every tracked object following its own predicted trajectory.
///
/// Three things this does that a simple "distance / closing speed" TTC does
/// not, and which matter on a real road:
///
///  * It propagates **both** bodies forward and tests their *footprints*, so
///    a car passing 3 m to the side at 20 m/s does not register as a
///    collision just because the gap is closing fast.
///  * It handles **lateral** motion explicitly, which is the whole reason
///    motorcycles filtering between lanes are dangerous: their longitudinal
///    closing speed can be zero while they are about to be in front of us.
///  * It propagates **uncertainty**: an object whose distance is poorly known
///    gets a wider footprint, so a doubtful measurement produces a cautious
///    assessment rather than a confident wrong one.
class CollisionPredictor {
  const CollisionPredictor({
    this.horizonSeconds = 5.0,
    this.stepSeconds = 0.2,
    this.egoHalfWidthMeters = 0.95,
    this.egoLengthMeters = 4.5,
    this.criticalTtcSeconds = 1.6,
    this.highTtcSeconds = 2.8,
    this.mediumTtcSeconds = 4.5,
    this.vulnerableTtcBonus = 1.0,
  });

  /// How far ahead to simulate. Beyond ~5 s a constant-velocity prediction of
  /// another road user is not worth much.
  final double horizonSeconds;

  final double stepSeconds;
  final double egoHalfWidthMeters;
  final double egoLengthMeters;

  final double criticalTtcSeconds;
  final double highTtcSeconds;
  final double mediumTtcSeconds;

  /// Vulnerable road users escalate one threshold earlier, because the
  /// consequence is not symmetric and their motion is less predictable.
  final double vulnerableTtcBonus;

  /// Assess every track against [path].
  List<CollisionAssessment> assess(WorldState world, PlannedPath path) {
    final List<CollisionAssessment> out = <CollisionAssessment>[];
    for (final ObjectTrack track in world.tracks) {
      if (!track.isConfirmed) continue;
      out.add(_assessOne(world, path, track));
    }
    // Most severe first: the HUD and the decision engine both want the worst.
    out.sort((CollisionAssessment a, CollisionAssessment b) {
      final int bySeverity = b.risk.severity.compareTo(a.risk.severity);
      if (bySeverity != 0) return bySeverity;
      final double at = a.timeToCollisionSeconds ?? double.infinity;
      final double bt = b.timeToCollisionSeconds ?? double.infinity;
      return at.compareTo(bt);
    });
    return out;
  }

  CollisionAssessment _assessOne(
    WorldState world,
    PlannedPath path,
    ObjectTrack track,
  ) {
    final (LaneRelation relation, double relationConfidence) =
        _classifyLaneRelation(world, path, track);

    if (track.objectClass.isInfrastructure) {
      return CollisionAssessment(
        trackId: track.id,
        laneRelation: LaneRelation.offRoad,
        laneRelationConfidence: 0.9,
        risk: CollisionRisk.low,
        timeToCollisionSeconds: null,
        predictedPath: const <Vec2>[],
        minimumGapMeters: double.infinity,
        willEnterPath: false,
        reason: 'infrastructure, not a collision target',
      );
    }

    // --- forward simulation ----------------------------------------------
    //
    // Everything here is in the **vehicle frame**, in which the ego vehicle is
    // stationary at the origin by definition and the object moves by its
    // relative velocity. Advancing the ego along its path as well would
    // double-count our own motion — relative velocity already contains it —
    // and would report a collision with any car keeping station ahead of us.
    final PhysicalSizePrior size = track.objectClass.sizePrior;

    // Uncertainty inflation: a poorly-localised object is treated as bigger,
    // and a poorly-known velocity makes the prediction fuzzier with time.
    final double distanceUncertainty =
        (1.0 - clampDouble(track.distanceConfidence.value, 0, 1)) *
            math.max(1.0, track.estimatedDistanceMeters * 0.12);
    final double velocityUncertainty =
        (1.0 - clampDouble(track.velocityConfidence, 0, 1)) * 1.5;

    final List<Vec2> predicted = <Vec2>[];
    double minimumGap = double.infinity;
    double? ttc;
    bool willEnterPath = false;
    // "Entering the path" means crossing *into* it. An object already in our
    // lane ahead is being followed, not cutting in, and conflating the two
    // would flag every car in front as a developing conflict.
    bool startsOutsidePath = true;
    bool firstSample = true;

    final double egoHalfLength = egoLengthMeters / 2;
    final double objectHalfLength = size.length / 2;
    final double objectHalfWidth = size.width / 2;

    // Intrusion is tested against the *lane* as well as the planned path.
    // Testing only the path would let the planner hide a hazard from the risk
    // assessment: having already nudged aside for a crossing pedestrian, the
    // pedestrian no longer reaches the (moved) path, and the situation would
    // be reported as low risk — when in fact the dodge is the evidence that
    // it is not.
    final Polynomial? laneCentre = world.referenceCenterline;
    final double laneHalfWidth = world.lanes.laneWidthMeters / 2;

    for (double t = 0; t <= horizonSeconds; t += stepSeconds) {
      final Vec2 objectPosition = Vec2(
        track.position.x + track.relativeVelocity.x * t,
        track.position.y + track.relativeVelocity.y * t,
      );
      predicted.add(objectPosition);

      // Where our own path sits at the object's longitudinal position. For a
      // straight path this is zero; for a curve it is what stops a car on the
      // inside of a bend from reading as a collision.
      final double pathLateral = objectPosition.y <= 0
          ? 0.0
          : (path.lateralAt(objectPosition.y) ??
              (path.isEmpty ? 0.0 : path.curve.evaluate(objectPosition.y)));

      final double growingUncertainty =
          distanceUncertainty + velocityUncertainty * t;

      // Nominal separation, from the estimates as they stand.
      final double nominalLateralGap = (objectPosition.x - pathLateral).abs() -
          (objectHalfWidth + egoHalfWidthMeters);

      // Inflated separation, widened by how unsure we are. Used for TTC and
      // for the reported closest approach, so doubt always makes the
      // assessment more cautious, never less.
      final double lateralGap =
          nominalLateralGap - growingUncertainty * 0.35;

      final double longitudinalGap = objectPosition.y.abs() -
          (objectHalfLength + egoHalfLength) -
          growingUncertainty;

      // The relevant separation is the larger of the two: clearing either axis
      // clears the collision.
      final double gap = math.max(lateralGap, longitudinalGap);
      if (gap < minimumGap) minimumGap = gap;

      if (ttc == null && lateralGap < 0 && longitudinalGap < 0) {
        // Only count objects that started ahead of us. Something already
        // alongside or behind is not something we are going to run into.
        if (track.position.y > -egoHalfLength) ttc = t;
      }

      // Cut-in detection uses the *nominal* geometry. Using the inflated gap
      // here would be backwards: a growing uncertainty cone would make an
      // object look as though it had always been in our path, so it would
      // never count as entering it, and a less certain observation would
      // produce a lower risk than a confident one.
      final double laneLateral = objectPosition.y <= 0
          ? 0.0
          : (laneCentre?.evaluate(objectPosition.y) ?? 0.0);
      final bool insideLane =
          (objectPosition.x - laneLateral).abs() < laneHalfWidth;

      if (firstSample) {
        startsOutsidePath = nominalLateralGap > 0.4 && !insideLane;
        firstSample = false;
      }

      if (startsOutsidePath &&
          !willEnterPath &&
          objectPosition.y > 0 &&
          objectPosition.y < math.max(path.maxRangeMeters, 20) + 5 &&
          (nominalLateralGap < 0.4 || insideLane)) {
        willEnterPath = true;
      }
    }

    final (CollisionRisk risk, String reason) = _classifyRisk(
      track: track,
      relation: relation,
      ttc: ttc,
      minimumGap: minimumGap,
      willEnterPath: willEnterPath,
      egoSpeed: math.max(0.0, world.ego.speedMps),
    );

    return CollisionAssessment(
      trackId: track.id,
      laneRelation: relation,
      laneRelationConfidence: relationConfidence,
      risk: risk,
      timeToCollisionSeconds: ttc,
      predictedPath: predicted,
      minimumGapMeters: minimumGap,
      willEnterPath: willEnterPath,
      reason: reason,
    );
  }

  /// Where the object sits relative to our lane and our path.
  (LaneRelation, double) _classifyLaneRelation(
    WorldState world,
    PlannedPath path,
    ObjectTrack track,
  ) {
    final double distance = track.position.y;
    if (distance < -egoLengthMeters) {
      return (LaneRelation.unknown, 0.3); // behind us; camera cannot see it
    }

    final double pathLateral = path.lateralAt(math.max(0, distance)) ??
        (path.isEmpty ? 0.0 : path.curve.evaluate(math.max(0, distance)));
    final double offset = track.position.x - pathLateral;
    final double laneHalfWidth = world.lanes.laneWidthMeters / 2;

    // Confidence in the relation depends on both the object's localisation
    // and the road model's — a lane relation from a guessed lane is a guess.
    final double geometryConfidence = clampDouble(
      math.min(
        track.distanceConfidence.value + 0.2,
        math.max(world.lanes.overallConfidence,
            world.corridor?.confidence ?? 0) + 0.25,
      ),
      0.1,
      0.95,
    );

    // Crossing beats position: an object moving laterally through our path is
    // "crossing" even while it is still in the next lane.
    if (track.direction.isLateral || track.direction == MotionDirection.enteringPath) {
      final double lateralSpeed = track.relativeVelocity.x;
      if (lateralSpeed.abs() > 0.5) {
        final double timeToCross = offset.abs() / lateralSpeed.abs();
        final bool movingTowardsUs = (offset > 0 && lateralSpeed < 0) ||
            (offset < 0 && lateralSpeed > 0);
        if (movingTowardsUs && timeToCross < horizonSeconds) {
          return (LaneRelation.crossing, geometryConfidence);
        }
      }
    }

    if (offset.abs() <= laneHalfWidth) {
      return (LaneRelation.egoLane, geometryConfidence);
    }

    // Oncoming: moving towards us over the ground on the other side.
    if (track.velocityWorld.y < -2.0 && offset < 0) {
      return (LaneRelation.oncoming, geometryConfidence * 0.9);
    }

    if (offset.abs() <= laneHalfWidth * 3) {
      return (
        offset < 0 ? LaneRelation.leftLane : LaneRelation.rightLane,
        geometryConfidence * 0.9,
      );
    }

    // Beyond the adjacent lanes: is it even on the road?
    if (world.drivableArea.contains(track.position)) {
      return (
        offset < 0 ? LaneRelation.leftLane : LaneRelation.rightLane,
        geometryConfidence * 0.6,
      );
    }
    return (LaneRelation.offRoad, geometryConfidence * 0.8);
  }

  (CollisionRisk, String) _classifyRisk({
    required ObjectTrack track,
    required LaneRelation relation,
    required double? ttc,
    required double minimumGap,
    required bool willEnterPath,
    required double egoSpeed,
  }) {
    final double bonus =
        track.objectClass.isVulnerable ? vulnerableTtcBonus : 0;

    if (ttc != null) {
      if (ttc <= criticalTtcSeconds + bonus) {
        return (
          CollisionRisk.critical,
          'collision predicted in ${ttc.toStringAsFixed(1)} s',
        );
      }
      if (ttc <= highTtcSeconds + bonus) {
        return (
          CollisionRisk.high,
          'closing, TTC ${ttc.toStringAsFixed(1)} s',
        );
      }
      if (ttc <= mediumTtcSeconds + bonus) {
        return (
          CollisionRisk.medium,
          'closing, TTC ${ttc.toStringAsFixed(1)} s',
        );
      }
      return (
        CollisionRisk.low,
        'closing slowly, TTC ${ttc.toStringAsFixed(1)} s',
      );
    }

    // No predicted contact, but a near miss is still a hazard.
    if (minimumGap < 0.3 && relation.blocksEgoPath) {
      return (
        CollisionRisk.high,
        'passes within ${minimumGap.toStringAsFixed(2)} m',
      );
    }
    if (willEnterPath && track.objectClass.isVulnerable) {
      return (
        CollisionRisk.high,
        '${track.objectClass.label} entering the planned path',
      );
    }

    // Headway: the gap to a lead vehicle in seconds. Checked before the
    // generic cut-in rule because it is the more specific statement about the
    // same geometry, and it reports a number a driver can act on.
    if (relation == LaneRelation.egoLane &&
        track.position.y > 0 &&
        egoSpeed > 2.0) {
      final double headway = track.position.y / egoSpeed;
      if (headway < 1.0) {
        return (
          CollisionRisk.high,
          'headway ${headway.toStringAsFixed(1)} s',
        );
      }
      if (headway < 2.0) {
        return (
          CollisionRisk.medium,
          'headway ${headway.toStringAsFixed(1)} s',
        );
      }
    }

    if (willEnterPath) {
      return (CollisionRisk.medium, 'will enter the planned path');
    }

    if (minimumGap < 1.0) {
      return (
        CollisionRisk.medium,
        'closest approach ${minimumGap.toStringAsFixed(1)} m',
      );
    }

    return (CollisionRisk.low, 'no predicted conflict');
  }
}
