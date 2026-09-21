import 'dart:math' as math;

import '../ai/interfaces/decision_engine.dart';
import '../core/confidence.dart';
import '../core/geometry.dart';
import '../navigation/maneuver.dart';
import '../perception/traffic_light.dart';
import '../planning/collision_predictor.dart';
import '../planning/planned_path.dart';
import '../road/intersection_detector.dart';
import '../road/road_marking.dart';
import '../tracking/object_track.dart';
import '../world_model/hazard.dart';
import '../world_model/world_state.dart';
import 'driving_decision.dart';

/// Tuning for [RuleBasedDecisionEngine].
class DecisionConfig {
  const DecisionConfig({
    this.emergencyTtcSeconds = 1.6,
    this.yieldTtcSeconds = 3.5,
    this.followHeadwaySeconds = 2.0,
    this.minFollowDistanceMeters = 6.0,
    this.stopDistanceMeters = 12.0,
    this.turnAnnounceMeters = 35.0,
    this.overSpeedToleranceKph = 5.0,
    this.crosswalkYieldMeters = 25.0,
    this.minDwellSeconds = 0.4,
    this.emergencyDwellSeconds = 1.2,
    this.uncertainDwellSeconds = 0.6,
  });

  final double emergencyTtcSeconds;
  final double yieldTtcSeconds;

  /// Target time gap to the vehicle ahead. Two seconds is the standard
  /// recommendation and is what the follow controller aims for.
  final double followHeadwaySeconds;

  final double minFollowDistanceMeters;
  final double stopDistanceMeters;
  final double turnAnnounceMeters;
  final double overSpeedToleranceKph;

  /// Within this distance of a crossing, someone waiting at its edge is
  /// treated as about to step onto it rather than as scenery.
  final double crosswalkYieldMeters;

  /// Minimum time a state is held before a *lower* priority state can take
  /// over. Without it the display flickers between states every frame as
  /// perception noise moves a TTC across a threshold.
  final double minDwellSeconds;

  /// Emergency braking is held longer: releasing it the instant the TTC blips
  /// back above the threshold would be exactly wrong.
  final double emergencyDwellSeconds;

  final double uncertainDwellSeconds;
}

/// Rule-based driving decision state machine.
///
/// Candidates are generated independently and the highest-priority one wins,
/// rather than a nest of if/else that is impossible to reason about. Each
/// candidate carries its own reason and confidence, so the winning decision
/// can always explain itself — which is the whole point of a system whose
/// output a human is supposed to evaluate.
///
/// Hysteresis is applied on the way *down* only: escalating to a more urgent
/// state is immediate, de-escalating requires the situation to have been
/// resolved for a minimum dwell time.
class RuleBasedDecisionEngine implements DecisionEngine {
  RuleBasedDecisionEngine({this.config = const DecisionConfig()});

  final DecisionConfig config;

  DrivingDecision? _current;
  int _stateEnteredMicros = 0;

  @override
  String get engineName => 'Rule-based state machine';

  @override
  DrivingDecision? get current => _current;

  @override
  void reset() {
    _current = null;
    _stateEnteredMicros = 0;
  }

  @override
  DrivingDecision decide({
    required WorldState world,
    required PlannedPath path,
    required List<CollisionAssessment> collisions,
  }) {
    final List<_Candidate> candidates = <_Candidate>[
      ..._uncertaintyCandidates(world, path),
      ..._collisionCandidates(world, path, collisions),
      ..._trafficControlCandidates(world),
      ..._roadMarkingCandidates(world),
      ..._intersectionCandidates(world),
      ..._pathCandidates(world, path),
      ..._navigationCandidates(world),
      ..._speedCandidates(world, path),
      _cruiseCandidate(world, path),
    ];

    candidates.sort((_Candidate a, _Candidate b) {
      final int byPriority = b.state.priority.compareTo(a.state.priority);
      if (byPriority != 0) return byPriority;
      return b.confidence.compareTo(a.confidence);
    });

    final _Candidate winner = _applyHysteresis(candidates, world);

    final double held = _current?.state == winner.state
        ? (world.timestampMicros - _stateEnteredMicros) / 1e6
        : 0;
    if (_current?.state != winner.state) {
      _stateEnteredMicros = world.timestampMicros;
    }

    final DrivingDecision decision = DrivingDecision(
      state: winner.state,
      reason: winner.reason,
      confidence: winner.confidence,
      timestampMicros: world.timestampMicros,
      frameId: world.frameId,
      targetSpeedMps: winner.targetSpeedMps,
      triggeringHazard: winner.hazard,
      triggeringTrackId: winner.trackId,
      heldForSeconds: held,
      alternativesConsidered: <String>[
        for (final _Candidate c in candidates.take(4))
          if (c.state != winner.state)
            '${c.state.label} (${(c.confidence * 100).round()}%)',
      ],
    );

    _current = decision;
    return decision;
  }

  // --- Candidate generators ----------------------------------------------

  /// The stack must be able to say "I do not know".
  List<_Candidate> _uncertaintyCandidates(WorldState world, PlannedPath path) {
    final List<_Candidate> out = <_Candidate>[];

    if (world.autonomy.isLow) {
      out.add(_Candidate(
        state: DrivingState.uncertain,
        reason: 'Autonomy confidence low '
            '(${(world.autonomy.overall * 100).round()}%, weakest: '
            '${world.autonomy.weakestSubsystem})',
        // Confidence *in the decision to be uncertain* is high precisely when
        // confidence in everything else is low.
        confidence: clampDouble(
          1 - world.autonomy.overall / ConfidenceThresholds.autonomyFloor,
          0.5,
          0.98,
        ),
        targetSpeedMps: math.min(world.ego.speedMps, 8.0),
        hazard: const Hazard(
          type: HazardType.lowAutonomyConfidence,
          severity: HazardSeverity.warning,
          description: 'Perception confidence below the usable threshold',
          confidence: 0.9,
        ),
      ));
    }

    if (!path.isUsable && !world.ego.isStationary) {
      out.add(_Candidate(
        state: DrivingState.uncertain,
        reason: path.blockReason ?? 'No usable planned path',
        confidence: 0.85,
        targetSpeedMps: math.min(world.ego.speedMps, 5.0),
      ));
    }

    if (world.degradedSubsystems.isNotEmpty) {
      out.add(_Candidate(
        state: DrivingState.uncertain,
        reason: 'Degraded: ${world.degradedSubsystems.join(', ')}',
        confidence: 0.7,
        targetSpeedMps: math.min(world.ego.speedMps, 8.0),
        hazard: Hazard(
          type: HazardType.perceptionDegraded,
          severity: HazardSeverity.warning,
          description: world.degradedSubsystems.join(', '),
          confidence: 0.8,
        ),
      ));
    }

    return out;
  }

  List<_Candidate> _collisionCandidates(
    WorldState world,
    PlannedPath path,
    List<CollisionAssessment> collisions,
  ) {
    final List<_Candidate> out = <_Candidate>[];

    for (final CollisionAssessment c in collisions) {
      final ObjectTrack? track = _trackById(world, c.trackId);
      if (track == null) continue;

      final double ttc = c.timeToCollisionSeconds ?? double.infinity;
      final double evidence = track.confidence.value *
          clampDouble(track.distanceConfidence.value, 0.2, 1.0) *
          clampDouble(c.laneRelationConfidence, 0.2, 1.0);

      if (c.risk == CollisionRisk.critical &&
          ttc <= config.emergencyTtcSeconds) {
        out.add(_Candidate(
          state: DrivingState.emergencyBrakeSimulation,
          reason: '${track.displayLabel}: ${c.reason}',
          confidence: clampDouble(evidence, 0.3, 0.99),
          targetSpeedMps: 0,
          trackId: track.id,
          hazard: Hazard.fromTrack(track),
        ));
        continue;
      }

      if (track.objectClass.isVulnerable &&
          (c.laneRelation.blocksEgoPath || c.willEnterPath) &&
          ttc <= config.yieldTtcSeconds) {
        out.add(_Candidate(
          state: DrivingState.pedestrianYield,
          reason: '${track.objectClass.label} '
              '${track.direction.label.toLowerCase()} at '
              '${track.estimatedDistanceMeters.toStringAsFixed(1)} m'
              '${c.timeToCollisionSeconds == null ? '' : ', TTC '
                  '${c.timeToCollisionSeconds!.toStringAsFixed(1)} s'}',
          confidence: clampDouble(evidence, 0.25, 0.95),
          targetSpeedMps: 0,
          trackId: track.id,
          hazard: Hazard.fromTrack(track),
        ));
        continue;
      }

      if (c.risk == CollisionRisk.high) {
        out.add(_Candidate(
          state: DrivingState.slowDown,
          reason: '${track.displayLabel}: ${c.reason}',
          confidence: clampDouble(evidence * 0.9, 0.2, 0.92),
          targetSpeedMps: _speedForTtc(world.ego.speedMps, ttc),
          trackId: track.id,
          hazard: Hazard.fromTrack(track),
        ));
      } else if (c.risk == CollisionRisk.medium &&
          c.laneRelation.blocksEgoPath) {
        out.add(_Candidate(
          state: DrivingState.slowDown,
          reason: '${track.displayLabel}: ${c.reason}',
          confidence: clampDouble(evidence * 0.7, 0.15, 0.8),
          targetSpeedMps: _speedForTtc(world.ego.speedMps, ttc),
          trackId: track.id,
        ));
      }
    }

    // Following: a lead vehicle at a sensible distance is not a hazard, it is
    // a speed target.
    final ObjectTrack? lead = world.leadVehicle;
    if (lead != null && lead.position.y > config.minFollowDistanceMeters) {
      final double targetSpeed = _followSpeed(world, lead);
      out.add(_Candidate(
        state: DrivingState.followVehicle,
        reason: '${lead.displayLabel} at '
            '${lead.estimatedDistanceMeters.toStringAsFixed(1)} m, '
            'matching ${(targetSpeed * 3.6).toStringAsFixed(0)} km/h',
        confidence: clampDouble(
          lead.confidence.value * lead.distanceConfidence.value,
          0.2,
          0.9,
        ),
        targetSpeedMps: targetSpeed,
        trackId: lead.id,
      ));
    }

    return out;
  }

  List<_Candidate> _trafficControlCandidates(WorldState world) {
    final List<_Candidate> out = <_Candidate>[];

    final TrafficLight? light = world.governingTrafficLight;
    if (light != null && light.color.requiresStop) {
      final double distance = light.distanceMeters ?? 25;
      out.add(_Candidate(
        state: distance < 6 && world.ego.speedMps < 0.6
            ? DrivingState.wait
            : DrivingState.stop,
        reason: 'Red light ahead at ${distance.toStringAsFixed(0)} m '
            '(${light.colorConfidence.percent}% colour, '
            '${light.relevanceConfidence.percent}% relevance)',
        confidence: clampDouble(
          light.colorConfidence.value * light.relevanceConfidence.value,
          0.2,
          0.95,
        ),
        targetSpeedMps: 0,
        hazard: Hazard(
          type: HazardType.redLight,
          severity: HazardSeverity.warning,
          description: 'Red light at ${distance.toStringAsFixed(0)} m',
          confidence: light.colorConfidence.value,
          distanceMeters: distance,
        ),
      ));
    }

    if (light != null &&
        light.color == TrafficLightColor.yellow &&
        (light.distanceMeters ?? 0) > 20) {
      out.add(_Candidate(
        state: DrivingState.slowDown,
        reason: 'Amber light at '
            '${light.distanceMeters!.toStringAsFixed(0)} m',
        confidence: light.colorConfidence.value * 0.8,
        targetSpeedMps: world.ego.speedMps * 0.5,
      ));
    }

    if (world.regulatory.pendingStop) {
      out.add(_Candidate(
        state: DrivingState.stop,
        reason: 'Stop sign ahead',
        confidence: 0.8,
        targetSpeedMps: 0,
        hazard: const Hazard(
          type: HazardType.stopSign,
          severity: HazardSeverity.warning,
          description: 'Stop sign ahead',
          confidence: 0.8,
        ),
      ));
    } else if (world.regulatory.pendingGiveWay) {
      out.add(_Candidate(
        state: DrivingState.slowDown,
        reason: 'Give way ahead',
        confidence: 0.7,
        targetSpeedMps: 4.0,
        hazard: const Hazard(
          type: HazardType.giveWay,
          severity: HazardSeverity.caution,
          description: 'Give way ahead',
          confidence: 0.7,
        ),
      ));
    }

    return out;
  }

  /// Paint on the road: crossings and speed bumps.
  ///
  /// These are not hazards in the collision sense — nothing is going to hit
  /// us — but they are exactly the kind of thing a driver reads the road for,
  /// and a stack that ignored them would be obviously wrong to anyone
  /// watching it drive.
  List<_Candidate> _roadMarkingCandidates(WorldState world) {
    final List<_Candidate> out = <_Candidate>[];

    // --- Crossings ---------------------------------------------------------
    final RoadMarking? crossing =
        world.markingAhead(RoadMarkingType.crosswalk);
    if (crossing != null &&
        crossing.distanceMeters <= RoadMarkingType.crosswalk.approachMeters) {
      final List<ObjectTrack> waiting = world.pedestriansAtCrossing;

      if (waiting.isNotEmpty &&
          crossing.distanceMeters <= config.crosswalkYieldMeters) {
        // Someone is on or at the crossing. That is a yield, not an advisory.
        final ObjectTrack nearest = waiting.reduce((ObjectTrack a,
                ObjectTrack b) =>
            a.position.y < b.position.y ? a : b);
        out.add(_Candidate(
          state: DrivingState.pedestrianYield,
          reason: '${waiting.length} '
              '${waiting.length == 1 ? 'person' : 'people'} at the crossing '
              '${crossing.distanceMeters.toStringAsFixed(0)} m ahead',
          confidence: clampDouble(
            crossing.confidence.value * nearest.confidence.value,
            0.2,
            0.92,
          ),
          targetSpeedMps: 0,
          trackId: nearest.id,
          hazard: Hazard(
            type: HazardType.crosswalkAhead,
            severity: HazardSeverity.warning,
            description: 'Pedestrian crossing with people at it',
            confidence: crossing.confidence.value,
            distanceMeters: crossing.distanceMeters,
            relatedTrackId: nearest.id,
          ),
        ));
      } else {
        final double target = _approachSpeed(
          world.ego.speedMps,
          crossing.distanceMeters,
          RoadMarkingType.crosswalk.advisorySpeedMps,
        );
        if (world.ego.speedMps > target + 1.0) {
          out.add(_Candidate(
            state: DrivingState.slowDown,
            reason: 'Pedestrian crossing in '
                '${crossing.distanceMeters.toStringAsFixed(0)} m',
            confidence: clampDouble(crossing.confidence.value, 0.2, 0.85),
            targetSpeedMps: target,
            hazard: Hazard(
              type: HazardType.crosswalkAhead,
              severity: HazardSeverity.caution,
              description: 'Pedestrian crossing ahead',
              confidence: crossing.confidence.value,
              distanceMeters: crossing.distanceMeters,
            ),
          ));
        }
      }
    }

    // --- Speed bumps -------------------------------------------------------
    final RoadMarking? bump = world.markingAhead(RoadMarkingType.speedBump);
    if (bump != null &&
        bump.distanceMeters <= RoadMarkingType.speedBump.approachMeters) {
      final double target = _approachSpeed(
        world.ego.speedMps,
        bump.distanceMeters,
        RoadMarkingType.speedBump.advisorySpeedMps,
      );
      if (world.ego.speedMps > target + 0.8) {
        out.add(_Candidate(
          state: DrivingState.slowDown,
          reason: 'Speed bump in ${bump.distanceMeters.toStringAsFixed(0)} m, '
              'easing to '
              '${(RoadMarkingType.speedBump.advisorySpeedKph).round()} km/h',
          confidence: clampDouble(bump.confidence.value, 0.2, 0.85),
          targetSpeedMps: target,
          hazard: Hazard(
            type: HazardType.speedBumpAhead,
            severity: HazardSeverity.caution,
            description: 'Speed bump ahead',
            confidence: bump.confidence.value,
            distanceMeters: bump.distanceMeters,
          ),
        ));
      }
    }

    return out;
  }

  /// Approaching a junction.
  ///
  /// The important case is the *uncontrolled* one. A green light tells us the
  /// junction is ours; no signal at all tells us nothing, and the correct
  /// response to knowing nothing about who has priority is to arrive slowly
  /// enough to react.
  List<_Candidate> _intersectionCandidates(WorldState world) {
    final IntersectionEstimate? junction = world.intersection;
    if (junction == null) return const <_Candidate>[];

    final double postedLimit = world.effectiveSpeedLimitKph != null
        ? world.effectiveSpeedLimitKph! / 3.6
        : math.max(world.ego.speedMps, 8.0);
    final double onset = junction.cautionOnsetMeters(
      speedMps: world.ego.speedMps,
      postedLimitMps: postedLimit,
    );
    if (junction.distanceMeters > onset) return const <_Candidate>[];

    final double approach =
        junction.approachSpeedMps(postedLimitMps: postedLimit);
    final double target = _approachSpeed(
        world.ego.speedMps, junction.distanceMeters, approach);

    // A signalled junction on green with nothing crossing needs no candidate
    // of its own — the red-light branch handles the case that matters.
    if (world.ego.speedMps <= target + 0.8 && !junction.hasCrossingTraffic) {
      return const <_Candidate>[];
    }

    final HazardSeverity severity = junction.hasCrossingTraffic
        ? HazardSeverity.warning
        : HazardSeverity.caution;

    return <_Candidate>[
      _Candidate(
        state: DrivingState.slowDown,
        reason: '${junction.control.label} junction in '
            '${junction.distanceMeters.toStringAsFixed(0)} m'
            '${junction.hasCrossingTraffic ? ', traffic crossing' : ''} '
            '— ${junction.evidence.first}',
        confidence: clampDouble(junction.confidence.value, 0.2, 0.9),
        targetSpeedMps: target,
        hazard: Hazard(
          type: junction.hasCrossingTraffic
              ? HazardType.crossingTraffic
              : HazardType.intersectionAhead,
          severity: severity,
          description: '${junction.control.label} junction: '
              '${junction.evidence.join('; ')}',
          confidence: junction.confidence.value,
          distanceMeters: junction.distanceMeters,
        ),
      ),
    ];
  }

  List<_Candidate> _pathCandidates(WorldState world, PlannedPath path) {
    final List<_Candidate> out = <_Candidate>[];

    // Two cases where a blocked path must not produce a specific action:
    //
    //  * `PathSource.none` means there was nothing to plan *from*, which is a
    //    statement about perception, not about an obstruction. Reporting
    //    "path blocked" would give the driver a confidently wrong reason.
    //  * Low autonomy confidence means this inference rests on perception we
    //    have already decided not to trust. Acting decisively on an untrusted
    //    inference is worse than admitting uncertainty, so the UNCERTAIN
    //    candidate is allowed to win instead. Directly observed hazards
    //    (emergency braking, yielding) still outrank it on priority.
    if (path.isBlocked &&
        path.source != PathSource.none &&
        !world.autonomy.isLow) {
      final double blockedAt = path.blockedAtMeters ?? 0;
      if (blockedAt < config.stopDistanceMeters) {
        out.add(_Candidate(
          state: world.ego.speedMps < 0.6
              ? DrivingState.wait
              : DrivingState.stop,
          reason: path.blockReason ?? 'Path blocked ahead',
          confidence: clampDouble(path.confidence + 0.3, 0.3, 0.9),
          targetSpeedMps: 0,
          hazard: Hazard(
            type: HazardType.obstacleInPath,
            severity: HazardSeverity.warning,
            description: path.blockReason ?? 'Path blocked',
            confidence: 0.8,
            distanceMeters: blockedAt,
          ),
        ));
      } else {
        out.add(_Candidate(
          state: DrivingState.slowDown,
          reason: 'Path blocked at ${blockedAt.toStringAsFixed(0)} m',
          confidence: clampDouble(path.confidence, 0.2, 0.85),
          targetSpeedMps: _speedToStopWithin(blockedAt),
        ));
      }
    }

    // A meaningful lateral offset means the planner moved over for something.
    if (path.source == PathSource.obstacleAvoidance &&
        path.lateralOffsetFromReference.abs() > 0.35) {
      out.add(_Candidate(
        state: DrivingState.obstacleAvoidanceSimulation,
        reason: 'Offsetting '
            '${path.lateralOffsetFromReference.abs().toStringAsFixed(2)} m '
            '${path.lateralOffsetFromReference < 0 ? 'left' : 'right'} '
            'around an obstruction',
        confidence: clampDouble(path.confidence, 0.2, 0.9),
        targetSpeedMps: path.limitingSpeedMps,
      ));
    }

    // Drivable area ending is a genuine hazard, distinct from an obstacle.
    if (world.drivableArea.maxRangeMeters > 0 &&
        world.drivableArea.maxRangeMeters < 15 &&
        world.ego.speedMps > 5) {
      out.add(_Candidate(
        state: DrivingState.slowDown,
        reason: 'Drivable area ends at '
            '${world.drivableArea.maxRangeMeters.toStringAsFixed(0)} m',
        confidence: clampDouble(world.drivableArea.confidence, 0.2, 0.8),
        targetSpeedMps: _speedToStopWithin(world.drivableArea.maxRangeMeters),
        hazard: Hazard(
          type: HazardType.roadEnds,
          severity: HazardSeverity.caution,
          description: 'Drivable area ends ahead',
          confidence: world.drivableArea.confidence,
          distanceMeters: world.drivableArea.maxRangeMeters,
        ),
      ));
    }

    return out;
  }

  List<_Candidate> _navigationCandidates(WorldState world) {
    final double? distance = world.routeProgress?.distanceToManeuverMeters;
    if (distance == null || distance > config.turnAnnounceMeters) {
      return const <_Candidate>[];
    }

    final ManeuverIntent intent = world.navigationIntent;
    if (!intent.isTurn) return const <_Candidate>[];

    // A turn is only executed when the *road* supports it; navigation intent
    // alone is never sufficient, because the route geometry is not accurate
    // enough to steer by.
    final double roadCurvature =
        world.referenceCenterline?.curvatureAt(math.max(5, distance)) ?? 0;
    final bool roadAgrees = intent == ManeuverIntent.turnLeft
        ? roadCurvature < -0.004
        : roadCurvature > 0.004;

    final double matchQuality = world.routeProgress?.matchQuality ?? 0;

    return <_Candidate>[
      _Candidate(
        state: intent == ManeuverIntent.turnLeft
            ? DrivingState.turnLeft
            : DrivingState.turnRight,
        reason: roadAgrees
            ? '${intent.label} in ${distance.toStringAsFixed(0)} m '
                '(road curvature agrees)'
            : '${intent.label} in ${distance.toStringAsFixed(0)} m '
                '(following the visible road, not the route geometry)',
        confidence: clampDouble(
          matchQuality * (roadAgrees ? 0.9 : 0.45),
          0.1,
          0.9,
        ),
        targetSpeedMps: math.min(world.ego.speedMps, 8.0),
      ),
    ];
  }

  List<_Candidate> _speedCandidates(WorldState world, PlannedPath path) {
    final List<_Candidate> out = <_Candidate>[];

    // A limit only constrains us once we believe we read it. A sign we are
    // 20 % sure said "50" is not a reason to brake — it is a reason to say we
    // do not know the limit. A limit that came from the map carries its own
    // provenance and does not need the sign reader's confidence.
    final int? limit = world.effectiveSpeedLimitKph;
    final bool limitTrusted = world.regulatory.hasSpeedLimit ||
        (world.regulatory.speedLimitKph == null &&
            world.routeProgress?.mapSpeedLimitKph != null);
    if (limit != null &&
        limitTrusted &&
        world.ego.speedKph > limit + config.overSpeedToleranceKph &&
        world.ego.speedConfidence > 0.5) {
      out.add(_Candidate(
        state: DrivingState.slowDown,
        reason: '${world.ego.speedKph.toStringAsFixed(0)} km/h in a '
            '$limit km/h limit',
        confidence: clampDouble(
          world.regulatory.speedLimitConfidence * world.ego.speedConfidence,
          0.2,
          0.9,
        ),
        targetSpeedMps: limit / 3.6,
        hazard: Hazard(
          type: HazardType.speedLimitExceeded,
          severity: HazardSeverity.caution,
          description: 'Over the $limit km/h limit',
          confidence: world.regulatory.speedLimitConfidence,
        ),
      ));
    }

    // The path's own curvature limit.
    if (path.isUsable) {
      final double limitingSpeed = path.limitingSpeedMps;
      if (world.ego.speedMps > limitingSpeed + 1.5) {
        out.add(_Candidate(
          state: DrivingState.slowDown,
          reason: 'Curve ahead limits speed to '
              '${(limitingSpeed * 3.6).toStringAsFixed(0)} km/h',
          confidence: clampDouble(path.confidence * 0.85, 0.2, 0.85),
          targetSpeedMps: limitingSpeed,
        ));
      }
    }

    return out;
  }

  _Candidate _cruiseCandidate(WorldState world, PlannedPath path) {
    final double target = path.isUsable
        ? path.limitingSpeedMps
        : math.min(world.ego.speedMps, 10.0);

    return _Candidate(
      state: DrivingState.cruise,
      reason: path.isUsable
          ? 'Path clear for ${path.maxRangeMeters.toStringAsFixed(0)} m '
              '(${path.source.label})'
          : 'Holding speed',
      confidence: clampDouble(
        path.confidence * world.autonomy.overall + 0.1,
        0.05,
        0.95,
      ),
      targetSpeedMps: target,
    );
  }

  // --- Helpers ------------------------------------------------------------

  /// Prefer the winner, but do not let a lower-priority state take over until
  /// the current one has been held long enough.
  _Candidate _applyHysteresis(List<_Candidate> candidates, WorldState world) {
    final _Candidate winner = candidates.first;
    final DrivingDecision? current = _current;
    if (current == null) return winner;

    if (winner.state.priority >= current.state.priority) return winner;

    final double held = (world.timestampMicros - _stateEnteredMicros) / 1e6;
    final double required = switch (current.state) {
      DrivingState.emergencyBrakeSimulation => config.emergencyDwellSeconds,
      DrivingState.uncertain => config.uncertainDwellSeconds,
      _ => config.minDwellSeconds,
    };
    if (held >= required) return winner;

    // Hold the current state, but keep the fresh reasoning so the HUD does not
    // show a stale explanation.
    final _Candidate? sameState = candidates
        .cast<_Candidate?>()
        .firstWhere((_Candidate? c) => c!.state == current.state,
            orElse: () => null);

    return sameState ??
        _Candidate(
          state: current.state,
          reason: '${current.reason} (held '
              '${held.toStringAsFixed(1)} s / '
              '${required.toStringAsFixed(1)} s)',
          confidence: current.confidence * 0.9,
          targetSpeedMps: current.targetSpeedMps,
          trackId: current.triggeringTrackId,
          hazard: current.triggeringHazard,
        );
  }

  ObjectTrack? _trackById(WorldState world, int id) {
    for (final ObjectTrack t in world.tracks) {
      if (t.id == id) return t;
    }
    return null;
  }

  /// Speed to aim for given a time to collision: scale down so that the gap
  /// reopens, without demanding an unachievable deceleration.
  double _speedForTtc(double currentSpeed, double ttc) {
    if (!ttc.isFinite) return currentSpeed;
    if (ttc <= 1.0) return 0;
    final double factor = clampDouble((ttc - 1.0) / 3.0, 0, 1);
    return currentSpeed * factor;
  }

  /// Headway-keeping speed for a lead vehicle.
  double _followSpeed(WorldState world, ObjectTrack lead) {
    final double gap = lead.position.y;
    final double leadSpeed = lead.velocityWorld.y;
    final double desiredGap = math.max(
      config.minFollowDistanceMeters,
      world.ego.speedMps * config.followHeadwaySeconds,
    );

    // Proportional on gap error, added to the lead's own speed. This is the
    // standard ACC formulation and settles at the desired headway.
    final double gapError = gap - desiredGap;
    final double target = leadSpeed + 0.45 * gapError;

    final double ceiling = world.effectiveSpeedLimitKph != null
        ? world.effectiveSpeedLimitKph! / 3.6
        : math.max(world.ego.speedMps, 8.0);

    return clampDouble(target, 0, ceiling);
  }

  /// Speed we may be doing now and still be down to [targetSpeed] on arrival.
  ///
  /// Ramping in like this is what stops the stack from ignoring a bump at
  /// 40 m and then demanding 20 km/h at 5 m.
  double _approachSpeed(
    double currentSpeed,
    double distance,
    double targetSpeed,
  ) {
    if (distance <= 0) return targetSpeed;
    const double comfortDecel = 1.6;
    final double allowed = math.sqrt(
      targetSpeed * targetSpeed + 2 * comfortDecel * distance,
    );
    return math.min(math.max(targetSpeed, currentSpeed), allowed);
  }

  /// Speed from which the vehicle could comfortably stop within [distance].
  double _speedToStopWithin(double distance) {
    if (distance <= 0.5) return 0;
    const double comfortDecel = 2.2;
    return math.sqrt(2 * comfortDecel * math.max(0, distance - 2));
  }
}

class _Candidate {
  const _Candidate({
    required this.state,
    required this.reason,
    required this.confidence,
    this.targetSpeedMps,
    this.trackId,
    this.hazard,
  });

  final DrivingState state;
  final String reason;
  final double confidence;
  final double? targetSpeedMps;
  final int? trackId;
  final Hazard? hazard;
}
