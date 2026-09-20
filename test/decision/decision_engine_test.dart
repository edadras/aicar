import 'package:aicar/core/confidence.dart';
import 'package:aicar/core/geometry.dart';
import 'package:aicar/decision/decision_engine_impl.dart';
import 'package:aicar/decision/driving_decision.dart';
import 'package:aicar/perception/object_class.dart';
import 'package:aicar/perception/traffic_light.dart';
import 'package:aicar/perception/traffic_sign.dart';
import 'package:aicar/planning/collision_predictor.dart';
import 'package:aicar/planning/local_path_planner.dart';
import 'package:aicar/planning/planned_path.dart';
import 'package:aicar/road/lane.dart';
import 'package:aicar/road/road_segmentation.dart';
import 'package:aicar/tracking/object_track.dart';
import 'package:aicar/world_model/world_state.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/world_builder.dart';

/// Run planning, collision prediction and the decision engine over a world,
/// exactly as the pipeline does.
DrivingDecision decide(WorldState world, {RuleBasedDecisionEngine? engine}) {
  final PlannedPath path = LocalPathPlanner().plan(world);
  final List<CollisionAssessment> collisions =
      const CollisionPredictor().assess(world, path);
  final List<ObjectTrack> annotated = <ObjectTrack>[
    for (final ObjectTrack t in world.tracks)
      t.copyWith(
        laneRelation: collisions
            .firstWhere((CollisionAssessment c) => c.trackId == t.id)
            .laneRelation,
        collisionRisk: collisions
            .firstWhere((CollisionAssessment c) => c.trackId == t.id)
            .risk,
        timeToCollisionSeconds: collisions
            .firstWhere((CollisionAssessment c) => c.trackId == t.id)
            .timeToCollisionSeconds,
      ),
  ];
  return (engine ?? RuleBasedDecisionEngine()).decide(
    world: world.copyWith(tracks: annotated),
    path: path,
    collisions: collisions,
  );
}

void main() {
  group('RuleBasedDecisionEngine basic states', () {
    test('cruises on a clear road', () {
      final DrivingDecision d = decide(testWorld());
      expect(d.state, DrivingState.cruise);
      expect(d.reason, isNotEmpty);
      expect(d.confidence, greaterThan(0));
    });

    test('follows a vehicle ahead', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 14),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.car,
            position: const Vec2(0, 34),
            worldVelocity: const Vec2(0, 11),
            egoSpeed: 14,
          ),
        ],
      ));
      expect(d.state, DrivingState.followVehicle);
      expect(d.targetSpeedMps, isNotNull);
      expect(d.targetSpeedMps!, lessThan(14));
      expect(d.triggeringTrackId, 1);
    });

    test('emergency brakes for a stopped vehicle at close range', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 16),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.car,
            position: const Vec2(0, 18),
            worldVelocity: const Vec2.zero(),
            direction: MotionDirection.stationary,
            egoSpeed: 16,
          ),
        ],
      ));
      expect(d.state, DrivingState.emergencyBrakeSimulation);
      expect(d.targetSpeedMps, 0);
      expect(d.reason, contains('CAR #1'));
    });

    test('yields to a pedestrian crossing into the path', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 10),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.person,
            position: const Vec2(-4.2, 20),
            worldVelocity: const Vec2(1.5, 0),
            relativeVelocity: const Vec2(1.5, -10),
            direction: MotionDirection.crossingLeftToRight,
            egoSpeed: 10,
          ),
        ],
      ));
      expect(
        d.state,
        anyOf(DrivingState.pedestrianYield,
            DrivingState.emergencyBrakeSimulation),
      );
      expect(d.reason.toLowerCase(), contains('person'));
    });

    test('stops for a red light', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 9),
        lights: <TrafficLight>[
          TrafficLight(
            id: 1,
            color: TrafficLightColor.red,
            arrow: TrafficLightArrow.none,
            box: const BoundingBox(
                left: 0.45, top: 0.2, width: 0.03, height: 0.08),
            confidence: Confidence(0.9),
            colorConfidence: Confidence(0.9),
            relevance: TrafficLightRelevance.egoPath,
            relevanceConfidence: Confidence(0.85),
            frameId: 1,
            timestampMicros: 0,
            distanceMeters: 30,
            stableColorFrames: 5,
          ),
        ],
      ));
      expect(d.state, DrivingState.stop);
      expect(d.reason.toLowerCase(), contains('red light'));
      expect(d.targetSpeedMps, 0);
    });

    test('ignores a red light that governs another path', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 9),
        lights: <TrafficLight>[
          TrafficLight(
            id: 1,
            color: TrafficLightColor.red,
            arrow: TrafficLightArrow.none,
            box: const BoundingBox(
                left: 0.05, top: 0.2, width: 0.03, height: 0.08),
            confidence: Confidence(0.9),
            colorConfidence: Confidence(0.9),
            relevance: TrafficLightRelevance.otherPath,
            relevanceConfidence: Confidence(0.8),
            frameId: 1,
            timestampMicros: 0,
            distanceMeters: 30,
            stableColorFrames: 5,
          ),
        ],
      ));
      expect(d.state, isNot(DrivingState.stop));
    });

    test('slows when over the posted limit', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 22), // ~79 km/h
        regulatory: const RegulatoryContext(
          speedLimitKph: 50,
          speedLimitConfidence: 0.9,
        ),
      ));
      expect(d.state, DrivingState.slowDown);
      expect(d.targetSpeedMps, closeTo(50 / 3.6, 0.01));
    });
  });

  group('RuleBasedDecisionEngine uncertainty', () {
    test('goes to UNCERTAIN when autonomy confidence is low', () {
      final DrivingDecision d = decide(testWorld(
        autonomy: AutonomyConfidence.compute(
          perception: 0.1,
          lanes: 0.2,
          depth: 0.2,
          egoMotion: 0.4,
          planning: 0.1,
        ),
      ));
      expect(d.state, DrivingState.uncertain);
      expect(d.reason.toLowerCase(), contains('confidence'));
    });

    test('goes to UNCERTAIN with no road model at all', () {
      final DrivingDecision d = decide(testWorld(
        lanes: LaneDetectionResult.empty(frameId: 1, timestampMicros: 0),
        drivableArea: DrivableArea.empty(frameId: 1, timestampMicros: 0),
      ));
      expect(d.state, DrivingState.uncertain);
    });

    test('a degraded subsystem is named in the reason', () {
      final DrivingDecision d = decide(testWorld(
        degraded: <String>['detection: no object detection model installed'],
      ));
      expect(d.state, DrivingState.uncertain);
      expect(d.reason, contains('no object detection model installed'));
    });

    test('a critical hazard still overrides UNCERTAIN', () {
      // Safety states must outrank uncertainty: not knowing is not a reason
      // to ignore something we *can* see.
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 16),
        autonomy: AutonomyConfidence.compute(
          perception: 0.15,
          lanes: 0.2,
          depth: 0.2,
          egoMotion: 0.4,
          planning: 0.2,
        ),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.car,
            position: const Vec2(0, 16),
            worldVelocity: const Vec2.zero(),
            direction: MotionDirection.stationary,
            egoSpeed: 16,
          ),
        ],
      ));
      expect(d.state, DrivingState.emergencyBrakeSimulation);
    });
  });

  group('RuleBasedDecisionEngine hysteresis', () {
    test('holds emergency braking briefly after the hazard clears', () {
      final RuleBasedDecisionEngine engine = RuleBasedDecisionEngine();

      final DrivingDecision braking = decide(
        testWorld(
          timestampMicros: 0,
          ego: testEgo(speedMps: 16),
          tracks: <ObjectTrack>[
            testTrack(
              id: 1,
              objectClass: ObjectClass.car,
              position: const Vec2(0, 16),
              worldVelocity: const Vec2.zero(),
              direction: MotionDirection.stationary,
              egoSpeed: 16,
            ),
          ],
        ),
        engine: engine,
      );
      expect(braking.state, DrivingState.emergencyBrakeSimulation);

      // 0.2 s later the object has vanished (a dropped detection).
      final DrivingDecision shortlyAfter = decide(
        testWorld(timestampMicros: 200000, ego: testEgo(speedMps: 16)),
        engine: engine,
      );
      expect(shortlyAfter.state, DrivingState.emergencyBrakeSimulation,
          reason: 'must not release immediately');

      // 2 s later it is genuinely gone.
      final DrivingDecision muchLater = decide(
        testWorld(timestampMicros: 2000000, ego: testEgo(speedMps: 16)),
        engine: engine,
      );
      expect(muchLater.state, isNot(DrivingState.emergencyBrakeSimulation));
    });

    test('escalation is immediate, with no dwell requirement', () {
      final RuleBasedDecisionEngine engine = RuleBasedDecisionEngine();
      final DrivingDecision cruising =
          decide(testWorld(timestampMicros: 0), engine: engine);
      expect(cruising.state, DrivingState.cruise);

      final DrivingDecision emergency = decide(
        testWorld(
          timestampMicros: 50000, // 50 ms later
          ego: testEgo(speedMps: 16),
          tracks: <ObjectTrack>[
            testTrack(
              id: 1,
              objectClass: ObjectClass.car,
              position: const Vec2(0, 15),
              worldVelocity: const Vec2.zero(),
              direction: MotionDirection.stationary,
              egoSpeed: 16,
            ),
          ],
        ),
        engine: engine,
      );
      expect(emergency.state, DrivingState.emergencyBrakeSimulation);
    });
  });

  group('decision reporting', () {
    test('every decision carries a reason and a confidence', () {
      for (final WorldState world in <WorldState>[
        testWorld(),
        testWorld(ego: testEgo(speedMps: 0)),
        testWorld(degraded: <String>['segmentation: no model']),
      ]) {
        final DrivingDecision d = decide(world);
        expect(d.reason, isNotEmpty);
        expect(d.confidence, inInclusiveRange(0, 1));
        expect(d.displayText, contains(d.state.label));
      }
    });

    test('alternatives considered are recorded for the debug overlay', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 14),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.car,
            position: const Vec2(0, 30),
            worldVelocity: const Vec2(0, 10),
            egoSpeed: 14,
          ),
        ],
      ));
      expect(d.alternativesConsidered, isNotEmpty);
    });

    test('round-trips through JSON', () {
      final DrivingDecision d = decide(testWorld());
      final DrivingDecision back =
          DrivingDecision.fromJson(d.toJson());
      expect(back.state, d.state);
      expect(back.reason, d.reason);
      expect(back.confidence, closeTo(d.confidence, 1e-3));
    });
  });
}
