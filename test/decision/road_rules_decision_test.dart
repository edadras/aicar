import 'package:aicar/core/confidence.dart';
import 'package:aicar/core/geometry.dart';
import 'package:aicar/decision/decision_engine_impl.dart';
import 'package:aicar/decision/driving_decision.dart';
import 'package:aicar/perception/object_class.dart';
import 'package:aicar/perception/traffic_sign.dart';
import 'package:aicar/planning/collision_predictor.dart';
import 'package:aicar/road/intersection_detector.dart';
import 'package:aicar/road/road_marking.dart';
import 'package:aicar/tracking/object_track.dart';
import 'package:aicar/world_model/hazard.dart';
import 'package:aicar/world_model/world_state.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/world_builder.dart';

/// How the stack reacts to the things a driver reads the road for: the posted
/// limit, paint on the surface, and junctions.
void main() {
  DrivingDecision decide(WorldState world) => RuleBasedDecisionEngine().decide(
        world: world,
        path: testPath(),
        collisions: const <CollisionAssessment>[],
      );

  group('speed limit', () {
    test('over the posted limit produces a slow-down with the number in it',
        () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 22),
        regulatory: const RegulatoryContext(
          speedLimitKph: 50,
          speedLimitConfidence: 0.85,
        ),
      ));
      expect(d.state, DrivingState.slowDown);
      expect(d.reason, contains('50 km/h'));
      expect(d.targetSpeedMps, closeTo(50 / 3.6, 0.01));
    });

    test('a school zone tightens the posted limit', () {
      const RegulatoryContext r = RegulatoryContext(
        speedLimitKph: 50,
        speedLimitConfidence: 0.9,
        inSchoolZone: true,
      );
      expect(r.targetSpeedMps, closeTo(30 / 3.6, 0.01));
    });

    test('a limit we are not confident in does not drive a decision', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 22),
        regulatory: const RegulatoryContext(
          speedLimitKph: 50,
          speedLimitConfidence: 0.2,
        ),
      ));
      // The candidate may still exist, but it must not win on a reading we
      // do not trust.
      expect(d.state, isNot(DrivingState.slowDown));
    });
  });

  group('crossings', () {
    test('an empty crossing is approached, not stopped for', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 14),
        roadMarkings: <RoadMarking>[
          testMarking(
              type: RoadMarkingType.crosswalk, distanceMeters: 30),
        ],
      ));
      expect(d.state, DrivingState.slowDown);
      expect(d.reason.toLowerCase(), contains('crossing'));
      expect(d.targetSpeedMps, greaterThan(0),
          reason: 'an empty crossing is not a stop line');
    });

    test('someone at the crossing is yielded to', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 10),
        roadMarkings: <RoadMarking>[
          testMarking(
              type: RoadMarkingType.crosswalk, distanceMeters: 18),
        ],
        tracks: <ObjectTrack>[
          testTrack(
            id: 3,
            objectClass: ObjectClass.person,
            position: const Vec2(-2.6, 19),
            direction: MotionDirection.movingRight,
            laneRelation: LaneRelation.leftLane,
          ),
        ],
      ));
      expect(d.state, DrivingState.pedestrianYield);
      expect(d.targetSpeedMps, 0);
    });

    test('a pedestrian nowhere near the crossing is not yielded to', () {
      final WorldState world = testWorld(
        ego: testEgo(speedMps: 10),
        roadMarkings: <RoadMarking>[
          testMarking(
              type: RoadMarkingType.crosswalk, distanceMeters: 18),
        ],
        tracks: <ObjectTrack>[
          testTrack(
            id: 4,
            objectClass: ObjectClass.person,
            position: const Vec2(-3.0, 45),
            direction: MotionDirection.parallel,
            laneRelation: LaneRelation.offRoad,
          ),
        ],
      );
      expect(world.pedestriansAtCrossing, isEmpty);
      expect(decide(world).state, isNot(DrivingState.pedestrianYield));
    });
  });

  group('speed bumps', () {
    test('a bump ahead eases the speed down towards its advisory', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 14),
        roadMarkings: <RoadMarking>[
          testMarking(type: RoadMarkingType.speedBump, distanceMeters: 20),
        ],
      ));
      expect(d.state, DrivingState.slowDown);
      expect(d.reason.toLowerCase(), contains('speed bump'));
      expect(d.targetSpeedMps, lessThan(14));
      expect(d.targetSpeedMps,
          greaterThanOrEqualTo(RoadMarkingType.speedBump.advisorySpeedMps));
    });

    test('a bump far away does not brake yet', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 12),
        roadMarkings: <RoadMarking>[
          testMarking(type: RoadMarkingType.speedBump, distanceMeters: 48),
        ],
      ));
      expect(d.state, isNot(DrivingState.slowDown));
    });

    test('a bump already behind us is ignored', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 14),
        roadMarkings: <RoadMarking>[
          testMarking(type: RoadMarkingType.speedBump, distanceMeters: -5),
        ],
      ));
      expect(d.state, isNot(DrivingState.slowDown));
    });
  });

  group('junctions', () {
    IntersectionEstimate junction({
      required IntersectionControl control,
      double distance = 25,
      double confidence = 0.7,
      List<int> crossing = const <int>[],
    }) =>
        IntersectionEstimate(
          distanceMeters: distance,
          confidence: Confidence(confidence),
          control: control,
          evidence: const <String>['stop line at 25 m'],
          crossingTrafficTrackIds: crossing,
        );

    test('an uncontrolled junction is approached with caution', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 14),
        intersection: junction(control: IntersectionControl.uncontrolled),
      ));
      expect(d.state, DrivingState.slowDown);
      expect(d.reason, contains('UNCONTROLLED'));
      expect(d.reason, contains('stop line at 25 m'),
          reason: 'the reason must name the evidence it acted on');
    });

    test('crossing traffic at a green light still slows us', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 14),
        intersection: junction(
          control: IntersectionControl.signalised,
          crossing: const <int>[9],
        ),
      ));
      expect(d.state, DrivingState.slowDown);
      expect(d.reason, contains('traffic crossing'));
    });

    test('a clear signalled junction at a sane speed needs no action', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 10),
        regulatory: const RegulatoryContext(
          speedLimitKph: 50,
          speedLimitConfidence: 0.9,
        ),
        intersection: junction(control: IntersectionControl.signalised),
      ));
      expect(d.state, isNot(DrivingState.slowDown));
    });

    test('a junction 200 m away is not acted on yet', () {
      final DrivingDecision d = decide(testWorld(
        ego: testEgo(speedMps: 14),
        intersection: junction(
          control: IntersectionControl.uncontrolled,
          distance: 200,
        ),
      ));
      expect(d.state, isNot(DrivingState.slowDown));
    });
  });

  group('what reaches the HUD', () {
    test('markings and junctions become hazards with their distance', () {
      final WorldState world = testWorld(
        roadMarkings: <RoadMarking>[
          testMarking(type: RoadMarkingType.speedBump, distanceMeters: 22),
        ],
      );
      final DrivingDecision d = decide(world);
      expect(d.triggeringHazard?.type, HazardType.speedBumpAhead);
      expect(d.triggeringHazard?.distanceMeters, closeTo(22, 0.01));
    });
  });
}
