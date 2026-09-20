import 'package:aicar/core/geometry.dart';
import 'package:aicar/perception/object_class.dart';
import 'package:aicar/planning/collision_predictor.dart';
import 'package:aicar/planning/local_path_planner.dart';
import 'package:aicar/planning/planned_path.dart';
import 'package:aicar/tracking/object_track.dart';
import 'package:aicar/world_model/world_state.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/world_builder.dart';

CollisionAssessment assess(WorldState world, {int trackId = 1}) {
  final PlannedPath path = LocalPathPlanner().plan(world);
  const CollisionPredictor predictor = CollisionPredictor();
  final List<CollisionAssessment> results = predictor.assess(world, path);
  return results.firstWhere((CollisionAssessment a) => a.trackId == trackId);
}

void main() {
  group('CollisionPredictor risk levels', () {
    test('a stopped car directly ahead at speed is critical', () {
      final WorldState world = testWorld(
        ego: testEgo(speedMps: 14),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.car,
            position: const Vec2(0, 18),
            worldVelocity: const Vec2.zero(),
            direction: MotionDirection.stationary,
            egoSpeed: 14,
          ),
        ],
      );
      final CollisionAssessment a = assess(world);
      expect(a.risk, CollisionRisk.critical);
      expect(a.timeToCollisionSeconds, isNotNull);
      expect(a.timeToCollisionSeconds!, lessThan(1.6));
      expect(a.laneRelation, LaneRelation.egoLane);
    });

    test('a car ahead at the same speed is not a collision risk', () {
      final WorldState world = testWorld(
        ego: testEgo(speedMps: 14),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.car,
            position: const Vec2(0, 40),
            worldVelocity: const Vec2(0, 14),
            egoSpeed: 14,
          ),
        ],
      );
      final CollisionAssessment a = assess(world);
      expect(a.timeToCollisionSeconds, isNull);
      expect(a.risk, CollisionRisk.low);
    });

    test('a car in the next lane at speed is not a collision risk', () {
      final WorldState world = testWorld(
        ego: testEgo(speedMps: 14),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.car,
            position: const Vec2(3.5, 25),
            worldVelocity: const Vec2(0, 22),
            egoSpeed: 14,
          ),
        ],
      );
      final CollisionAssessment a = assess(world);
      expect(a.risk, CollisionRisk.low);
      expect(a.laneRelation, LaneRelation.rightLane);
      expect(a.timeToCollisionSeconds, isNull);
    });

    test('a pedestrian crossing into the path is high risk', () {
      final WorldState world = testWorld(
        ego: testEgo(speedMps: 11),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.person,
            position: const Vec2(-4.0, 18),
            // Walking right, into our path.
            worldVelocity: const Vec2(1.6, 0),
            relativeVelocity: const Vec2(1.6, -11),
            direction: MotionDirection.crossingLeftToRight,
            egoSpeed: 11,
          ),
        ],
      );
      final CollisionAssessment a = assess(world);
      expect(a.laneRelation, LaneRelation.crossing);
      expect(a.risk.severity,
          greaterThanOrEqualTo(CollisionRisk.high.severity));
      expect(a.willEnterPath, isTrue);
    });

    test('a pedestrian walking away from the road is low risk', () {
      final WorldState world = testWorld(
        ego: testEgo(speedMps: 11),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.person,
            position: const Vec2(-5.0, 20),
            worldVelocity: const Vec2(-1.4, 0),
            relativeVelocity: const Vec2(-1.4, -11),
            direction: MotionDirection.movingLeft,
            egoSpeed: 11,
          ),
        ],
      );
      final CollisionAssessment a = assess(world);
      expect(a.risk, CollisionRisk.low);
      expect(a.willEnterPath, isFalse);
    });

    test('a motorcycle filtering across the lane is flagged', () {
      final WorldState world = testWorld(
        ego: testEgo(speedMps: 8),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.motorcycle,
            position: const Vec2(2.6, 14),
            worldVelocity: const Vec2(-2.4, 9),
            relativeVelocity: const Vec2(-2.4, 1),
            direction: MotionDirection.crossingRightToLeft,
            egoSpeed: 8,
          ),
        ],
      );
      final CollisionAssessment a = assess(world);
      expect(a.laneRelation, LaneRelation.crossing);
      expect(a.risk.severity,
          greaterThanOrEqualTo(CollisionRisk.medium.severity));
    });

    test('close following is flagged even with nothing closing', () {
      final WorldState world = testWorld(
        ego: testEgo(speedMps: 20),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.car,
            position: const Vec2(0, 12), // 0.6 s headway at 20 m/s
            worldVelocity: const Vec2(0, 20),
            egoSpeed: 20,
          ),
        ],
      );
      final CollisionAssessment a = assess(world);
      expect(a.risk.severity,
          greaterThanOrEqualTo(CollisionRisk.high.severity));
      expect(a.reason, contains('headway'));
    });

    test('traffic infrastructure is never a collision target', () {
      final WorldState world = testWorld(
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.trafficSign,
            position: const Vec2(0.2, 12),
            direction: MotionDirection.stationary,
          ),
        ],
      );
      final CollisionAssessment a = assess(world);
      expect(a.risk, CollisionRisk.low);
      expect(a.laneRelation, LaneRelation.offRoad);
    });
  });

  group('CollisionPredictor uncertainty handling', () {
    test('a poorly-localised object yields a more cautious assessment', () {
      WorldState worldWith(double distanceConfidence) => testWorld(
            ego: testEgo(speedMps: 14),
            tracks: <ObjectTrack>[
              testTrack(
                id: 1,
                objectClass: ObjectClass.car,
                position: const Vec2(1.9, 26),
                worldVelocity: const Vec2(0, 12),
                egoSpeed: 14,
                distanceConfidence: distanceConfidence,
              ),
            ],
          );

      final CollisionAssessment confident = assess(worldWith(0.95));
      final CollisionAssessment doubtful = assess(worldWith(0.15));

      // The uncertain case must never be assessed as *safer* than the
      // confident one.
      expect(doubtful.risk.severity,
          greaterThanOrEqualTo(confident.risk.severity));
      expect(doubtful.minimumGapMeters,
          lessThanOrEqualTo(confident.minimumGapMeters));
    });

    test('lane-relation confidence reflects the road model quality', () {
      final WorldState good = testWorld(
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.car,
            position: const Vec2(0, 30),
            worldVelocity: const Vec2(0, 14),
          ),
        ],
      );
      final WorldState poor = testWorld(
        lanes: straightLanes(confidence: 0.2),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.car,
            position: const Vec2(0, 30),
            worldVelocity: const Vec2(0, 14),
            distanceConfidence: 0.3,
          ),
        ],
      );
      expect(assess(poor).laneRelationConfidence,
          lessThan(assess(good).laneRelationConfidence));
    });
  });

  group('CollisionPredictor ordering', () {
    test('returns the most severe assessment first', () {
      final WorldState world = testWorld(
        ego: testEgo(speedMps: 14),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.car,
            position: const Vec2(3.6, 45),
            worldVelocity: const Vec2(0, 14),
            egoSpeed: 14,
          ),
          testTrack(
            id: 2,
            objectClass: ObjectClass.car,
            position: const Vec2(0, 16),
            worldVelocity: const Vec2.zero(),
            direction: MotionDirection.stationary,
            egoSpeed: 14,
          ),
        ],
      );
      final PlannedPath path = LocalPathPlanner().plan(world);
      final List<CollisionAssessment> results =
          const CollisionPredictor().assess(world, path);
      expect(results.first.trackId, 2);
      expect(results.first.risk, CollisionRisk.critical);
    });
  });
}
