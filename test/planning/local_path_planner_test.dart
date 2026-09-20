import 'package:aicar/core/geometry.dart';
import 'package:aicar/perception/object_class.dart';
import 'package:aicar/planning/local_path_planner.dart';
import 'package:aicar/planning/planned_path.dart';
import 'package:aicar/road/lane.dart';
import 'package:aicar/road/no_lane_corridor.dart';
import 'package:aicar/perception/traffic_sign.dart';
import 'package:aicar/road/road_segmentation.dart';
import 'package:aicar/tracking/object_track.dart';
import 'package:aicar/world_model/world_state.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/world_builder.dart';

void main() {
  group('LocalPathPlanner on a clear road', () {
    test('follows the lane centreline', () {
      final LocalPathPlanner planner = LocalPathPlanner();
      final PlannedPath path = planner.plan(testWorld());

      expect(path.isUsable, isTrue);
      expect(path.source, PathSource.laneCenterline);
      expect(path.lateralAt(10), closeTo(0, 0.2));
      expect(path.lateralAt(25), closeTo(0, 0.2));
      expect(path.confidence, greaterThan(0.3));
      expect(path.isBlocked, isFalse);
    });

    test('follows a curved lane', () {
      final LocalPathPlanner planner = LocalPathPlanner();
      final PlannedPath path = planner.plan(
        testWorld(lanes: straightLanes(curvature: 0.002)),
      );
      expect(path.isUsable, isTrue);
      // 0.002 * 20² = 0.8 m at 20 m.
      expect(path.lateralAt(20), closeTo(0.8, 0.35));
      expect(path.curvatureAt(20), greaterThan(0));
    });

    test('a lane offset moves the path with it', () {
      final LocalPathPlanner planner = LocalPathPlanner();
      final PlannedPath path =
          planner.plan(testWorld(lanes: straightLanes(offset: 1.0)));
      expect(path.lateralAt(15), closeTo(1.0, 0.35));
    });

    test('target speed respects the posted limit', () {
      final LocalPathPlanner planner = LocalPathPlanner();
      final PlannedPath path = planner.plan(testWorld(
        regulatory: const RegulatoryContext(
          speedLimitKph: 50,
          speedLimitConfidence: 0.9,
        ),
      ));
      expect(path.limitingSpeedMps, lessThanOrEqualTo(50 / 3.6 + 0.01));
    });

    test('a school zone tightens the target speed further', () {
      final LocalPathPlanner planner = LocalPathPlanner();
      final PlannedPath path = planner.plan(testWorld(
        regulatory: const RegulatoryContext(
          speedLimitKph: 50,
          speedLimitConfidence: 0.9,
          inSchoolZone: true,
        ),
      ));
      expect(path.limitingSpeedMps, lessThanOrEqualTo(30 / 3.6 + 0.01));
    });

    test('a tight curve limits speed below the posted limit', () {
      final LocalPathPlanner planner = LocalPathPlanner();
      final PlannedPath path = planner.plan(testWorld(
        lanes: straightLanes(curvature: 0.008),
        regulatory: const RegulatoryContext(
          speedLimitKph: 90,
          speedLimitConfidence: 0.9,
        ),
      ));
      expect(path.limitingSpeedMps, lessThan(90 / 3.6));
    });
  });

  group('LocalPathPlanner with obstacles', () {
    test('offsets around a cyclist at the lane edge', () {
      final LocalPathPlanner planner = LocalPathPlanner();
      final PlannedPath path = planner.plan(testWorld(
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.bicycle,
            position: const Vec2(1.3, 20),
            worldVelocity: const Vec2(0, 5),
          ),
        ],
      ));

      expect(path.isUsable, isTrue);
      // Must move left, away from the cyclist on the right.
      expect(path.lateralOffsetFromReference, lessThan(-0.05));
      expect(path.source, PathSource.obstacleAvoidance);
    });

    test('does not swerve for a vehicle in the next lane', () {
      final LocalPathPlanner planner = LocalPathPlanner();
      final PlannedPath path = planner.plan(testWorld(
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.car,
            position: const Vec2(3.5, 22),
            worldVelocity: const Vec2(0, 14),
            laneRelation: LaneRelation.rightLane,
          ),
        ],
      ));
      expect(path.lateralOffsetFromReference.abs(), lessThan(0.3));
    });

    test('reports the path as blocked when the lane is fully obstructed', () {
      final LocalPathPlanner planner = LocalPathPlanner();
      final PlannedPath path = planner.plan(testWorld(
        // A stopped lorry squarely in the lane, with a narrow corridor so
        // there is nowhere to go around it.
        drivableArea: straightCorridor(halfWidth: 1.9),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.truck,
            position: const Vec2(0, 14),
            worldVelocity: const Vec2.zero(),
            direction: MotionDirection.stationary,
          ),
        ],
      ));

      expect(path.isBlocked, isTrue);
      expect(path.blockedAtMeters, isNotNull);
      expect(path.blockedAtMeters!, lessThan(20));
      expect(path.blockReason, isNotNull);
    });

    test('slows for tight clearance', () {
      final LocalPathPlanner planner = LocalPathPlanner();
      final PlannedPath clear = planner.plan(testWorld());
      final LocalPathPlanner planner2 = LocalPathPlanner();
      final PlannedPath tight = planner2.plan(testWorld(
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.trafficCone,
            position: const Vec2(1.15, 18),
            direction: MotionDirection.stationary,
          ),
        ],
      ));
      expect(tight.limitingSpeedMps, lessThan(clear.limitingSpeedMps));
    });
  });

  group('LocalPathPlanner without a road model', () {
    test('returns no path when there is nothing to follow', () {
      final LocalPathPlanner planner = LocalPathPlanner();
      final WorldState world = testWorld(
        lanes: LaneDetectionResult.empty(frameId: 1, timestampMicros: 0),
        drivableArea: DrivableArea.empty(frameId: 1, timestampMicros: 0),
      );
      final PlannedPath path = planner.plan(world);

      expect(path.isUsable, isFalse);
      expect(path.source, PathSource.none);
      expect(path.blockReason, isNotNull);
    });

    test('uses the NO_LANE_MODE corridor when lanes are unavailable', () {
      final LocalPathPlanner planner = LocalPathPlanner();
      final WorldState base = testWorld(
        lanes: LaneDetectionResult.empty(frameId: 1, timestampMicros: 0),
      );
      final WorldState world = base.copyWith(
        corridor: CorridorEstimate(
          centerline: const Polynomial(<double>[0.4, 0.01, 0]),
          halfWidthMeters: 1.8,
          minRangeMeters: 4,
          maxRangeMeters: 35,
          confidence: 0.55,
          evidence: <CorridorEvidence>{
            CorridorEvidence.drivableArea,
            CorridorEvidence.leadVehicles,
          },
        ),
      );

      final PlannedPath path = planner.plan(world);
      expect(path.isUsable, isTrue);
      expect(path.source, PathSource.corridorCenterline);
      expect(path.lateralAt(20), closeTo(0.4 + 0.01 * 20, 0.35));
      // NO_LANE_MODE must be less confident than a marked lane.
      final PlannedPath lanePath = LocalPathPlanner().plan(testWorld());
      expect(path.confidence, lessThan(lanePath.confidence));
    });

    test('holds the previous path briefly, then gives up', () {
      final LocalPathPlanner planner = LocalPathPlanner();
      final PlannedPath good = planner.plan(testWorld(timestampMicros: 0));
      expect(good.isUsable, isTrue);

      final WorldState lost = testWorld(
        timestampMicros: 200000, // 0.2 s later
        lanes: LaneDetectionResult.empty(frameId: 2, timestampMicros: 200000),
        drivableArea:
            DrivableArea.empty(frameId: 2, timestampMicros: 200000),
      );
      final PlannedPath held = planner.plan(lost, previous: good);
      expect(held.source, PathSource.previousPathHold);
      expect(held.confidence, lessThan(good.confidence));

      final WorldState longGone = testWorld(
        timestampMicros: 1200000, // 1.2 s later
        lanes: LaneDetectionResult.empty(frameId: 3, timestampMicros: 1200000),
        drivableArea:
            DrivableArea.empty(frameId: 3, timestampMicros: 1200000),
      );
      final PlannedPath abandoned = planner.plan(longGone, previous: good);
      expect(abandoned.source, PathSource.none);
      expect(abandoned.isUsable, isFalse);
    });
  });

  group('LocalPathPlanner continuity', () {
    test('does not jump laterally between frames', () {
      final LocalPathPlanner planner = LocalPathPlanner();
      PlannedPath? previous;
      final List<double> offsets = <double>[];

      for (int i = 0; i < 10; i++) {
        // An obstacle appears halfway through; the response must be gradual.
        final PlannedPath path = planner.plan(
          testWorld(
            frameId: i,
            timestampMicros: i * 50000,
            tracks: i < 5
                ? const <ObjectTrack>[]
                : <ObjectTrack>[
                    testTrack(
                      id: 1,
                      objectClass: ObjectClass.trafficCone,
                      position: const Vec2(1.2, 20),
                      direction: MotionDirection.stationary,
                    ),
                  ],
          ),
          previous: previous,
        );
        offsets.add(path.lateralOffsetFromReference);
        previous = path;
      }

      for (int i = 1; i < offsets.length; i++) {
        expect((offsets[i] - offsets[i - 1]).abs(), lessThan(0.35),
            reason: 'frame $i jumped from ${offsets[i - 1]} to ${offsets[i]}');
      }
      // It should nevertheless have moved by the end.
      expect(offsets.last, lessThan(-0.05));
    });
  });
}
