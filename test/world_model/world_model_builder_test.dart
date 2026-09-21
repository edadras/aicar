import 'package:aicar/core/confidence.dart';
import 'package:aicar/core/geometry.dart';
import 'package:aicar/perception/detection.dart';
import 'package:aicar/perception/object_class.dart';
import 'package:aicar/perception/traffic_sign.dart';
import 'package:aicar/road/lane.dart';
import 'package:aicar/road/road_segmentation.dart';
import 'package:aicar/tracking/object_track.dart';
import 'package:aicar/world_model/hazard.dart';
import 'package:aicar/world_model/world_model_builder.dart';
import 'package:aicar/world_model/world_state.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/world_builder.dart';

const WorldModelBuilder builder = WorldModelBuilder();

WorldState build({
  DetectionResult? detections,
  LaneDetectionResult? lanes,
  List<ObjectTrack> tracks = const <ObjectTrack>[],
  RegulatoryContext regulatory = const RegulatoryContext(),
  double ambientLuminance = 140,
  double planningConfidence = 0.8,
  List<String> degraded = const <String>[],
  double egoSpeed = 14,
}) =>
    builder.build(
      frameId: 1,
      timestampMicros: 1000,
      ego: testEgo(speedMps: egoSpeed),
      calibration: testCalibration,
      lanes: lanes ?? straightLanes(),
      drivableArea: straightCorridor(),
      roadEdges: const <RoadEdge>[],
      tracks: tracks,
      signs: const <TrafficSign>[],
      lights: const <dynamic>[].cast(),
      regulatory: regulatory,
      detections: detections ??
          const DetectionResult(
            detections: <Detection>[],
            frameId: 1,
            timestampMicros: 1000,
            inferenceMicros: 9000,
            modelName: 'test',
          ),
      ambientLuminance: ambientLuminance,
      planningConfidence: planningConfidence,
      degradedSubsystems: degraded,
    );

void main() {
  group('autonomy confidence', () {
    test('a healthy stack is confident', () {
      final WorldState world = build();
      expect(world.autonomy.overall, greaterThan(0.5));
      expect(world.autonomy.isLow, isFalse);
    });

    test('a missing detector collapses confidence, not the road', () {
      final WorldState world = build(
        detections: DetectionResult.noModel(
          frameId: 1,
          timestampMicros: 1000,
          reason: 'no object detection model installed',
        ),
        degraded: <String>['detection: no object detection model installed'],
      );

      expect(world.autonomy.perception, 0);
      expect(world.autonomy.isLow, isTrue);
      expect(world.autonomy.weakestSubsystem, 'perception');
      expect(
        world.hazards.map((Hazard h) => h.type),
        contains(HazardType.lowAutonomyConfidence),
      );
    });

    test('an empty but working detector is a confident observation', () {
      // "The road ahead is clear" is a real observation, not an absence.
      final WorldState world = build();
      expect(world.autonomy.perception, greaterThan(0.7));
    });

    test('the roll-up is dragged down by its weakest subsystem', () {
      final WorldState good = build();
      final WorldState oneBadSubsystem = build(
        lanes: LaneDetectionResult.empty(frameId: 1, timestampMicros: 1000),
      );
      expect(oneBadSubsystem.autonomy.overall,
          lessThan(good.autonomy.overall));
      expect(oneBadSubsystem.autonomy.weakestSubsystem, 'lanes');
    });

    test('night reduces every camera-derived score', () {
      final WorldState day = build(ambientLuminance: 150);
      final WorldState night = build(ambientLuminance: 25);
      expect(night.autonomy.perception, lessThan(day.autonomy.perception));
      expect(night.autonomy.lanes, lessThan(day.autonomy.lanes));
      // Ego motion does not come from the camera and must be unaffected.
      expect(night.autonomy.egoMotion, day.autonomy.egoMotion);
      expect(night.isNight, isTrue);
    });

    test('depth has a geometric floor, lower when uncalibrated', () {
      final WorldState calibrated = build();
      expect(calibrated.autonomy.depth, greaterThan(0.4));

      final WorldState uncalibrated = builder.build(
        frameId: 1,
        timestampMicros: 1000,
        ego: testEgo(),
        calibration: testCalibration.copyWith(isCalibrated: false),
        lanes: straightLanes(),
        drivableArea: straightCorridor(),
        roadEdges: const <RoadEdge>[],
        tracks: const <ObjectTrack>[],
        signs: const <TrafficSign>[],
        lights: const <dynamic>[].cast(),
        regulatory: const RegulatoryContext(),
        planningConfidence: 0.8,
      );
      expect(uncalibrated.autonomy.depth,
          lessThan(calibrated.autonomy.depth));
    });
  });

  group('hazards', () {
    test('a critical track becomes a critical hazard', () {
      final WorldState world = build(
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.car,
            position: const Vec2(0, 14),
            risk: CollisionRisk.critical,
            ttc: 1.1,
            direction: MotionDirection.stationary,
          ),
        ],
      );
      final Hazard? primary = world.primaryHazard;
      expect(primary, isNotNull);
      expect(primary!.severity, HazardSeverity.critical);
      expect(primary.relatedTrackId, 1);
      expect(primary.timeToCollisionSeconds, 1.1);
    });

    test('a crossing motorcycle gets its own warning', () {
      final WorldState world = build(
        tracks: <ObjectTrack>[
          testTrack(
            id: 4,
            objectClass: ObjectClass.motorcycle,
            position: const Vec2(2.0, 16),
            relativeVelocity: const Vec2(-2.2, -1),
            direction: MotionDirection.crossingRightToLeft,
            laneRelation: LaneRelation.crossing,
          ),
        ],
      );
      expect(
        world.hazards.map((Hazard h) => h.type),
        contains(HazardType.motorcycleCrossing),
      );
    });

    test('lane departure is only raised on a trustworthy lane model', () {
      final WorldState drifting = build(
        lanes: straightLanes(offset: 0.8, confidence: 0.9),
      );
      expect(
        drifting.hazards.map((Hazard h) => h.type),
        contains(HazardType.laneDeparture),
      );

      final WorldState unsure = build(
        lanes: straightLanes(offset: 0.8, confidence: 0.3),
      );
      expect(
        unsure.hazards.map((Hazard h) => h.type),
        isNot(contains(HazardType.laneDeparture)),
      );
    });

    test('lane departure is not raised when stationary', () {
      final WorldState world = build(
        lanes: straightLanes(offset: 0.8, confidence: 0.9),
        egoSpeed: 0.2,
      );
      expect(
        world.hazards.map((Hazard h) => h.type),
        isNot(contains(HazardType.laneDeparture)),
      );
    });

    test('exceeding a confident speed limit is a hazard', () {
      final WorldState world = build(
        egoSpeed: 22,
        regulatory: const RegulatoryContext(
          speedLimitKph: 50,
          speedLimitConfidence: 0.9,
        ),
      );
      expect(
        world.hazards.map((Hazard h) => h.type),
        contains(HazardType.speedLimitExceeded),
      );
    });

    test('hazards are ordered most severe first', () {
      final WorldState world = build(
        egoSpeed: 22,
        lanes: straightLanes(offset: 0.8, confidence: 0.9),
        regulatory: const RegulatoryContext(
          speedLimitKph: 50,
          speedLimitConfidence: 0.9,
        ),
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.person,
            position: const Vec2(0, 10),
            risk: CollisionRisk.critical,
            ttc: 0.9,
          ),
        ],
      );
      expect(world.hazards.length, greaterThan(1));
      for (int i = 1; i < world.hazards.length; i++) {
        expect(
          world.hazards[i - 1].severity.level,
          greaterThanOrEqualTo(world.hazards[i].severity.level),
        );
      }
      expect(world.worstHazardSeverity, HazardSeverity.critical);
    });
  });

  group('world views', () {
    test('the lead vehicle is the nearest one in our lane ahead', () {
      final WorldState world = build(
        tracks: <ObjectTrack>[
          testTrack(
            id: 1,
            objectClass: ObjectClass.car,
            position: const Vec2(0, 40),
          ),
          testTrack(
            id: 2,
            objectClass: ObjectClass.car,
            position: const Vec2(0, 22),
          ),
          testTrack(
            id: 3,
            objectClass: ObjectClass.car,
            position: const Vec2(3.5, 15),
            laneRelation: LaneRelation.rightLane,
          ),
        ],
      );
      expect(world.leadVehicle?.id, 2);
    });

    test('vulnerable road users are separated out', () {
      final WorldState world = build(
        tracks: <ObjectTrack>[
          testTrack(
              id: 1, objectClass: ObjectClass.car, position: const Vec2(0, 20)),
          testTrack(
              id: 2,
              objectClass: ObjectClass.person,
              position: const Vec2(-3, 12)),
          testTrack(
              id: 3,
              objectClass: ObjectClass.motorcycle,
              position: const Vec2(2, 18)),
        ],
      );
      expect(world.pedestrians.map((ObjectTrack t) => t.id), <int>[2]);
      expect(world.motorcycles.map((ObjectTrack t) => t.id), <int>[3]);
      expect(world.vulnerableRoadUsers.length, 2);
      expect(world.vehicles.length, 2); // car and motorcycle
    });

    test('the reference centreline prefers lanes over the corridor', () {
      final WorldState world = build();
      expect(world.referenceCenterline, isNotNull);
      expect(world.referenceCenterline!.evaluate(20), closeTo(0, 0.01));
    });

    test('json round-trips through a recording', () {
      final WorldState world = build(
        tracks: <ObjectTrack>[
          testTrack(
            id: 9,
            objectClass: ObjectClass.truck,
            position: const Vec2(0.4, 28),
            risk: CollisionRisk.medium,
            ttc: 3.2,
          ),
        ],
        regulatory: const RegulatoryContext(
          speedLimitKph: 80,
          speedLimitConfidence: 0.85,
        ),
      );

      final WorldState back = WorldState.fromRecordedJson(
        world.toJson(),
        calibration: testCalibration,
      );

      expect(back.frameId, world.frameId);
      expect(back.tracks, hasLength(1));
      expect(back.tracks.first.id, 9);
      expect(back.tracks.first.objectClass, ObjectClass.truck);
      expect(back.tracks.first.collisionRisk, CollisionRisk.medium);
      expect(back.tracks.first.timeToCollisionSeconds, closeTo(3.2, 0.01));
      expect(back.regulatory.speedLimitKph, 80);
      expect(back.autonomy.overall, closeTo(world.autonomy.overall, 0.01));
      expect(back.lanes.laneWidthMeters,
          closeTo(world.lanes.laneWidthMeters, 0.01));
    });
  });
}
