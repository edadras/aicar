import 'package:aicar/core/confidence.dart';
import 'package:aicar/core/geometry.dart';
import 'package:aicar/perception/object_class.dart';
import 'package:aicar/perception/traffic_light.dart';
import 'package:aicar/perception/traffic_sign.dart';
import 'package:aicar/road/intersection_detector.dart';
import 'package:aicar/road/lane.dart';
import 'package:aicar/road/road_marking.dart';
import 'package:aicar/road/road_segmentation.dart';
import 'package:aicar/tracking/object_track.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/world_builder.dart';

void main() {
  const IntersectionDetector detector = IntersectionDetector();

  IntersectionEstimate? run({
    List<RoadMarking> markings = const <RoadMarking>[],
    List<TrafficLight> lights = const <TrafficLight>[],
    RegulatoryContext regulatory = const RegulatoryContext(),
    List<ObjectTrack> tracks = const <ObjectTrack>[],
    LaneDetectionResult? lanes,
    DrivableArea? drivableArea,
    double speed = 13.9,
  }) =>
      detector.detect(
        lanes: lanes ?? straightLanes(),
        drivableArea: drivableArea ?? straightCorridor(),
        markings: markings,
        lights: lights,
        regulatory: regulatory,
        tracks: tracks,
        egoSpeedMps: speed,
      );

  group('inference', () {
    test('an open road with no cues infers nothing', () {
      expect(run(), isNull);
    });

    test('a stop line alone is not enough on its own', () {
      // One weak cue must not reach the reporting threshold — otherwise a
      // tar seam becomes a junction every few hundred metres.
      final IntersectionEstimate? e = run(markings: <RoadMarking>[
        testMarking(
          type: RoadMarkingType.stopLine,
          distanceMeters: 30,
          confidence: 0.5,
        ),
      ]);
      expect(e, isNull);
    });

    test('a stop line and a crossing together do reach it', () {
      final IntersectionEstimate? e = run(markings: <RoadMarking>[
        testMarking(type: RoadMarkingType.stopLine, distanceMeters: 26),
        testMarking(type: RoadMarkingType.crosswalk, distanceMeters: 28),
      ]);
      expect(e, isNotNull);
      expect(e!.distanceMeters, closeTo(26, 0.1),
          reason: 'the nearest cue sets the distance');
      expect(e.control, IntersectionControl.uncontrolled);
      expect(e.evidence.length, greaterThanOrEqualTo(2));
    });

    test('lane markings ending while the road continues is a cue', () {
      final IntersectionEstimate? e = run(
        lanes: straightLanes(range: 22),
        drivableArea: straightCorridor(range: 60),
        markings: <RoadMarking>[
          testMarking(type: RoadMarkingType.stopLine, distanceMeters: 24),
        ],
      );
      expect(e, isNotNull);
      expect(
        e!.evidence.any((String s) => s.contains('lane markings end')),
        isTrue,
        reason: 'evidence: ${e.evidence}',
      );
    });
  });

  group('control', () {
    TrafficLight light(TrafficLightColor colour, double distance) =>
        TrafficLight(
          id: 1,
          color: colour,
          arrow: TrafficLightArrow.none,
          box: const BoundingBox(
              left: 0.45, top: 0.2, width: 0.05, height: 0.1),
          confidence: Confidence(0.9),
          colorConfidence: Confidence(0.9),
          relevance: TrafficLightRelevance.egoPath,
          relevanceConfidence: Confidence(0.8),
          frameId: 1,
          timestampMicros: 0,
          distanceMeters: distance,
          stableColorFrames: 5,
        );

    test('a resolved signal head makes the junction signalised', () {
      final IntersectionEstimate? e =
          run(lights: <TrafficLight>[light(TrafficLightColor.green, 30)]);
      expect(e, isNotNull);
      expect(e!.control, IntersectionControl.signalised);
      expect(e.control.requiresYield, isFalse);
    });

    test('a stop sign in force makes it a stop junction', () {
      final IntersectionEstimate? e = run(
        regulatory: const RegulatoryContext(pendingStop: true),
        markings: <RoadMarking>[
          testMarking(type: RoadMarkingType.stopLine, distanceMeters: 20),
        ],
      );
      expect(e!.control, IntersectionControl.stopSign);
      expect(e.approachSpeedMps(postedLimitMps: 13.9), lessThan(4));
    });

    test('an uncontrolled junction is approached slowly even when clear', () {
      final IntersectionEstimate? e = run(markings: <RoadMarking>[
        testMarking(type: RoadMarkingType.stopLine, distanceMeters: 26),
        testMarking(type: RoadMarkingType.crosswalk, distanceMeters: 28),
      ]);
      // Not knowing who has priority is not the same as having it.
      expect(e!.approachSpeedMps(postedLimitMps: 13.9), lessThan(13.9));
      expect(e.control.requiresYield, isTrue);
    });

    test('a green light does not license speed past crossing traffic', () {
      final IntersectionEstimate? e = run(
        lights: <TrafficLight>[light(TrafficLightColor.green, 25)],
        tracks: <ObjectTrack>[
          testTrack(
            id: 7,
            objectClass: ObjectClass.car,
            position: const Vec2(-6, 22),
            worldVelocity: const Vec2(5.0, 0),
            direction: MotionDirection.crossingLeftToRight,
            laneRelation: LaneRelation.oncoming,
          ),
        ],
      );
      expect(e!.hasCrossingTraffic, isTrue);
      expect(e.crossingTrafficTrackIds, contains(7));
      expect(e.approachSpeedMps(postedLimitMps: 13.9), lessThanOrEqualTo(4.5));
    });

    test('a car travelling with us is not crossing traffic', () {
      final IntersectionEstimate? e = run(
        lights: <TrafficLight>[light(TrafficLightColor.green, 25)],
        tracks: <ObjectTrack>[
          testTrack(
            id: 8,
            objectClass: ObjectClass.car,
            position: const Vec2(0, 20),
            worldVelocity: const Vec2(0, 12),
            direction: MotionDirection.parallel,
          ),
        ],
      );
      expect(e!.hasCrossingTraffic, isFalse);
    });
  });

  group('approach profile', () {
    test('caution starts further out the faster we are going', () {
      final IntersectionEstimate e = run(markings: <RoadMarking>[
        testMarking(type: RoadMarkingType.stopLine, distanceMeters: 40),
        testMarking(type: RoadMarkingType.crosswalk, distanceMeters: 42),
      ])!;
      final double slow =
          e.cautionOnsetMeters(speedMps: 8, postedLimitMps: 13.9);
      final double fast =
          e.cautionOnsetMeters(speedMps: 22, postedLimitMps: 25);
      expect(fast, greaterThan(slow));
    });

    test('serialises and comes back intact', () {
      final IntersectionEstimate e = run(markings: <RoadMarking>[
        testMarking(type: RoadMarkingType.stopLine, distanceMeters: 26),
        testMarking(type: RoadMarkingType.crosswalk, distanceMeters: 28),
      ])!;
      final IntersectionEstimate back =
          IntersectionEstimate.fromJson(e.toJson());
      expect(back.distanceMeters, closeTo(e.distanceMeters, 0.1));
      expect(back.control, e.control);
      expect(back.evidence, e.evidence);
    });
  });
}
