import 'dart:typed_data';

import 'package:aicar/camera/camera_calibration.dart';
import 'package:aicar/road/classical_lane_detector.dart';
import 'package:aicar/road/lane.dart';
import 'package:aicar/road/road_marking.dart';
import 'package:aicar/road/road_marking_detector.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/conditions.dart';
import '../support/painted_road.dart';
import '../support/synthetic_road.dart';

/// What actually degrades, and by how much.
///
/// The point is not that the stack copes with everything — it does not. The
/// point is that when it stops coping it *says so*, and the confidence it
/// reports tracks how hard the scene really is. A detector that returns a
/// confident lane from a rain-blurred image is far more dangerous than one
/// that returns nothing.
void main() {
  final CameraCalibration cal =
      CameraCalibration.galaxyS23Default().scaledTo(640, 360);

  Uint8List cleanRoad({
    double curvature = 0,
    int markingLuma = 225,
    Set<int> dashed = const <int>{},
  }) =>
      SyntheticRoad(
        calibration: cal,
        curvature: curvature,
        markingLuma: markingLuma,
        dashedLanes: dashed,
        noiseAmplitude: 4,
      ).renderGray();

  final ClassicalLaneDetector detector =
      ClassicalLaneDetector(calibration: cal);

  Future<LaneDetectionResult> detectLanes(Uint8List gray) =>
      detector.detectLanes(Conditions.frameOf(gray, cal));

  /// Confidence in daylight, as the baseline every other case is judged
  /// against.
  late double baseline;

  setUpAll(() async {
    baseline = (await detectLanes(cleanRoad())).overallConfidence;
  });

  group('the baseline', () {
    test('a clean daylight road is found confidently', () async {
      final LaneDetectionResult r = await detectLanes(cleanRoad());
      expect(r.mode, LaneMode.bothBoundaries);
      expect(r.overallConfidence, greaterThan(0.6));
      expect(r.laneWidthMeters, closeTo(3.5, 0.5));
    });
  });

  group('night', () {
    test('the scene is dark enough for the stack to call it night', () async {
      final Uint8List gray = Conditions.night(cleanRoad(), cal);
      // 55 is the threshold WorldState.isNight uses.
      expect(Conditions.meanLuma(gray), lessThan(55));
    });

    test('markings inside the headlight cone are still found', () async {
      final LaneDetectionResult r =
          await detectLanes(Conditions.night(cleanRoad(), cal));
      expect(r.mode, isNot(LaneMode.none));
    });

    test('confidence is lower than in daylight, and says so', () async {
      final LaneDetectionResult r =
          await detectLanes(Conditions.night(cleanRoad(), cal));
      expect(r.overallConfidence, lessThan(baseline),
          reason: 'night is harder and the number must admit it');
    });

    test('the usable range shrinks to roughly the headlight reach', () async {
      final LaneDetectionResult night =
          await detectLanes(Conditions.night(cleanRoad(), cal));
      final LaneDetectionResult day = await detectLanes(cleanRoad());
      expect(night.usableRangeMeters, lessThan(day.usableRangeMeters),
          reason: 'you cannot see past your headlights');
    });
  });

  group('rain', () {
    test('a soaked scene never produces a confident wrong lane', () async {
      final LaneDetectionResult r = await detectLanes(
        Conditions.rain(cleanRoad(), cal, contrastLoss: 0.6),
      );
      // Either it finds the lane, or it says it cannot. What it must never
      // do is report high confidence in something else.
      if (r.mode == LaneMode.bothBoundaries) {
        expect(r.laneWidthMeters, closeTo(3.5, 1.0),
            reason: 'a confident answer has to be the right answer');
      } else {
        expect(r.overallConfidence, lessThan(0.6));
      }
    });

    test('heavy rain costs confidence', () async {
      final LaneDetectionResult light = await detectLanes(
          Conditions.rain(cleanRoad(), cal, contrastLoss: 0.2));
      final LaneDetectionResult heavy = await detectLanes(
        Conditions.rain(cleanRoad(), cal,
            contrastLoss: 0.7, streakCount: 600, reflectionCount: 200),
      );
      expect(heavy.overallConfidence, lessThan(light.overallConfidence));
    });

    test('wet reflections do not invent a crossing', () async {
      // Specular patches on wet tarmac are bright, road-coloured and roughly
      // the right size. This is the false positive that matters.
      final RoadMarkingResult r = RoadMarkingDetector().detect(
        Conditions.frameOf(
          Conditions.rain(cleanRoad(), cal, reflectionCount: 300),
          cal,
        ),
      );
      expect(r.nearestOf(RoadMarkingType.crosswalk), isNull,
          reason: 'markings found: ${r.markings}');
    });
  });

  group('glare', () {
    test('a low sun does not produce a confident wrong lane', () async {
      final LaneDetectionResult r =
          await detectLanes(Conditions.glare(cleanRoad(), cal));
      if (r.mode == LaneMode.bothBoundaries) {
        expect(r.laneWidthMeters, closeTo(3.5, 1.0));
      } else {
        expect(r.overallConfidence, lessThan(0.7));
      }
    });

    test('a blown-out frame is reported, not guessed at', () async {
      final LaneDetectionResult r = await detectLanes(
        Conditions.glare(cleanRoad(), cal,
            radiusNormalized: 0.75, bloom: 1.0),
      );
      expect(r.overallConfidence, lessThan(baseline * 0.9));
    });
  });

  group('curves', () {
    test('a moderate curve is still tracked', () async {
      final LaneDetectionResult r =
          await detectLanes(cleanRoad(curvature: 0.002));
      expect(r.mode, isNot(LaneMode.none));
      expect(r.overallConfidence, greaterThan(0.4));
    });

    test('a tight curve costs range, not correctness', () async {
      final LaneDetectionResult tight =
          await detectLanes(cleanRoad(curvature: 0.006));
      if (tight.mode == LaneMode.bothBoundaries) {
        expect(tight.laneWidthMeters, closeTo(3.5, 0.8));
      }
      final LaneDetectionResult straight = await detectLanes(cleanRoad());
      expect(tight.usableRangeMeters,
          lessThanOrEqualTo(straight.usableRangeMeters));
    });
  });

  group('worn paint', () {
    test('faded markings lower confidence rather than vanishing', () async {
      final LaneDetectionResult worn =
          await detectLanes(cleanRoad(markingLuma: 120));
      expect(worn.overallConfidence, lessThan(baseline));
    });

    test('nearly invisible paint gives up instead of guessing', () async {
      final LaneDetectionResult gone =
          await detectLanes(cleanRoad(markingLuma: 80));
      if (gone.mode == LaneMode.bothBoundaries) {
        expect(gone.laneWidthMeters, closeTo(3.5, 1.0));
      } else {
        expect(gone.overallConfidence, lessThan(0.5));
      }
    });

    test('dashed markings are still found', () async {
      final LaneDetectionResult r =
          await detectLanes(cleanRoad(dashed: const <int>{0}));
      expect(r.mode, isNot(LaneMode.none));
    });
  });

  group('traffic', () {
    test('a lorry covering one boundary drops to single-edge, not to a '
        'fabricated lane', () async {
      final Uint8List gray = Conditions.occlude(
        cleanRoad(),
        cal,
        const <GroundRect>[
          GroundRect(
            nearMeters: 8,
            farMeters: 30,
            leftMeters: 1.2,
            rightMeters: 5.0,
          ),
        ],
      );
      final LaneDetectionResult r = await detectLanes(gray);
      expect(r.mode, isNot(LaneMode.bothBoundaries),
          reason: 'the right boundary is under a lorry');
      if (r.mode == LaneMode.singleBoundary) {
        expect(r.laneWidthMeters, closeTo(3.5, 1.0),
            reason: 'the width comes from the learned prior, not invention');
      }
    });

    test('a queue covering both boundaries produces NO_LANE_MODE', () async {
      final Uint8List gray = Conditions.occlude(
        cleanRoad(),
        cal,
        const <GroundRect>[
          GroundRect(
            nearMeters: 6,
            farMeters: 40,
            leftMeters: -6,
            rightMeters: 6,
          ),
        ],
      );
      final LaneDetectionResult r = await detectLanes(gray);
      expect(
        r.mode == LaneMode.noLane || r.mode == LaneMode.none,
        isTrue,
        reason: 'mode was ${r.mode}, confidence ${r.overallConfidence}',
      );
    });
  });

  group('stacked conditions', () {
    test('night plus rain is worse than either alone, and admits it', () async {
      final double night =
          (await detectLanes(Conditions.night(cleanRoad(), cal)))
              .overallConfidence;
      final double rain =
          (await detectLanes(Conditions.rain(cleanRoad(), cal)))
              .overallConfidence;
      final double both = (await detectLanes(
        Conditions.rain(Conditions.night(cleanRoad(), cal), cal),
      )).overallConfidence;

      expect(both, lessThanOrEqualTo(night));
      expect(both, lessThanOrEqualTo(rain));
    });

    test('no condition ever produces a crossing on an empty road', () async {
      // Across every hard scene, the marking detector must not hallucinate.
      final List<Uint8List> scenes = <Uint8List>[
        Conditions.night(cleanRoad(), cal),
        Conditions.rain(cleanRoad(), cal),
        Conditions.glare(cleanRoad(), cal),
        Conditions.rain(Conditions.night(cleanRoad(), cal), cal),
        cleanRoad(curvature: 0.004),
        cleanRoad(markingLuma: 110),
      ];
      for (final Uint8List gray in scenes) {
        final RoadMarkingResult r =
            RoadMarkingDetector().detect(Conditions.frameOf(gray, cal));
        expect(r.nearestOf(RoadMarkingType.crosswalk), isNull,
            reason: 'markings: ${r.markings}');
        expect(r.nearestOf(RoadMarkingType.speedBump), isNull,
            reason: 'markings: ${r.markings}');
      }
    });

    test('a real crossing is still found at night', () async {
      final PaintedRoad road = PaintedRoad(
        calibration: cal,
        patches: PaintedRoad.crosswalk(nearMeters: 14, depthMeters: 3),
      );
      final RoadMarkingResult r = RoadMarkingDetector().detect(
        Conditions.frameOf(
          Conditions.night(road.renderGray(), cal, headlightReachMeters: 30),
          cal,
        ),
      );
      expect(r.nearestOf(RoadMarkingType.crosswalk), isNotNull,
          reason: 'markings: ${r.markings}');
    });
  });
}
