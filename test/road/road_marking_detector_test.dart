import 'package:aicar/camera/camera_calibration.dart';
import 'package:aicar/camera/camera_frame.dart';
import 'package:aicar/road/road_marking.dart';
import 'package:aicar/road/road_marking_detector.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/painted_road.dart';

void main() {
  final CameraCalibration calib = CameraCalibration.galaxyS23Default()
      .scaledTo(640, 360);

  RoadMarkingResult run(List<PaintPatch> patches, {int noise = 0}) {
    final PaintedRoad road = PaintedRoad(
      calibration: calib,
      patches: patches,
      noiseAmplitude: noise,
    );
    final CameraFrame frame = road.renderFrame();
    return RoadMarkingDetector().detect(frame);
  }

  group('crosswalk', () {
    test('is found at the distance it was painted', () {
      final RoadMarkingResult r =
          run(PaintedRoad.crosswalk(nearMeters: 15, depthMeters: 3));
      final RoadMarking? c = r.nearestOf(RoadMarkingType.crosswalk);
      expect(c, isNotNull, reason: 'markings: ${r.markings}');
      expect(c!.distanceMeters, closeTo(15, 1.2));
      expect(c.widthMeters, greaterThan(4.0));
      expect(c.confidence.value, greaterThan(0.3));
    });

    test('survives sensor noise', () {
      final RoadMarkingResult r = run(
        PaintedRoad.crosswalk(nearMeters: 12, depthMeters: 3),
        noise: 12,
      );
      expect(r.nearestOf(RoadMarkingType.crosswalk), isNotNull,
          reason: 'markings: ${r.markings}');
    });
  });

  group('speed bump', () {
    test('repeated transverse bands read as a bump, not a crossing', () {
      final RoadMarkingResult r =
          run(PaintedRoad.speedBump(nearMeters: 12, bands: 4));
      final RoadMarking? b = r.nearestOf(RoadMarkingType.speedBump);
      expect(b, isNotNull, reason: 'markings: ${r.markings}');
      expect(b!.distanceMeters, closeTo(12, 1.2));
      expect(r.nearestOf(RoadMarkingType.crosswalk), isNull);
    });
  });

  group('stop line', () {
    test('a single solid bar is a stop line, not a bump', () {
      final RoadMarkingResult r =
          run(PaintedRoad.stopLine(nearMeters: 14));
      final RoadMarking? s = r.nearestOf(RoadMarkingType.stopLine);
      expect(s, isNotNull, reason: 'markings: ${r.markings}');
      expect(s!.distanceMeters, closeTo(14, 1.2));
      expect(r.nearestOf(RoadMarkingType.speedBump), isNull);
    });
  });

  group('range', () {
    // A crossing's stripes repeat across the image where resolution is good;
    // a hump's bands repeat into the vanishing point where it is not. The
    // asymmetry is physical, and pinning it here keeps it a stated property
    // rather than a surprise on the road.
    RoadMarkingResult atResolution(int width, List<PaintPatch> patches) {
      final CameraCalibration c = CameraCalibration.galaxyS23Default()
          .scaledTo(width, (width * 9 / 16).round());
      return RoadMarkingDetector()
          .detect(PaintedRoad(calibration: c, patches: patches).renderFrame());
    }

    test('crossings are seen well beyond speed bumps', () {
      final RoadMarkingResult crossing = atResolution(
          640, PaintedRoad.crosswalk(nearMeters: 24, depthMeters: 3));
      expect(crossing.nearestOf(RoadMarkingType.crosswalk), isNotNull);

      final RoadMarkingResult bump =
          atResolution(640, PaintedRoad.speedBump(nearMeters: 24, bands: 4));
      expect(bump.nearestOf(RoadMarkingType.speedBump), isNull,
          reason: 'at 640x360 a hump at 24 m is under two image rows deep');
    });

    test('a higher capture resolution extends the bump range', () {
      final List<PaintPatch> patches =
          PaintedRoad.speedBump(nearMeters: 18, bands: 4);
      expect(atResolution(640, patches).nearestOf(RoadMarkingType.speedBump),
          isNull);
      expect(atResolution(1280, patches).nearestOf(RoadMarkingType.speedBump),
          isNotNull);
    });
  });

  group('honesty', () {
    test('plain asphalt with lane lines produces nothing', () {
      final RoadMarkingResult r = run(const <PaintPatch>[], noise: 8);
      expect(r.markings, isEmpty, reason: 'markings: ${r.markings}');
      expect(r.isDegraded, isFalse);
    });

    test('a camera seeing no road reports degraded, not empty', () {
      // Pitched hard up: almost no ground plane in frame.
      final CameraCalibration skyward =
          CameraCalibration.galaxyS23Default().scaledTo(640, 360).copyWith(
                pitchDegrees: 40,
              );
      final CameraFrame frame =
          PaintedRoad(calibration: skyward).renderFrame();
      final RoadMarkingResult r = RoadMarkingDetector().detect(frame);
      expect(r.isDegraded, isTrue);
      expect(r.degradedReason, isNotNull);
    });
  });
}
