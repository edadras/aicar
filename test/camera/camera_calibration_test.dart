import 'dart:math' as math;

import 'package:aicar/camera/camera_calibration.dart';
import 'package:aicar/core/geometry.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  const CameraCalibration cal = CameraCalibration(
    imageWidth: 1280,
    imageHeight: 720,
    horizontalFovDegrees: 73.7,
    cameraHeightMeters: 1.2,
    pitchDegrees: 3.0,
    rollDegrees: 0,
    yawDegrees: 0,
    lateralOffsetMeters: 0.0,
    longitudinalOffsetMeters: 2.0,
    isCalibrated: true,
  );

  group('CameraCalibration intrinsics', () {
    test('focal length matches the field of view', () {
      // fx = (w/2) / tan(hfov/2)
      final double expected = 640 / math.tan(degToRad(73.7) / 2);
      expect(cal.fx, closeTo(expected, 1e-9));
      expect(cal.fy, cal.fx);
    });

    test('vertical FOV follows from the sensor aspect ratio', () {
      expect(cal.verticalFovDegrees, greaterThan(40));
      expect(cal.verticalFovDegrees, lessThan(50));
    });

    test('horizon sits above the image centre for a downward pitch', () {
      expect(cal.horizonY, lessThan(cal.cy));
    });

    test('a level camera puts the horizon exactly at the centre row', () {
      final CameraCalibration level = cal.copyWith(pitchDegrees: 0);
      expect(level.horizonY, closeTo(level.cy, 1e-9));
    });
  });

  group('ground-plane projection', () {
    test('round-trips metric points through the image and back', () {
      for (final Vec2 point in <Vec2>[
        const Vec2(0, 10),
        const Vec2(-1.75, 25),
        const Vec2(3.5, 50),
        const Vec2(0.5, 8),
      ]) {
        final PixelPoint? pixel = cal.projectGroundToImage(point);
        expect(pixel, isNotNull, reason: '$point should be visible');
        final Vec2? back = cal.projectToGround(pixel!);
        expect(back, isNotNull);
        expect(back!.x, closeTo(point.x, 1e-6));
        expect(back.y, closeTo(point.y, 1e-6));
      }
    });

    test('returns null above the horizon instead of inventing a distance', () {
      final PixelPoint aboveHorizon = PixelPoint(cal.cx, cal.horizonY - 5);
      expect(cal.projectToGround(aboveHorizon), isNull);
      expect(cal.projectToGround(PixelPoint(cal.cx, 0)), isNull);
    });

    test('distance grows monotonically as the contact row rises', () {
      double? previous;
      for (double v = 719; v > cal.horizonY + 20; v -= 20) {
        final double? d = cal.groundDistanceForImageRow(v);
        expect(d, isNotNull);
        if (previous != null) {
          expect(d!, greaterThan(previous));
        }
        previous = d;
      }
    });

    test('lateral offset shifts the projected point', () {
      final CameraCalibration offset =
          cal.copyWith(lateralOffsetMeters: 0.4);
      // A camera 0.4 m right of the centreline sees the centreline 0.4 m to
      // its left, so a point at vehicle x=0.4 appears straight ahead.
      final PixelPoint? p = offset.projectGroundToImage(const Vec2(0.4, 20));
      expect(p!.u, closeTo(offset.cx, 1e-6));
    });

    test('roll is undone symmetrically', () {
      final CameraCalibration rolled = cal.copyWith(rollDegrees: 7);
      const Vec2 point = Vec2(1.5, 30);
      final PixelPoint? pixel = rolled.projectGroundToImage(point);
      expect(pixel, isNotNull);
      final Vec2? back = rolled.projectToGround(pixel!);
      expect(back!.x, closeTo(point.x, 1e-5));
      expect(back.y, closeTo(point.y, 1e-5));
    });

    test('yaw is undone symmetrically', () {
      final CameraCalibration yawed = cal.copyWith(yawDegrees: -4);
      const Vec2 point = Vec2(-2.0, 35);
      final PixelPoint? pixel = yawed.projectGroundToImage(point);
      final Vec2? back = yawed.projectToGround(pixel!);
      expect(back!.x, closeTo(point.x, 1e-5));
      expect(back.y, closeTo(point.y, 1e-5));
    });
  });

  group('size-based distance', () {
    test('apparent height inverts the pinhole relation', () {
      const double realHeight = 1.5; // a car
      const double distance = 20;
      final double pixels = cal.fy * realHeight / distance;
      final double? recovered = cal.distanceFromApparentHeight(
        boxHeightPixels: pixels,
        realHeightMeters: realHeight,
      );
      expect(recovered, closeTo(distance, 1e-6));
    });

    test('degenerate boxes return null rather than a huge distance', () {
      expect(
        cal.distanceFromApparentHeight(
            boxHeightPixels: 0, realHeightMeters: 1.5),
        isNull,
      );
    });
  });

  group('confidence model', () {
    test('collapses at the horizon and is highest at the bottom row', () {
      expect(cal.groundDepthConfidenceAtRow(cal.horizonY + 1), 0);
      expect(cal.groundDepthConfidenceAtRow(719), greaterThan(0.9));
      expect(
        cal.groundDepthConfidenceAtRow(500),
        lessThan(cal.groundDepthConfidenceAtRow(700)),
      );
    });

    test('uncalibrated defaults are explicitly discounted', () {
      final CameraCalibration guessed = cal.copyWith(isCalibrated: false);
      expect(
        guessed.groundDepthConfidenceAtRow(700),
        lessThan(cal.groundDepthConfidenceAtRow(700)),
      );
    });
  });

  group('rescaling', () {
    test('scaling to another resolution keeps the same world geometry', () {
      final CameraCalibration small = cal.scaledTo(640, 360);
      const Vec2 point = Vec2(1.0, 22);
      final PixelPoint? big = cal.projectGroundToImage(point);
      final PixelPoint? sml = small.projectGroundToImage(point);
      expect(sml!.u / small.imageWidth, closeTo(big!.u / cal.imageWidth, 1e-6));
      expect(sml.v / small.imageHeight, closeTo(big.v / cal.imageHeight, 1e-6));
    });

    test('json round-trips', () {
      final CameraCalibration back =
          CameraCalibration.fromJson(cal.toJson());
      expect(back.fx, closeTo(cal.fx, 1e-9));
      expect(back.cameraHeightMeters, cal.cameraHeightMeters);
      expect(back.isCalibrated, isTrue);
    });
  });
}
