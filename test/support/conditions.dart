import 'dart:math' as math;
import 'dart:typed_data';

import 'package:aicar/camera/camera_calibration.dart';
import 'package:aicar/camera/camera_frame.dart';
import 'package:aicar/core/geometry.dart';

/// Post-processing that turns a clean synthetic scene into a hard one.
///
/// Applied to the rendered image rather than built into the renderers so the
/// same conditions can be layered onto any scene — a lane test and a crossing
/// test face the same rain.
///
/// None of this claims to be photorealistic, and it is not a substitute for
/// driving at night in the rain. What it is good for is the thing a real
/// drive is bad at: producing the *same* difficult scene twice, so a change
/// to the code can be attributed to the code. See `docs/FIELD_TESTING.md`
/// for what still has to be driven.
class Conditions {
  const Conditions._();

  /// Night: the cone a headlight lights, and darkness that destroys contrast
  /// rather than merely dimming it.
  ///
  /// The distinction matters and is easy to get wrong. Scaling every pixel by
  /// a constant makes a dark image whose *contrast ratio* is untouched, so a
  /// marking-vs-tarmac step survives perfectly and any detector still finds
  /// the lane out to the horizon — which is not what night is like. Beyond
  /// the headlights there is no illumination, so there is no contrast: paint
  /// and tarmac return the same near-black, and what is left is sensor noise.
  ///
  /// So this models illumination properly. Scene contrast is multiplied by an
  /// illumination factor that falls from 1 inside the beam to almost nothing
  /// past it, and noise is added at a level that does not fall with it. That
  /// is why lane range collapses at night in the tests, as it does on a road.
  ///
  /// The cone is projected through the calibration rather than drawn as a
  /// triangle in image space, so it covers the right patch of *ground*.
  static Uint8List night(
    Uint8List gray,
    CameraCalibration cal, {
    double ambientScale = 0.28,
    double headlightReachMeters = 28,
    double headlightHalfWidthMeters = 4.5,
    double beyondBeamIllumination = 0.10,
    int noiseAmplitude = 9,
    int seed = 11,
  }) {
    final Uint8List out = Uint8List(gray.length);
    final math.Random rng = math.Random(seed);
    final double mean = meanLuma(gray);

    for (int v = 0; v < cal.imageHeight; v++) {
      for (int u = 0; u < cal.imageWidth; u++) {
        final int i = v * cal.imageWidth + u;

        double illumination = beyondBeamIllumination;
        final Vec2? ground =
            cal.projectToGround(PixelPoint(u + 0.5, v + 0.5));
        if (ground != null &&
            ground.y > 0.5 &&
            ground.y < headlightReachMeters &&
            ground.x.abs() < headlightHalfWidthMeters) {
          final double falloff =
              1 - math.pow(ground.y / headlightReachMeters, 1.6).toDouble();
          final double lateral =
              1 - (ground.x.abs() / headlightHalfWidthMeters);
          illumination = math.max(
            beyondBeamIllumination,
            falloff * lateral,
          );
        }

        // Illumination scales the *contrast*, not just the level.
        final double lit =
            mean * ambientScale + (gray[i] - mean) * illumination;
        // Noise does not get darker with the scene, which is exactly why
        // night is hard.
        final double noisy =
            lit + rng.nextInt(noiseAmplitude * 2 + 1) - noiseAmplitude;
        out[i] = noisy.round().clamp(0, 255);
      }
    }
    return out;
  }

  /// Rain: lower contrast, streaks, and specular reflections on wet tarmac.
  ///
  /// The reflections matter more than the streaks. A wet road mirrors the
  /// sky and the street lights, and those bright patches are exactly what a
  /// marking detector is looking for.
  static Uint8List rain(
    Uint8List gray,
    CameraCalibration cal, {
    double contrastLoss = 0.45,
    int streakCount = 240,
    int reflectionCount = 60,
    int seed = 23,
  }) {
    final Uint8List out = Uint8List.fromList(gray);
    final math.Random rng = math.Random(seed);
    final int w = cal.imageWidth;
    final int h = cal.imageHeight;

    // Haze: pull everything towards the mean.
    double sum = 0;
    for (final int v in gray) {
      sum += v;
    }
    final double mean = sum / gray.length;
    for (int i = 0; i < out.length; i++) {
      out[i] = (mean + (out[i] - mean) * (1 - contrastLoss))
          .round()
          .clamp(0, 255);
    }

    // Streaks on the lens and in the air.
    for (int s = 0; s < streakCount; s++) {
      final int x = rng.nextInt(w);
      final int y = rng.nextInt(h);
      final int len = 4 + rng.nextInt(14);
      final int lean = rng.nextInt(3) - 1;
      for (int k = 0; k < len; k++) {
        final int yy = y + k;
        final int xx = x + (k * lean) ~/ 4;
        if (yy >= h || xx < 0 || xx >= w) break;
        final int i = yy * w + xx;
        out[i] = math.min(255, out[i] + 55);
      }
    }

    // Specular patches on the road surface, below the horizon.
    final double horizon = cal.horizonYNormalized * h;
    for (int s = 0; s < reflectionCount; s++) {
      final int y = (horizon + rng.nextDouble() * (h - horizon)).round();
      final int x = rng.nextInt(w);
      final int rx = 2 + rng.nextInt(9);
      final int ry = 1 + rng.nextInt(3);
      for (int dy = -ry; dy <= ry; dy++) {
        for (int dx = -rx; dx <= rx; dx++) {
          final int xx = x + dx;
          final int yy = y + dy;
          if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
          final int i = yy * w + xx;
          out[i] = math.min(255, out[i] + 90);
        }
      }
    }
    return out;
  }

  /// Low sun straight ahead: a saturated blob and a washed-out surround.
  static Uint8List glare(
    Uint8List gray,
    CameraCalibration cal, {
    double sunXNormalized = 0.52,
    double? sunYNormalized,
    double radiusNormalized = 0.16,
    double bloom = 0.55,
  }) {
    final Uint8List out = Uint8List.fromList(gray);
    final int w = cal.imageWidth;
    final int h = cal.imageHeight;
    // Default: just above the horizon, which is where it blinds a driver and
    // a camera alike.
    final double cy =
        (sunYNormalized ?? (cal.horizonYNormalized - 0.03)) * h;
    final double cx = sunXNormalized * w;
    final double r = radiusNormalized * math.min(w, h);

    for (int v = 0; v < h; v++) {
      for (int u = 0; u < w; u++) {
        final double dx = u - cx;
        final double dy = v - cy;
        final double d = math.sqrt(dx * dx + dy * dy);
        if (d > r * 3) continue;
        final int i = v * w + u;
        if (d <= r) {
          out[i] = 255;
        } else {
          final double t = 1 - (d - r) / (r * 2);
          out[i] = (out[i] + (255 - out[i]) * bloom * t)
              .round()
              .clamp(0, 255);
        }
      }
    }
    return out;
  }

  /// A vehicle alongside or ahead, covering part of the road.
  ///
  /// Given in metres so the occlusion lands on the right piece of road at any
  /// resolution: a lorry in the next lane hides the lane marking beside it,
  /// and that is the failure worth testing.
  static Uint8List occlude(
    Uint8List gray,
    CameraCalibration cal,
    List<GroundRect> boxes, {
    int luma = 45,
  }) {
    final Uint8List out = Uint8List.fromList(gray);
    for (int v = 0; v < cal.imageHeight; v++) {
      for (int u = 0; u < cal.imageWidth; u++) {
        final Vec2? ground =
            cal.projectToGround(PixelPoint(u + 0.5, v + 0.5));
        if (ground == null) continue;
        for (final GroundRect b in boxes) {
          if (ground.y >= b.nearMeters &&
              ground.y <= b.farMeters &&
              ground.x >= b.leftMeters &&
              ground.x <= b.rightMeters) {
            out[v * cal.imageWidth + u] = luma;
            break;
          }
        }
      }
    }
    return out;
  }

  static CameraFrame frameOf(
    Uint8List gray,
    CameraCalibration cal, {
    int id = 0,
    int timestampMicros = 0,
  }) =>
      CameraFrame(
        id: id,
        timestampMicros: timestampMicros,
        width: cal.imageWidth,
        height: cal.imageHeight,
        bytes: gray,
        format: PixelFormat.gray8,
        calibration: cal,
      );

  /// Mean luminance, which is how the stack itself decides it is night.
  static double meanLuma(Uint8List gray) {
    if (gray.isEmpty) return 0;
    int sum = 0;
    for (final int v in gray) {
      sum += v;
    }
    return sum / gray.length;
  }
}

/// A rectangle on the road plane, in metres.
class GroundRect {
  const GroundRect({
    required this.nearMeters,
    required this.farMeters,
    required this.leftMeters,
    required this.rightMeters,
  });

  final double nearMeters;
  final double farMeters;
  final double leftMeters;
  final double rightMeters;
}
