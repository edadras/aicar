import 'dart:math' as math;
import 'dart:typed_data';

import 'package:aicar/camera/camera_calibration.dart';
import 'package:aicar/camera/camera_frame.dart';
import 'package:aicar/core/geometry.dart';

/// Renders a synthetic road scene by projecting a metric road model through
/// the same camera model the perception code inverts.
///
/// This gives tests a ground truth they can assert against: if the detector
/// recovers a lane at -1.75 m from an image drawn with a lane at -1.75 m, the
/// whole IPM → filter → window → fit chain is consistent.
class SyntheticRoad {
  const SyntheticRoad({
    required this.calibration,
    this.laneCenters = const <double>[-1.75, 1.75],
    this.markingWidthMeters = 0.12,
    this.asphaltLuma = 70,
    this.markingLuma = 225,
    this.skyLuma = 200,
    this.curvature = 0.0,
    this.heading = 0.0,
    this.lateralOffset = 0.0,
    this.dashedLanes = const <int>{},
    this.dashPeriodMeters = 12.0,
    this.dashOnMeters = 3.0,
    this.noiseAmplitude = 0,
    this.seed = 1234,
  });

  final CameraCalibration calibration;

  /// Lateral positions of the lane markings at the vehicle, metres.
  final List<double> laneCenters;
  final double markingWidthMeters;
  final int asphaltLuma;
  final int markingLuma;
  final int skyLuma;

  /// Quadratic term of the road curve, `x = offset + heading*y + curvature*y²`.
  final double curvature;
  final double heading;
  final double lateralOffset;

  /// Indices into [laneCenters] that should be drawn dashed.
  final Set<int> dashedLanes;
  final double dashPeriodMeters;
  final double dashOnMeters;

  final int noiseAmplitude;
  final int seed;

  double laneLateralAt(int laneIndex, double forward) =>
      laneCenters[laneIndex] +
      lateralOffset +
      heading * forward +
      curvature * forward * forward;

  bool _isPainted(int laneIndex, double forward) {
    if (!dashedLanes.contains(laneIndex)) return true;
    final double phase = forward % dashPeriodMeters;
    return phase < dashOnMeters;
  }

  /// Render an 8-bit grayscale image.
  Uint8List renderGray() {
    final int w = calibration.imageWidth;
    final int h = calibration.imageHeight;
    final Uint8List out = Uint8List(w * h);
    final math.Random rng = math.Random(seed);
    final double halfWidth = markingWidthMeters / 2;

    for (int v = 0; v < h; v++) {
      for (int u = 0; u < w; u++) {
        final Vec2? ground =
            calibration.projectToGround(PixelPoint(u + 0.5, v + 0.5));
        int luma;
        if (ground == null || ground.y > 120 || ground.y < 0.5) {
          luma = skyLuma;
        } else {
          luma = asphaltLuma;
          for (int i = 0; i < laneCenters.length; i++) {
            final double centre = laneLateralAt(i, ground.y);
            if ((ground.x - centre).abs() <= halfWidth &&
                _isPainted(i, ground.y)) {
              luma = markingLuma;
              break;
            }
          }
        }
        if (noiseAmplitude > 0) {
          luma += rng.nextInt(noiseAmplitude * 2 + 1) - noiseAmplitude;
        }
        out[v * w + u] = luma.clamp(0, 255);
      }
    }
    return out;
  }

  CameraFrame renderFrame({int id = 0, int timestampMicros = 0}) => CameraFrame(
        id: id,
        timestampMicros: timestampMicros,
        width: calibration.imageWidth,
        height: calibration.imageHeight,
        bytes: renderGray(),
        format: PixelFormat.gray8,
        calibration: calibration,
      );
}
