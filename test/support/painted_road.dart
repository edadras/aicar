import 'dart:math' as math;
import 'dart:typed_data';

import 'package:aicar/camera/camera_calibration.dart';
import 'package:aicar/camera/camera_frame.dart';
import 'package:aicar/core/geometry.dart';

/// One axis-aligned rectangle of paint on the road plane, in metres.
class PaintPatch {
  const PaintPatch({
    required this.nearMeters,
    required this.farMeters,
    required this.leftMeters,
    required this.rightMeters,
    this.luma = 225,
  });

  final double nearMeters;
  final double farMeters;
  final double leftMeters;
  final double rightMeters;
  final int luma;

  bool contains(double lateral, double forward) =>
      forward >= nearMeters &&
      forward <= farMeters &&
      lateral >= leftMeters &&
      lateral <= rightMeters;
}

/// Renders road-surface markings by projecting metric paint through the same
/// camera model the detector inverts.
///
/// The point is a ground truth the test can assert against: paint a crossing
/// whose near edge is at 18.0 m and the detector must say 18.0 m, through the
/// whole projection → bird's-eye → row-profile chain.
class PaintedRoad {
  const PaintedRoad({
    required this.calibration,
    this.patches = const <PaintPatch>[],
    this.laneCenters = const <double>[-1.75, 1.75],
    this.markingWidthMeters = 0.12,
    this.asphaltLuma = 70,
    this.laneLuma = 220,
    this.skyLuma = 200,
    this.noiseAmplitude = 0,
    this.seed = 99,
  });

  final CameraCalibration calibration;
  final List<PaintPatch> patches;
  final List<double> laneCenters;
  final double markingWidthMeters;
  final int asphaltLuma;
  final int laneLuma;
  final int skyLuma;
  final int noiseAmplitude;
  final int seed;

  /// A zebra crossing: bars **along** the direction of travel, repeated
  /// across the road. That is the standard layout, and it is what makes a
  /// crossing distinguishable from a speed bump in bird's-eye space.
  static List<PaintPatch> crosswalk({
    required double nearMeters,
    double depthMeters = 3.0,
    double halfWidthMeters = 3.2,
    double stripeWidthMeters = 0.5,
    double gapMeters = 0.5,
  }) {
    final List<PaintPatch> out = <PaintPatch>[];
    double x = -halfWidthMeters;
    while (x + stripeWidthMeters <= halfWidthMeters) {
      out.add(PaintPatch(
        nearMeters: nearMeters,
        farMeters: nearMeters + depthMeters,
        leftMeters: x,
        rightMeters: x + stripeWidthMeters,
      ));
      x += stripeWidthMeters + gapMeters;
    }
    return out;
  }

  /// A speed hump: bands **across** the road, repeated along travel.
  static List<PaintPatch> speedBump({
    required double nearMeters,
    double bandDepthMeters = 0.35,
    double gapMeters = 0.45,
    int bands = 3,
    double halfWidthMeters = 2.6,
  }) {
    final List<PaintPatch> out = <PaintPatch>[];
    double y = nearMeters;
    for (int i = 0; i < bands; i++) {
      out.add(PaintPatch(
        nearMeters: y,
        farMeters: y + bandDepthMeters,
        leftMeters: -halfWidthMeters,
        rightMeters: halfWidthMeters,
      ));
      y += bandDepthMeters + gapMeters;
    }
    return out;
  }

  /// A single solid bar across the road.
  static List<PaintPatch> stopLine({
    required double nearMeters,
    double depthMeters = 0.4,
    double halfWidthMeters = 2.6,
  }) =>
      <PaintPatch>[
        PaintPatch(
          nearMeters: nearMeters,
          farMeters: nearMeters + depthMeters,
          leftMeters: -halfWidthMeters,
          rightMeters: halfWidthMeters,
        ),
      ];

  Uint8List renderGray() {
    final int w = calibration.imageWidth;
    final int h = calibration.imageHeight;
    final Uint8List out = Uint8List(w * h);
    final math.Random rng = math.Random(seed);
    final double halfMarking = markingWidthMeters / 2;

    for (int v = 0; v < h; v++) {
      for (int u = 0; u < w; u++) {
        final Vec2? ground =
            calibration.projectToGround(PixelPoint(u + 0.5, v + 0.5));
        int luma;
        if (ground == null || ground.y > 120 || ground.y < 0.5) {
          luma = skyLuma;
        } else {
          luma = asphaltLuma;
          for (final double centre in laneCenters) {
            if ((ground.x - centre).abs() <= halfMarking) {
              luma = laneLuma;
              break;
            }
          }
          for (final PaintPatch p in patches) {
            if (p.contains(ground.x, ground.y)) {
              luma = p.luma;
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

  CameraFrame renderFrame({int id = 0, int timestampMicros = 0}) =>
      CameraFrame(
        id: id,
        timestampMicros: timestampMicros,
        width: calibration.imageWidth,
        height: calibration.imageHeight,
        bytes: renderGray(),
        format: PixelFormat.gray8,
        calibration: calibration,
      );
}
