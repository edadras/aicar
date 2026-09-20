import 'dart:math' as math;
import 'dart:typed_data';

import '../camera/camera_calibration.dart';
import '../camera/camera_frame.dart';
import '../camera/image_preprocessing.dart';
import '../core/geometry.dart';
import 'birds_eye_view.dart';
import 'road_segmentation.dart';

/// Finds the physical edges of the carriageway — kerbs, verges, barriers —
/// as opposed to the painted lines a lane detector looks for.
///
/// On unmarked roads (and there are a lot of them) this is the primary
/// structure the corridor is built from, which is why it is a separate stage
/// rather than a fallback inside the lane detector.
///
/// Two independent cues are combined:
///  * the boundary of the drivable region from segmentation, which is
///    semantically right but spatially coarse;
///  * a strong, sustained vertical gradient in bird's-eye space, which is
///    spatially sharp but semantically blind.
/// Agreement between them is what produces a confident edge.
class RoadEdgeDetector {
  const RoadEdgeDetector({
    this.minGradient = 26,
    this.minSupportPoints = 6,
    this.maxLateralMeters = 9.0,
    this.ransacInlierMeters = 0.35,
  });

  final int minGradient;
  final int minSupportPoints;
  final double maxLateralMeters;
  final double ransacInlierMeters;

  List<RoadEdge> detect({
    required CameraFrame frame,
    required BirdsEyeView bev,
    RoadSegmentation? segmentation,
    DrivableArea? drivableArea,
  }) {
    final List<RoadEdge> edges = <RoadEdge>[];

    final Uint8List gray = frame.format == PixelFormat.gray8
        ? frame.bytes
        : ImagePreprocessing.rgbToGray(frame.bytes, frame.width, frame.height);
    final Uint8List warped = bev.warpGray(gray, frame.width, frame.height);
    final Uint8List blurred =
        ImagePreprocessing.boxBlur3(warped, bev.width, bev.height);
    final Int16List gradient =
        ImagePreprocessing.sobelX(blurred, bev.width, bev.height);

    for (final bool isLeft in <bool>[true, false]) {
      final RoadEdge? edge = _detectSide(
        isLeft: isLeft,
        gradient: gradient,
        bev: bev,
        calibration: frame.calibration,
        drivableArea: drivableArea,
        segmentation: segmentation,
      );
      if (edge != null) edges.add(edge);
    }
    return edges;
  }

  RoadEdge? _detectSide({
    required bool isLeft,
    required Int16List gradient,
    required BirdsEyeView bev,
    required CameraCalibration calibration,
    DrivableArea? drivableArea,
    RoadSegmentation? segmentation,
  }) {
    final List<double> forwards = <double>[];
    final List<double> laterals = <double>[];
    final List<double> weights = <double>[];
    int segmentationAgreements = 0;

    for (int row = 0; row < bev.height; row += 2) {
      final double forward = bev.forwardAtRow(row.toDouble());
      if (forward < bev.minForward || forward > bev.maxForward) continue;

      // Seed the search from the drivable corridor when we have one; the
      // gradient search then only has to refine a position, not find it.
      final (double, double)? limits = drivableArea?.limitsAt(forward);
      final double searchFrom = limits == null
          ? 0.0
          : (isLeft ? limits.$1 : limits.$2);
      final double searchWindow = limits == null ? maxLateralMeters : 1.5;

      final double? hit = _strongestGradientNear(
        gradient: gradient,
        bev: bev,
        row: row,
        centreLateral: searchFrom,
        windowMeters: searchWindow,
        // A kerb on the left is a dark-to-bright or bright-to-dark step; the
        // sign tells us which side of the road we are looking at.
        preferNegative: isLeft,
      );
      if (hit == null) continue;
      if (hit.abs() > maxLateralMeters) continue;

      forwards.add(forward);
      laterals.add(hit);
      // Near-field rows are worth more: metres per pixel grows quickly.
      weights.add(1.0 + 2.0 * (row / bev.height));

      if (limits != null && (hit - searchFrom).abs() < 0.8) {
        segmentationAgreements++;
      }
    }

    if (forwards.length < minSupportPoints) return null;

    final Polynomial? fit = fitPolynomialRansac(
      forwards,
      laterals,
      degree: 2,
      iterations: 40,
      inlierThreshold: ransacInlierMeters,
      seed: isLeft ? 1 : 2,
    );
    if (fit == null) return null;

    double residual = 0;
    for (int i = 0; i < forwards.length; i++) {
      residual += (fit.evaluate(forwards[i]) - laterals[i]).abs();
    }
    residual /= forwards.length;

    final double minRange = forwards.reduce(math.min);
    final double maxRange = forwards.reduce(math.max);

    // Confidence: fit quality, extent, and agreement with segmentation.
    final double residualScore =
        clampDouble(1 - residual / (ransacInlierMeters * 2), 0, 1);
    final double supportScore =
        clampDouble(forwards.length / (minSupportPoints * 3), 0, 1);
    final double agreementScore = drivableArea == null
        ? 0.4
        : clampDouble(segmentationAgreements / forwards.length, 0, 1);

    final double confidence = clampDouble(
      0.4 * residualScore + 0.25 * supportScore + 0.35 * agreementScore,
      0,
      1,
    );
    if (confidence < 0.2) return null;

    return RoadEdge(
      isLeft: isLeft,
      curve: fit,
      confidence: confidence,
      kind: _classifyKind(segmentation, calibration, fit, minRange, maxRange),
      minRangeMeters: minRange,
      maxRangeMeters: maxRange,
    );
  }

  /// Strongest signed gradient within a lateral window of one bird's-eye row.
  double? _strongestGradientNear({
    required Int16List gradient,
    required BirdsEyeView bev,
    required int row,
    required double centreLateral,
    required double windowMeters,
    required bool preferNegative,
  }) {
    final int centreCol = bev.columnAtLateral(centreLateral).round();
    final int halfWindow =
        math.max(2, (windowMeters / bev.metresPerPixelLateral).round());
    final int from = math.max(1, centreCol - halfWindow);
    final int to = math.min(bev.width - 1, centreCol + halfWindow);
    if (from >= to) return null;

    int bestCol = -1;
    int bestMagnitude = 0;
    for (int col = from; col < to; col++) {
      if (!bev.isValid(col, row)) continue;
      final int g = gradient[row * bev.width + col];
      // Sign encodes which way the brightness steps. Road-to-verge on the left
      // and road-to-verge on the right have opposite signs, so respecting the
      // expected sign halves the false-positive rate.
      final int signed = preferNegative ? -g : g;
      if (signed > bestMagnitude) {
        bestMagnitude = signed;
        bestCol = col;
      }
    }
    if (bestCol < 0 || bestMagnitude < minGradient) return null;
    return bev.lateralAtColumn(bestCol.toDouble());
  }

  /// Name the edge from what the segmenter sees just outside it.
  RoadEdgeKind _classifyKind(
    RoadSegmentation? segmentation,
    CameraCalibration calibration,
    Polynomial fit,
    double minRange,
    double maxRange,
  ) {
    if (segmentation == null || !segmentation.isUsable) {
      return RoadEdgeKind.unknown;
    }
    final Map<SurfaceClass, int> votes = <SurfaceClass, int>{};
    for (double d = minRange; d <= maxRange; d += 2) {
      final double lateral = fit.evaluate(d);
      // Sample a little beyond the edge, where the non-road surface is.
      for (final double offset in <double>[0.5, 1.0]) {
        final double probe = lateral + (lateral < 0 ? -offset : offset);
        final PixelPoint? p =
            calibration.projectGroundToImage(Vec2(probe, d));
        if (p == null) continue;
        final SurfaceClass cls = segmentation.classAtNormalized(
          p.u / calibration.imageWidth,
          p.v / calibration.imageHeight,
        );
        votes.update(cls, (int v) => v + 1, ifAbsent: () => 1);
      }
    }

    SurfaceClass best = SurfaceClass.unknown;
    int bestVotes = 0;
    for (final MapEntry<SurfaceClass, int> e in votes.entries) {
      if (e.value > bestVotes) {
        bestVotes = e.value;
        best = e.key;
      }
    }

    return switch (best) {
      SurfaceClass.curb || SurfaceClass.sidewalk => RoadEdgeKind.curb,
      SurfaceClass.grass => RoadEdgeKind.verge,
      SurfaceClass.building => RoadEdgeKind.barrier,
      SurfaceClass.vehicle => RoadEdgeKind.parkedVehicles,
      _ => RoadEdgeKind.unknown,
    };
  }
}
