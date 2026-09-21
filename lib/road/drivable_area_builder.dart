import 'dart:math' as math;

import '../camera/camera_calibration.dart';
import '../core/geometry.dart';
import '../tracking/object_track.dart';
import 'road_segmentation.dart';

/// Turns a pixel-space segmentation into the metric corridor the planner uses.
///
/// Sampling is done the other way round from the obvious approach: rather than
/// projecting every road pixel into metric space (which piles up samples near
/// the horizon and leaves the near field sparse), it walks a regular metric
/// grid and asks the segmentation what is at each point. That gives uniform
/// metric resolution, which is what the planner actually needs.
class DrivableAreaBuilder {
  const DrivableAreaBuilder({
    this.minRangeMeters = 3.0,
    this.maxRangeMeters = 60.0,
    this.rangeStepMeters = 2.0,
    this.maxLateralMeters = 9.0,
    this.lateralStepMeters = 0.25,
    this.minCorridorWidthMeters = 2.2,
    this.maxSurfaceGapMeters = 6.0,
  });

  final double minRangeMeters;
  final double maxRangeMeters;
  final double rangeStepMeters;
  final double maxLateralMeters;
  final double lateralStepMeters;

  /// Narrower than this and the "corridor" is noise, not a road.
  final double minCorridorWidthMeters;

  /// How far the corridor may be interrupted and still be bridged.
  ///
  /// Paint is not the end of the road. A zebra crossing, a junction box, a
  /// big painted arrow — any appearance-based segmenter sees a bright band
  /// across the carriageway and stops there, and without this the planner
  /// would then report "path blocked" and brake at every crossing in town.
  /// Six metres covers the widest of them; a genuine end of the road has
  /// nothing beyond it, so a gap only bridges once road is found on the far
  /// side.
  final double maxSurfaceGapMeters;

  DrivableArea build({
    required RoadSegmentation segmentation,
    required CameraCalibration calibration,
    List<ObjectTrack> obstacles = const <ObjectTrack>[],
    double? egoLateralOffset,
  }) {
    if (!segmentation.isUsable) {
      return DrivableArea.empty(
        frameId: segmentation.frameId,
        timestampMicros: segmentation.timestampMicros,
      );
    }

    final double usefulRange = math.min(
      maxRangeMeters,
      segmentationUsefulRangeMeters(calibration, segmentation.height),
    );
    if (usefulRange < minRangeMeters + rangeStepMeters) {
      return DrivableArea.empty(
        frameId: segmentation.frameId,
        timestampMicros: segmentation.timestampMicros,
      );
    }

    final List<DrivableSample> samples = <DrivableSample>[];
    // Seed the search at the vehicle's own lateral position: the corridor is
    // the connected region containing us, not any road-coloured blob.
    double searchCentre = egoLateralOffset ?? 0.0;

    // Ranges the scan could not resolve, held back until we know whether the
    // road resumes beyond them.
    final List<double> pendingGap = <double>[];

    for (double range = minRangeMeters;
        range <= usefulRange;
        range += rangeStepMeters) {
      final _Extent? extent =
          _scanRow(segmentation, calibration, range, searchCentre);

      double left = 0;
      double right = 0;
      bool usable = extent != null;
      if (extent != null) {
        left = extent.left;
        right = extent.right;

        // Obstacles occupy road: a lorry stopped on the carriageway is road
        // surface but is not drivable, which is exactly the distinction the
        // SurfaceClass.road / drivableRoad split exists for.
        (left, right) = _applyObstacles(left, right, range, obstacles);
        usable = right - left >= minCorridorWidthMeters;
      }

      if (!usable) {
        // Hold the range open rather than ending the road here. If nothing
        // resolves within maxSurfaceGapMeters the loop stops below, and the
        // corridor ends at the last row we actually saw.
        if (samples.isEmpty) break;
        pendingGap.add(range);
        if (pendingGap.length * rangeStepMeters > maxSurfaceGapMeters) break;
        continue;
      }

      // The road resumed, so whatever interrupted it was on the surface.
      // Fill the gap by interpolating between the two sides, at a confidence
      // that says plainly this was inferred rather than observed.
      if (pendingGap.isNotEmpty) {
        final DrivableSample before = samples.last;
        for (int i = 0; i < pendingGap.length; i++) {
          final double t = (i + 1) / (pendingGap.length + 1);
          samples.add(DrivableSample(
            distanceAhead: pendingGap[i],
            leftEdge: lerpDouble(before.leftEdge, left, t),
            rightEdge: lerpDouble(before.rightEdge, right, t),
            confidence:
                math.min(before.confidence, extent!.confidence) * 0.6,
          ));
        }
        pendingGap.clear();
      }

      samples.add(DrivableSample(
        distanceAhead: range,
        leftEdge: left,
        rightEdge: right,
        confidence: extent!.confidence,
      ));
      searchCentre = (left + right) / 2;
    }

    if (samples.isEmpty) {
      return DrivableArea.empty(
        frameId: segmentation.frameId,
        timestampMicros: segmentation.timestampMicros,
      );
    }

    _smooth(samples);

    double confidenceSum = 0;
    for (final DrivableSample s in samples) {
      confidenceSum += s.confidence;
    }
    final double meanConfidence = confidenceSum / samples.length;
    // Range is itself evidence: a corridor that only reaches 8 m ahead tells
    // the planner very little, however confident each sample is.
    final double rangeFactor =
        clampDouble(samples.last.distanceAhead / 30.0, 0.25, 1.0);

    return DrivableArea(
      samples: samples,
      confidence: clampDouble(meanConfidence * rangeFactor, 0, 1),
      frameId: segmentation.frameId,
      timestampMicros: segmentation.timestampMicros,
      source: segmentation.modelName,
    );
  }

  /// Grow left and right from [searchCentre] while the surface stays drivable.
  _Extent? _scanRow(
    RoadSegmentation segmentation,
    CameraCalibration calibration,
    double range,
    double searchCentre,
  ) {
    bool drivableAt(double lateral) {
      final PixelPoint? p =
          calibration.projectGroundToImage(Vec2(lateral, range));
      if (p == null) return false;
      final double nx = p.u / calibration.imageWidth;
      final double ny = p.v / calibration.imageHeight;
      if (nx < 0 || nx > 1 || ny < 0 || ny > 1) return false;
      return segmentation.classAtNormalized(nx, ny).isDrivable;
    }

    double confidenceAt(double lateral) {
      final PixelPoint? p =
          calibration.projectGroundToImage(Vec2(lateral, range));
      if (p == null) return 0;
      return segmentation.confidenceAt(
        (p.u / calibration.imageWidth * segmentation.width)
            .round()
            .clamp(0, segmentation.width - 1),
        (p.v / calibration.imageHeight * segmentation.height)
            .round()
            .clamp(0, segmentation.height - 1),
      );
    }

    // If the seed itself is not drivable, look a little either side before
    // giving up — a lane marking or a manhole cover is not the end of the road.
    double centre = searchCentre;
    if (!drivableAt(centre)) {
      bool recovered = false;
      for (double d = lateralStepMeters; d <= 1.2; d += lateralStepMeters) {
        if (drivableAt(centre - d)) {
          centre -= d;
          recovered = true;
          break;
        }
        if (drivableAt(centre + d)) {
          centre += d;
          recovered = true;
          break;
        }
      }
      if (!recovered) return null;
    }

    double left = centre;
    int gap = 0;
    for (double x = centre; x >= -maxLateralMeters; x -= lateralStepMeters) {
      if (drivableAt(x)) {
        left = x;
        gap = 0;
      } else {
        gap++;
        // Tolerate a marking-width gap; stop at a real boundary.
        if (gap > 2) break;
      }
    }

    double right = centre;
    gap = 0;
    for (double x = centre; x <= maxLateralMeters; x += lateralStepMeters) {
      if (drivableAt(x)) {
        right = x;
        gap = 0;
      } else {
        gap++;
        if (gap > 2) break;
      }
    }

    if (right - left < minCorridorWidthMeters) return null;

    final double confidence = (confidenceAt(centre) +
            confidenceAt((centre + left) / 2) +
            confidenceAt((centre + right) / 2)) /
        3;

    return _Extent(left: left, right: right, confidence: confidence);
  }

  /// Narrow the corridor around obstacles that sit at this range.
  ///
  /// An obstacle in the middle of the corridor cannot be modelled by a single
  /// left/right pair, so the wider free side is kept. The planner is told the
  /// remaining gap, and the collision predictor separately handles the object
  /// itself — belt and braces, because a corridor is a weaker representation
  /// than an explicit obstacle.
  (double, double) _applyObstacles(
    double left,
    double right,
    double range,
    List<ObjectTrack> obstacles,
  ) {
    for (final ObjectTrack o in obstacles) {
      if (!o.objectClass.isObstacle) continue;
      if (!o.isConfirmed) continue;

      final double halfLength = o.objectClass.sizePrior.length / 2;
      final double nearEdge = o.position.y - halfLength;
      final double farEdge = o.position.y + halfLength;
      if (range < nearEdge - 1.0 || range > farEdge + 1.0) continue;

      final double halfWidth = o.objectClass.sizePrior.width / 2 + 0.3;
      final double oLeft = o.position.x - halfWidth;
      final double oRight = o.position.x + halfWidth;

      if (oRight <= left || oLeft >= right) continue; // clear of the corridor

      final double freeLeft = oLeft - left;
      final double freeRight = right - oRight;
      if (freeLeft >= freeRight) {
        right = math.max(left, oLeft);
      } else {
        left = math.min(right, oRight);
      }
    }
    return (left, right);
  }

  /// Median-filter the edges. A single bad segmentation row otherwise puts a
  /// notch in the corridor that the planner would try to steer around.
  void _smooth(List<DrivableSample> samples) {
    if (samples.length < 3) return;
    final List<double> lefts =
        samples.map((DrivableSample s) => s.leftEdge).toList();
    final List<double> rights =
        samples.map((DrivableSample s) => s.rightEdge).toList();

    for (int i = 1; i < samples.length - 1; i++) {
      final List<double> l = <double>[lefts[i - 1], lefts[i], lefts[i + 1]]
        ..sort();
      final List<double> r = <double>[rights[i - 1], rights[i], rights[i + 1]]
        ..sort();
      samples[i] = DrivableSample(
        distanceAhead: samples[i].distanceAhead,
        leftEdge: l[1],
        rightEdge: r[1],
        confidence: samples[i].confidence,
      );
    }
  }
}

class _Extent {
  const _Extent({
    required this.left,
    required this.right,
    required this.confidence,
  });

  final double left;
  final double right;
  final double confidence;
}
