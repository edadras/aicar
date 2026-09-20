import 'dart:math' as math;
import 'dart:typed_data';

import '../ai/interfaces/lane_detector.dart';
import '../ai/model_descriptor.dart';
import '../camera/camera_calibration.dart';
import '../camera/camera_frame.dart';
import '../camera/image_preprocessing.dart';
import '../core/confidence.dart';
import '../core/geometry.dart';
import '../core/logging.dart';
import 'birds_eye_view.dart';
import 'lane.dart';
import 'road_segmentation.dart';

/// Tuning for [ClassicalLaneDetector].
class ClassicalLaneConfig {
  const ClassicalLaneConfig({
    this.markingWidthMeters = 0.12,
    this.minMarkingResponse = 22,
    this.slidingWindowCount = 12,
    this.windowHalfWidthMeters = 0.55,
    this.minPointsPerLane = 18,
    this.minLaneWidthMeters = 2.4,
    this.maxLaneWidthMeters = 5.2,
    this.defaultLaneWidthMeters = 3.5,
    this.maxCurvature = 0.02,
    this.ransacIterations = 48,
    this.ransacInlierMeters = 0.18,
    this.temporalSmoothing = 0.45,
  });

  /// Typical painted-line width. The matched filter is tuned to this, which is
  /// what makes it reject kerb shadows and tar-seam repairs.
  final double markingWidthMeters;

  /// Minimum top-hat response, on a 0..255 scale, before a cell counts as
  /// marking. Combined with an Otsu-derived floor at runtime.
  final int minMarkingResponse;

  final int slidingWindowCount;
  final double windowHalfWidthMeters;
  final int minPointsPerLane;

  final double minLaneWidthMeters;
  final double maxLaneWidthMeters;
  final double defaultLaneWidthMeters;

  /// Curvature beyond this (radius < 50 m) is not a motorway lane; it is a
  /// bad fit. Rejecting it prevents wild steering commands.
  final double maxCurvature;

  final int ransacIterations;
  final double ransacInlierMeters;

  /// Blend factor with the previous frame's fit. Lanes are physically
  /// continuous; a fit that jumps between frames is noise, not a new road.
  final double temporalSmoothing;
}

/// Lane detector built from classical computer vision, requiring no weights.
///
/// This is the baseline that keeps the whole stack useful before (and
/// independently of) a neural lane model, and it is also the reference the
/// learned detector is compared against in replay.
///
/// Pipeline, all in metric bird's-eye space:
///  1. Inverse-perspective warp of the luminance channel.
///  2. A top-hat matched filter sized to a real lane marking — bright stripe
///     with darker asphalt on both sides at a fixed metric offset.
///  3. Otsu thresholding, so it adapts to night, tunnels and low sun instead
///     of relying on a hand-tuned constant.
///  4. Histogram peak finding in the near field to seed left/right.
///  5. Sliding-window search upward, collecting marking centroids.
///  6. RANSAC quadratic fit in metres, then geometric validation.
///  7. Line-type (solid/dashed) and colour classification.
class ClassicalLaneDetector extends LaneDetector {
  ClassicalLaneDetector({
    required CameraCalibration calibration,
    this.config = const ClassicalLaneConfig(),
  }) : _calibration = calibration;

  static const String _tag = 'ClassicalLaneDetector';

  CameraCalibration _calibration;
  final ClassicalLaneConfig config;

  BirdsEyeView? _bev;
  double _trackedLaneWidth = 3.5;
  double _laneWidthConfidence = 0;

  set calibration(CameraCalibration value) {
    _calibration = value;
    _bev = null; // force the sampling table to be rebuilt
  }

  CameraCalibration get calibration => _calibration;

  @override
  String get modelId => 'classical-bev-lane';

  @override
  String get displayName => 'Classical lane detector (IPM + matched filter)';

  @override
  ModelRole get role => ModelRole.laneDetection;

  @override
  ModelDescriptor? get descriptor => null;

  @override
  bool get isReady => true;

  @override
  String? get unavailableReason => null;

  @override
  Future<void> load() async {}

  @override
  Future<void> close() async {}

  BirdsEyeView _ensureBev(CameraFrame frame) {
    final BirdsEyeView? existing = _bev;
    if (existing != null && existing.matches(frame.calibration)) return existing;
    final BirdsEyeView built =
        BirdsEyeView.build(calibration: frame.calibration);
    _bev = built;
    Log.info(
      _tag,
      'built IPM table ${built.width}x${built.height} '
      '(coverage ${(built.coverage * 100).round()}%)',
    );
    return built;
  }

  @override
  Future<LaneDetectionResult> detectLanes(
    CameraFrame frame, {
    RoadSegmentation? segmentation,
    LaneDetectionResult? previous,
  }) async {
    final BirdsEyeView bev = _ensureBev(frame);
    if (bev.coverage < 0.15) {
      return LaneDetectionResult.empty(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: 'camera geometry shows almost no road surface',
      );
    }

    final Uint8List gray = frame.format == PixelFormat.gray8
        ? frame.bytes
        : ImagePreprocessing.rgbToGray(frame.bytes, frame.width, frame.height);

    final Uint8List warped = bev.warpGray(gray, frame.width, frame.height);
    final Uint8List response = _markingResponse(warped, bev);
    final Uint8List mask =
        _thresholdResponse(response, warped, bev, segmentation);

    final (int? leftSeed, int? rightSeed) = _findSeeds(mask, bev);
    if (leftSeed == null && rightSeed == null) {
      return LaneDetectionResult.empty(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: 'no lane marking peaks in the near field',
      );
    }

    final _LaneTrace? leftTrace =
        leftSeed == null ? null : _slidingWindow(mask, bev, leftSeed);
    final _LaneTrace? rightTrace =
        rightSeed == null ? null : _slidingWindow(mask, bev, rightSeed);

    LaneBoundary? left = _fitBoundary(
      leftTrace,
      LanePosition.egoLeft,
      bev,
      frame,
      mask,
      previous?.left,
    );
    LaneBoundary? right = _fitBoundary(
      rightTrace,
      LanePosition.egoRight,
      bev,
      frame,
      mask,
      previous?.right,
    );

    (left, right) = _validatePair(left, right);

    final List<LaneBoundary> boundaries = <LaneBoundary>[
      if (left != null) left,
      if (right != null) right,
    ];

    // Adjacent lanes: search outward from each ego boundary by one lane width.
    boundaries.addAll(_findAdjacent(mask, bev, frame, left, right));

    final LaneMode mode = switch ((left, right)) {
      (final LaneBoundary _, final LaneBoundary _) => LaneMode.bothBoundaries,
      (null, null) => LaneMode.noLane,
      _ => LaneMode.singleBoundary,
    };

    _updateLaneWidth(left, right);

    final (double offset, double headingError) =
        _egoPose(left, right, mode);

    return LaneDetectionResult(
      boundaries: boundaries,
      mode: mode,
      frameId: frame.id,
      timestampMicros: frame.timestampMicros,
      laneWidthMeters: _trackedLaneWidth,
      laneWidthConfidence: _laneWidthConfidence,
      egoLateralOffsetMeters: offset,
      egoHeadingErrorRadians: headingError,
      modelName: modelId,
    );
  }

  // --- Step 2: matched filter --------------------------------------------

  /// Top-hat response: a lane marking is brighter than the asphalt a fixed
  /// *metric* distance to either side. In bird's-eye space that offset is a
  /// constant number of cells, which is precisely why the warp is done first.
  Uint8List _markingResponse(Uint8List warped, BirdsEyeView bev) {
    final int w = bev.width;
    final int h = bev.height;
    final int offset = math.max(
      2,
      (config.markingWidthMeters * 1.6 / bev.metresPerPixelLateral).round(),
    );

    final Uint8List out = Uint8List(w * h);
    for (int row = 0; row < h; row++) {
      final int base = row * w;
      for (int col = offset; col < w - offset; col++) {
        final int centre = warped[base + col];
        if (centre == 0) continue; // outside the valid warp region
        final int leftBg = warped[base + col - offset];
        final int rightBg = warped[base + col + offset];
        if (leftBg == 0 || rightBg == 0) continue;

        // Require the marking to be brighter than *both* sides: a single-sided
        // step is a shadow boundary or a kerb, not a painted line.
        final int diff = math.min(centre - leftBg, centre - rightBg);
        out[base + col] = diff <= 0 ? 0 : (diff > 255 ? 255 : diff);
      }
    }
    return out;
  }

  /// Threshold with an Otsu floor so the detector adapts to the light level.
  Uint8List _thresholdResponse(
    Uint8List response,
    Uint8List warped,
    BirdsEyeView bev,
    RoadSegmentation? segmentation,
  ) {
    final int otsu = ImagePreprocessing.otsuThreshold(response, stride: 3);
    final int threshold = math.max(config.minMarkingResponse, otsu);

    final Uint8List mask = Uint8List(response.length);
    for (int i = 0; i < response.length; i++) {
      mask[i] = response[i] >= threshold ? 1 : 0;
    }

    // When a segmenter is available, discard markings that are not on drivable
    // surface. This removes the single biggest false-positive source: bright
    // pavement edges, white kerbs and parked-car trim.
    if (segmentation != null && segmentation.isUsable) {
      for (int row = 0; row < bev.height; row++) {
        final double forward = bev.forwardAtRow(row.toDouble());
        for (int col = 0; col < bev.width; col++) {
          final int i = row * bev.width + col;
          if (mask[i] == 0) continue;
          final double lateral = bev.lateralAtColumn(col.toDouble());
          final PixelPoint? p = _calibration
              .projectGroundToImage(Vec2(lateral, forward));
          if (p == null) continue;
          final SurfaceClass cls = segmentation.classAtNormalized(
            p.u / _calibration.imageWidth,
            p.v / _calibration.imageHeight,
          );
          if (!cls.isDrivable && cls != SurfaceClass.unknown) mask[i] = 0;
        }
      }
    }
    return mask;
  }

  // --- Step 4: seeds ------------------------------------------------------

  /// Column histogram over the near field, split at the vehicle centreline.
  (int?, int?) _findSeeds(Uint8List mask, BirdsEyeView bev) {
    final int startRow = (bev.height * 0.55).round();
    final Int32List histogram = Int32List(bev.width);
    for (int row = startRow; row < bev.height; row++) {
      final int base = row * bev.width;
      for (int col = 0; col < bev.width; col++) {
        histogram[col] += mask[base + col];
      }
    }

    final int centreColumn = bev.columnAtLateral(0).round();
    // Search no more than ~3.5 m either side: beyond that we would lock onto
    // the neighbouring lane's far boundary and report a 7 m lane.
    final int searchHalfWidth =
        (3.6 / bev.metresPerPixelLateral).round();

    final int? left = _peakIn(
      histogram,
      math.max(0, centreColumn - searchHalfWidth),
      math.max(0, centreColumn - 4),
    );
    final int? right = _peakIn(
      histogram,
      math.min(bev.width - 1, centreColumn + 4),
      math.min(bev.width, centreColumn + searchHalfWidth),
    );
    return (left, right);
  }

  int? _peakIn(Int32List histogram, int from, int to) {
    int bestCol = -1;
    int bestValue = 0;
    for (int c = from; c < to; c++) {
      if (histogram[c] > bestValue) {
        bestValue = histogram[c];
        bestCol = c;
      }
    }
    // A handful of stray pixels is not a lane marking.
    return bestValue >= 6 ? bestCol : null;
  }

  // --- Step 5: sliding window --------------------------------------------

  _LaneTrace? _slidingWindow(Uint8List mask, BirdsEyeView bev, int seedColumn) {
    final int windows = config.slidingWindowCount;
    final int windowHeight = math.max(1, bev.height ~/ windows);
    final int halfWidth = math.max(
      2,
      (config.windowHalfWidthMeters / bev.metresPerPixelLateral).round(),
    );

    final List<double> lateral = <double>[];
    final List<double> forward = <double>[];
    final List<double> weights = <double>[];
    final List<int> rowsWithSupport = <int>[];

    int current = seedColumn;
    int consecutiveEmpty = 0;

    // Start at the bottom (near field) and climb.
    for (int w = 0; w < windows; w++) {
      final int rowEnd = bev.height - w * windowHeight;
      final int rowStart = math.max(0, rowEnd - windowHeight);
      if (rowStart >= rowEnd) break;

      int sum = 0;
      int count = 0;
      for (int row = rowStart; row < rowEnd; row++) {
        final int base = row * bev.width;
        final int from = math.max(0, current - halfWidth);
        final int to = math.min(bev.width, current + halfWidth + 1);
        for (int col = from; col < to; col++) {
          if (mask[base + col] == 1) {
            sum += col;
            count++;
          }
        }
      }

      if (count >= 4) {
        final double centroid = sum / count;
        current = centroid.round();
        final double rowCentre = (rowStart + rowEnd) / 2;
        lateral.add(bev.lateralAtColumn(centroid));
        forward.add(bev.forwardAtRow(rowCentre));
        // Near-field samples are geometrically far more reliable, so they
        // carry more weight in the fit.
        weights.add(1.0 + 2.0 * (rowCentre / bev.height));
        rowsWithSupport.add(w);
        consecutiveEmpty = 0;
      } else {
        consecutiveEmpty++;
        // Dashed markings leave gaps; three empty windows in a row means the
        // line has genuinely ended, not that we are between dashes.
        if (consecutiveEmpty >= 3) break;
      }
    }

    if (lateral.length < 3) return null;
    return _LaneTrace(
      lateral: lateral,
      forward: forward,
      weights: weights,
      windowsSearched: math.min(windows, rowsWithSupport.isEmpty
          ? windows
          : rowsWithSupport.last + 1),
      windowsWithSupport: rowsWithSupport.length,
    );
  }

  // --- Step 6: fit and validate ------------------------------------------

  LaneBoundary? _fitBoundary(
    _LaneTrace? trace,
    LanePosition position,
    BirdsEyeView bev,
    CameraFrame frame,
    Uint8List mask,
    LaneBoundary? previous,
  ) {
    if (trace == null) return null;

    Polynomial? fit = fitPolynomialRansac(
      trace.forward,
      trace.lateral,
      degree: 2,
      iterations: config.ransacIterations,
      inlierThreshold: config.ransacInlierMeters,
      seed: position.index,
    );
    fit ??= fitPolynomial(trace.forward, trace.lateral,
        degree: 2, weights: trace.weights);
    if (fit == null) return null;

    final double minRange = trace.forward.reduce(math.min);
    final double maxRange = trace.forward.reduce(math.max);

    // Reject physically impossible geometry rather than steering by it.
    final double curvature = fit.curvatureAt((minRange + maxRange) / 2);
    if (curvature.abs() > config.maxCurvature) {
      Log.debug(_tag,
          'rejected ${position.name}: curvature ${curvature.toStringAsExponential(1)}');
      return null;
    }
    final double nearOffset = fit.evaluate(math.max(minRange, 5));
    if (nearOffset.abs() > 6.0) return null;

    // Temporal smoothing: a lane cannot teleport between frames.
    if (previous != null) {
      final double jump =
          (previous.curve.evaluate(math.max(minRange, 6)) - nearOffset).abs();
      if (jump < 1.2) {
        fit = previous.curve.lerp(fit, 1 - config.temporalSmoothing);
      }
    }

    final double residual = _meanResidual(fit, trace);
    final (LineType type, double dashConfidence) =
        _classifyLineType(trace, bev);
    final LineColor color = LineColor.unknown;

    final double confidence = _boundaryConfidence(
      trace: trace,
      residual: residual,
      rangeMeters: maxRange - minRange,
      bevCoverage: bev.coverage,
      calibrated: frame.calibration.isCalibrated,
      dashConfidence: dashConfidence,
    );

    return LaneBoundary(
      position: position,
      curve: fit,
      confidence: Confidence(confidence, source: 'cv-lane'),
      lineType: type,
      color: color,
      minRangeMeters: minRange,
      maxRangeMeters: maxRange,
      supportPointCount: trace.lateral.length,
    );
  }

  double _meanResidual(Polynomial fit, _LaneTrace trace) {
    double sum = 0;
    for (int i = 0; i < trace.forward.length; i++) {
      sum += (fit.evaluate(trace.forward[i]) - trace.lateral[i]).abs();
    }
    return sum / trace.forward.length;
  }

  /// Solid versus dashed from the fraction of search windows that found
  /// support. A solid line is present in essentially every window; a dashed
  /// line, at typical 3 m mark / 9 m gap proportions, in roughly a third.
  (LineType, double) _classifyLineType(_LaneTrace trace, BirdsEyeView bev) {
    if (trace.windowsSearched < 4) return (LineType.unknown, 0.2);
    final double supportRatio =
        trace.windowsWithSupport / trace.windowsSearched;
    if (supportRatio > 0.82) return (LineType.solid, 0.8);
    if (supportRatio < 0.55) return (LineType.dashed, 0.7);
    // In between is genuinely ambiguous — a worn solid line or a long dash.
    return (LineType.unknown, 0.4);
  }

  double _boundaryConfidence({
    required _LaneTrace trace,
    required double residual,
    required double rangeMeters,
    required double bevCoverage,
    required bool calibrated,
    required double dashConfidence,
  }) {
    // Residual: a good fit sits within a few centimetres of its support.
    final double residualScore =
        clampDouble(1.0 - residual / config.ransacInlierMeters / 2, 0, 1);
    // Support: more points and a longer range are strictly better evidence.
    final double supportScore =
        clampDouble(trace.lateral.length / config.minPointsPerLane, 0, 1);
    final double rangeScore = clampDouble(rangeMeters / 25.0, 0, 1);
    final double coverageScore = clampDouble(bevCoverage / 0.5, 0, 1);

    double c = 0.35 * residualScore +
        0.25 * supportScore +
        0.25 * rangeScore +
        0.15 * coverageScore;

    // An uncalibrated camera means the metric geometry is a guess, and every
    // metric claim built on it inherits that doubt.
    if (!calibrated) c *= 0.8;
    return clampDouble(c, 0, 1);
  }

  /// Cross-check the two boundaries against each other.
  ///
  /// Two independently plausible fits can still be jointly absurd — crossing
  /// each other, 8 m apart, or diverging. Rejecting the weaker one is much
  /// safer than handing the planner an impossible lane.
  (LaneBoundary?, LaneBoundary?) _validatePair(
    LaneBoundary? left,
    LaneBoundary? right,
  ) {
    if (left == null || right == null) return (left, right);

    bool widthOk = true;
    bool parallelOk = true;
    for (final double d in <double>[6, 12, 20, 30]) {
      final double? l = left.lateralAt(d);
      final double? r = right.lateralAt(d);
      if (l == null || r == null) continue;
      final double width = r - l;
      if (width < config.minLaneWidthMeters ||
          width > config.maxLaneWidthMeters) {
        widthOk = false;
        break;
      }
      final double headingDelta = (left.headingAt(d) - right.headingAt(d)).abs();
      if (headingDelta > 0.12) parallelOk = false;
    }

    if (widthOk && parallelOk) return (left, right);

    // Keep whichever boundary has the better evidence and drop the other.
    Log.debug(_tag,
        'lane pair rejected (width ok: $widthOk, parallel ok: $parallelOk)');
    return left.confidence.value >= right.confidence.value
        ? (left, null)
        : (null, right);
  }

  /// Search one lane width outward for the adjacent lanes' outer boundaries.
  List<LaneBoundary> _findAdjacent(
    Uint8List mask,
    BirdsEyeView bev,
    CameraFrame frame,
    LaneBoundary? left,
    LaneBoundary? right,
  ) {
    final List<LaneBoundary> out = <LaneBoundary>[];
    final double laneWidth = _trackedLaneWidth;

    void search(LaneBoundary? reference, bool toLeft) {
      if (reference == null) return;
      final double nearForward = math.max(reference.minRangeMeters, 6);
      final double expected = reference.curve.evaluate(nearForward) +
          (toLeft ? -laneWidth : laneWidth);
      final int seed = bev.columnAtLateral(expected).round();
      if (seed < 2 || seed >= bev.width - 2) return;

      final _LaneTrace? trace = _slidingWindow(mask, bev, seed);
      if (trace == null || trace.lateral.length < 4) return;

      final LaneBoundary? boundary = _fitBoundary(
        trace,
        toLeft ? LanePosition.leftAdjacent : LanePosition.rightAdjacent,
        bev,
        frame,
        mask,
        null,
      );
      if (boundary == null) return;

      // It must actually be outside the ego boundary, not a re-detection of it.
      final double separation =
          (boundary.curve.evaluate(nearForward) - reference.curve.evaluate(nearForward))
              .abs();
      if (separation < config.minLaneWidthMeters * 0.7) return;
      out.add(boundary);
    }

    search(left, true);
    search(right, false);
    return out;
  }

  // --- Lane width and ego pose -------------------------------------------

  /// Track the lane width over time. After a few good frames the measured
  /// width is a far better prior than a hard-coded 3.5 m, and it is what makes
  /// single-boundary mode usable.
  void _updateLaneWidth(LaneBoundary? left, LaneBoundary? right) {
    if (left == null || right == null) {
      _laneWidthConfidence *= 0.97;
      return;
    }
    final List<double> widths = <double>[];
    for (final double d in <double>[6, 10, 15, 22, 30]) {
      final double? l = left.lateralAt(d);
      final double? r = right.lateralAt(d);
      if (l != null && r != null) widths.add(r - l);
    }
    if (widths.isEmpty) return;

    widths.sort();
    final double median = widths[widths.length ~/ 2];
    if (median < config.minLaneWidthMeters ||
        median > config.maxLaneWidthMeters) {
      return;
    }

    final double evidence =
        math.min(left.confidence.value, right.confidence.value);
    final double alpha = 0.12 * evidence;
    _trackedLaneWidth = _trackedLaneWidth * (1 - alpha) + median * alpha;
    _laneWidthConfidence =
        clampDouble(_laneWidthConfidence * 0.9 + evidence * 0.15, 0, 1);
  }

  /// Lateral offset from the lane centre and heading error, both at the
  /// vehicle, not at some arbitrary look-ahead.
  (double, double) _egoPose(
    LaneBoundary? left,
    LaneBoundary? right,
    LaneMode mode,
  ) {
    const double atDistance = 5.0;
    double? centre;
    double? heading;

    if (left != null && right != null) {
      final double l = left.lateralAtUnchecked(atDistance);
      final double r = right.lateralAtUnchecked(atDistance);
      centre = (l + r) / 2;
      heading = (left.headingAt(atDistance) + right.headingAt(atDistance)) / 2;
    } else if (left != null) {
      centre = left.lateralAtUnchecked(atDistance) + _trackedLaneWidth / 2;
      heading = left.headingAt(atDistance);
    } else if (right != null) {
      centre = right.lateralAtUnchecked(atDistance) - _trackedLaneWidth / 2;
      heading = right.headingAt(atDistance);
    }

    if (centre == null || heading == null) return (0, 0);
    // The vehicle is at x = 0 by definition, so its offset from the centre is
    // the negative of the centre's offset from the vehicle.
    return (-centre, -heading);
  }
}

/// Raw sliding-window output before fitting.
class _LaneTrace {
  const _LaneTrace({
    required this.lateral,
    required this.forward,
    required this.weights,
    required this.windowsSearched,
    required this.windowsWithSupport,
  });

  final List<double> lateral;
  final List<double> forward;
  final List<double> weights;
  final int windowsSearched;
  final int windowsWithSupport;
}
