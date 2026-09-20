import 'dart:math' as math;
import 'dart:typed_data';

import '../ai/inference_backend.dart';
import '../ai/interfaces/lane_detector.dart';
import '../ai/model_descriptor.dart';
import '../camera/camera_frame.dart';
import '../camera/image_preprocessing.dart';
import '../core/confidence.dart';
import '../core/geometry.dart';
import '../core/logging.dart';
import 'lane.dart';
import 'road_segmentation.dart';

/// Lane detector backed by a row-anchor network (Ultra-Fast-Lane-Detection
/// and its derivatives).
///
/// Row-anchor models predict, for each of a fixed set of image rows and each
/// lane slot, a probability distribution over horizontal grid cells. Decoding
/// takes the expectation over that distribution rather than the argmax, which
/// gives sub-cell precision — worth roughly a lane-marking width at 30 m.
///
/// The decoded image points are then projected onto the road plane through
/// the calibration, so the output is metric and interchangeable with the
/// classical detector's.
class NeuralLaneDetector extends LaneDetector {
  NeuralLaneDetector({
    required ModelDescriptor descriptor,
    required InferenceBackend backend,
  })  : _descriptor = descriptor,
        _backend = backend;

  static const String _tag = 'NeuralLaneDetector';

  final ModelDescriptor _descriptor;
  final InferenceBackend _backend;

  int? _handle;
  String? _unavailableReason;
  double _trackedLaneWidth = 3.5;
  double _laneWidthConfidence = 0;

  @override
  String get modelId => _descriptor.id;

  @override
  String get displayName => _descriptor.name;

  @override
  ModelRole get role => ModelRole.laneDetection;

  @override
  ModelDescriptor? get descriptor => _descriptor;

  @override
  bool get isReady => _handle != null;

  @override
  String? get unavailableReason => _unavailableReason;

  @override
  Future<void> load() async {
    try {
      if (!await _backend.isAvailable()) {
        _unavailableReason = 'inference backend unavailable';
        return;
      }
      _handle = await _backend.loadModel(_descriptor);
      _unavailableReason = null;
    } catch (e) {
      _unavailableReason = '$e';
      Log.error(_tag, 'load failed', e);
    }
  }

  @override
  Future<void> close() async {
    final int? h = _handle;
    _handle = null;
    if (h != null) await _backend.unload(h);
  }

  @override
  Future<LaneDetectionResult> detectLanes(
    CameraFrame frame, {
    RoadSegmentation? segmentation,
    LaneDetectionResult? previous,
  }) async {
    final int? handle = _handle;
    if (handle == null) {
      return LaneDetectionResult.empty(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: _unavailableReason ?? 'lane model not loaded',
      );
    }

    try {
      final Uint8List resized = ImagePreprocessing.resize(
        frame.bytes,
        frame.width,
        frame.height,
        _descriptor.inputWidth,
        _descriptor.inputHeight,
        channels: frame.bytesPerPixel,
      );
      final Float32List input = ImagePreprocessing.toFloatTensor(
        resized,
        _descriptor.inputWidth,
        _descriptor.inputHeight,
        channels: _descriptor.inputChannels,
        mean: _descriptor.inputMean,
        std: _descriptor.inputStd,
        channelsFirst: _descriptor.channelsFirst,
      );

      final InferenceOutput out = await _backend.run(handle, input);
      if (out.outputCount == 0) {
        return LaneDetectionResult.empty(
          frameId: frame.id,
          timestampMicros: frame.timestampMicros,
          reason: 'lane model produced no output',
        );
      }

      final List<_LaneCandidate> candidates = _decodeRowAnchors(out, frame);
      return _toResult(candidates, frame, previous);
    } catch (e, st) {
      Log.error(_tag, 'lane inference failed', e, st);
      return LaneDetectionResult.empty(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: 'lane inference error: $e',
      );
    }
  }

  /// Decode `[1, gridding+1, rowAnchors, lanes]` into metric point sets.
  ///
  /// The extra grid cell is the "no lane in this row" class; a row whose
  /// highest probability lands there contributes nothing, which is what lets
  /// the model represent a lane that starts partway up the image.
  List<_LaneCandidate> _decodeRowAnchors(
    InferenceOutput out,
    CameraFrame frame,
  ) {
    final Float32List t = out.tensor(0);
    final int gridding = _descriptor.extraInt('griddingNum', 200);
    final int rowAnchors = _descriptor.extraInt('rowAnchorCount', 72);
    final int laneCount = _descriptor.extraInt('laneCount', 4);
    final double anchorStart = _descriptor.extraDouble('rowAnchorStart', 0.42);
    final double anchorEnd = _descriptor.extraDouble('rowAnchorEnd', 1.0);

    final int channels = gridding + 1;
    final int expected = channels * rowAnchors * laneCount;
    if (t.length < expected) {
      Log.warn(_tag,
          'output tensor has ${t.length} values, expected $expected');
      return const <_LaneCandidate>[];
    }

    final List<_LaneCandidate> lanes = <_LaneCandidate>[];

    for (int lane = 0; lane < laneCount; lane++) {
      final List<PixelPoint> imagePoints = <PixelPoint>[];
      final List<double> forwards = <double>[];
      final List<double> laterals = <double>[];
      final List<double> weights = <double>[];
      double probabilitySum = 0;

      for (int row = 0; row < rowAnchors; row++) {
        // Softmax over the gridding classes (excluding the "absent" class).
        double maxLogit = -double.infinity;
        for (int g = 0; g < gridding; g++) {
          final double v = t[(g * rowAnchors + row) * laneCount + lane];
          if (v > maxLogit) maxLogit = v;
        }
        final double absentLogit =
            t[(gridding * rowAnchors + row) * laneCount + lane];

        double sum = 0;
        double weightedIndex = 0;
        for (int g = 0; g < gridding; g++) {
          final double e =
              math.exp(t[(g * rowAnchors + row) * laneCount + lane] - maxLogit);
          sum += e;
          weightedIndex += e * g;
        }
        final double absentE = math.exp(absentLogit - maxLogit);
        final double presence = sum / (sum + absentE);
        if (presence < 0.5 || sum <= 0) continue;

        // Expectation over grid cells gives sub-cell precision.
        final double cell = weightedIndex / sum;
        final double nx = cell / (gridding - 1);
        final double ny =
            anchorStart + (anchorEnd - anchorStart) * (row / (rowAnchors - 1));

        final PixelPoint pixel = PixelPoint(
          nx * frame.width,
          ny * frame.height,
        );
        final Vec2? ground = frame.calibration.projectToGround(pixel);
        if (ground == null || ground.y < 3 || ground.y > 70) continue;

        imagePoints.add(pixel);
        forwards.add(ground.y);
        laterals.add(ground.x);
        weights.add(presence *
            frame.calibration.groundDepthConfidenceAtRow(pixel.v));
        probabilitySum += presence;
      }

      if (forwards.length < 4) continue;
      lanes.add(_LaneCandidate(
        forwards: forwards,
        laterals: laterals,
        weights: weights,
        imagePoints: imagePoints,
        meanProbability: probabilitySum / forwards.length,
      ));
    }

    return lanes;
  }

  /// Assign decoded lane candidates to positions by their lateral offset at
  /// the vehicle, rather than trusting the model's own slot ordering — slot
  /// order differs between checkpoints and is not worth encoding per model.
  LaneDetectionResult _toResult(
    List<_LaneCandidate> candidates,
    CameraFrame frame,
    LaneDetectionResult? previous,
  ) {
    if (candidates.isEmpty) {
      return LaneDetectionResult.empty(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: 'model produced no lane rows above threshold',
      );
    }

    final List<_FittedLane> fitted = <_FittedLane>[];
    for (final _LaneCandidate c in candidates) {
      final Polynomial? fit = fitPolynomialRansac(
        c.forwards,
        c.laterals,
        degree: 2,
        inlierThreshold: 0.25,
        seed: fitted.length,
      );
      if (fit == null) continue;
      final double near = fit.evaluate(math.max(c.forwards.reduce(math.min), 5));
      if (near.abs() > 8) continue;
      if (fit.curvatureAt(15).abs() > 0.02) continue;
      fitted.add(_FittedLane(
        curve: fit,
        nearOffset: near,
        candidate: c,
        minRange: c.forwards.reduce(math.min),
        maxRange: c.forwards.reduce(math.max),
      ));
    }

    if (fitted.isEmpty) {
      return LaneDetectionResult.empty(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: 'no lane candidate survived geometric validation',
      );
    }

    fitted.sort((_FittedLane a, _FittedLane b) =>
        a.nearOffset.compareTo(b.nearOffset));

    final List<_FittedLane> leftSide =
        fitted.where((_FittedLane f) => f.nearOffset < 0).toList();
    final List<_FittedLane> rightSide =
        fitted.where((_FittedLane f) => f.nearOffset >= 0).toList();

    final List<LaneBoundary> boundaries = <LaneBoundary>[];
    LaneBoundary? egoLeft;
    LaneBoundary? egoRight;

    if (leftSide.isNotEmpty) {
      egoLeft = _toBoundary(leftSide.last, LanePosition.egoLeft);
      boundaries.add(egoLeft);
      if (leftSide.length >= 2) {
        boundaries.add(_toBoundary(
            leftSide[leftSide.length - 2], LanePosition.leftAdjacent));
      }
    }
    if (rightSide.isNotEmpty) {
      egoRight = _toBoundary(rightSide.first, LanePosition.egoRight);
      boundaries.add(egoRight);
      if (rightSide.length >= 2) {
        boundaries.add(_toBoundary(rightSide[1], LanePosition.rightAdjacent));
      }
    }

    _updateLaneWidth(egoLeft, egoRight);

    final LaneMode mode = switch ((egoLeft, egoRight)) {
      (final LaneBoundary _, final LaneBoundary _) => LaneMode.bothBoundaries,
      (null, null) => LaneMode.noLane,
      _ => LaneMode.singleBoundary,
    };

    double offset = 0;
    double headingError = 0;
    if (egoLeft != null && egoRight != null) {
      final double centre = (egoLeft.lateralAtUnchecked(5) +
              egoRight.lateralAtUnchecked(5)) /
          2;
      offset = -centre;
      headingError =
          -(egoLeft.headingAt(5) + egoRight.headingAt(5)) / 2;
    } else if (egoLeft != null) {
      offset = -(egoLeft.lateralAtUnchecked(5) + _trackedLaneWidth / 2);
      headingError = -egoLeft.headingAt(5);
    } else if (egoRight != null) {
      offset = -(egoRight.lateralAtUnchecked(5) - _trackedLaneWidth / 2);
      headingError = -egoRight.headingAt(5);
    }

    return LaneDetectionResult(
      boundaries: boundaries,
      mode: mode,
      frameId: frame.id,
      timestampMicros: frame.timestampMicros,
      laneWidthMeters: _trackedLaneWidth,
      laneWidthConfidence: _laneWidthConfidence,
      egoLateralOffsetMeters: offset,
      egoHeadingErrorRadians: headingError,
      modelName: _descriptor.id,
    );
  }

  LaneBoundary _toBoundary(_FittedLane f, LanePosition position) {
    // The network's own confidence, discounted by how far the fit extends and
    // how much the road plane can be trusted at that range.
    final double rangeScore =
        clampDouble((f.maxRange - f.minRange) / 25, 0, 1);
    final double confidence = clampDouble(
      0.6 * f.candidate.meanProbability + 0.4 * rangeScore,
      0,
      1,
    );
    return LaneBoundary(
      position: position,
      curve: f.curve,
      confidence: Confidence(confidence, source: 'nn-lane'),
      lineType: LineType.unknown,
      color: LineColor.unknown,
      minRangeMeters: f.minRange,
      maxRangeMeters: f.maxRange,
      imagePoints: f.candidate.imagePoints,
      supportPointCount: f.candidate.forwards.length,
    );
  }

  void _updateLaneWidth(LaneBoundary? left, LaneBoundary? right) {
    if (left == null || right == null) {
      _laneWidthConfidence *= 0.97;
      return;
    }
    final List<double> widths = <double>[];
    for (final double d in <double>[6, 12, 20, 30]) {
      final double? l = left.lateralAt(d);
      final double? r = right.lateralAt(d);
      if (l != null && r != null) widths.add(r - l);
    }
    if (widths.isEmpty) return;
    widths.sort();
    final double median = widths[widths.length ~/ 2];
    if (median < 2.4 || median > 5.2) return;
    final double evidence =
        math.min(left.confidence.value, right.confidence.value);
    final double alpha = 0.12 * evidence;
    _trackedLaneWidth = _trackedLaneWidth * (1 - alpha) + median * alpha;
    _laneWidthConfidence =
        clampDouble(_laneWidthConfidence * 0.9 + evidence * 0.15, 0, 1);
  }
}

class _LaneCandidate {
  const _LaneCandidate({
    required this.forwards,
    required this.laterals,
    required this.weights,
    required this.imagePoints,
    required this.meanProbability,
  });

  final List<double> forwards;
  final List<double> laterals;
  final List<double> weights;
  final List<PixelPoint> imagePoints;
  final double meanProbability;
}

class _FittedLane {
  const _FittedLane({
    required this.curve,
    required this.nearOffset,
    required this.candidate,
    required this.minRange,
    required this.maxRange,
  });

  final Polynomial curve;
  final double nearOffset;
  final _LaneCandidate candidate;
  final double minRange;
  final double maxRange;
}
