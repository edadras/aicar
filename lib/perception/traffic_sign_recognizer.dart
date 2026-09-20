import 'dart:math' as math;
import 'dart:typed_data';

import '../ai/inference_backend.dart';
import '../ai/interfaces/traffic_sign_detector.dart';
import '../ai/model_descriptor.dart';
import '../camera/camera_calibration.dart';
import '../camera/camera_frame.dart';
import '../camera/image_preprocessing.dart';
import '../core/confidence.dart';
import '../core/geometry.dart';
import '../core/logging.dart';
import 'detection.dart';
import 'object_class.dart';
import 'speed_limit_reader.dart';
import 'traffic_sign.dart';

/// Recognises traffic signs from candidate boxes produced by the object
/// detector.
///
/// Two classification paths, tried in order:
///  1. A trained classifier (GTSRB-style), when one is installed. This is the
///     accurate path and the one that can name specific warning signs.
///  2. Shape and colour analysis, which needs no weights. A red octagon is a
///     stop sign; a red-rimmed white circle with digits is a speed limit; a
///     red-rimmed triangle is a warning. This path deliberately returns coarse
///     categories with modest confidence rather than guessing at specifics.
///
/// Running on detector crops rather than the full frame keeps the cost at
/// about a millisecond per candidate instead of a second full-frame pass.
class TrafficSignRecognizer extends TrafficSignDetector {
  TrafficSignRecognizer({
    ModelDescriptor? classifierDescriptor,
    InferenceBackend? backend,
    this.speedLimitReader = const SpeedLimitReader(),
    this.minBoxHeightPixels = 14,
  })  : _classifierDescriptor = classifierDescriptor,
        _backend = backend;

  static const String _tag = 'TrafficSignRecognizer';

  final ModelDescriptor? _classifierDescriptor;
  final InferenceBackend? _backend;
  final SpeedLimitReader speedLimitReader;

  /// Below this the sign face has too few pixels to classify, let alone read.
  final int minBoxHeightPixels;

  int? _handle;
  String? _unavailableReason;
  int _nextId = 1;
  final Map<int, _SignMemory> _memory = <int, _SignMemory>{};

  /// Planned-path geometry, set by the pipeline, used to decide whether a
  /// sign applies to our carriageway or to a side road.
  Polynomial? egoPathCenterline;

  @override
  String get modelId => _classifierDescriptor?.id ?? 'classical-sign';

  @override
  String get displayName => _handle != null
      ? _classifierDescriptor!.name
      : 'Sign recogniser (shape + colour, no classifier installed)';

  @override
  ModelRole get role => ModelRole.trafficSignDetection;

  @override
  ModelDescriptor? get descriptor => _classifierDescriptor;

  /// Always ready: the shape/colour path needs nothing. [hasClassifier]
  /// distinguishes the accurate path from the fallback.
  @override
  bool get isReady => true;

  bool get hasClassifier => _handle != null;

  @override
  String? get unavailableReason => _handle == null
      ? (_unavailableReason ?? 'no sign classifier installed — using '
          'shape/colour fallback with reduced accuracy')
      : null;

  @override
  Future<void> load() async {
    final ModelDescriptor? d = _classifierDescriptor;
    final InferenceBackend? b = _backend;
    if (d == null || b == null) return;
    try {
      if (!await b.isAvailable()) {
        _unavailableReason = 'inference backend unavailable';
        return;
      }
      _handle = await b.loadModel(d);
      _unavailableReason = null;
      Log.info(_tag, 'sign classifier ${d.id} loaded');
    } catch (e) {
      _unavailableReason = '$e';
      Log.error(_tag, 'classifier load failed', e);
    }
  }

  @override
  Future<void> close() async {
    final int? h = _handle;
    _handle = null;
    if (h != null) await _backend?.unload(h);
  }

  @override
  Future<List<TrafficSign>> detectSigns(
    CameraFrame frame, {
    List<Detection> candidates = const <Detection>[],
  }) async {
    final List<TrafficSign> out = <TrafficSign>[];
    final Set<int> seen = <int>{};

    for (final Detection d in candidates) {
      if (d.objectClass != ObjectClass.trafficSign) continue;
      final double heightPx = d.box.height * frame.height;
      if (heightPx < minBoxHeightPixels) continue;

      final _SignCrop? crop = _cropSign(frame, d.box);
      if (crop == null) continue;

      _SignClassification classification =
          await _classifyWithModel(crop) ?? _classifyByShapeAndColor(crop);

      SpeedLimitReading? reading;
      if (classification.type.carriesValue) {
        reading = _readValue(crop);
        if (reading == null &&
            classification.type == TrafficSignType.speedLimit &&
            !hasClassifier) {
          // A red circle we cannot read is a prohibition sign of unknown
          // value. Reporting "SPEED LIMIT" with no number would imply we know
          // something we do not.
          classification = _SignClassification(
            type: TrafficSignType.unknown,
            confidence: classification.confidence * 0.6,
            source: '${classification.source}:unreadable',
          );
        }
      }

      final int id = _matchOrCreateId(d.box);
      seen.add(id);
      final _SignMemory memory = _memory.putIfAbsent(
        id,
        () => _SignMemory(box: d.box, firstSeenMicros: frame.timestampMicros),
      );
      memory.update(
        box: d.box,
        type: classification.type,
        valueKph: reading?.valueKph,
      );

      final double? distance = frame.calibration.distanceFromApparentHeight(
        boxHeightPixels: heightPx,
        realHeightMeters: ObjectClass.trafficSign.sizePrior.height,
      );

      out.add(TrafficSign(
        id: id,
        type: memory.stableType,
        box: d.box,
        confidence: Confidence(
          math.min(d.score, classification.confidence),
          source: classification.source,
        ),
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        speedLimitKph: memory.stableValue,
        valueConfidence: reading == null
            ? null
            : Confidence(
                // Repeated agreement across frames is what makes a reading
                // trustworthy, far more than one frame's match score.
                clampDouble(
                  reading.confidence *
                      (0.5 + 0.5 * clampDouble(memory.valueAgreement, 0, 1)),
                  0,
                  0.95,
                ),
                source: hasClassifier ? 'classifier' : 'template',
              ),
        distanceMeters: distance,
        appliesToEgoLane:
            _appliesToEgoLane(frame.calibration, d.box, distance),
        firstSeenMicros: memory.firstSeenMicros,
        observationCount: memory.observations,
      ));
    }

    _memory.removeWhere((int id, _SignMemory m) => !seen.contains(id));
    return out;
  }

  // --- Cropping -----------------------------------------------------------

  _SignCrop? _cropSign(CameraFrame frame, BoundingBox box) {
    final int x0 = (box.left * frame.width).floor().clamp(0, frame.width - 1);
    final int x1 = (box.right * frame.width).ceil().clamp(x0 + 1, frame.width);
    final int y0 = (box.top * frame.height).floor().clamp(0, frame.height - 1);
    final int y1 =
        (box.bottom * frame.height).ceil().clamp(y0 + 1, frame.height);
    final int w = x1 - x0;
    final int h = y1 - y0;
    if (w < 6 || h < 6) return null;

    final int channels = frame.bytesPerPixel;
    final Uint8List pixels = Uint8List(w * h * channels);
    for (int row = 0; row < h; row++) {
      pixels.setRange(
        row * w * channels,
        (row + 1) * w * channels,
        frame.bytes,
        ((y0 + row) * frame.width + x0) * channels,
      );
    }

    return _SignCrop(
      pixels: pixels,
      width: w,
      height: h,
      channels: channels,
      hasColor: frame.format == PixelFormat.rgb888,
    );
  }

  // --- Path 1: trained classifier ----------------------------------------

  Future<_SignClassification?> _classifyWithModel(_SignCrop crop) async {
    final int? handle = _handle;
    final ModelDescriptor? d = _classifierDescriptor;
    final InferenceBackend? b = _backend;
    if (handle == null || d == null || b == null) return null;

    try {
      final Uint8List resized = ImagePreprocessing.resize(
        crop.pixels,
        crop.width,
        crop.height,
        d.inputWidth,
        d.inputHeight,
        channels: crop.channels,
      );
      final Float32List input = ImagePreprocessing.toFloatTensor(
        resized,
        d.inputWidth,
        d.inputHeight,
        channels: d.inputChannels,
        mean: d.inputMean,
        std: d.inputStd,
        channelsFirst: d.channelsFirst,
      );
      final InferenceOutput out = await b.run(handle, input);
      if (out.outputCount == 0) return null;

      final Float32List logits = out.tensor(0);
      final int count = math.min(logits.length, d.labels.length);
      if (count == 0) return null;

      double best = -double.infinity;
      double second = -double.infinity;
      int bestIndex = 0;
      for (int i = 0; i < count; i++) {
        if (logits[i] > best) {
          second = best;
          best = logits[i];
          bestIndex = i;
        } else if (logits[i] > second) {
          second = logits[i];
        }
      }

      // Softmax over the top two: the margin is what matters for a decision.
      final double margin = best - second;
      final double probability = 1 / (1 + math.exp(-margin));
      if (probability < d.scoreThreshold) return null;

      return _SignClassification(
        type: _typeFromLabel(d.labels[bestIndex]),
        confidence: probability,
        source: 'classifier',
      );
    } catch (e) {
      Log.warn(_tag, 'sign classification failed: $e');
      return null;
    }
  }

  static TrafficSignType _typeFromLabel(String label) {
    final String l = label.toLowerCase().trim();
    if (l.startsWith('speed_limit')) return TrafficSignType.speedLimit;
    if (l.startsWith('end_speed_limit')) {
      return TrafficSignType.endOfSpeedLimit;
    }
    return switch (l) {
      'stop' => TrafficSignType.stop,
      'give_way' || 'yield' => TrafficSignType.giveWay,
      'no_entry' || 'no_vehicles' || 'no_trucks' => TrafficSignType.noEntry,
      'no_overtaking' || 'no_overtaking_trucks' =>
        TrafficSignType.noOvertaking,
      'end_no_overtaking' || 'end_no_overtaking_trucks' =>
        TrafficSignType.endOfNoOvertaking,
      'pedestrian_crossing' => TrafficSignType.pedestrianCrossing,
      'school_zone' => TrafficSignType.schoolZone,
      'road_work' => TrafficSignType.roadWork,
      'slippery_road' || 'ice_snow' => TrafficSignType.slipperyRoad,
      'bumpy_road' => TrafficSignType.bumpyRoad,
      'road_narrows' => TrafficSignType.narrowRoad,
      'curve_left' => TrafficSignType.curveLeft,
      'curve_right' => TrafficSignType.curveRight,
      'double_curve' => TrafficSignType.generalWarning,
      'roundabout' => TrafficSignType.roundabout,
      'traffic_signals' => TrafficSignType.trafficSignalAhead,
      'animal_crossing' => TrafficSignType.animalCrossing,
      'bicycle_crossing' => TrafficSignType.pedestrianCrossing,
      'mandatory_left' || 'keep_left' => TrafficSignType.mandatoryLeft,
      'mandatory_right' || 'keep_right' => TrafficSignType.mandatoryRight,
      'mandatory_straight' => TrafficSignType.mandatoryStraight,
      'general_warning' => TrafficSignType.generalWarning,
      _ => TrafficSignType.unknown,
    };
  }

  // --- Path 2: shape and colour ------------------------------------------

  /// Coarse classification from the sign's dominant colour and its outline.
  ///
  /// This recovers the categories that actually change behaviour — stop, give
  /// way, prohibition, warning, mandatory — without attempting to name a
  /// specific pictogram, which shape analysis genuinely cannot do.
  _SignClassification _classifyByShapeAndColor(_SignCrop crop) {
    if (!crop.hasColor) {
      return const _SignClassification(
        type: TrafficSignType.unknown,
        confidence: 0.15,
        source: 'shape:mono',
      );
    }

    final _ColorProfile profile = _profileColors(crop);
    final _ShapeProfile shape = _profileShape(crop, profile);

    // Red-dominant, roughly octagonal, ink covering most of the face: stop.
    if (profile.redFraction > 0.35 && shape.cornerCount >= 7) {
      return const _SignClassification(
        type: TrafficSignType.stop,
        confidence: 0.6,
        source: 'shape:octagon+red',
      );
    }

    // Downward triangle with a red rim: give way.
    if (profile.redFraction > 0.18 && shape.isDownwardTriangle) {
      return const _SignClassification(
        type: TrafficSignType.giveWay,
        confidence: 0.55,
        source: 'shape:triangle-down+red',
      );
    }

    // Upward triangle with a red rim: a warning sign, kind unknown.
    if (profile.redFraction > 0.15 && shape.isUpwardTriangle) {
      return const _SignClassification(
        type: TrafficSignType.generalWarning,
        confidence: 0.5,
        source: 'shape:triangle-up+red',
      );
    }

    if (shape.isCircular) {
      if (profile.redFraction > 0.20) {
        // Red ring: a prohibition. A white interior with dark marks is a
        // speed limit; a solid red or white bar is no-entry.
        if (profile.whiteFraction > 0.30) {
          return const _SignClassification(
            type: TrafficSignType.speedLimit,
            confidence: 0.5,
            source: 'shape:circle+red-ring',
          );
        }
        return const _SignClassification(
          type: TrafficSignType.noEntry,
          confidence: 0.45,
          source: 'shape:circle+red',
        );
      }
      if (profile.blueFraction > 0.35) {
        return const _SignClassification(
          type: TrafficSignType.mandatoryStraight,
          confidence: 0.35,
          source: 'shape:circle+blue',
        );
      }
      if (profile.blackFraction > 0.12 && profile.whiteFraction > 0.45) {
        // White circle with a black diagonal: end of restrictions.
        return const _SignClassification(
          type: TrafficSignType.endOfSpeedLimit,
          confidence: 0.35,
          source: 'shape:circle+white',
        );
      }
    }

    if (profile.yellowFraction > 0.35 || profile.orangeFraction > 0.3) {
      return const _SignClassification(
        type: TrafficSignType.roadWork,
        confidence: 0.4,
        source: 'shape:yellow/orange',
      );
    }

    return const _SignClassification(
      type: TrafficSignType.unknown,
      confidence: 0.2,
      source: 'shape:unmatched',
    );
  }

  _ColorProfile _profileColors(_SignCrop crop) {
    int red = 0;
    int blue = 0;
    int yellow = 0;
    int orange = 0;
    int white = 0;
    int black = 0;
    int total = 0;

    for (int i = 0; i < crop.width * crop.height; i++) {
      final int r = crop.pixels[i * crop.channels];
      final int g = crop.pixels[i * crop.channels + 1];
      final int b = crop.pixels[i * crop.channels + 2];
      total++;

      final double maxC = math.max(r, math.max(g, b)) / 255;
      final double minC = math.min(r, math.min(g, b)) / 255;
      final double delta = maxC - minC;
      final double value = maxC;
      final double saturation = maxC < 1e-6 ? 0 : delta / maxC;

      if (saturation < 0.22) {
        if (value > 0.62) {
          white++;
        } else if (value < 0.28) {
          black++;
        }
        continue;
      }

      double hue;
      final double rf = r / 255;
      final double gf = g / 255;
      final double bf = b / 255;
      if (maxC == rf) {
        hue = 60 * (((gf - bf) / delta) % 6);
      } else if (maxC == gf) {
        hue = 60 * ((bf - rf) / delta + 2);
      } else {
        hue = 60 * ((rf - gf) / delta + 4);
      }
      if (hue < 0) hue += 360;

      if (hue >= 345 || hue <= 12) {
        red++;
      } else if (hue > 12 && hue <= 38) {
        orange++;
      } else if (hue > 38 && hue <= 68) {
        yellow++;
      } else if (hue >= 195 && hue <= 255) {
        blue++;
      }
    }

    if (total == 0) {
      return const _ColorProfile(0, 0, 0, 0, 0, 0);
    }
    return _ColorProfile(
      red / total,
      blue / total,
      yellow / total,
      orange / total,
      white / total,
      black / total,
    );
  }

  /// Outline shape from the saturated/dark foreground mask.
  _ShapeProfile _profileShape(_SignCrop crop, _ColorProfile profile) {
    final int w = crop.width;
    final int h = crop.height;

    // Row-wise extent of the sign face: a triangle's width varies linearly
    // with row, a circle's as a chord, an octagon's barely at all.
    final List<double> widths = <double>[];
    for (int y = 0; y < h; y++) {
      int first = -1;
      int last = -1;
      for (int x = 0; x < w; x++) {
        final int i = (y * w + x) * crop.channels;
        final int r = crop.pixels[i];
        final int g = crop.pixels[i + 1];
        final int b = crop.pixels[i + 2];
        final double maxC = math.max(r, math.max(g, b)) / 255;
        final double minC = math.min(r, math.min(g, b)) / 255;
        final bool foreground = (maxC - minC) > 0.18 || maxC > 0.6;
        if (foreground) {
          if (first < 0) first = x;
          last = x;
        }
      }
      widths.add(first < 0 ? 0 : (last - first + 1) / w);
    }

    final double topWidth = _meanOf(widths, 0, h ~/ 5);
    final double middleWidth = _meanOf(widths, (h * 2) ~/ 5, (h * 3) ~/ 5);
    final double bottomWidth = _meanOf(widths, (h * 4) ~/ 5, h);

    // Upward triangle: narrow at the top, wide at the bottom.
    final bool upwardTriangle =
        topWidth < 0.45 && bottomWidth > 0.75 && middleWidth > topWidth;
    // Downward triangle: the reverse.
    final bool downwardTriangle =
        topWidth > 0.75 && bottomWidth < 0.45 && middleWidth < topWidth;
    // Circle: widest in the middle, tapering symmetrically.
    final bool circular = !upwardTriangle &&
        !downwardTriangle &&
        middleWidth > 0.8 &&
        topWidth < middleWidth * 0.85 &&
        bottomWidth < middleWidth * 0.85 &&
        (topWidth - bottomWidth).abs() < 0.22;

    // Octagon: like a circle but with flatter top and bottom edges.
    int cornerCount = 0;
    if (!upwardTriangle && !downwardTriangle) {
      if (middleWidth > 0.85 && topWidth > 0.45 && bottomWidth > 0.45) {
        cornerCount = (topWidth - bottomWidth).abs() < 0.12 ? 8 : 4;
      }
    }

    return _ShapeProfile(
      isUpwardTriangle: upwardTriangle,
      isDownwardTriangle: downwardTriangle,
      isCircular: circular,
      cornerCount: cornerCount,
    );
  }

  static double _meanOf(List<double> values, int from, int to) {
    final int lo = from.clamp(0, values.length);
    final int hi = to.clamp(lo, values.length);
    if (hi <= lo) return 0;
    double sum = 0;
    for (int i = lo; i < hi; i++) {
      sum += values[i];
    }
    return sum / (hi - lo);
  }

  // --- Value reading ------------------------------------------------------

  /// Read the number from the middle of a speed-limit face.
  ///
  /// The red annulus is excluded by cropping to the central 62%, which is
  /// where the numerals sit on every standard speed-limit design.
  SpeedLimitReading? _readValue(_SignCrop crop) {
    final int inset = (math.min(crop.width, crop.height) * 0.19).round();
    final int x0 = inset;
    final int y0 = inset;
    final int w = crop.width - 2 * inset;
    final int h = crop.height - 2 * inset;
    if (w < 8 || h < 8) return null;

    final Uint8List gray = Uint8List(w * h);
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        final int i = ((y0 + y) * crop.width + (x0 + x)) * crop.channels;
        if (crop.channels == 1) {
          gray[y * w + x] = crop.pixels[i];
        } else {
          gray[y * w + x] = (crop.pixels[i] * 77 +
                  crop.pixels[i + 1] * 150 +
                  crop.pixels[i + 2] * 29) >>
              8;
        }
      }
    }

    // Upscale small faces so the digit segmenter has something to work with.
    if (h < 24) {
      final int scale = (24 / h).ceil();
      final Uint8List big = ImagePreprocessing.resize(
        gray,
        w,
        h,
        w * scale,
        h * scale,
        channels: 1,
      );
      return speedLimitReader.read(big, w * scale, h * scale);
    }
    return speedLimitReader.read(gray, w, h);
  }

  // --- Relevance and identity --------------------------------------------

  /// A sign mounted far to the side of our path governs a different road.
  bool _appliesToEgoLane(
    CameraCalibration cal,
    BoundingBox box,
    double? distance,
  ) {
    if (distance == null) return true; // no evidence either way
    final double bearing = (box.centerX * cal.imageWidth - cal.cx) / cal.fx;
    final double lateral = bearing * distance + cal.lateralOffsetMeters;
    final double pathLateral = egoPathCenterline?.evaluate(distance) ?? 0;
    // Signs sit on the verge or overhead; 6 m either side of our path covers
    // both without swallowing the signs for a parallel slip road.
    return (lateral - pathLateral).abs() < 6.0;
  }

  int _matchOrCreateId(BoundingBox box) {
    int bestId = -1;
    double bestIou = 0.2;
    for (final MapEntry<int, _SignMemory> e in _memory.entries) {
      final double iou = e.value.box.iou(box);
      if (iou > bestIou) {
        bestIou = iou;
        bestId = e.key;
      }
    }
    return bestId >= 0 ? bestId : _nextId++;
  }

  void reset() => _memory.clear();
}

class _SignCrop {
  const _SignCrop({
    required this.pixels,
    required this.width,
    required this.height,
    required this.channels,
    required this.hasColor,
  });

  final Uint8List pixels;
  final int width;
  final int height;
  final int channels;
  final bool hasColor;
}

class _SignClassification {
  const _SignClassification({
    required this.type,
    required this.confidence,
    required this.source,
  });

  final TrafficSignType type;
  final double confidence;
  final String source;
}

class _ColorProfile {
  const _ColorProfile(
    this.redFraction,
    this.blueFraction,
    this.yellowFraction,
    this.orangeFraction,
    this.whiteFraction,
    this.blackFraction,
  );

  final double redFraction;
  final double blueFraction;
  final double yellowFraction;
  final double orangeFraction;
  final double whiteFraction;
  final double blackFraction;
}

class _ShapeProfile {
  const _ShapeProfile({
    required this.isUpwardTriangle,
    required this.isDownwardTriangle,
    required this.isCircular,
    required this.cornerCount,
  });

  final bool isUpwardTriangle;
  final bool isDownwardTriangle;
  final bool isCircular;
  final int cornerCount;
}

/// Per-sign memory so a type and a value must recur before they are believed.
class _SignMemory {
  _SignMemory({required this.box, required this.firstSeenMicros});

  BoundingBox box;
  final int firstSeenMicros;
  int observations = 0;

  final Map<TrafficSignType, int> _typeVotes = <TrafficSignType, int>{};
  final Map<int, int> _valueVotes = <int, int>{};

  TrafficSignType stableType = TrafficSignType.unknown;
  int? stableValue;

  /// Fraction of value readings that agreed with the winner. This is the
  /// number that actually justifies acting on a speed limit.
  double valueAgreement = 0;

  void update({
    required BoundingBox box,
    required TrafficSignType type,
    int? valueKph,
  }) {
    this.box = box;
    observations++;

    _typeVotes.update(type, (int v) => v + 1, ifAbsent: () => 1);
    TrafficSignType bestType = TrafficSignType.unknown;
    int bestTypeVotes = 0;
    for (final MapEntry<TrafficSignType, int> e in _typeVotes.entries) {
      if (e.value > bestTypeVotes) {
        bestTypeVotes = e.value;
        bestType = e.key;
      }
    }
    stableType = bestType;

    if (valueKph != null) {
      _valueVotes.update(valueKph, (int v) => v + 1, ifAbsent: () => 1);
      int bestValue = valueKph;
      int bestVotes = 0;
      int totalVotes = 0;
      for (final MapEntry<int, int> e in _valueVotes.entries) {
        totalVotes += e.value;
        if (e.value > bestVotes) {
          bestVotes = e.value;
          bestValue = e.key;
        }
      }
      stableValue = bestValue;
      valueAgreement = totalVotes == 0 ? 0 : bestVotes / totalVotes;
    }
  }
}
