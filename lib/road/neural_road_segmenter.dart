import 'dart:math' as math;
import 'dart:typed_data';

import '../ai/inference_backend.dart';
import '../ai/interfaces/road_segmenter.dart';
import '../ai/model_descriptor.dart';
import '../camera/camera_frame.dart';
import '../camera/image_preprocessing.dart';
import '../core/logging.dart';
import 'road_segmentation.dart';

/// Semantic segmenter backed by a neural network.
///
/// Output is `[1, H, W, C]` (or `[1, C, H, W]`) class logits. Decoding takes
/// the argmax per pixel and keeps the softmax probability of the winner as the
/// per-pixel confidence — the margin between the top two classes is what makes
/// a road/pavement boundary trustworthy or not, and discarding it would throw
/// away the most useful signal the model produces.
class NeuralRoadSegmenter extends RoadSegmenter {
  NeuralRoadSegmenter({
    required ModelDescriptor descriptor,
    required InferenceBackend backend,
  })  : _descriptor = descriptor,
        _backend = backend;

  static const String _tag = 'NeuralRoadSegmenter';

  final ModelDescriptor _descriptor;
  final InferenceBackend _backend;

  int? _handle;
  String? _unavailableReason;

  /// Maps the model's own class indices onto [SurfaceClass]. Built from the
  /// descriptor's label list, so a binary road/not-road model and a full
  /// Cityscapes model both work unchanged.
  late final List<SurfaceClass> _classMap = _buildClassMap();

  List<SurfaceClass> _buildClassMap() {
    if (_descriptor.labels.isEmpty) {
      // A model with no labels is assumed binary: 0 = background, 1 = road.
      return const <SurfaceClass>[
        SurfaceClass.unknown,
        SurfaceClass.drivableRoad,
      ];
    }
    return <SurfaceClass>[
      for (final String label in _descriptor.labels)
        switch (label.toLowerCase().trim()) {
          'road' => SurfaceClass.road,
          'drivable road' || 'drivable' || 'direct' => SurfaceClass.drivableRoad,
          'sidewalk' || 'pavement' => SurfaceClass.sidewalk,
          'curb' || 'kerb' => SurfaceClass.curb,
          'grass' || 'terrain' || 'vegetation' => SurfaceClass.grass,
          'building' || 'wall' || 'fence' => SurfaceClass.building,
          'vehicle' || 'car' || 'truck' || 'bus' => SurfaceClass.vehicle,
          'pedestrian' || 'person' || 'rider' => SurfaceClass.pedestrian,
          'obstacle' || 'pole' || 'object' => SurfaceClass.obstacle,
          _ => SurfaceClass.unknown,
        },
    ];
  }

  @override
  String get modelId => _descriptor.id;

  @override
  String get displayName => _descriptor.name;

  @override
  ModelRole get role => ModelRole.roadSegmentation;

  @override
  ModelDescriptor? get descriptor => _descriptor;

  @override
  bool get isReady => _handle != null;

  @override
  String? get unavailableReason => _unavailableReason;

  @override
  List<SurfaceClass> get supportedClasses => _classMap.toSet().toList();

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
  Future<RoadSegmentation> segment(CameraFrame frame) async {
    final int? handle = _handle;
    if (handle == null) {
      return RoadSegmentation.unavailable(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: _unavailableReason ?? 'segmentation model not loaded',
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
        return RoadSegmentation.unavailable(
          frameId: frame.id,
          timestampMicros: frame.timestampMicros,
          reason: 'segmentation model produced no output',
        );
      }

      return _decode(out, frame);
    } catch (e, st) {
      Log.error(_tag, 'segmentation failed', e, st);
      return RoadSegmentation.unavailable(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: 'segmentation error: $e',
      );
    }
  }

  RoadSegmentation _decode(InferenceOutput out, CameraFrame frame) {
    final Float32List t = out.tensor(0);
    final List<int> shape = out.shape(0);
    final int classes = _classMap.length;

    final (int w, int h, bool channelsLast) = _resolveShape(shape, classes);
    if (w <= 0 || h <= 0) {
      return RoadSegmentation.unavailable(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: 'unrecognised segmentation output shape $shape',
      );
    }

    final Uint8List indices = Uint8List(w * h);
    final Float32List confidence = Float32List(w * h);
    final int plane = w * h;

    for (int p = 0; p < plane; p++) {
      double best = -double.infinity;
      double second = -double.infinity;
      int bestClass = 0;

      for (int c = 0; c < classes; c++) {
        final double v =
            channelsLast ? t[p * classes + c] : t[c * plane + p];
        if (v > best) {
          second = best;
          best = v;
          bestClass = c;
        } else if (v > second) {
          second = v;
        }
      }

      indices[p] = _classMap[bestClass].index;
      // Two-class softmax between the winner and the runner-up. This is the
      // decision margin, which is exactly what "how sure are you this is road
      // rather than pavement" means.
      final double margin = best - second;
      confidence[p] = margin.isFinite
          ? (1.0 / (1.0 + _expNeg(margin)))
          : 0.5;
    }

    return RoadSegmentation(
      width: w,
      height: h,
      classIndices: indices,
      classConfidence: confidence,
      frameId: frame.id,
      timestampMicros: frame.timestampMicros,
      modelName: _descriptor.id,
    );
  }

  /// `exp(-x)` guarded against overflow for large decision margins.
  static double _expNeg(double x) {
    if (x > 30) return 0;
    if (x < -30) return double.maxFinite;
    return math.exp(-x);
  }

  /// Work out whether the output is NHWC or NCHW, and its spatial size.
  ///
  /// The descriptor can state the output size explicitly (which is what the
  /// catalog does); otherwise it is inferred, preferring the interpretation
  /// whose element count matches the tensor exactly.
  (int, int, bool) _resolveShape(List<int> shape, int classes) {
    final int declaredW = _descriptor.extraInt('outputWidth', 0);
    final int declaredH = _descriptor.extraInt('outputHeight', 0);
    if (declaredW > 0 && declaredH > 0) {
      final bool channelsLast = shape.isNotEmpty && shape.last == classes;
      return (declaredW, declaredH, channelsLast);
    }

    final List<int> dims = shape.where((int d) => d > 1).toList();
    if (dims.length < 3) return (0, 0, true);

    if (dims.last == classes) {
      return (dims[dims.length - 2], dims[dims.length - 3], true);
    }
    if (dims.first == classes) {
      return (dims[2], dims[1], false);
    }
    return (0, 0, true);
  }
}
