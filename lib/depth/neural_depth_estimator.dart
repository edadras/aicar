import 'dart:typed_data';

import '../ai/inference_backend.dart';
import '../ai/interfaces/depth_estimator.dart';
import '../ai/model_descriptor.dart';
import '../camera/camera_frame.dart';
import '../camera/image_preprocessing.dart';
import '../core/logging.dart';
import 'depth_map.dart';

/// Monocular depth network wrapper (MiDaS, Depth Anything, or any model whose
/// single output is a dense depth or inverse-depth image).
///
/// The output is exposed as [DepthScale.relativeInverse] unless the descriptor
/// declares metric depth. That distinction is load-bearing: a relative map
/// carries no metres at all until [DepthFusion] fits it against ground-plane
/// anchors, and [DepthMap.distanceAt] refuses to answer before it has been.
class NeuralDepthEstimator extends DepthEstimator {
  NeuralDepthEstimator({
    required ModelDescriptor descriptor,
    required InferenceBackend backend,
  })  : _descriptor = descriptor,
        _backend = backend;

  static const String _tag = 'NeuralDepthEstimator';

  final ModelDescriptor _descriptor;
  final InferenceBackend _backend;

  int? _handle;
  String? _unavailableReason;

  @override
  String get modelId => _descriptor.id;

  @override
  String get displayName => _descriptor.name;

  @override
  ModelRole get role => ModelRole.depthEstimation;

  @override
  ModelDescriptor? get descriptor => _descriptor;

  @override
  bool get isReady => _handle != null;

  @override
  String? get unavailableReason => _unavailableReason;

  @override
  DepthScale get outputScale =>
      _descriptor.outputFormat == ModelOutputFormat.metricDepthMap
          ? DepthScale.metric
          : DepthScale.relativeInverse;

  @override
  (int, int) get outputSize => (
        _descriptor.extraInt('outputWidth', _descriptor.inputWidth),
        _descriptor.extraInt('outputHeight', _descriptor.inputHeight),
      );

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
  Future<DepthMap> estimate(CameraFrame frame) async {
    final int? handle = _handle;
    if (handle == null) {
      return DepthMap.unavailable(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
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
      if (out.outputCount == 0 || out.tensor(0).isEmpty) {
        return DepthMap.unavailable(
          frameId: frame.id,
          timestampMicros: frame.timestampMicros,
        );
      }

      final (int w, int h) = _resolveOutputSize(out.shape(0), out.tensor(0).length);
      if (w <= 0 || h <= 0) {
        Log.warn(_tag, 'unrecognised depth output shape ${out.shape(0)}');
        return DepthMap.unavailable(
          frameId: frame.id,
          timestampMicros: frame.timestampMicros,
        );
      }

      return DepthMap(
        width: w,
        height: h,
        values: Float32List.sublistView(out.tensor(0), 0, w * h),
        scale: outputScale,
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        // A metric model is usable immediately; a relative one is not usable
        // until DepthFusion has fitted it, and says so via isFitted.
        globalConfidence: outputScale == DepthScale.metric ? 0.6 : 0.0,
        metricGain: _descriptor.extraDouble('depthScale', 1.0),
        metricBias: _descriptor.extraDouble('depthOffset', 0.0),
        isFitted: outputScale == DepthScale.metric,
      );
    } catch (e, st) {
      Log.error(_tag, 'depth inference failed', e, st);
      return DepthMap.unavailable(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
      );
    }
  }

  (int, int) _resolveOutputSize(List<int> shape, int elementCount) {
    final int declaredW = _descriptor.extraInt('outputWidth', 0);
    final int declaredH = _descriptor.extraInt('outputHeight', 0);
    if (declaredW > 0 && declaredH > 0 && declaredW * declaredH <= elementCount) {
      return (declaredW, declaredH);
    }

    final List<int> dims = shape.where((int d) => d > 1).toList();
    if (dims.length >= 2) {
      final int w = dims.last;
      final int h = dims[dims.length - 2];
      if (w * h <= elementCount) return (w, h);
    }
    if (dims.length == 1 && dims.first == elementCount) {
      return (_descriptor.inputWidth, _descriptor.inputHeight);
    }
    return (0, 0);
  }
}

/// Installed when no depth model is available.
///
/// Every query returns `null`, which forces distance estimation onto its
/// geometric cues — a degradation the fusion layer represents explicitly in
/// its confidence rather than hiding.
class UnavailableDepthEstimator extends DepthEstimator {
  UnavailableDepthEstimator([
    this._reason = 'no depth estimation model installed',
  ]);

  final String _reason;

  @override
  String get modelId => 'none';

  @override
  String get displayName => 'No depth model installed';

  @override
  ModelRole get role => ModelRole.depthEstimation;

  @override
  ModelDescriptor? get descriptor => null;

  @override
  bool get isReady => false;

  @override
  String? get unavailableReason => _reason;

  @override
  DepthScale get outputScale => DepthScale.relativeInverse;

  @override
  (int, int) get outputSize => (0, 0);

  @override
  Future<void> load() async {}

  @override
  Future<void> close() async {}

  @override
  Future<DepthMap> estimate(CameraFrame frame) async => DepthMap.unavailable(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
      );
}
