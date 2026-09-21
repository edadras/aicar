import 'dart:typed_data';

import '../ai/inference_backend.dart';
import '../ai/interfaces/object_detector.dart';
import '../ai/model_descriptor.dart';
import '../camera/camera_frame.dart';
import '../camera/image_preprocessing.dart';
import '../core/logging.dart';
import 'detection.dart';
import 'nms.dart';
import 'object_class.dart';
import 'yolo_decoder.dart';

/// Object detector backed by a neural network running on an
/// [InferenceBackend].
///
/// Everything architecture-specific lives in the [ModelDescriptor] and
/// [YoloDecoder], so installing a different detector is a matter of dropping
/// in new weights plus a descriptor — no code change.
class NeuralObjectDetector extends ObjectDetector {
  NeuralObjectDetector({
    required ModelDescriptor descriptor,
    required InferenceBackend backend,
  })  : _descriptor = descriptor,
        _backend = backend;

  static const String _tag = 'NeuralObjectDetector';

  final ModelDescriptor _descriptor;
  final InferenceBackend _backend;

  int? _handle;
  String? _unavailableReason;
  late final ObjectClassMapping _mapping = YoloDecoder.mappingFor(_descriptor);

  /// Reused across frames so a steady-state pipeline allocates nothing per
  /// frame for the input tensor.
  Float32List? _inputBuffer;

  @override
  String get modelId => _descriptor.id;

  @override
  String get displayName => _descriptor.name;

  @override
  ModelRole get role => ModelRole.objectDetection;

  @override
  ModelDescriptor? get descriptor => _descriptor;

  @override
  bool get isReady => _handle != null;

  @override
  String? get unavailableReason => _unavailableReason;

  @override
  double get scoreThreshold => _descriptor.scoreThreshold;

  @override
  List<String> get supportedLabels => _descriptor.labels;

  @override
  Future<void> load() async {
    try {
      if (!await _backend.isAvailable()) {
        _unavailableReason = 'inference backend unavailable';
        return;
      }
      if (_descriptor.labels.isEmpty) {
        _unavailableReason = 'model has no label list';
        return;
      }
      _handle = await _backend.loadModel(_descriptor);
      _unavailableReason = null;
      Log.info(_tag, 'loaded ${_descriptor.id} '
          '(${_descriptor.inputWidth}x${_descriptor.inputHeight}, '
          '${_descriptor.labels.length} classes)');
    } catch (e) {
      _unavailableReason = '$e';
      Log.error(_tag, 'load failed for ${_descriptor.id}', e);
    }
  }

  @override
  Future<DetectionResult> detect(CameraFrame frame) async {
    final int? handle = _handle;
    if (handle == null) {
      return DetectionResult.noModel(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: _unavailableReason ?? 'detector not loaded',
      );
    }

    try {
      // Fit the frame to the input tensor the way this head was trained,
      // and keep the transform so the boxes can be mapped back exactly.
      LetterboxResult? lb;
      final Uint8List input;
      if (_descriptor.inputFit == InputFit.letterbox) {
        lb = ImagePreprocessing.letterbox(
          frame.bytes,
          frame.width,
          frame.height,
          _descriptor.inputWidth,
          channels: frame.bytesPerPixel,
        );
        input = lb.bytes;
      } else {
        input = ImagePreprocessing.resize(
          frame.bytes,
          frame.width,
          frame.height,
          _descriptor.inputWidth,
          _descriptor.inputHeight,
          channels: frame.bytesPerPixel,
        );
      }

      // The camera can hand us a grayscale frame; the input tensor's channel
      // count is fixed. Reconcile them here, where the mismatch is still
      // visible — the native runtime would just copy a short buffer into a
      // long tensor and leave the tail holding the previous frame.
      final Uint8List fitted = ImagePreprocessing.toChannels(
        input,
        frame.bytesPerPixel,
        _descriptor.inputChannels,
      );

      final InferenceOutput out;
      if (_descriptor.quantized) {
        // A quantised input tensor takes the raw bytes: the interpreter's own
        // scale and zero point turn them back into the range the network was
        // trained on, so normalising here would apply it twice.
        out = await _backend.runQuantized(handle, fitted);
      } else {
        _inputBuffer = ImagePreprocessing.toFloatTensor(
          fitted,
          _descriptor.inputWidth,
          _descriptor.inputHeight,
          channels: _descriptor.inputChannels,
          mean: _descriptor.inputMean,
          std: _descriptor.inputStd,
          channelsFirst: _descriptor.channelsFirst,
        );
        out = await _backend.run(handle, _inputBuffer!);
      }

      if (out.outputCount == 0) {
        return DetectionResult.noModel(
          frameId: frame.id,
          timestampMicros: frame.timestampMicros,
          reason: 'model produced no output tensors',
        );
      }

      List<Detection> detections = _decode(out, lb, frame);
      detections = sanitizeDetections(detections);
      detections = NonMaximumSuppression.filterImplausible(detections);
      detections = NonMaximumSuppression.softNms(
        detections,
        scoreThreshold: _descriptor.scoreThreshold * 0.6,
      );
      detections =
          NonMaximumSuppression.mergeCrossClassDuplicates(detections);

      return DetectionResult(
        detections: detections,
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        inferenceMicros: out.inferenceMicros,
        modelName: _descriptor.id,
        isSaturated: _isSaturated(out),
      );
    } catch (e, st) {
      // A failed inference must degrade the frame, never kill the pipeline.
      Log.error(_tag, 'inference failed on frame ${frame.id}', e, st);
      return DetectionResult.noModel(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: 'inference error: $e',
      );
    }
  }

  List<Detection> _decode(
    InferenceOutput out,
    LetterboxResult? lb,
    CameraFrame frame,
  ) {
    switch (_descriptor.outputFormat) {
      case ModelOutputFormat.yoloV8:
        if (lb == null) return const <Detection>[];
        return YoloDecoder.toDetections(
          raw: YoloDecoder.decodeV8(
            output: out.tensor(0),
            shape: out.shape(0),
            numClasses: _descriptor.labels.length,
            scoreThreshold: _descriptor.scoreThreshold,
          ),
          letterbox: lb,
          originalWidth: frame.width,
          originalHeight: frame.height,
          labels: _descriptor.labels,
          mapping: _mapping,
          frameId: frame.id,
          timestampMicros: frame.timestampMicros,
        );

      case ModelOutputFormat.yoloV5:
        if (lb == null) return const <Detection>[];
        return YoloDecoder.toDetections(
          raw: YoloDecoder.decodeV5(
            output: out.tensor(0),
            shape: out.shape(0),
            numClasses: _descriptor.labels.length,
            scoreThreshold: _descriptor.scoreThreshold,
          ),
          letterbox: lb,
          originalWidth: frame.width,
          originalHeight: frame.height,
          labels: _descriptor.labels,
          mapping: _mapping,
          frameId: frame.id,
          timestampMicros: frame.timestampMicros,
        );

      case ModelOutputFormat.ssdMobileNet:
        if (out.outputCount < 4) return const <Detection>[];
        return YoloDecoder.decodeSsd(
          boxes: out.tensor(0),
          classes: out.tensor(1),
          scores: out.tensor(2),
          count: out.tensor(3).isEmpty ? 0 : out.tensor(3)[0].round(),
          labels: _descriptor.labels,
          mapping: _mapping,
          scoreThreshold: _descriptor.scoreThreshold,
          frameId: frame.id,
          timestampMicros: frame.timestampMicros,
          letterbox: lb,
          originalWidth: frame.width,
          originalHeight: frame.height,
        );

      default:
        Log.warn(_tag,
            'output format ${_descriptor.outputFormat.name} is not a detector head');
        return const <Detection>[];
    }
  }

  /// Whether this frame exhausted the model's output capacity.
  ///
  /// Only fixed-slot heads can run out: a YOLO grid emits a box for every
  /// anchor, so its output is always as complete as the network is.
  bool _isSaturated(InferenceOutput out) {
    if (_descriptor.outputFormat != ModelOutputFormat.ssdMobileNet) return false;
    if (out.outputCount < 4) return false;
    return YoloDecoder.isSsdSaturated(
      scores: out.tensor(2),
      count: out.tensor(3).isEmpty ? 0 : out.tensor(3)[0].round(),
      scoreThreshold: _descriptor.scoreThreshold,
    );
  }

  @override
  Future<void> close() async {
    final int? h = _handle;
    _handle = null;
    _inputBuffer = null;
    if (h != null) await _backend.unload(h);
  }
}

/// The detector installed when no object-detection model is available.
///
/// It exists so that "we cannot see" is represented explicitly. Returning an
/// empty non-degraded result instead would tell the rest of the stack that the
/// road is clear, which is the single most dangerous lie a perception system
/// can tell.
class UnavailableObjectDetector extends ObjectDetector {
  UnavailableObjectDetector([
    this._reason = 'no object detection model installed',
  ]);

  final String _reason;

  @override
  String get modelId => 'none';

  @override
  String get displayName => 'No detector installed';

  @override
  ModelRole get role => ModelRole.objectDetection;

  @override
  ModelDescriptor? get descriptor => null;

  @override
  bool get isReady => false;

  @override
  String? get unavailableReason => _reason;

  @override
  double get scoreThreshold => 1.0;

  @override
  List<String> get supportedLabels => const <String>[];

  @override
  Future<void> load() async {}

  @override
  Future<DetectionResult> detect(CameraFrame frame) async =>
      DetectionResult.noModel(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: _reason,
      );

  @override
  Future<void> close() async {}
}
