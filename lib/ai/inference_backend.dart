import 'dart:async';
import 'dart:typed_data';

import 'package:flutter/services.dart';

import '../core/logging.dart';
import 'model_descriptor.dart';

/// One model's output tensors, keyed by index.
class InferenceOutput {
  const InferenceOutput({
    required this.tensors,
    required this.shapes,
    required this.inferenceMicros,
  });

  final List<Float32List> tensors;
  final List<List<int>> shapes;
  final int inferenceMicros;

  Float32List tensor(int i) => tensors[i];
  List<int> shape(int i) => shapes[i];
  int get outputCount => tensors.length;

  static const InferenceOutput empty = InferenceOutput(
    tensors: <Float32List>[],
    shapes: <List<int>>[],
    inferenceMicros: 0,
  );
}

/// Abstract neural-network runtime.
///
/// Keeping the runtime behind an interface is what makes the model-swap
/// requirement real: a TFLite backend, an ONNX/NNAPI backend or a pure-Dart
/// stub all satisfy it, and nothing above this line knows the difference.
abstract class InferenceBackend {
  String get name;

  /// Whether this backend can run at all on the current device.
  Future<bool> isAvailable();

  /// Load a model and return an opaque handle.
  Future<int> loadModel(ModelDescriptor descriptor);

  /// Run inference on a prepared float tensor.
  Future<InferenceOutput> run(int handle, Float32List input);

  /// Run inference on a prepared uint8 tensor (quantised models).
  Future<InferenceOutput> runQuantized(int handle, Uint8List input);

  Future<void> unload(int handle);

  Future<void> dispose();
}

/// Talks to the Kotlin `TfLiteRunner` over a method channel.
///
/// Inference itself must not cross the channel as JSON — the input tensor for
/// a 640x640 detector is 4.9 MB of floats. Flutter's standard message codec
/// passes `Float32List`/`Uint8List` as raw byte buffers with no per-element
/// boxing, which is exactly the fast path we want, and the Kotlin side writes
/// directly into the interpreter's direct `ByteBuffer`.
class TfLiteBackend implements InferenceBackend {
  TfLiteBackend({MethodChannel? channel})
      : _channel = channel ?? const MethodChannel(channelName);

  static const String channelName = 'com.aicar/inference';
  static const String _tag = 'TfLiteBackend';

  final MethodChannel _channel;
  bool? _available;

  @override
  String get name => 'TensorFlow Lite (native)';

  @override
  Future<bool> isAvailable() async {
    if (_available != null) return _available!;
    try {
      final bool? ok = await _channel.invokeMethod<bool>('isAvailable');
      _available = ok ?? false;
    } on MissingPluginException {
      // Desktop/test host: the native side simply is not there.
      _available = false;
    } catch (e) {
      Log.warn(_tag, 'availability probe failed: $e');
      _available = false;
    }
    return _available!;
  }

  /// Which delegates the device actually accepted, for the AI Models screen.
  Future<List<String>> supportedDelegates() async {
    try {
      final List<Object?>? list =
          await _channel.invokeMethod<List<Object?>>('supportedDelegates');
      return list?.map((Object? e) => '$e').toList() ?? const <String>[];
    } catch (_) {
      return const <String>[];
    }
  }

  @override
  Future<int> loadModel(ModelDescriptor descriptor) async {
    final int? handle = await _channel.invokeMethod<int>('loadModel', <String, dynamic>{
      'path': descriptor.assetOrFilePath,
      'isAsset': descriptor.isBundledAsset,
      'delegate': descriptor.delegate.name,
      'numThreads': descriptor.numThreads,
      'quantized': descriptor.quantized,
    });
    if (handle == null || handle < 0) {
      throw StateError('Failed to load model ${descriptor.id}');
    }
    Log.info(_tag, 'loaded ${descriptor.id} as handle $handle');
    return handle;
  }

  @override
  Future<InferenceOutput> run(int handle, Float32List input) async {
    final Map<Object?, Object?>? res =
        await _channel.invokeMethod<Map<Object?, Object?>>(
      'run',
      <String, dynamic>{'handle': handle, 'input': input},
    );
    return _decode(res);
  }

  @override
  Future<InferenceOutput> runQuantized(int handle, Uint8List input) async {
    final Map<Object?, Object?>? res =
        await _channel.invokeMethod<Map<Object?, Object?>>(
      'runQuantized',
      <String, dynamic>{'handle': handle, 'input': input},
    );
    return _decode(res);
  }

  static InferenceOutput _decode(Map<Object?, Object?>? res) {
    if (res == null) return InferenceOutput.empty;
    final List<Object?> raw = (res['outputs'] as List<Object?>?) ?? <Object?>[];
    final List<Object?> rawShapes =
        (res['shapes'] as List<Object?>?) ?? <Object?>[];
    return InferenceOutput(
      tensors: <Float32List>[
        for (final Object? t in raw)
          t is Float32List ? t : Float32List.fromList(<double>[]),
      ],
      shapes: <List<int>>[
        for (final Object? s in rawShapes)
          <int>[for (final Object? d in (s as List<Object?>)) (d as num).toInt()],
      ],
      inferenceMicros: (res['micros'] as num?)?.toInt() ?? 0,
    );
  }

  @override
  Future<void> unload(int handle) async {
    try {
      await _channel.invokeMethod<void>('unload', <String, dynamic>{'handle': handle});
    } catch (e) {
      Log.warn(_tag, 'unload($handle) failed: $e');
    }
  }

  @override
  Future<void> dispose() async {
    try {
      await _channel.invokeMethod<void>('disposeAll');
    } catch (_) {
      // Nothing to clean up if the native side never came up.
    }
  }
}

/// Backend that reports itself unavailable. Installed when the device has no
/// usable native runtime so that callers take the "no model" path explicitly
/// rather than crashing at the first inference.
class UnavailableBackend implements InferenceBackend {
  const UnavailableBackend([this.reason = 'no native inference runtime']);

  final String reason;

  @override
  String get name => 'unavailable ($reason)';

  @override
  Future<bool> isAvailable() async => false;

  @override
  Future<int> loadModel(ModelDescriptor descriptor) async =>
      throw StateError('Inference backend unavailable: $reason');

  @override
  Future<InferenceOutput> run(int handle, Float32List input) async =>
      throw StateError('Inference backend unavailable: $reason');

  @override
  Future<InferenceOutput> runQuantized(int handle, Uint8List input) async =>
      throw StateError('Inference backend unavailable: $reason');

  @override
  Future<void> unload(int handle) async {}

  @override
  Future<void> dispose() async {}
}
