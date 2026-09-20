import 'dart:async';
import 'dart:isolate';
import 'dart:typed_data';
import 'dart:ui' show Size;

import 'package:camera/camera.dart';
import 'package:flutter/services.dart' show DeviceOrientation;

import '../core/logging.dart';
import '../core/time_sync.dart';
import 'camera_calibration.dart';
import 'camera_frame.dart';

/// Inference resolutions offered in Settings. Lower resolutions trade small
/// distant objects for frame rate; on a Galaxy S23 the 512-wide preset holds
/// ~20 FPS through the full stack while 768 holds ~12.
enum InferenceResolution {
  low(320, 192, 'Low 320x192'),
  medium(512, 288, 'Medium 512x288'),
  high(640, 384, 'High 640x384'),
  ultra(768, 448, 'Ultra 768x448');

  const InferenceResolution(this.width, this.height, this.label);
  final int width;
  final int height;
  final String label;
}

/// Capture-side configuration.
class CameraConfig {
  const CameraConfig({
    this.captureResolution = ResolutionPreset.high, // 1280x720
    this.inferenceResolution = InferenceResolution.medium,
    this.targetInferenceFps = 20,
    this.enableAudio = false,
    this.lockExposureWhileDriving = false,
  });

  final ResolutionPreset captureResolution;
  final InferenceResolution inferenceResolution;
  final double targetInferenceFps;
  final bool enableAudio;

  /// Auto-exposure hunting between sky and asphalt causes visible detection
  /// flicker. Locking AE after a few seconds of driving is a meaningful
  /// stability win, at the cost of blown highlights entering a tunnel.
  final bool lockExposureWhileDriving;

  CameraConfig copyWith({
    ResolutionPreset? captureResolution,
    InferenceResolution? inferenceResolution,
    double? targetInferenceFps,
    bool? enableAudio,
    bool? lockExposureWhileDriving,
  }) =>
      CameraConfig(
        captureResolution: captureResolution ?? this.captureResolution,
        inferenceResolution: inferenceResolution ?? this.inferenceResolution,
        targetInferenceFps: targetInferenceFps ?? this.targetInferenceFps,
        enableAudio: enableAudio ?? this.enableAudio,
        lockExposureWhileDriving:
            lockExposureWhileDriving ?? this.lockExposureWhileDriving,
      );
}

/// Owns the physical camera and emits raw [YuvFrame]s.
///
/// Deliberately does **no** pixel work: the conversion to RGB happens in the
/// perception isolate so that the platform's image-stream callback (which runs
/// on the Dart main isolate) returns immediately and the UI keeps its 60 FPS.
class CameraService {
  CameraService({MonotonicClock? clock, CameraConfig? config})
      : _clock = clock ?? MonotonicClock(),
        _config = config ?? const CameraConfig();

  static const String _tag = 'CameraService';

  final MonotonicClock _clock;
  CameraConfig _config;

  CameraController? _controller;
  CameraDescription? _description;
  final StreamController<YuvFrame> _frames =
      StreamController<YuvFrame>.broadcast();

  int _frameCounter = 0;
  bool _streaming = false;
  CameraCalibration _calibration = CameraCalibration.galaxyS23Default();

  Stream<YuvFrame> get frames => _frames.stream;
  CameraConfig get config => _config;
  CameraController? get controller => _controller;
  bool get isStreaming => _streaming;
  bool get isInitialized => _controller?.value.isInitialized ?? false;
  MonotonicClock get clock => _clock;

  /// Calibration matching the **capture** resolution. The pipeline rescales it
  /// to the inference resolution.
  CameraCalibration get calibration => _calibration;

  set calibration(CameraCalibration value) {
    final Size? s = _controller?.value.previewSize;
    _calibration = s == null
        ? value
        : value.scaledTo(s.height.toInt(), s.width.toInt());
  }

  /// Pick the rear camera. On a Galaxy S23 `availableCameras()` returns the
  /// ultrawide, the main and the telephoto as separate back-facing entries;
  /// the first back camera is the main sensor, which is the one with the FOV
  /// our calibration defaults assume.
  Future<CameraDescription?> selectRearCamera() async {
    final List<CameraDescription> cameras = await availableCameras();
    if (cameras.isEmpty) {
      Log.error(_tag, 'no cameras reported by the platform');
      return null;
    }
    for (final CameraDescription c in cameras) {
      if (c.lensDirection == CameraLensDirection.back) return c;
    }
    return cameras.first;
  }

  Future<void> initialize({CameraDescription? description}) async {
    await dispose();

    _description = description ?? await selectRearCamera();
    if (_description == null) {
      throw StateError('No usable camera found on this device');
    }

    final CameraController controller = CameraController(
      _description!,
      _config.captureResolution,
      enableAudio: _config.enableAudio,
      // YUV420 is what the sensor produces; asking for anything else forces a
      // conversion inside the plugin that we would immediately have to undo.
      imageFormatGroup: ImageFormatGroup.yuv420,
    );
    await controller.initialize();

    // The HUD is landscape-locked; pinning the capture orientation keeps the
    // frame geometry (and therefore the calibration) constant regardless of
    // how the phone reports its rotation in the cradle.
    await controller.lockCaptureOrientation(DeviceOrientation.landscapeLeft);
    await controller.setFocusMode(FocusMode.auto);
    await controller.setExposureMode(ExposureMode.auto);

    _controller = controller;

    final Size preview = controller.value.previewSize ?? const Size(1280, 720);
    // `previewSize` is reported in sensor orientation (portrait), so swap.
    final int w = preview.height.toInt();
    final int h = preview.width.toInt();
    _calibration = _calibration.scaledTo(w, h);

    Log.info(
      _tag,
      'initialised ${_description!.name} at ${w}x$h '
      '(sensor orientation ${_description!.sensorOrientation}°)',
    );
  }

  Future<void> startStream() async {
    final CameraController? c = _controller;
    if (c == null || !c.value.isInitialized) {
      throw StateError('CameraService.initialize() must be called first');
    }
    if (_streaming) return;

    await c.startImageStream(_onImage);
    _streaming = true;
    Log.info(_tag, 'image stream started');

    if (_config.lockExposureWhileDriving) {
      // Give AE a few seconds to settle on the road scene before locking.
      Timer(const Duration(seconds: 5), () async {
        try {
          await c.setExposureMode(ExposureMode.locked);
          Log.info(_tag, 'exposure locked');
        } catch (e) {
          Log.warn(_tag, 'exposure lock unsupported: $e');
        }
      });
    }
  }

  Future<void> stopStream() async {
    if (!_streaming) return;
    try {
      await _controller?.stopImageStream();
    } catch (e) {
      Log.warn(_tag, 'stopImageStream failed: $e');
    }
    _streaming = false;
  }

  void _onImage(CameraImage image) {
    if (_frames.isClosed) return;
    // Timestamp as early as possible. CameraX does not surface the sensor
    // exposure timestamp through the Flutter plugin, so this is delivery time
    // minus nothing — the pipeline records it as such and the ego-motion
    // aligner treats it as having a fixed unknown bias, not as exact.
    final int ts = _clock.micros;

    if (image.planes.length < 3) {
      Log.warn(_tag, 'unexpected plane count ${image.planes.length}');
      return;
    }

    _frames.add(
      YuvFrame(
        id: _frameCounter++,
        timestampMicros: ts,
        width: image.width,
        height: image.height,
        yPlane: image.planes[0].bytes,
        uPlane: image.planes[1].bytes,
        vPlane: image.planes[2].bytes,
        yRowStride: image.planes[0].bytesPerRow,
        uvRowStride: image.planes[1].bytesPerRow,
        uvPixelStride: image.planes[1].bytesPerPixel ?? 1,
        sensorRotationDegrees: _description?.sensorOrientation ?? 0,
      ),
    );
  }

  Future<void> applyConfig(CameraConfig next) async {
    final bool needsRestart =
        next.captureResolution != _config.captureResolution;
    _config = next;
    if (needsRestart && _controller != null) {
      final bool wasStreaming = _streaming;
      await initialize(description: _description);
      if (wasStreaming) await startStream();
    }
  }

  Future<void> dispose() async {
    await stopStream();
    await _controller?.dispose();
    _controller = null;
  }

  Future<void> close() async {
    await dispose();
    await _frames.close();
  }
}

/// Copies a [YuvFrame]'s planes into one contiguous buffer so it can cross an
/// isolate boundary as a single [TransferableTypedData] with no per-plane
/// copies on the receiving side.
class YuvTransfer {
  const YuvTransfer._();

  static (TransferableTypedData, Map<String, int>) pack(YuvFrame f) {
    final int total = f.yPlane.length + f.uPlane.length + f.vPlane.length;
    final Uint8List buf = Uint8List(total);
    buf.setRange(0, f.yPlane.length, f.yPlane);
    buf.setRange(f.yPlane.length, f.yPlane.length + f.uPlane.length, f.uPlane);
    buf.setRange(f.yPlane.length + f.uPlane.length, total, f.vPlane);
    return (
      TransferableTypedData.fromList(<Uint8List>[buf]),
      <String, int>{
        'id': f.id,
        'ts': f.timestampMicros,
        'w': f.width,
        'h': f.height,
        'ySize': f.yPlane.length,
        'uSize': f.uPlane.length,
        'vSize': f.vPlane.length,
        'yStride': f.yRowStride,
        'uvStride': f.uvRowStride,
        'uvPixel': f.uvPixelStride,
        'rotation': f.sensorRotationDegrees,
      },
    );
  }

  static YuvFrame unpack(TransferableTypedData data, Map<String, int> meta) {
    final Uint8List buf = data.materialize().asUint8List();
    final int ySize = meta['ySize']!;
    final int uSize = meta['uSize']!;
    final int vSize = meta['vSize']!;
    return YuvFrame(
      id: meta['id']!,
      timestampMicros: meta['ts']!,
      width: meta['w']!,
      height: meta['h']!,
      yPlane: Uint8List.sublistView(buf, 0, ySize),
      uPlane: Uint8List.sublistView(buf, ySize, ySize + uSize),
      vPlane: Uint8List.sublistView(buf, ySize + uSize, ySize + uSize + vSize),
      yRowStride: meta['yStride']!,
      uvRowStride: meta['uvStride']!,
      uvPixelStride: meta['uvPixel']!,
      sensorRotationDegrees: meta['rotation']!,
    );
  }
}
