import 'dart:async';

import 'camera_calibration.dart';
import 'camera_frame.dart';

/// Where frames come from. The live camera and the replay player both
/// implement this, which is what makes "re-run the AI over a recorded drive"
/// exercise exactly the same pipeline code as a live drive.
abstract class FrameSource {
  /// Human readable name for the debug overlay.
  String get name;

  /// Broadcast stream of frames, already preprocessed to the configured
  /// inference resolution and orientation-corrected.
  Stream<CameraFrame> get frames;

  /// Nominal capture rate, used to size buffers and to sanity-check the
  /// measured FPS.
  double get nominalFps;

  /// Calibration for the frames this source produces.
  CameraCalibration get calibration;

  bool get isRunning;

  Future<void> start();
  Future<void> stop();
  Future<void> dispose();
}

/// Frame source backed by a fixed in-memory list. Used by tests and by the
/// dataset tools; keeps the pipeline honest by never needing a real device.
class ListFrameSource implements FrameSource {
  ListFrameSource(this._frames, {required this.calibration, this.nominalFps = 30});

  final List<CameraFrame> _frames;

  @override
  final CameraCalibration calibration;

  @override
  final double nominalFps;

  final StreamController<CameraFrame> _controller =
      StreamController<CameraFrame>.broadcast();

  bool _running = false;

  @override
  String get name => 'ListFrameSource(${_frames.length} frames)';

  @override
  Stream<CameraFrame> get frames => _controller.stream;

  @override
  bool get isRunning => _running;

  @override
  Future<void> start() async {
    _running = true;
    for (final CameraFrame f in _frames) {
      if (!_running) break;
      _controller.add(f);
      await Future<void>.delayed(Duration.zero);
    }
  }

  @override
  Future<void> stop() async => _running = false;

  @override
  Future<void> dispose() async {
    _running = false;
    await _controller.close();
  }
}
