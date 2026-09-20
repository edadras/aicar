import '../core/geometry.dart';
import '../core/logging.dart';
import 'camera_frame.dart';

/// Decides which camera frames the perception pipeline is allowed to consume.
///
/// The camera runs at 30 FPS and must never block. Inference is slower than
/// that, so frames have to be dropped — the question is only *which* ones.
/// This scheduler implements "latest wins with a target interval":
///
///  * If the pipeline is busy, the incoming frame replaces any frame already
///    waiting, so the stack always works on the freshest view of the road.
///    A stale frame is worse than no frame when you are computing TTC.
///  * A target interval throttles the pipeline below the camera rate when the
///    user asks for a lower inference FPS to save battery or heat.
///
/// The counters it keeps feed the FPS and drop-rate readouts in the debug
/// overlay.
class FrameScheduler {
  FrameScheduler({
    double targetFps = 20,
    this.maxQueueDepth = 1,
  })  : assert(maxQueueDepth >= 1),
        _targetIntervalMicros = targetFps <= 0 ? 0 : (1e6 ~/ targetFps);

  int _targetIntervalMicros;
  final int maxQueueDepth;

  CameraFrame? _pending;
  bool _busy = false;
  int _lastAcceptedMicros = -1;

  int _accepted = 0;
  int _droppedBusy = 0;
  int _droppedThrottle = 0;
  int _replaced = 0;

  int get accepted => _accepted;
  int get droppedBusy => _droppedBusy;
  int get droppedThrottle => _droppedThrottle;
  int get replaced => _replaced;
  int get totalDropped => _droppedBusy + _droppedThrottle + _replaced;
  bool get isBusy => _busy;
  bool get hasPending => _pending != null;

  double get targetFps =>
      _targetIntervalMicros == 0 ? 0 : 1e6 / _targetIntervalMicros;

  set targetFps(double fps) {
    _targetIntervalMicros = fps <= 0 ? 0 : (1e6 ~/ clampDouble(fps, 1, 60));
    Log.info('FrameScheduler',
        'inference target set to ${targetFps.toStringAsFixed(1)} FPS');
  }

  /// Offer a freshly captured frame. Returns the frame the pipeline should
  /// start on right now, or `null` if the frame was queued or dropped.
  CameraFrame? offer(CameraFrame frame) {
    if (_targetIntervalMicros > 0 &&
        _lastAcceptedMicros >= 0 &&
        frame.timestampMicros - _lastAcceptedMicros < _targetIntervalMicros) {
      _droppedThrottle++;
      return null;
    }

    if (_busy) {
      if (_pending != null) _replaced++;
      _pending = frame; // latest wins
      return null;
    }

    _busy = true;
    _lastAcceptedMicros = frame.timestampMicros;
    _accepted++;
    return frame;
  }

  /// Signal that the pipeline finished. Returns the next frame to process
  /// immediately, if one is waiting.
  CameraFrame? complete() {
    _busy = false;
    final CameraFrame? next = _pending;
    _pending = null;
    if (next == null) return null;

    if (_targetIntervalMicros > 0 &&
        _lastAcceptedMicros >= 0 &&
        next.timestampMicros - _lastAcceptedMicros < _targetIntervalMicros) {
      _droppedThrottle++;
      return null;
    }
    _busy = true;
    _lastAcceptedMicros = next.timestampMicros;
    _accepted++;
    return next;
  }

  /// Abandon the in-flight frame after an error so the pipeline does not wedge.
  void abort() {
    _busy = false;
    if (_pending != null) {
      _droppedBusy++;
      _pending = null;
    }
  }

  void reset() {
    _pending = null;
    _busy = false;
    _lastAcceptedMicros = -1;
    _accepted = 0;
    _droppedBusy = 0;
    _droppedThrottle = 0;
    _replaced = 0;
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'targetFps': double.parse(targetFps.toStringAsFixed(1)),
        'accepted': _accepted,
        'droppedBusy': _droppedBusy,
        'droppedThrottle': _droppedThrottle,
        'replaced': _replaced,
      };
}
