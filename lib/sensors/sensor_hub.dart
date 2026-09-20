import 'dart:async';

import '../core/logging.dart';
import '../core/time_sync.dart';
import 'ego_motion.dart';
import 'gps_service.dart';
import 'imu_service.dart';
import 'sensor_fusion.dart';

/// Single owner of the IMU, the GNSS receiver and the fusion filter.
///
/// Everything downstream asks the hub for `stateAt(frameTimestamp)` rather
/// than reading sensors directly, which is what keeps the pipeline
/// deterministic during replay: a replayed session swaps the hub's live inputs
/// for recorded ones and nothing else changes.
class SensorHub {
  SensorHub({MonotonicClock? clock})
      : clock = clock ?? MonotonicClock() {
    _imu = ImuService(clock: this.clock);
    _gps = GpsService(clock: this.clock);
    _estimator = EgoMotionEstimator(clock: this.clock);

    _imu.onSample = (Vec3 accel, Vec3 gyro, double? mag, int ts) {
      _estimator.onImu(
        rawAccelerationPhone: accel,
        angularRatePhone: gyro,
        magneticHeadingDegrees: mag,
        timestampMicros: ts,
      );
      _rawImu.add(ImuSample(
        accelerationMps2: accel,
        angularRateRadPerS: gyro,
        timestampMicros: ts,
        magneticHeadingDegrees: mag,
      ));
    };
  }

  static const String _tag = 'SensorHub';

  final MonotonicClock clock;
  late final ImuService _imu;
  late final GpsService _gps;
  late final EgoMotionEstimator _estimator;

  final StreamController<ImuSample> _rawImu =
      StreamController<ImuSample>.broadcast();

  StreamSubscription<GeoPosition>? _gpsSub;
  String? _gpsError;
  bool _running = false;

  EgoMotionEstimator get estimator => _estimator;
  ImuService get imu => _imu;
  GpsService get gps => _gps;

  /// Raw (phone-frame) IMU samples, for the recorder and the debug screen.
  Stream<ImuSample> get rawImuSamples => _rawImu.stream;

  Stream<GeoPosition> get gpsFixes => _gps.fixes;

  GeoPosition? get lastFix => _gps.lastFix;
  String? get gpsError => _gpsError;
  bool get isRunning => _running;

  /// Fused ego state aligned to [timestampMicros] (normally a camera frame's
  /// exposure timestamp).
  EgoMotionState stateAt(int timestampMicros) =>
      _estimator.stateAt(timestampMicros);

  EgoMotionState get currentState => _estimator.stateAt(clock.micros);

  Future<void> start() async {
    if (_running) return;
    _running = true;
    await _imu.start();
    try {
      await _gps.start();
      _gpsError = null;
      _gpsSub = _gps.fixes.listen(_estimator.onGpsFix);
    } catch (e) {
      // The perception stack is designed to work without GNSS: speed
      // confidence drops and navigation is unavailable, but object detection,
      // lanes, depth, planning and simulation all continue.
      _gpsError = '$e';
      Log.warn(_tag, 'continuing without GNSS: $e');
    }
  }

  Future<void> stop() async {
    _running = false;
    await _gpsSub?.cancel();
    _gpsSub = null;
    await _imu.stop();
    await _gps.stop();
  }

  Future<void> dispose() async {
    await stop();
    await _gps.dispose();
    await _rawImu.close();
  }
}
