import 'dart:async';
import 'dart:math' as math;

import 'package:sensors_plus/sensors_plus.dart';

import '../core/geometry.dart';
import '../core/logging.dart';
import '../core/time_sync.dart';
import 'ego_motion.dart';

/// Subscribes to the phone's inertial sensors and forwards them to an
/// [EgoMotionEstimator].
///
/// The **raw** accelerometer stream is used (gravity included) because
/// [PhoneToVehicleFrame] needs gravity to find "down"; Android's fused
/// linear-acceleration sensor throws exactly that information away.
class ImuService {
  ImuService({
    required MonotonicClock clock,
    this.samplingPeriod = SensorInterval.gameInterval,
  }) : _clock = clock;

  static const String _tag = 'ImuService';

  final MonotonicClock _clock;

  /// ~50 Hz. The UI interval (~15 Hz) is too slow to catch a hard brake, and
  /// the fastest interval burns battery for no gain at our pipeline rate.
  final Duration samplingPeriod;

  StreamSubscription<AccelerometerEvent>? _accelSub;
  StreamSubscription<GyroscopeEvent>? _gyroSub;
  StreamSubscription<MagnetometerEvent>? _magSub;

  Vec3 _lastGyro = const Vec3.zero();
  Vec3? _lastMagnetometer;
  double? _magneticHeadingDegrees;
  bool _running = false;
  int _sampleCount = 0;

  bool get isRunning => _running;
  int get sampleCount => _sampleCount;
  double? get magneticHeadingDegrees => _magneticHeadingDegrees;
  Vec3? get lastMagnetometer => _lastMagnetometer;

  /// Invoked on every accelerometer tick with the most recent gyroscope and
  /// magnetometer readings attached. The accelerometer paces the callback
  /// because gravity estimation is what frame resolution depends on.
  void Function(Vec3 accel, Vec3 gyro, double? magHeadingDeg, int tsMicros)?
      onSample;

  Future<void> start() async {
    if (_running) return;
    _running = true;

    _gyroSub = gyroscopeEventStream(samplingPeriod: samplingPeriod).listen(
      (GyroscopeEvent e) => _lastGyro = Vec3(e.x, e.y, e.z),
      onError: (Object e) => Log.warn(_tag, 'gyroscope error: $e'),
      cancelOnError: false,
    );

    _magSub =
        magnetometerEventStream(samplingPeriod: SensorInterval.uiInterval)
            .listen(
      (MagnetometerEvent e) => _lastMagnetometer = Vec3(e.x, e.y, e.z),
      onError: (Object e) => Log.warn(_tag, 'magnetometer error: $e'),
      cancelOnError: false,
    );

    _accelSub = accelerometerEventStream(samplingPeriod: samplingPeriod).listen(
      (AccelerometerEvent e) {
        _sampleCount++;
        final Vec3 accel = Vec3(e.x, e.y, e.z);
        final Vec3? mag = _lastMagnetometer;
        _magneticHeadingDegrees =
            mag == null ? null : tiltCompensatedHeading(accel: accel, mag: mag);
        onSample?.call(accel, _lastGyro, _magneticHeadingDegrees, _clock.micros);
      },
      onError: (Object e) => Log.warn(_tag, 'accelerometer error: $e'),
      cancelOnError: false,
    );

    Log.info(_tag, 'IMU streams started at ${samplingPeriod.inMilliseconds}ms');
  }

  /// Magnetic azimuth corrected for the phone's tilt.
  ///
  /// Without tilt compensation a cradle-mounted phone reports a heading that
  /// swings tens of degrees with pitch, which would drag the fused heading
  /// around every time the road changes gradient.
  ///
  /// Returns degrees clockwise from magnetic north, or `null` when the device
  /// is close to free-fall (gravity unusable).
  static double? tiltCompensatedHeading({
    required Vec3 accel,
    required Vec3 mag,
  }) {
    final double norm = accel.length;
    if (norm < 1.0) return null;

    // Accelerometer measures the reaction to gravity, so +accel is "up".
    final double ax = accel.x / norm;
    final double ay = accel.y / norm;
    final double az = accel.z / norm;

    // pitch/roll of the device relative to the horizontal plane.
    final double pitch = math.asin(-ay.clamp(-1.0, 1.0));
    final double roll = math.atan2(ax, az);

    final double cp = math.cos(pitch);
    final double sp = math.sin(pitch);
    final double cr = math.cos(roll);
    final double sr = math.sin(roll);

    final double xh = mag.x * cr - mag.z * sr;
    final double yh = mag.x * sp * sr + mag.y * cp + mag.z * sp * cr;

    double deg = radToDeg(math.atan2(-xh, yh));
    if (deg < 0) deg += 360;
    return deg;
  }

  Future<void> stop() async {
    _running = false;
    await _accelSub?.cancel();
    await _gyroSub?.cancel();
    await _magSub?.cancel();
    _accelSub = null;
    _gyroSub = null;
    _magSub = null;
    Log.info(_tag, 'IMU streams stopped');
  }
}
