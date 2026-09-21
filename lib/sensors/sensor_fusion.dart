import 'dart:math' as math;

import '../core/geometry.dart';
import '../core/kalman.dart';
import '../core/logging.dart';
import '../core/ring_buffer.dart';
import '../core/time_sync.dart';
import 'ego_motion.dart';

/// Resolves the rigid rotation between the **phone** sensor frame and the
/// **vehicle** frame.
///
/// A phone in a dashboard cradle sits at an arbitrary roll/pitch/yaw, so raw
/// accelerometer and gyroscope axes mean nothing on their own. Two references
/// pin the rotation down:
///
///  * **Gravity** gives "down" exactly, from a low-passed accelerometer. This
///    is reliable within a degree or two even while driving, because braking
///    and cornering accelerations average out over a few seconds.
///  * **Forward** is seeded from the mounting hint (the camera's optical axis
///    is, by construction, roughly the direction of travel) and then refined
///    by correlating the horizontal acceleration with GPS speed changes:
///    when the car accelerates, the horizontal acceleration vector *is*
///    forward.
class PhoneToVehicleFrame {
  PhoneToVehicleFrame({
    Vec3 forwardHintPhoneFrame = const Vec3(0, 0, -1),
    this.gravityTimeConstantSeconds = 2.0,
  }) : _forwardHint = forwardHintPhoneFrame;

  /// The camera looks out of the back of the phone, so `-z` in the Android
  /// sensor frame is "forward" for a cradle-mounted device.
  Vec3 _forwardHint;

  final double gravityTimeConstantSeconds;

  Vec3? _gravity;
  Vec3 _up = const Vec3(0, 1, 0);
  Vec3 _forward = const Vec3(0, 0, -1);
  Vec3 _right = const Vec3(1, 0, 0);
  bool _hasOrientation = false;
  double _forwardRefinementWeight = 0;

  bool get isResolved => _hasOrientation;

  /// How well the frame is pinned down, 0..1. Used to discount IMU-derived
  /// quantities until the estimate has settled.
  double get confidence {
    if (!_hasOrientation) return 0;
    return clampDouble(0.55 + 0.45 * _forwardRefinementWeight, 0, 1);
  }

  Vec3 get up => _up;
  Vec3 get forward => _forward;
  Vec3 get right => _right;
  Vec3? get gravity => _gravity;

  /// Feed a raw accelerometer sample **including** gravity.
  void updateGravity(Vec3 rawAccel, double dtSeconds) {
    if (_gravity == null) {
      _gravity = rawAccel;
    } else {
      final double alpha = dtSeconds <= 0
          ? 0.02
          : 1 - math.exp(-dtSeconds / gravityTimeConstantSeconds);
      _gravity = Vec3(
        _gravity!.x + alpha * (rawAccel.x - _gravity!.x),
        _gravity!.y + alpha * (rawAccel.y - _gravity!.y),
        _gravity!.z + alpha * (rawAccel.z - _gravity!.z),
      );
    }
    _rebuild();
  }

  /// Refine "forward" using a known longitudinal acceleration event.
  ///
  /// [horizontalAccel] is the gravity-free acceleration projected onto the
  /// horizontal plane; [referenceLongitudinalAccel] is the same quantity
  /// derived independently from GPS speed. When the car is genuinely speeding
  /// up or slowing down, the two must point the same way.
  void refineForward(Vec3 horizontalAccel, double referenceLongitudinalAccel) {
    if (referenceLongitudinalAccel.abs() < 0.6) return; // too weak to trust
    final double mag = horizontalAccel.length;
    if (mag < 0.4) return;

    final Vec3 observed = referenceLongitudinalAccel > 0
        ? horizontalAccel * (1 / mag)
        : horizontalAccel * (-1 / mag);

    // Slow blend: a single bump in the road should not rotate the frame.
    const double blend = 0.05;
    final Vec3 mixed = Vec3(
      _forward.x + blend * (observed.x - _forward.x),
      _forward.y + blend * (observed.y - _forward.y),
      _forward.z + blend * (observed.z - _forward.z),
    );
    _forwardHint = mixed;
    _forwardRefinementWeight =
        clampDouble(_forwardRefinementWeight + 0.02, 0, 1);
    _rebuild();
  }

  void _rebuild() {
    final Vec3? g = _gravity;
    if (g == null || g.length < 1e-3) return;

    final double gl = g.length;
    // Gravity read by an accelerometer points *up* in sensor terms (the device
    // measures the normal force), so "up" is +g normalised.
    _up = Vec3(g.x / gl, g.y / gl, g.z / gl);

    final double proj = _forwardHint.x * _up.x +
        _forwardHint.y * _up.y +
        _forwardHint.z * _up.z;
    Vec3 fwd = Vec3(
      _forwardHint.x - proj * _up.x,
      _forwardHint.y - proj * _up.y,
      _forwardHint.z - proj * _up.z,
    );
    final double fl = fwd.length;
    if (fl < 1e-4) return; // hint is parallel to gravity: unusable
    fwd = Vec3(fwd.x / fl, fwd.y / fl, fwd.z / fl);
    _forward = fwd;
    // Right-handed with x=right, y=forward, z=up  =>  right = forward x up.
    _right = _cross(fwd, _up);
    _hasOrientation = true;
  }

  static Vec3 _cross(Vec3 a, Vec3 b) => Vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
      );

  /// Rotate a phone-frame vector into the vehicle frame
  /// (x right, y forward, z up).
  Vec3 toVehicle(Vec3 phoneFrame) => Vec3(
        phoneFrame.x * _right.x + phoneFrame.y * _right.y + phoneFrame.z * _right.z,
        phoneFrame.x * _forward.x +
            phoneFrame.y * _forward.y +
            phoneFrame.z * _forward.z,
        phoneFrame.x * _up.x + phoneFrame.y * _up.y + phoneFrame.z * _up.z,
      );

  /// Remove gravity from a raw accelerometer reading, in the vehicle frame.
  Vec3 linearAccelerationVehicle(Vec3 rawAccelPhone) {
    final Vec3 g = _gravity ?? const Vec3.zero();
    return toVehicle(Vec3(
      rawAccelPhone.x - g.x,
      rawAccelPhone.y - g.y,
      rawAccelPhone.z - g.z,
    ));
  }
}

/// Fuses GPS, IMU and (optionally) visual odometry into an [EgoMotionState].
///
/// Speed uses a 2-state Kalman filter (speed, longitudinal acceleration):
/// the IMU drives the prediction at ~100 Hz and GPS corrects it at ~1 Hz. That
/// combination gives both the low latency the planner needs and the absolute
/// scale that only GNSS provides.
///
/// Heading uses a complementary filter: the gyroscope integrates smoothly but
/// drifts; GPS course is drift-free but only meaningful above walking pace and
/// is noisy; the magnetometer is drift-free but is wrecked by the car's own
/// steel and electronics. Weights reflect exactly that ordering.
class EgoMotionEstimator {
  EgoMotionEstimator({
    MonotonicClock? clock,
    PhoneToVehicleFrame? frame,
  })  : _clock = clock ?? MonotonicClock(),
        _frame = frame ?? PhoneToVehicleFrame() {
    _speedFilter = KalmanFilter(
      stateSize: 2,
      measurementSize: 1,
      initialState: Matrix.fromList(2, 1, <double>[0, 0]),
      initialCovariance: Matrix.diagonal(<double>[25, 4]),
    );
  }

  static const String _tag = 'EgoMotion';

  /// Below this speed GPS course is meaningless and heading is held.
  static const double _gpsHeadingMinSpeed = 2.0;

  /// Stationary detection: speed and |accel| both below these for a while.
  static const double _stationarySpeedThreshold = 0.35;

  final MonotonicClock _clock;
  final PhoneToVehicleFrame _frame;
  late final KalmanFilter _speedFilter;

  final TimeAlignedBuffer<ImuSample> _imuBuffer = TimeAlignedBuffer<ImuSample>(
    capacity: 256,
    interpolate: ImuSample.lerp,
  );

  final RingBuffer<double> _yawRateWindow = RingBuffer<double>(16);

  double _headingDegrees = 0;
  double _headingConfidence = 0;
  bool _headingInitialised = false;

  GeoPosition? _lastFix;
  GeoPosition? _currentFix;
  int _lastImuMicros = -1;
  int _lastPredictMicros = -1;
  double _yawRate = 0;
  double _lateralAccel = 0;
  double _longitudinalAccel = 0;
  int _stationarySinceMicros = -1;

  PhoneToVehicleFrame get frame => _frame;
  TimeAlignedBuffer<ImuSample> get imuHistory => _imuBuffer;

  /// Feed a raw accelerometer + gyroscope sample in the **phone** frame.
  void onImu({
    required Vec3 rawAccelerationPhone,
    required Vec3 angularRatePhone,
    double? magneticHeadingDegrees,
    int? timestampMicros,
  }) {
    final int ts = timestampMicros ?? _clock.micros;
    final double dt =
        _lastImuMicros < 0 ? 0.01 : (ts - _lastImuMicros) / 1e6;
    _lastImuMicros = ts;
    if (dt <= 0 || dt > 0.5) {
      // Sensor hiccup or app resume; resync without integrating a huge step.
      _frame.updateGravity(rawAccelerationPhone, 0.01);
      return;
    }

    _frame.updateGravity(rawAccelerationPhone, dt);

    final Vec3 linAccel =
        _frame.linearAccelerationVehicle(rawAccelerationPhone);
    final Vec3 gyroVehicle = _frame.toVehicle(angularRatePhone);

    _longitudinalAccel = linAccel.y;
    _lateralAccel = linAccel.x;
    // Positive yaw rate = turning right; rotation about "up" is positive to
    // the left under the right-hand rule, hence the sign flip.
    final double yaw = -gyroVehicle.z;
    _yawRateWindow.add(yaw);
    _yawRate = _median(_yawRateWindow.toList());

    _imuBuffer.add(
      Stamped<ImuSample>(
        ImuSample(
          accelerationMps2: linAccel,
          angularRateRadPerS: Vec3(gyroVehicle.x, gyroVehicle.y, gyroVehicle.z),
          timestampMicros: ts,
          magneticHeadingDegrees: magneticHeadingDegrees,
        ),
        ts,
      ),
    );

    _predictSpeed(ts);
    _integrateHeading(yaw, dt, magneticHeadingDegrees);
  }

  void _predictSpeed(int ts) {
    final double dt =
        _lastPredictMicros < 0 ? 0.01 : (ts - _lastPredictMicros) / 1e6;
    _lastPredictMicros = ts;
    if (dt <= 0 || dt > 0.5) return;

    // x = [speed, accel]; constant-acceleration model.
    final Matrix f = Matrix.fromList(2, 2, <double>[1, dt, 0, 1]);
    // Process noise: accelerometer bias walks, and the vehicle's own jerk is
    // unmodelled. Scaled with dt so the filter behaves the same at any rate.
    final double q = 0.6 * dt;
    final Matrix qm = Matrix.fromList(
      2,
      2,
      <double>[q * dt * dt, 0, 0, q],
    );
    _speedFilter.predict(f, qm);

    // The accelerometer is a direct (noisy) measurement of state[1].
    _speedFilter.update(
      Matrix.fromList(1, 1, <double>[_longitudinalAccel]),
      Matrix.fromList(1, 2, <double>[0, 1]),
      Matrix.fromList(1, 1, <double>[0.45]),
    );

    // Speed cannot go negative: this stack has no reverse-gear signal, and a
    // negative estimate would flip every TTC sign downstream.
    if (_speedFilter.state(0) < 0) {
      _speedFilter.x.set(0, 0, 0);
    }
  }

  void _integrateHeading(
    double yawRate,
    double dt,
    double? magneticHeadingDegrees,
  ) {
    if (!_headingInitialised) {
      if (magneticHeadingDegrees != null) {
        _headingDegrees = magneticHeadingDegrees;
        _headingInitialised = true;
        _headingConfidence = 0.3;
      }
      return;
    }
    _headingDegrees = (_headingDegrees + radToDeg(yawRate) * dt) % 360;
    if (_headingDegrees < 0) _headingDegrees += 360;

    // The magnetometer inside a car is badly disturbed, so it only gets a
    // small pull — enough to bound gyro drift over minutes, not enough to
    // yank the heading when passing a truck.
    if (magneticHeadingDegrees != null) {
      _headingDegrees = _blendAngles(
        _headingDegrees,
        magneticHeadingDegrees,
        0.004,
      );
    }
    _headingConfidence = clampDouble(_headingConfidence - 0.0005 * dt, 0.2, 1);
  }

  /// Feed a GNSS fix.
  void onGpsFix(GeoPosition fix) {
    _lastFix = _currentFix;
    _currentFix = fix;

    final double accuracyConfidence = fix.confidence.value;

    // --- speed ------------------------------------------------------------
    double? measuredSpeed = fix.speedMps;
    if (measuredSpeed == null && _lastFix != null) {
      final double dtSec =
          (fix.timestampMicros - _lastFix!.timestampMicros) / 1e6;
      if (dtSec > 0.2 && dtSec < 10) {
        measuredSpeed = _lastFix!.distanceTo(fix) / dtSec;
      }
    }

    if (measuredSpeed != null && measuredSpeed >= 0) {
      // Doppler speed from a GNSS chip is far better than its position, so the
      // measurement noise is modest even when horizontal accuracy is poor.
      final double r = fix.speedAccuracyMps != null
          ? math.max(0.05, fix.speedAccuracyMps! * fix.speedAccuracyMps!)
          : clampDouble(1.5 / math.max(accuracyConfidence, 0.05), 0.2, 25.0);
      _speedFilter.update(
        Matrix.fromList(1, 1, <double>[measuredSpeed]),
        Matrix.fromList(1, 2, <double>[1, 0]),
        Matrix.fromList(1, 1, <double>[r]),
      );
      if (_speedFilter.state(0) < 0) _speedFilter.x.set(0, 0, 0);

      // Use a real acceleration event to pin down which way "forward" is.
      if (_lastFix?.speedMps != null) {
        final double dtSec =
            (fix.timestampMicros - _lastFix!.timestampMicros) / 1e6;
        if (dtSec > 0.3) {
          final double gpsAccel =
              (measuredSpeed - _lastFix!.speedMps!) / dtSec;
          final ImuSample? latest = _imuBuffer.latest?.value;
          if (latest != null) {
            _frame.refineForward(
              Vec3(latest.accelerationMps2.x, latest.accelerationMps2.y, 0),
              gpsAccel,
            );
          }
        }
      }
    }

    // --- heading ----------------------------------------------------------
    final double speed = _speedFilter.state(0);
    if (fix.headingDegrees != null && speed > _gpsHeadingMinSpeed) {
      final double headingQuality = clampDouble(
        accuracyConfidence * clampDouble((speed - _gpsHeadingMinSpeed) / 6, 0, 1),
        0,
        1,
      );
      if (!_headingInitialised) {
        _headingDegrees = fix.headingDegrees!;
        _headingInitialised = true;
      } else {
        _headingDegrees = _blendAngles(
          _headingDegrees,
          fix.headingDegrees!,
          0.25 * headingQuality,
        );
      }
      _headingConfidence =
          clampDouble(math.max(_headingConfidence, headingQuality), 0, 1);
    }
  }

  /// Ego state at [timestampMicros] — typically a camera frame's exposure
  /// time, which is why the IMU history is interpolated rather than sampled.
  EgoMotionState stateAt(int timestampMicros) {
    final ImuSample? aligned = _imuBuffer.sampleAt(timestampMicros);
    final double speed = math.max(0, _speedFilter.state(0));
    final double speedSigma = _speedFilter.uncertainty(0);

    final bool stationary = _updateStationary(speed, timestampMicros);

    double speedConfidence = clampDouble(1.0 - speedSigma / 6.0, 0.05, 1.0);
    if (_currentFix == null) {
      // Without GNSS the speed is a pure IMU integration and will drift; say so.
      speedConfidence = math.min(speedConfidence, 0.35);
    } else {
      final double staleSec =
          (timestampMicros - _currentFix!.timestampMicros) / 1e6;
      if (staleSec > 3) {
        speedConfidence *= clampDouble(1.0 - (staleSec - 3) / 10, 0.1, 1.0);
      }
    }
    speedConfidence *= clampDouble(0.4 + 0.6 * _frame.confidence, 0, 1);

    return EgoMotionState(
      speedMps: stationary ? 0 : speed,
      headingDegrees: _headingDegrees,
      yawRateRadPerS: stationary ? 0 : (aligned?.yawRate ?? _yawRate),
      longitudinalAccelMps2:
          aligned?.longitudinalAcceleration ?? _longitudinalAccel,
      lateralAccelMps2: aligned?.lateralAcceleration ?? _lateralAccel,
      verticalAccelMps2: aligned?.verticalAcceleration ?? 0,
      position: _currentFix,
      timestampMicros: timestampMicros,
      speedConfidence: speedConfidence,
      headingConfidence:
          _headingInitialised ? _headingConfidence : 0.0,
      isStationary: stationary,
      source: _currentFix == null ? 'imu-only' : 'gps+imu',
    );
  }

  bool _updateStationary(double speed, int ts) {
    final bool looksStopped = speed < _stationarySpeedThreshold &&
        _longitudinalAccel.abs() < 0.5 &&
        _yawRate.abs() < 0.05;
    if (!looksStopped) {
      _stationarySinceMicros = -1;
      return false;
    }
    if (_stationarySinceMicros < 0) _stationarySinceMicros = ts;
    // Require half a second of quiet before declaring a stop, so that a brief
    // dip at the bottom of a gear change does not read as "stopped".
    return ts - _stationarySinceMicros > 500000;
  }

  static double _blendAngles(double current, double target, double weight) {
    double delta = (target - current) % 360;
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;
    double out = (current + delta * clampDouble(weight, 0, 1)) % 360;
    if (out < 0) out += 360;
    return out;
  }

  static double _median(List<double> values) {
    if (values.isEmpty) return 0;
    final List<double> s = List<double>.from(values)..sort();
    final int mid = s.length ~/ 2;
    return s.length.isOdd ? s[mid] : (s[mid - 1] + s[mid]) / 2;
  }

  void reset() {
    _imuBuffer.clear();
    _yawRateWindow.clear();
    _headingInitialised = false;
    _headingConfidence = 0;
    _lastFix = null;
    _currentFix = null;
    _lastImuMicros = -1;
    _lastPredictMicros = -1;
    _speedFilter.x = Matrix.fromList(2, 1, <double>[0, 0]);
    _speedFilter.p = Matrix.diagonal(<double>[25, 4]);
    Log.info(_tag, 'estimator reset');
  }
}
