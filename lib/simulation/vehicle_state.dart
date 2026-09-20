import 'dart:math' as math;

import '../core/geometry.dart';

/// State of the simulated vehicle.
///
/// This is a *model* of what the car would be doing if the stack were driving
/// it. It runs alongside the real vehicle, which is under human control, and
/// is periodically re-anchored to the measured ego motion so the two do not
/// diverge without bound.
class VehicleState {
  const VehicleState({
    required this.speedMps,
    required this.accelerationMps2,
    required this.steeringAngleRadians,
    required this.yawRadians,
    required this.yawRateRadPerS,
    required this.position,
    required this.throttlePercent,
    required this.brakePercent,
    required this.timestampMicros,
    this.distanceTravelledMeters = 0,
    this.driftFromMeasuredMps = 0,
  });

  factory VehicleState.stationary({int timestampMicros = 0}) => VehicleState(
        speedMps: 0,
        accelerationMps2: 0,
        steeringAngleRadians: 0,
        yawRadians: 0,
        yawRateRadPerS: 0,
        position: const Vec2.zero(),
        throttlePercent: 0,
        brakePercent: 0,
        timestampMicros: timestampMicros,
      );

  final double speedMps;
  final double accelerationMps2;

  /// Road-wheel angle (not steering-wheel angle), radians. Positive = right.
  final double steeringAngleRadians;

  /// Heading in the simulation's own world frame, radians.
  final double yawRadians;
  final double yawRateRadPerS;

  /// Position in the simulation's world frame, metres.
  final Vec2 position;

  final double throttlePercent;
  final double brakePercent;
  final int timestampMicros;
  final double distanceTravelledMeters;

  /// How far the simulated speed has drifted from the measured one, m/s.
  /// Shown on the performance screen: a large, persistent drift means the
  /// vehicle model's parameters do not match the real car.
  final double driftFromMeasuredMps;

  double get speedKph => speedMps * 3.6;
  double get steeringAngleDegrees => radToDeg(steeringAngleRadians);
  double get yawDegrees => radToDeg(yawRadians);

  /// Radius of the circle the vehicle is turning on, metres.
  double get turnRadiusMeters {
    if (yawRateRadPerS.abs() < 1e-4) return double.infinity;
    return speedMps / yawRateRadPerS.abs();
  }

  /// Lateral acceleration the occupants would feel, m/s².
  double get lateralAccelerationMps2 => speedMps * yawRateRadPerS;

  VehicleState copyWith({
    double? speedMps,
    double? accelerationMps2,
    double? steeringAngleRadians,
    double? yawRadians,
    double? yawRateRadPerS,
    Vec2? position,
    double? throttlePercent,
    double? brakePercent,
    int? timestampMicros,
    double? distanceTravelledMeters,
    double? driftFromMeasuredMps,
  }) =>
      VehicleState(
        speedMps: speedMps ?? this.speedMps,
        accelerationMps2: accelerationMps2 ?? this.accelerationMps2,
        steeringAngleRadians:
            steeringAngleRadians ?? this.steeringAngleRadians,
        yawRadians: yawRadians ?? this.yawRadians,
        yawRateRadPerS: yawRateRadPerS ?? this.yawRateRadPerS,
        position: position ?? this.position,
        throttlePercent: throttlePercent ?? this.throttlePercent,
        brakePercent: brakePercent ?? this.brakePercent,
        timestampMicros: timestampMicros ?? this.timestampMicros,
        distanceTravelledMeters:
            distanceTravelledMeters ?? this.distanceTravelledMeters,
        driftFromMeasuredMps:
            driftFromMeasuredMps ?? this.driftFromMeasuredMps,
      );

  Map<String, dynamic> toJson() => <String, dynamic>{
        'speed': double.parse(speedMps.toStringAsFixed(3)),
        'accel': double.parse(accelerationMps2.toStringAsFixed(3)),
        'steer': double.parse(steeringAngleRadians.toStringAsFixed(4)),
        'yaw': double.parse(yawRadians.toStringAsFixed(4)),
        'yawRate': double.parse(yawRateRadPerS.toStringAsFixed(4)),
        'x': double.parse(position.x.toStringAsFixed(2)),
        'y': double.parse(position.y.toStringAsFixed(2)),
        'throttle': double.parse(throttlePercent.toStringAsFixed(1)),
        'brake': double.parse(brakePercent.toStringAsFixed(1)),
        'ts': timestampMicros,
        'odo': double.parse(distanceTravelledMeters.toStringAsFixed(1)),
        'drift': double.parse(driftFromMeasuredMps.toStringAsFixed(2)),
      };

  static VehicleState fromJson(Map<String, dynamic> j) => VehicleState(
        speedMps: (j['speed'] as num).toDouble(),
        accelerationMps2: (j['accel'] as num).toDouble(),
        steeringAngleRadians: (j['steer'] as num).toDouble(),
        yawRadians: (j['yaw'] as num).toDouble(),
        yawRateRadPerS: (j['yawRate'] as num).toDouble(),
        position: Vec2((j['x'] as num).toDouble(), (j['y'] as num).toDouble()),
        throttlePercent: (j['throttle'] as num).toDouble(),
        brakePercent: (j['brake'] as num).toDouble(),
        timestampMicros: (j['ts'] as num).toInt(),
        distanceTravelledMeters: (j['odo'] as num?)?.toDouble() ?? 0,
        driftFromMeasuredMps: (j['drift'] as num?)?.toDouble() ?? 0,
      );

  @override
  String toString() => 'VehicleState(${speedKph.toStringAsFixed(1)} km/h, '
      'steer ${steeringAngleDegrees.toStringAsFixed(1)}°, '
      'yaw ${yawDegrees.toStringAsFixed(0)}°)';
}

/// Physical parameters of the simulated vehicle.
///
/// Defaults describe a typical mid-size saloon. They are editable in Settings
/// because the simulated trajectory is only meaningful if the model roughly
/// matches the car the phone is sitting in.
class VehicleParameters {
  const VehicleParameters({
    this.wheelbaseMeters = 2.70,
    this.trackWidthMeters = 1.55,
    this.lengthMeters = 4.60,
    this.widthMeters = 1.82,
    this.massKg = 1500,
    this.maxSteeringAngleDegrees = 35,
    this.steeringRatio = 15.0,
    this.maxSteeringRateDegPerS = 220,
    this.maxAccelerationMps2 = 3.0,
    this.maxDecelerationMps2 = 8.0,
    this.maxPowerKw = 110,
    this.dragCoefficient = 0.30,
    this.frontalAreaM2 = 2.2,
    this.rollingResistance = 0.013,
  });

  /// Distance between the axles. The single most important parameter for a
  /// kinematic bicycle model — it sets the turn radius for a given steer.
  final double wheelbaseMeters;

  final double trackWidthMeters;
  final double lengthMeters;
  final double widthMeters;
  final double massKg;

  /// Maximum **road wheel** angle.
  final double maxSteeringAngleDegrees;

  /// Steering-wheel degrees per road-wheel degree. Used only to convert the
  /// simulated road-wheel angle into the number shown on the HUD wheel.
  final double steeringRatio;

  /// How fast the steering can be moved, degrees per second at the road
  /// wheel. Without this limit the simulation would snap instantly to any
  /// commanded angle, which no real steering system does.
  final double maxSteeringRateDegPerS;

  /// Peak acceleration at low speed, m/s². Traction- and torque-limited.
  final double maxAccelerationMps2;

  final double maxDecelerationMps2;

  /// Engine/motor power, kW.
  ///
  /// Without a power limit the model sustains [maxAccelerationMps2] at any
  /// speed, and the only thing stopping it is aerodynamic drag — which puts
  /// the terminal speed of an ordinary saloon north of 350 km/h. Real vehicles
  /// are force-limited at low speed and *power*-limited at high speed, so
  /// available acceleration falls as P/(m·v).
  final double maxPowerKw;

  final double dragCoefficient;
  final double frontalAreaM2;
  final double rollingResistance;

  double get maxSteeringAngleRadians => degToRad(maxSteeringAngleDegrees);

  /// Tightest circle the vehicle can trace, metres.
  double get minimumTurnRadiusMeters =>
      wheelbaseMeters / math.tan(maxSteeringAngleRadians);

  Map<String, dynamic> toJson() => <String, dynamic>{
        'wheelbase': wheelbaseMeters,
        'trackWidth': trackWidthMeters,
        'length': lengthMeters,
        'width': widthMeters,
        'mass': massKg,
        'maxSteer': maxSteeringAngleDegrees,
        'steeringRatio': steeringRatio,
        'maxSteerRate': maxSteeringRateDegPerS,
        'maxAccel': maxAccelerationMps2,
        'maxDecel': maxDecelerationMps2,
        'maxPower': maxPowerKw,
      };

  static VehicleParameters fromJson(Map<String, dynamic> j) =>
      VehicleParameters(
        wheelbaseMeters: (j['wheelbase'] as num?)?.toDouble() ?? 2.70,
        trackWidthMeters: (j['trackWidth'] as num?)?.toDouble() ?? 1.55,
        lengthMeters: (j['length'] as num?)?.toDouble() ?? 4.60,
        widthMeters: (j['width'] as num?)?.toDouble() ?? 1.82,
        massKg: (j['mass'] as num?)?.toDouble() ?? 1500,
        maxSteeringAngleDegrees: (j['maxSteer'] as num?)?.toDouble() ?? 35,
        steeringRatio: (j['steeringRatio'] as num?)?.toDouble() ?? 15.0,
        maxSteeringRateDegPerS:
            (j['maxSteerRate'] as num?)?.toDouble() ?? 220,
        maxAccelerationMps2: (j['maxAccel'] as num?)?.toDouble() ?? 3.0,
        maxDecelerationMps2: (j['maxDecel'] as num?)?.toDouble() ?? 8.0,
        maxPowerKw: (j['maxPower'] as num?)?.toDouble() ?? 110,
      );
}
