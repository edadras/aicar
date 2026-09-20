import 'dart:math' as math;

import '../ai/interfaces/vehicle_simulator.dart';
import '../core/geometry.dart';
import 'simulated_control.dart';
import 'vehicle_state.dart';

/// Kinematic bicycle model of the vehicle.
///
/// The two axles are collapsed onto a single centreline wheel each, which is
/// the standard approximation for vehicle motion below roughly 0.4 g of
/// lateral acceleration — comfortably covering every situation this stack
/// would propose. It reproduces the one thing that matters for a plausible
/// trajectory: the coupling between steering angle, speed and yaw rate,
///
///   yawRate = v * tan(delta) / L
///
/// A dynamic (tyre-slip) model would add sideslip and load transfer; those
/// change the *feel* of a manoeuvre, not where the car ends up at these
/// accelerations, and they would need tyre parameters we have no way to know.
///
/// Rate and travel limits on the steering, plus drag and rolling resistance on
/// the longitudinal axis, are included because without them the simulation
/// produces trajectories no vehicle could follow, which would make the whole
/// display misleading.
class KinematicBicycleModel implements VehicleSimulator {
  const KinematicBicycleModel({
    this.parameters = const VehicleParameters(),
    this.airDensity = 1.225,
    this.gravity = 9.81,
  });

  final VehicleParameters parameters;
  final double airDensity;
  final double gravity;

  @override
  String get modelName => 'Kinematic bicycle '
      '(L=${parameters.wheelbaseMeters.toStringAsFixed(2)} m)';

  @override
  VehicleState step({
    required VehicleState state,
    required SimulatedControlCommand command,
    required double dtSeconds,
  }) {
    if (dtSeconds <= 0 || dtSeconds > 1.0) return state;

    // --- steering ---------------------------------------------------------
    // `SimulatedControlCommand.steeringAngleDegrees` is the **road-wheel**
    // angle, not the steering-wheel angle. That is the quantity the vehicle
    // model needs, and it is the quantity the HUD shows (the ±35° range the
    // spec's examples use); the graphical steering wheel multiplies it by
    // `steeringRatio` for display. Keeping one convention end to end avoids a
    // 15x error hiding in the middle of the control loop.
    final double commandedRoadWheel = degToRad(
      clampDouble(
        command.steeringAngleDegrees,
        -parameters.maxSteeringAngleDegrees,
        parameters.maxSteeringAngleDegrees,
      ),
    );
    final double maxDelta =
        degToRad(parameters.maxSteeringRateDegPerS) * dtSeconds;
    final double steeringError = commandedRoadWheel - state.steeringAngleRadians;
    final double steeringAngle = state.steeringAngleRadians +
        clampDouble(steeringError, -maxDelta, maxDelta);

    // --- longitudinal -----------------------------------------------------
    // Force-limited at low speed, power-limited at high speed.
    final double speedForPower = math.max(state.speedMps, 1.0);
    final double powerLimitedAccel =
        parameters.maxPowerKw * 1000 / (parameters.massKg * speedForPower);
    final double availableAccel =
        math.min(parameters.maxAccelerationMps2, powerLimitedAccel);
    final double throttleAccel =
        (command.throttlePercent / 100) * availableAccel;
    final double brakeAccel =
        (command.brakePercent / 100) * parameters.maxDecelerationMps2;

    // Resistive forces, expressed as accelerations.
    final double speed = state.speedMps;
    final double dragAccel = 0.5 *
        airDensity *
        parameters.dragCoefficient *
        parameters.frontalAreaM2 *
        speed *
        speed /
        parameters.massKg;
    final double rollingAccel =
        speed > 0.05 ? parameters.rollingResistance * gravity : 0;

    double acceleration =
        throttleAccel - brakeAccel - dragAccel - rollingAccel;

    double newSpeed = speed + acceleration * dtSeconds;
    if (newSpeed < 0) {
      // The brake brings the vehicle to a stop; it does not reverse it. This
      // stack has no reverse gear and must never imply one.
      newSpeed = 0;
      acceleration = -speed / dtSeconds;
    }

    // --- lateral ----------------------------------------------------------
    // yawRate = v * tan(delta) / L
    final double yawRate =
        newSpeed * math.tan(steeringAngle) / parameters.wheelbaseMeters;
    final double newYaw = normalizeAngle(state.yawRadians + yawRate * dtSeconds);

    // Integrate position at the midpoint heading (second-order accurate,
    // which matters over a long simulated run).
    final double midYaw = state.yawRadians + yawRate * dtSeconds / 2;
    final double midSpeed = (speed + newSpeed) / 2;
    final Vec2 newPosition = Vec2(
      state.position.x + midSpeed * math.sin(midYaw) * dtSeconds,
      state.position.y + midSpeed * math.cos(midYaw) * dtSeconds,
    );

    return VehicleState(
      speedMps: newSpeed,
      accelerationMps2: acceleration,
      steeringAngleRadians: steeringAngle,
      yawRadians: newYaw,
      yawRateRadPerS: yawRate,
      position: newPosition,
      throttlePercent: command.throttlePercent,
      brakePercent: command.brakePercent,
      timestampMicros: command.timestampMicros,
      distanceTravelledMeters:
          state.distanceTravelledMeters + midSpeed * dtSeconds,
      driftFromMeasuredMps: state.driftFromMeasuredMps,
    );
  }

  @override
  VehicleState synchronize({
    required VehicleState simulated,
    required double measuredSpeedMps,
    required double measuredYawRateRadPerS,
    required double blend,
  }) {
    final double b = clampDouble(blend, 0, 1);
    final double drift = simulated.speedMps - measuredSpeedMps;
    return simulated.copyWith(
      speedMps: lerpDouble(simulated.speedMps, measuredSpeedMps, b),
      yawRateRadPerS:
          lerpDouble(simulated.yawRateRadPerS, measuredYawRateRadPerS, b),
      driftFromMeasuredMps: drift,
    );
  }

  /// Steering-wheel angle corresponding to a road-wheel angle, for the HUD.
  double roadWheelToSteeringWheelDegrees(double roadWheelRadians) =>
      radToDeg(roadWheelRadians) * parameters.steeringRatio;

  /// Road-wheel angle that would produce [curvature] (1/m) — the inverse of
  /// the model, used by the steering controller.
  double steeringForCurvature(double curvature) => math.atan(
        clampDouble(
          curvature * parameters.wheelbaseMeters,
          -math.tan(parameters.maxSteeringAngleRadians),
          math.tan(parameters.maxSteeringAngleRadians),
        ),
      );
}
