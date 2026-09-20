import 'dart:math' as math;

import '../core/geometry.dart';
import '../core/kalman.dart';
import '../decision/driving_decision.dart';
import '../planning/planned_path.dart';
import '../world_model/world_state.dart';
import 'bicycle_model.dart';
import 'pure_pursuit.dart';
import 'simulated_control.dart';
import 'vehicle_state.dart';

/// Turns a decision plus a planned path into the steering, throttle and brake
/// the stack would command.
///
/// Lateral and longitudinal control are separated, as in every real driving
/// stack: steering comes from the path geometry via pure pursuit, and the
/// pedals come from the speed error via a PI controller with anti-windup.
/// Decisions enter longitudinally, as a target speed, and only override the
/// lateral channel in the avoidance cases where they must.
///
/// The output is a [SimulatedControlCommand], which exists only to be drawn
/// and logged.
class SimulatedVehicleController {
  SimulatedVehicleController({
    this.parameters = const VehicleParameters(),
    this.purePursuit = const PurePursuitController(),
    this.speedProportionalGain = 0.55,
    this.speedIntegralGain = 0.12,
    this.integralLimit = 2.0,
    this.steeringSmoothingSeconds = 0.18,
    this.throttleSmoothingSeconds = 0.25,
    this.steeringLimitDegrees = 35.0,
  }) : _steeringFilter =
            ExponentialFilter(timeConstantSeconds: steeringSmoothingSeconds),
        _throttleFilter =
            ExponentialFilter(timeConstantSeconds: throttleSmoothingSeconds);

  final VehicleParameters parameters;
  final PurePursuitController purePursuit;

  final double speedProportionalGain;
  final double speedIntegralGain;
  final double integralLimit;
  final double steeringSmoothingSeconds;
  final double throttleSmoothingSeconds;

  /// Displayed steering range; configurable from Settings.
  final double steeringLimitDegrees;

  final ExponentialFilter _steeringFilter;
  final ExponentialFilter _throttleFilter;

  double _speedIntegral = 0;
  int _lastTimestampMicros = -1;

  SimulatedControlCommand compute({
    required WorldState world,
    required PlannedPath path,
    required DrivingDecision decision,
    required VehicleState simulated,
  }) {
    final double dt = _lastTimestampMicros < 0
        ? 0.05
        : clampDouble(
            (world.timestampMicros - _lastTimestampMicros) / 1e6,
            0.001,
            0.5,
          );
    _lastTimestampMicros = world.timestampMicros;

    // --- lateral ----------------------------------------------------------
    final double steeringRadians = purePursuit.computeSteering(
      path: path,
      vehicle: simulated,
      parameters: parameters,
      speedMps: math.max(world.ego.speedMps, simulated.speedMps),
    );

    // Commands carry the **road-wheel** angle throughout: it is what the
    // vehicle model integrates, and it is the quantity whose magnitude the
    // HUD reports (±35°, matching the spec's examples). The graphical
    // steering wheel multiplies by the steering ratio when it draws itself.
    double steeringWheelDegrees = clampDouble(
      radToDeg(steeringRadians),
      -steeringLimitDegrees,
      steeringLimitDegrees,
    );

    // An unusable path must not produce a steering command at all. Returning
    // zero here (rather than the last value) is deliberate: it makes the HUD
    // visibly stop proposing a direction when the stack cannot see the road.
    if (!path.isUsable) {
      _steeringFilter.reset(0);
      steeringWheelDegrees = 0;
    } else {
      steeringWheelDegrees =
          _steeringFilter.update(steeringWheelDegrees, dt);
    }

    // --- longitudinal -----------------------------------------------------
    final double targetSpeed = _resolveTargetSpeed(world, path, decision);
    final double currentSpeed =
        math.max(0, world.ego.speedConfidence > 0.3
            ? world.ego.speedMps
            : simulated.speedMps);
    final double error = targetSpeed - currentSpeed;

    // PI with conditional integration: the integral only accumulates when the
    // output is not saturated, which is what prevents wind-up during a long
    // full-brake event.
    final double proportional = speedProportionalGain * error;
    final bool saturated = (proportional > 1.2 && error > 0) ||
        (proportional < -1.2 && error < 0);
    if (!saturated) {
      _speedIntegral = clampDouble(
        _speedIntegral + error * dt,
        -integralLimit,
        integralLimit,
      );
    }
    final double control =
        proportional + speedIntegralGain * _speedIntegral;

    double throttlePercent = 0;
    double brakePercent = 0;

    if (decision.state == DrivingState.emergencyBrakeSimulation) {
      // No modulation, no filtering: an emergency stop is the one case where
      // smoothing would be actively wrong.
      _speedIntegral = 0;
      _throttleFilter.reset(0);
      return SimulatedControlCommand.emergencyStop(
        timestampMicros: world.timestampMicros,
        frameId: world.frameId,
        steeringAngleDegrees: steeringWheelDegrees,
        reason: decision.reason,
      );
    }

    if (control >= 0) {
      throttlePercent = clampDouble(control * 100 / 2.0, 0, 100);
      throttlePercent = _throttleFilter.update(throttlePercent, dt);
    } else {
      _throttleFilter.reset(0);
      // Map the required deceleration onto brake percentage against the
      // model's maximum, so 100 % means the model's full braking authority.
      final double requiredDecel = -control;
      brakePercent = clampDouble(
        requiredDecel / parameters.maxDecelerationMps2 * 100 * 2.0,
        0,
        100,
      );
    }

    // States that mean "stop" hold the brake once nearly stationary, rather
    // than oscillating around a zero speed error.
    if ((decision.state == DrivingState.stop ||
            decision.state == DrivingState.wait) &&
        currentSpeed < 0.6) {
      throttlePercent = 0;
      brakePercent = math.max(brakePercent, 35);
    }

    // UNCERTAIN never commands acceleration: if the stack does not know what
    // is ahead, the only defensible proposal is to ease off.
    if (decision.state == DrivingState.uncertain) {
      throttlePercent = 0;
      brakePercent = math.max(brakePercent, currentSpeed > 1 ? 15 : 0);
    }

    return SimulatedControlCommand(
      steeringAngleDegrees: steeringWheelDegrees,
      throttlePercent: throttlePercent,
      brakePercent: brakePercent,
      timestampMicros: world.timestampMicros,
      frameId: world.frameId,
      reason: decision.reason,
      steeringLimitDegrees: steeringLimitDegrees,
    );
  }

  double _resolveTargetSpeed(
    WorldState world,
    PlannedPath path,
    DrivingDecision decision,
  ) {
    double target = decision.targetSpeedMps ??
        (path.isUsable ? path.limitingSpeedMps : 0);

    // The path's own speed profile is a ceiling regardless of the decision:
    // a decision to cruise cannot override a curve.
    if (path.isUsable) {
      target = math.min(target, path.limitingSpeedMps);
    }

    // As is the posted limit.
    final double? regulatory = world.regulatory.targetSpeedMps;
    if (regulatory != null) target = math.min(target, regulatory);

    return clampDouble(target, 0, 45);
  }

  void reset() {
    _speedIntegral = 0;
    _steeringFilter.reset();
    _throttleFilter.reset();
    _lastTimestampMicros = -1;
  }

  /// Convenience: run the bicycle model one step under [command].
  VehicleState advanceSimulation({
    required KinematicBicycleModel model,
    required VehicleState state,
    required SimulatedControlCommand command,
    required WorldState world,
    double dtSeconds = 0.05,
  }) {
    VehicleState next = model.step(
      state: state,
      command: command,
      dtSeconds: dtSeconds,
    );
    // Re-anchor gently to the measured motion so the simulated speed does not
    // wander away from the car the phone is actually in over a long drive.
    if (world.ego.speedConfidence > 0.4) {
      next = model.synchronize(
        simulated: next,
        measuredSpeedMps: world.ego.speedMps,
        measuredYawRateRadPerS: world.ego.yawRateRadPerS,
        blend: 0.06,
      );
    }
    return next;
  }
}
