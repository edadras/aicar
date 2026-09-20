import 'dart:math' as math;

import 'package:aicar/core/geometry.dart';
import 'package:aicar/core/safety.dart';
import 'package:aicar/decision/driving_decision.dart';
import 'package:aicar/planning/local_path_planner.dart';
import 'package:aicar/planning/planned_path.dart';
import 'package:aicar/simulation/bicycle_model.dart';
import 'package:aicar/simulation/pure_pursuit.dart';
import 'package:aicar/simulation/simulated_control.dart';
import 'package:aicar/simulation/simulated_vehicle_controller.dart';
import 'package:aicar/simulation/vehicle_state.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/world_builder.dart';

VehicleState run(
  KinematicBicycleModel model,
  VehicleState start,
  SimulatedControlCommand Function(int step) command, {
  int steps = 100,
  double dt = 0.05,
}) {
  VehicleState state = start;
  for (int i = 0; i < steps; i++) {
    state = model.step(state: state, command: command(i), dtSeconds: dt);
  }
  return state;
}

void main() {
  const VehicleParameters parameters = VehicleParameters();
  const KinematicBicycleModel model =
      KinematicBicycleModel(parameters: parameters);

  group('SafetyMode', () {
    test('is hard-wired to simulation only', () {
      expect(SafetyMode.simulationOnly, isTrue);
      expect(SafetyMode.assertSimulationOnly, returnsNormally);
      expect(SafetyMode.forbiddenInterfaces, contains('CAN bus'));
    });

    test('every command declares itself as simulation output', () {
      final SimulatedControlCommand c = SimulatedControlCommand(
        steeringAngleDegrees: 10,
        throttlePercent: 40,
        brakePercent: 0,
        timestampMicros: 0,
        frameId: 0,
      );
      expect(c.isSimulationOnly, isTrue);
      expect(c.toJson()['mode'], SafetyMode.recordingTag);
    });

    test('commands are clamped to their declared ranges', () {
      final SimulatedControlCommand c = SimulatedControlCommand(
        steeringAngleDegrees: 900,
        throttlePercent: 500,
        brakePercent: -20,
        timestampMicros: 0,
        frameId: 0,
      );
      expect(c.steeringAngleDegrees, 35);
      expect(c.throttlePercent, 100);
      expect(c.brakePercent, 0);
    });
  });

  group('KinematicBicycleModel longitudinal', () {
    test('accelerates under throttle and stops under brake', () {
      final VehicleState accelerated = run(
        model,
        VehicleState.stationary(),
        (int i) => SimulatedControlCommand(
          steeringAngleDegrees: 0,
          throttlePercent: 100,
          brakePercent: 0,
          timestampMicros: i * 50000,
          frameId: i,
        ),
        steps: 60, // 3 s
      );
      // 3 m/s² for 3 s, minus drag: comfortably above 6 m/s.
      expect(accelerated.speedMps, greaterThan(6));
      expect(accelerated.speedMps, lessThan(10));

      final VehicleState braked = run(
        model,
        accelerated,
        (int i) => SimulatedControlCommand(
          steeringAngleDegrees: 0,
          throttlePercent: 0,
          brakePercent: 100,
          timestampMicros: (60 + i) * 50000,
          frameId: 60 + i,
        ),
        steps: 60,
      );
      expect(braked.speedMps, 0);
    });

    test('never reverses under braking', () {
      final VehicleState state = run(
        model,
        VehicleState.stationary().copyWith(speedMps: 2),
        (int i) => SimulatedControlCommand(
          steeringAngleDegrees: 0,
          throttlePercent: 0,
          brakePercent: 100,
          timestampMicros: i * 50000,
          frameId: i,
        ),
        steps: 100,
      );
      expect(state.speedMps, 0);
      expect(state.speedMps, isNot(lessThan(0)));
    });

    test('drag produces a terminal speed under constant throttle', () {
      final VehicleState state = run(
        model,
        VehicleState.stationary(),
        (int i) => SimulatedControlCommand(
          steeringAngleDegrees: 0,
          throttlePercent: 100,
          brakePercent: 0,
          timestampMicros: i * 50000,
          frameId: i,
        ),
        steps: 4000, // 200 s
      );
      // Terminal speed where 3 m/s² == drag + rolling resistance.
      expect(state.speedMps, greaterThan(40));
      expect(state.speedMps, lessThan(90));
      expect(state.accelerationMps2.abs(), lessThan(0.05));
    });
  });

  group('KinematicBicycleModel lateral', () {
    test('drives straight with zero steering', () {
      final VehicleState state = run(
        model,
        VehicleState.stationary().copyWith(speedMps: 10),
        (int i) => SimulatedControlCommand(
          steeringAngleDegrees: 0,
          throttlePercent: 20,
          brakePercent: 0,
          timestampMicros: i * 50000,
          frameId: i,
        ),
        steps: 100,
      );
      expect(state.yawRadians, closeTo(0, 1e-9));
      expect(state.position.x, closeTo(0, 1e-9));
      expect(state.position.y, greaterThan(40));
    });

    test('traces the turn radius the kinematic model predicts', () {
      // Commands carry the road-wheel angle directly.
      const double roadWheelDegrees = 10;

      VehicleState state =
          VehicleState.stationary().copyWith(speedMps: 8);
      // Let the rate-limited steering reach the commanded angle first.
      for (int i = 0; i < 200; i++) {
        state = model.step(
          state: state,
          command: SimulatedControlCommand(
            steeringAngleDegrees: roadWheelDegrees,
            throttlePercent: 18,
            brakePercent: 0,
            timestampMicros: i * 50000,
            frameId: i,
          ),
          dtSeconds: 0.05,
        );
      }

      final double expectedRadius = parameters.wheelbaseMeters /
          math.tan(degToRad(roadWheelDegrees));
      expect(state.turnRadiusMeters, closeTo(expectedRadius, expectedRadius * 0.08));
      expect(state.yawRateRadPerS, greaterThan(0), reason: 'positive = right');
    });

    test('steering is rate limited, not instantaneous', () {
      final VehicleState after = model.step(
        state: VehicleState.stationary().copyWith(speedMps: 10),
        command: SimulatedControlCommand(
          steeringAngleDegrees: 35,
          throttlePercent: 0,
          brakePercent: 0,
          timestampMicros: 0,
          frameId: 0,
        ),
        dtSeconds: 0.05,
      );
      final double maxStepDegrees =
          parameters.maxSteeringRateDegPerS * 0.05;
      expect(after.steeringAngleDegrees, lessThanOrEqualTo(maxStepDegrees + 1e-6));
      expect(after.steeringAngleDegrees, greaterThan(0));
    });

    test('a stationary vehicle does not yaw however hard it is steered', () {
      final VehicleState state = run(
        model,
        VehicleState.stationary(),
        (int i) => SimulatedControlCommand(
          steeringAngleDegrees: 35,
          throttlePercent: 0,
          brakePercent: 100,
          timestampMicros: i * 50000,
          frameId: i,
        ),
        steps: 40,
      );
      expect(state.yawRateRadPerS, 0);
      expect(state.yawRadians, 0);
    });
  });

  group('PurePursuitController', () {
    test('commands no steering on a straight path', () {
      final PlannedPath path = LocalPathPlanner().plan(testWorld());
      const PurePursuitController controller = PurePursuitController();
      final double steering = controller.computeSteering(
        path: path,
        vehicle: VehicleState.stationary().copyWith(speedMps: 14),
        parameters: parameters,
        speedMps: 14,
      );
      expect(steering, closeTo(0, 0.02));
    });

    test('steers towards a curve', () {
      final PlannedPath right = LocalPathPlanner()
          .plan(testWorld(lanes: straightLanes(curvature: 0.004)));
      final PlannedPath left = LocalPathPlanner()
          .plan(testWorld(lanes: straightLanes(curvature: -0.004)));
      const PurePursuitController controller = PurePursuitController();

      final double steerRight = controller.computeSteering(
        path: right,
        vehicle: VehicleState.stationary().copyWith(speedMps: 12),
        parameters: parameters,
        speedMps: 12,
      );
      final double steerLeft = controller.computeSteering(
        path: left,
        vehicle: VehicleState.stationary().copyWith(speedMps: 12),
        parameters: parameters,
        speedMps: 12,
      );

      expect(steerRight, greaterThan(0.01), reason: 'right curve → right steer');
      expect(steerLeft, lessThan(-0.01), reason: 'left curve → left steer');
    });

    test('look-ahead grows with speed', () {
      const PurePursuitController controller = PurePursuitController();
      expect(controller.lookaheadFor(30, 60),
          greaterThan(controller.lookaheadFor(5, 60)));
    });

    test('never exceeds the steering limit', () {
      final PlannedPath sharp = LocalPathPlanner()
          .plan(testWorld(lanes: straightLanes(curvature: 0.02)));
      const PurePursuitController controller = PurePursuitController();
      final double steering = controller.computeSteering(
        path: sharp,
        vehicle: VehicleState.stationary().copyWith(speedMps: 20),
        parameters: parameters,
        speedMps: 20,
      );
      expect(steering.abs(),
          lessThanOrEqualTo(parameters.maxSteeringAngleRadians + 1e-9));
    });
  });

  group('SimulatedVehicleController', () {
    test('emergency braking commands full brake and no throttle', () {
      final SimulatedVehicleController controller =
          SimulatedVehicleController(parameters: parameters);
      final PlannedPath path = LocalPathPlanner().plan(testWorld());
      final SimulatedControlCommand command = controller.compute(
        world: testWorld(),
        path: path,
        decision: const DrivingDecision(
          state: DrivingState.emergencyBrakeSimulation,
          reason: 'test',
          confidence: 0.9,
          timestampMicros: 0,
          frameId: 0,
          targetSpeedMps: 0,
        ),
        simulated: VehicleState.stationary().copyWith(speedMps: 14),
      );
      expect(command.brakePercent, 100);
      expect(command.throttlePercent, 0);
      expect(command.isEmergency, isTrue);
    });

    test('UNCERTAIN never commands acceleration', () {
      final SimulatedVehicleController controller =
          SimulatedVehicleController(parameters: parameters);
      final PlannedPath path = LocalPathPlanner().plan(testWorld());
      final SimulatedControlCommand command = controller.compute(
        world: testWorld(),
        path: path,
        decision: const DrivingDecision(
          state: DrivingState.uncertain,
          reason: 'test',
          confidence: 0.9,
          timestampMicros: 0,
          frameId: 0,
          targetSpeedMps: 5,
        ),
        simulated: VehicleState.stationary().copyWith(speedMps: 14),
      );
      expect(command.throttlePercent, 0);
      expect(command.brakePercent, greaterThan(0));
    });

    test('no usable path means no steering proposal', () {
      final SimulatedVehicleController controller =
          SimulatedVehicleController(parameters: parameters);
      final SimulatedControlCommand command = controller.compute(
        world: testWorld(),
        path: PlannedPath.none(frameId: 1, timestampMicros: 0),
        decision: const DrivingDecision(
          state: DrivingState.uncertain,
          reason: 'no path',
          confidence: 0.9,
          timestampMicros: 0,
          frameId: 0,
        ),
        simulated: VehicleState.stationary().copyWith(speedMps: 10),
      );
      expect(command.steeringAngleDegrees, 0);
    });

    test('closes the speed error over time without oscillating', () {
      final SimulatedVehicleController controller =
          SimulatedVehicleController(parameters: parameters);
      final PlannedPath path = LocalPathPlanner().plan(testWorld());

      VehicleState vehicle = VehicleState.stationary();
      final List<double> speeds = <double>[];
      for (int i = 0; i < 200; i++) {
        final SimulatedControlCommand command = controller.compute(
          world: testWorld(
            timestampMicros: i * 50000,
            ego: testEgo(speedMps: vehicle.speedMps, ts: i * 50000),
          ),
          path: path,
          decision: DrivingDecision(
            state: DrivingState.cruise,
            reason: 'test',
            confidence: 0.9,
            timestampMicros: i * 50000,
            frameId: i,
            targetSpeedMps: 12,
          ),
          simulated: vehicle,
        );
        vehicle = model.step(
          state: vehicle,
          command: command,
          dtSeconds: 0.05,
        );
        speeds.add(vehicle.speedMps);
      }

      expect(speeds.last, closeTo(12, 1.0));
      // No large overshoot: the peak must stay near the target.
      expect(speeds.reduce(math.max), lessThan(13.5));
    });
  });
}
