import '../../simulation/simulated_control.dart';
import '../../simulation/vehicle_state.dart';

/// Integrates a vehicle model forward under simulated controls.
///
/// This exists so the HUD can answer "if the stack had been driving, where
/// would the car be now?" — it never influences the real vehicle, which is
/// controlled entirely by its human driver.
abstract class VehicleSimulator {
  String get modelName;

  /// Advance the simulation by [dtSeconds] under [command].
  VehicleState step({
    required VehicleState state,
    required SimulatedControlCommand command,
    required double dtSeconds,
  });

  /// Re-anchor the simulated vehicle to the measured ego state, so the
  /// simulation does not drift arbitrarily far from reality over a long drive.
  VehicleState synchronize({
    required VehicleState simulated,
    required double measuredSpeedMps,
    required double measuredYawRateRadPerS,
    required double blend,
  });
}
