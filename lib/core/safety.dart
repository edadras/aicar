/// Global, compile-time-visible safety contract for the whole application.
///
/// This project is a **research / ADAS / simulation / data-collection** tool.
/// It observes the road through the phone camera and sensors and computes what
/// an autonomous driving stack *would* command. Those commands are never sent
/// anywhere: there is no CAN bus code, no ECU code, no actuator code, and no
/// Bluetooth/OBD writing path anywhere in this repository.
///
/// Every actuator-shaped value produced by the stack flows through
/// [SimulatedControlCommand] (see `lib/simulation/`), which can only be
/// rendered on screen or written to a recording file.
library;

/// Immutable description of the safety envelope the binary was built with.
class SafetyMode {
  const SafetyMode._();

  /// Hard-wired to `true`. There is deliberately no setter, no factory and no
  /// build flavour that can flip this. Code that would actuate a real vehicle
  /// must not exist; [assertSimulationOnly] documents and enforces the intent
  /// at the few places where control values are produced.
  static const bool simulationOnly = true;

  /// Human readable banner shown in the HUD and written into every recording.
  static const String banner = 'SIMULATION ONLY — no vehicle control output';

  /// Machine readable tag stored in recorded sessions so that any downstream
  /// dataset consumer can verify provenance.
  static const String recordingTag = 'SIMULATION_ONLY=true';

  /// Interfaces that this build is explicitly *not* linked against.
  static const List<String> forbiddenInterfaces = <String>[
    'CAN bus',
    'ECU / OBD-II write',
    'steering actuator',
    'brake actuator',
    'throttle actuator',
  ];

  /// Called from the control path. Kept as a real runtime check (not just an
  /// `assert`) so that the invariant also holds in release builds.
  static void assertSimulationOnly() {
    if (!simulationOnly) {
      throw StateError(
        'SafetyMode.simulationOnly was false. This build is not permitted to '
        'produce vehicle control output.',
      );
    }
  }
}
