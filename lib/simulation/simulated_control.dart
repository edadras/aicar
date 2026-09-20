import '../core/geometry.dart';
import '../core/safety.dart';

/// The control command the stack *would* issue.
///
/// Every field here is a number on a screen and a line in a log file. There is
/// no consumer of this class anywhere in the repository that writes to a bus,
/// a serial port, a Bluetooth socket or an actuator — see [SafetyMode] and
/// `docs/SAFETY.md`. The constructor asserts the invariant so that any future
/// build which tried to relax it would fail loudly at the point of creation.
class SimulatedControlCommand {
  SimulatedControlCommand({
    required double steeringAngleDegrees,
    required double throttlePercent,
    required double brakePercent,
    required this.timestampMicros,
    required this.frameId,
    this.reason = '',
    this.isEmergency = false,
    this.steeringLimitDegrees = 35.0,
  })  : steeringAngleDegrees =
            clampDouble(steeringAngleDegrees, -steeringLimitDegrees, steeringLimitDegrees),
        throttlePercent = clampDouble(throttlePercent, 0, 100),
        brakePercent = clampDouble(brakePercent, 0, 100) {
    // Runtime, not just an assert: the guarantee must hold in release builds.
    SafetyMode.assertSimulationOnly();
  }

  factory SimulatedControlCommand.neutral({
    required int timestampMicros,
    required int frameId,
    String reason = 'no command',
  }) =>
      SimulatedControlCommand(
        steeringAngleDegrees: 0,
        throttlePercent: 0,
        brakePercent: 0,
        timestampMicros: timestampMicros,
        frameId: frameId,
        reason: reason,
      );

  factory SimulatedControlCommand.emergencyStop({
    required int timestampMicros,
    required int frameId,
    required double steeringAngleDegrees,
    String reason = 'emergency braking (simulated)',
  }) =>
      SimulatedControlCommand(
        steeringAngleDegrees: steeringAngleDegrees,
        throttlePercent: 0,
        brakePercent: 100,
        timestampMicros: timestampMicros,
        frameId: frameId,
        reason: reason,
        isEmergency: true,
      );

  /// Simulated steering-wheel angle. `0` is straight ahead, negative is left,
  /// positive is right — matching the vehicle-frame convention used
  /// throughout, so the on-screen wheel turns the way the path bends.
  final double steeringAngleDegrees;

  /// Simulated accelerator position, 0–100 %.
  final double throttlePercent;

  /// Simulated brake pressure, 0–100 %.
  final double brakePercent;

  final int timestampMicros;
  final int frameId;
  final String reason;
  final bool isEmergency;

  /// Configurable steering range, shown in Settings.
  final double steeringLimitDegrees;

  /// Always true. Present so that anything consuming a command has to
  /// acknowledge what it is looking at.
  bool get isSimulationOnly => SafetyMode.simulationOnly;

  double get steeringNormalized =>
      steeringAngleDegrees / steeringLimitDegrees;

  String get steeringDisplay =>
      '${steeringAngleDegrees >= 0 ? '+' : ''}'
      '${steeringAngleDegrees.toStringAsFixed(0)}°';

  String get throttleDisplay => '${throttlePercent.round()}%';
  String get brakeDisplay => '${brakePercent.round()}%';

  SimulatedControlCommand copyWith({
    double? steeringAngleDegrees,
    double? throttlePercent,
    double? brakePercent,
    String? reason,
    bool? isEmergency,
  }) =>
      SimulatedControlCommand(
        steeringAngleDegrees:
            steeringAngleDegrees ?? this.steeringAngleDegrees,
        throttlePercent: throttlePercent ?? this.throttlePercent,
        brakePercent: brakePercent ?? this.brakePercent,
        timestampMicros: timestampMicros,
        frameId: frameId,
        reason: reason ?? this.reason,
        isEmergency: isEmergency ?? this.isEmergency,
        steeringLimitDegrees: steeringLimitDegrees,
      );

  Map<String, dynamic> toJson() => <String, dynamic>{
        'steering': double.parse(steeringAngleDegrees.toStringAsFixed(2)),
        'throttle': double.parse(throttlePercent.toStringAsFixed(1)),
        'brake': double.parse(brakePercent.toStringAsFixed(1)),
        'ts': timestampMicros,
        'frameId': frameId,
        if (reason.isNotEmpty) 'reason': reason,
        if (isEmergency) 'emergency': true,
        // Written into every record so a dataset can be audited for
        // provenance without reading the code that produced it.
        'mode': SafetyMode.recordingTag,
      };

  static SimulatedControlCommand fromJson(Map<String, dynamic> j) =>
      SimulatedControlCommand(
        steeringAngleDegrees: (j['steering'] as num).toDouble(),
        throttlePercent: (j['throttle'] as num).toDouble(),
        brakePercent: (j['brake'] as num).toDouble(),
        timestampMicros: (j['ts'] as num).toInt(),
        frameId: (j['frameId'] as num).toInt(),
        reason: j['reason'] as String? ?? '',
        isEmergency: j['emergency'] as bool? ?? false,
      );

  @override
  String toString() => 'SimulatedControl(steer $steeringDisplay, '
      'throttle $throttleDisplay, brake $brakeDisplay)';
}
