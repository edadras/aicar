import '../world_model/hazard.dart';

/// States of the driving decision state machine.
///
/// Every name ending in `Simulation` is a reminder in the type system itself:
/// the stack is describing what it *would* do. Nothing here reaches a vehicle.
enum DrivingState {
  cruise('CRUISE', 'Maintaining speed on the planned path'),
  followVehicle('FOLLOW_VEHICLE', 'Matching the vehicle ahead'),
  slowDown('SLOW_DOWN', 'Reducing speed'),
  stop('STOP', 'Coming to a stop'),
  wait('WAIT', 'Stopped, waiting for the way to clear'),
  turnLeft('TURN_LEFT', 'Executing a left turn'),
  turnRight('TURN_RIGHT', 'Executing a right turn'),
  laneChangeSimulation('LANE_CHANGE_SIMULATION', 'Simulated lane change'),
  pedestrianYield('PEDESTRIAN_YIELD', 'Yielding to a vulnerable road user'),
  obstacleAvoidanceSimulation(
      'OBSTACLE_AVOIDANCE_SIMULATION', 'Simulated avoidance manoeuvre'),
  emergencyBrakeSimulation(
      'EMERGENCY_BRAKE_SIMULATION', 'Simulated emergency braking'),
  uncertain('UNCERTAIN', 'Not confident enough to propose an action');

  const DrivingState(this.label, this.description);
  final String label;
  final String description;

  /// States in which the stack is actively reducing speed.
  bool get isBraking =>
      this == DrivingState.slowDown ||
      this == DrivingState.stop ||
      this == DrivingState.pedestrianYield ||
      this == DrivingState.emergencyBrakeSimulation;

  /// States that must not be interrupted by a lower-priority one.
  int get priority => switch (this) {
        DrivingState.emergencyBrakeSimulation => 100,
        DrivingState.pedestrianYield => 90,
        DrivingState.obstacleAvoidanceSimulation => 80,
        DrivingState.stop => 70,
        DrivingState.uncertain => 65,
        DrivingState.wait => 60,
        DrivingState.slowDown => 50,
        DrivingState.turnLeft || DrivingState.turnRight => 40,
        DrivingState.laneChangeSimulation => 30,
        DrivingState.followVehicle => 20,
        DrivingState.cruise => 10,
      };
}

/// One decision, with the reasoning attached.
///
/// A decision without a reason is not reviewable, and reviewability is the
/// entire point of a research tool like this. The reason string is shown on
/// the HUD, written into the recording and replayed later.
class DrivingDecision {
  const DrivingDecision({
    required this.state,
    required this.reason,
    required this.confidence,
    required this.timestampMicros,
    required this.frameId,
    this.targetSpeedMps,
    this.triggeringHazard,
    this.triggeringTrackId,
    this.heldForSeconds = 0,
    this.alternativesConsidered = const <String>[],
  });

  final DrivingState state;

  /// Human-readable justification, e.g. "Motorcycle entering predicted path".
  final String reason;

  final double confidence;
  final int timestampMicros;
  final int frameId;

  /// Speed this decision aims for, m/s.
  final double? targetSpeedMps;

  final Hazard? triggeringHazard;
  final int? triggeringTrackId;

  /// How long this state has been held, for hysteresis and for the HUD.
  final double heldForSeconds;

  /// Other states that were evaluated, for the debug overlay.
  final List<String> alternativesConsidered;

  int get confidencePercent => (confidence * 100).round();

  /// HUD block:
  /// ```
  /// Decision: SLOW_DOWN
  /// Reason:   Motorcycle entering predicted path
  /// Confidence: 89%
  /// ```
  String get displayText =>
      'Decision:\n${state.label}\n\nReason:\n$reason\n\n'
      'Confidence:\n$confidencePercent%';

  Map<String, dynamic> toJson() => <String, dynamic>{
        'state': state.name,
        'reason': reason,
        'conf': double.parse(confidence.toStringAsFixed(3)),
        'ts': timestampMicros,
        'frameId': frameId,
        if (targetSpeedMps != null)
          'targetSpeed': double.parse(targetSpeedMps!.toStringAsFixed(2)),
        if (triggeringTrackId != null) 'track': triggeringTrackId,
        if (triggeringHazard != null) 'hazard': triggeringHazard!.toJson(),
        'held': double.parse(heldForSeconds.toStringAsFixed(2)),
      };

  static DrivingDecision fromJson(Map<String, dynamic> j) => DrivingDecision(
        state: DrivingState.values.firstWhere(
          (DrivingState s) => s.name == j['state'],
          orElse: () => DrivingState.uncertain,
        ),
        reason: j['reason'] as String? ?? '',
        confidence: (j['conf'] as num?)?.toDouble() ?? 0,
        timestampMicros: (j['ts'] as num?)?.toInt() ?? 0,
        frameId: (j['frameId'] as num?)?.toInt() ?? 0,
        targetSpeedMps: (j['targetSpeed'] as num?)?.toDouble(),
        triggeringTrackId: (j['track'] as num?)?.toInt(),
        triggeringHazard: j['hazard'] == null
            ? null
            : Hazard.fromJson(j['hazard'] as Map<String, dynamic>),
        heldForSeconds: (j['held'] as num?)?.toDouble() ?? 0,
      );

  @override
  String toString() =>
      '${state.label} ($confidencePercent%): $reason';
}
