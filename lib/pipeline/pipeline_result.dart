import '../decision/driving_decision.dart';
import '../planning/collision_predictor.dart';
import '../planning/planned_path.dart';
import '../simulation/simulated_control.dart';
import '../simulation/vehicle_state.dart';
import '../world_model/world_state.dart';

/// Everything one pipeline cycle produced.
///
/// This single object is what the HUD renders, what the recorder writes and
/// what replay compares — so a recorded drive and a live drive are literally
/// the same data structure.
class PipelineResult {
  const PipelineResult({
    required this.world,
    required this.path,
    required this.decision,
    required this.command,
    required this.vehicle,
    required this.collisions,
    required this.totalLatencyMicros,
    required this.stageTimings,
  });

  final WorldState world;
  final PlannedPath path;
  final DrivingDecision decision;
  final SimulatedControlCommand command;
  final VehicleState vehicle;
  final List<CollisionAssessment> collisions;

  /// End-to-end latency from frame exposure to command, microseconds.
  final int totalLatencyMicros;

  /// Per-stage timings for this cycle, milliseconds.
  final Map<String, double> stageTimings;

  double get totalLatencyMs => totalLatencyMicros / 1000;

  int get frameId => world.frameId;

  Map<String, dynamic> toJson() => <String, dynamic>{
        'world': world.toJson(),
        'path': path.toJson(),
        'decision': decision.toJson(),
        'command': command.toJson(),
        'vehicle': vehicle.toJson(),
        'latencyMs': double.parse(totalLatencyMs.toStringAsFixed(2)),
        'stages': <String, double>{
          for (final MapEntry<String, double> e in stageTimings.entries)
            e.key: double.parse(e.value.toStringAsFixed(2)),
        },
      };

  @override
  String toString() => 'PipelineResult(#$frameId, ${decision.state.label}, '
      '${totalLatencyMs.toStringAsFixed(1)}ms)';
}
