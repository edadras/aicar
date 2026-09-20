import '../../planning/planned_path.dart';
import '../../world_model/world_state.dart';

/// Produces the trajectory the vehicle would follow.
///
/// Swappable like the perception models: a sampling lattice planner, an
/// optimisation-based planner or a learned policy all fit here. The contract
/// is that it consumes only the [WorldState] — never a camera frame — which
/// is what makes planning replayable and unit-testable.
abstract class LocalPlanner {
  String get plannerName;

  /// Plan for this cycle. [previous] is the last path, so the planner can
  /// stay continuous rather than re-deciding from scratch every frame.
  PlannedPath plan(WorldState world, {PlannedPath? previous});

  void reset();
}
