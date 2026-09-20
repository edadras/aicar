import '../../decision/driving_decision.dart';
import '../../planning/collision_predictor.dart';
import '../../planning/planned_path.dart';
import '../../world_model/world_state.dart';

/// Chooses what the vehicle would do.
///
/// Swappable like every other stage: a rule-based state machine (the default),
/// a behaviour tree or a learned policy all satisfy this. The interface takes
/// only the world model, the planned path and the collision assessments, so a
/// replacement cannot quietly reach around into raw perception.
abstract class DecisionEngine {
  String get engineName;

  DrivingDecision decide({
    required WorldState world,
    required PlannedPath path,
    required List<CollisionAssessment> collisions,
  });

  /// The current state, for hysteresis and for the UI.
  DrivingDecision? get current;

  void reset();
}
