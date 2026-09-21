import 'dart:math' as math;

import '../decision/driving_decision.dart';
import '../navigation/maneuver.dart';
import '../planning/planned_path.dart';
import '../road/lane.dart';
import '../world_model/world_state.dart';
import 'turn_signal.dart';

/// Decides when the indicator would be on.
///
/// Indicating is not a by-product of steering — it has to happen *before* the
/// manoeuvre, which means it is a decision in its own right and belongs here
/// rather than falling out of the controller. Three sources feed it, in
/// priority order: the driving state machine (a turn or a simulated lane
/// change is under way), the route (a turn is coming up), and the planner (we
/// are moving out around something).
///
/// Two behaviours make the difference between this looking right and looking
/// like a fault: a minimum on-time, so a flicker in the planner does not
/// produce a flicker in the lamp, and a cancel condition tied to the
/// manoeuvre finishing rather than to a timer.
class TurnSignalPlanner {
  TurnSignalPlanner({
    this.minOnSeconds = 1.5,
    this.preSignalSeconds = 3.0,
    this.preSignalMinMeters = 30.0,
    this.avoidanceOffsetMeters = 0.45,
    this.cancelOffsetMeters = 0.15,
  });

  /// Once lit, the lamp stays lit at least this long.
  final double minOnSeconds;

  /// Indicate this far ahead of a route turn, in time...
  final double preSignalSeconds;

  /// ...but never less than this in distance, which is what matters at low
  /// speed in town.
  final double preSignalMinMeters;

  /// Planner offset beyond which we are visibly moving over, not just
  /// tracking the lane centre.
  final double avoidanceOffsetMeters;

  /// Offset below which the manoeuvre is over and the lamp cancels.
  final double cancelOffsetMeters;

  TurnSignalState _state = TurnSignalState.off;
  double _heldSeconds = 0;

  TurnSignalState get state => _state;

  void reset() {
    _state = TurnSignalState.off;
    _heldSeconds = 0;
  }

  TurnSignalState update({
    required WorldState world,
    required DrivingDecision decision,
    required PlannedPath path,
    required double dtSeconds,
  }) {
    final TurnSignalState? request = _request(world, decision, path);

    if (request == null) {
      // Nothing wants the lamp on. Honour the minimum on-time before
      // cancelling, so a one-frame drop in the planner's offset does not
      // blink the indicator off and straight back on.
      if (_state.signal.isActive && _heldSeconds < minOnSeconds) {
        _heldSeconds += dtSeconds;
        _state = TurnSignalState(
          signal: _state.signal,
          reason: _state.reason,
          heldSeconds: _heldSeconds,
        );
        return _state;
      }
      _heldSeconds = 0;
      _state = TurnSignalState.off;
      return _state;
    }

    if (request.signal == _state.signal) {
      _heldSeconds += dtSeconds;
    } else {
      _heldSeconds = 0;
    }
    _state = TurnSignalState(
      signal: request.signal,
      reason: request.reason,
      heldSeconds: _heldSeconds,
    );
    return _state;
  }

  /// What, if anything, is asking for the indicator this frame.
  TurnSignalState? _request(
    WorldState world,
    DrivingDecision decision,
    PlannedPath path,
  ) {
    // 1. Stopped where we would be obstructing. Hazards, not an indicator.
    if (decision.state == DrivingState.emergencyBrakeSimulation) {
      return const TurnSignalState(
        signal: TurnSignal.hazard,
        reason: 'simulated emergency stop',
      );
    }
    if ((decision.state == DrivingState.wait ||
            decision.state == DrivingState.uncertain) &&
        world.ego.speedMps < 0.6 &&
        world.lanes.mode != LaneMode.none) {
      return TurnSignalState(
        signal: TurnSignal.hazard,
        reason: 'stopped in a live lane: ${decision.reason}',
      );
    }

    // 2. A turn the state machine has actually committed to.
    if (decision.state == DrivingState.turnLeft) {
      return TurnSignalState(signal: TurnSignal.left, reason: decision.reason);
    }
    if (decision.state == DrivingState.turnRight) {
      return TurnSignalState(signal: TurnSignal.right, reason: decision.reason);
    }

    // 3. A lane change or an avoidance offset. Direction comes from where the
    //    path is actually going, not from what triggered it.
    final double offset = path.lateralOffsetFromReference;
    final bool manoeuvring =
        decision.state == DrivingState.laneChangeSimulation ||
            decision.state == DrivingState.obstacleAvoidanceSimulation;
    final bool alreadyIndicating = _state.signal.isActive &&
        _state.signal != TurnSignal.hazard;
    // Asymmetric thresholds, for two different reasons. Starting needs a
    // clear commitment, so a planner wobble does not light the lamp. Once
    // lit, it stays lit until we are actually back on line — cancelling
    // halfway through a manoeuvre is worse than never indicating. And a
    // manoeuvre the state machine has declared counts from the first
    // centimetre, because a lane change begins before the offset grows.
    final double threshold = (manoeuvring || alreadyIndicating)
        ? cancelOffsetMeters
        : avoidanceOffsetMeters;
    if (offset.abs() > threshold) {
      return TurnSignalState(
        signal: offset < 0 ? TurnSignal.left : TurnSignal.right,
        reason: '${offset.abs().toStringAsFixed(2)} m '
            '${offset < 0 ? 'left' : 'right'} of the lane centre'
            '${manoeuvring ? ' (${decision.state.label})' : ''}',
      );
    }

    // 4. A route turn coming up. Announce it early, the way a driver would.
    final double? toManeuver = world.routeProgress?.distanceToManeuverMeters;
    final ManeuverIntent intent = world.navigationIntent;
    if (toManeuver != null && intent.lateralBiasSign != 0) {
      final double announceAt = math.max(
        preSignalMinMeters,
        world.ego.speedMps * preSignalSeconds,
      );
      if (toManeuver <= announceAt) {
        return TurnSignalState(
          signal: intent.lateralBiasSign < 0
              ? TurnSignal.left
              : TurnSignal.right,
          reason: '${intent.label} in ${toManeuver.toStringAsFixed(0)} m',
        );
      }
    }

    return null;
  }
}
