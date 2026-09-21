import 'package:aicar/decision/driving_decision.dart';
import 'package:aicar/navigation/maneuver.dart';
import 'package:aicar/planning/planned_path.dart';
import 'package:aicar/simulation/turn_signal.dart';
import 'package:aicar/simulation/turn_signal_planner.dart';
import 'package:aicar/world_model/world_state.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/world_builder.dart';

void main() {
  DrivingDecision decide(DrivingState state, {String reason = 'test'}) =>
      DrivingDecision(
        state: state,
        reason: reason,
        confidence: 0.8,
        timestampMicros: 0,
        frameId: 1,
      );

  PlannedPath straightPath({double offset = 0}) =>
      testPath(lateralOffset: offset);

  group('what turns it on', () {
    test('nothing happening means no indicator', () {
      final TurnSignalPlanner p = TurnSignalPlanner();
      final TurnSignalState s = p.update(
        world: testWorld(),
        decision: decide(DrivingState.cruise),
        path: straightPath(),
        dtSeconds: 0.05,
      );
      expect(s.signal, TurnSignal.none);
    });

    test('a committed left turn indicates left', () {
      final TurnSignalPlanner p = TurnSignalPlanner();
      final TurnSignalState s = p.update(
        world: testWorld(),
        decision: decide(DrivingState.turnLeft, reason: 'TURN_LEFT in 20 m'),
        path: straightPath(),
        dtSeconds: 0.05,
      );
      expect(s.signal, TurnSignal.left);
      expect(s.reason, contains('TURN_LEFT'));
    });

    test('moving out around an obstruction indicates that way', () {
      final TurnSignalPlanner p = TurnSignalPlanner();
      final TurnSignalState s = p.update(
        world: testWorld(),
        decision: decide(DrivingState.obstacleAvoidanceSimulation),
        path: straightPath(offset: -0.8),
        dtSeconds: 0.05,
      );
      expect(s.signal, TurnSignal.left);
    });

    test('a simulated emergency stop shows hazards, not an indicator', () {
      final TurnSignalPlanner p = TurnSignalPlanner();
      final TurnSignalState s = p.update(
        world: testWorld(),
        decision: decide(DrivingState.emergencyBrakeSimulation),
        path: straightPath(),
        dtSeconds: 0.05,
      );
      expect(s.signal, TurnSignal.hazard);
      expect(s.signal.showsLeft && s.signal.showsRight, isTrue);
    });

    test('a route turn is announced before the manoeuvre begins', () {
      final TurnSignalPlanner p = TurnSignalPlanner();
      final WorldState world = testWorld(
        ego: testEgo(speedMps: 8),
      ).copyWith(
        routeProgress: testRouteProgress(
          intent: ManeuverIntent.turnRight,
          distanceToManeuverMeters: 26,
        ),
      );
      final TurnSignalState s = p.update(
        world: world,
        decision: decide(DrivingState.cruise),
        path: straightPath(),
        dtSeconds: 0.05,
      );
      expect(s.signal, TurnSignal.right,
          reason: 'the lamp comes on before the wheel moves');
      expect(s.reason, contains('26'));
    });

    test('a route turn far away does not', () {
      final TurnSignalPlanner p = TurnSignalPlanner();
      final WorldState world = testWorld(ego: testEgo(speedMps: 8)).copyWith(
        routeProgress: testRouteProgress(
          intent: ManeuverIntent.turnRight,
          distanceToManeuverMeters: 180,
        ),
      );
      final TurnSignalState s = p.update(
        world: world,
        decision: decide(DrivingState.cruise),
        path: straightPath(),
        dtSeconds: 0.05,
      );
      expect(s.signal, TurnSignal.none);
    });
  });

  group('how it behaves over time', () {
    test('it does not strobe when the trigger flickers for one frame', () {
      final TurnSignalPlanner p = TurnSignalPlanner();
      p.update(
        world: testWorld(),
        decision: decide(DrivingState.obstacleAvoidanceSimulation),
        path: straightPath(offset: -0.8),
        dtSeconds: 0.05,
      );
      // One frame where the planner momentarily reports no offset.
      final TurnSignalState s = p.update(
        world: testWorld(),
        decision: decide(DrivingState.cruise),
        path: straightPath(),
        dtSeconds: 0.05,
      );
      expect(s.signal, TurnSignal.left,
          reason: 'the minimum on-time keeps the lamp lit');
    });

    test('it cancels once the manoeuvre is genuinely over', () {
      final TurnSignalPlanner p = TurnSignalPlanner();
      p.update(
        world: testWorld(),
        decision: decide(DrivingState.obstacleAvoidanceSimulation),
        path: straightPath(offset: -0.8),
        dtSeconds: 0.05,
      );
      for (int i = 0; i < 40; i++) {
        p.update(
          world: testWorld(),
          decision: decide(DrivingState.cruise),
          path: straightPath(),
          dtSeconds: 0.05,
        );
      }
      expect(p.state.signal, TurnSignal.none);
    });

    test('it keeps indicating while still out of line', () {
      final TurnSignalPlanner p = TurnSignalPlanner();
      p.update(
        world: testWorld(),
        decision: decide(DrivingState.obstacleAvoidanceSimulation),
        path: straightPath(offset: -0.8),
        dtSeconds: 0.05,
      );
      // Coming back in, but not back yet: below the start threshold, above
      // the cancel threshold.
      for (int i = 0; i < 40; i++) {
        p.update(
          world: testWorld(),
          decision: decide(DrivingState.cruise),
          path: straightPath(offset: -0.3),
          dtSeconds: 0.05,
        );
      }
      expect(p.state.signal, TurnSignal.left);
    });
  });

  group('blink phase', () {
    test('comes from the timestamp so a replay matches the drive', () {
      const TurnSignalState s =
          TurnSignalState(signal: TurnSignal.left, reason: 'x');
      expect(s.blinkOn(0), isTrue);
      expect(s.blinkOn(340000), isFalse);
      expect(s.blinkOn(700000), isTrue);
      // Same timestamp, same lamp, every time it is asked.
      expect(s.blinkOn(340000), s.blinkOn(340000));
    });

    test('an off indicator never blinks', () {
      expect(TurnSignalState.off.blinkOn(0), isFalse);
    });
  });

  test('it survives a recording round trip', () {
    const TurnSignalState s = TurnSignalState(
      signal: TurnSignal.right,
      reason: 'TURN_RIGHT in 18 m',
    );
    final TurnSignalState back = TurnSignalState.fromJson(s.toJson());
    expect(back.signal, TurnSignal.right);
    expect(back.reason, s.reason);
  });
}
