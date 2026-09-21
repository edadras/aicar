import 'package:aicar/core/confidence.dart';
import 'package:aicar/core/geometry.dart';
import 'package:aicar/localization/lateral_estimator.dart';
import 'package:aicar/localization/lateral_state.dart';
import 'package:aicar/navigation/maneuver.dart';
import 'package:aicar/road/lane.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/world_builder.dart';

/// Where we are across the road, and — more importantly — what we admit to
/// not knowing about it.
void main() {
  LaneDetectionResult noLanes() => LaneDetectionResult(
        boundaries: const <LaneBoundary>[],
        mode: LaneMode.noLane,
        frameId: 1,
        timestampMicros: 0,
        laneWidthMeters: 3.5,
        laneWidthConfidence: 0,
        egoLateralOffsetMeters: 0,
        egoHeadingErrorRadians: 0,
        modelName: 'test',
      );

  LaneDetectionResult lanesWithEdge({
    required LineType outerType,
    LineColor outerColor = LineColor.white,
    double offset = 0,
    double confidence = 0.85,
  }) {
    LaneBoundary side(LanePosition position, double lateral, LineType type,
            LineColor colour) =>
        LaneBoundary(
          position: position,
          curve: Polynomial(<double>[lateral + offset, 0, 0]),
          confidence: Confidence(confidence),
          lineType: type,
          color: colour,
          minRangeMeters: 4,
          maxRangeMeters: 40,
          supportPointCount: 20,
        );

    return LaneDetectionResult(
      boundaries: <LaneBoundary>[
        side(LanePosition.egoLeft, -1.75, LineType.dashed, LineColor.white),
        side(LanePosition.egoRight, 1.75, outerType, outerColor),
      ],
      mode: LaneMode.bothBoundaries,
      frameId: 1,
      timestampMicros: 0,
      laneWidthMeters: 3.5,
      laneWidthConfidence: 0.8,
      egoLateralOffsetMeters: -offset,
      egoHeadingErrorRadians: 0,
      modelName: 'test',
    );
  }

  group('what the camera gives', () {
    test('a clean lane observation is a confident offset', () {
      final LateralEstimator e = LateralEstimator();
      final LateralState s = e.update(
        lanes: straightLanes(offset: 0.4),
        ego: testEgo(),
        dtSeconds: 0.05,
      );
      expect(s.source, LateralSource.laneObservation);
      expect(s.offsetInLaneMeters, closeTo(-0.4, 0.01));
      expect(s.isUsable, isTrue);
    });

    test('no markings and no history means no answer', () {
      final LateralEstimator e = LateralEstimator();
      final LateralState s = e.update(
        lanes: noLanes(),
        ego: testEgo(),
        dtSeconds: 0.05,
      );
      expect(s.source, LateralSource.none);
      expect(s.isUsable, isFalse);
      expect(s.description, contains('unknown'));
    });
  });

  group('the IMU bridge', () {
    test('carries the offset across a short dropout', () {
      final LateralEstimator e = LateralEstimator();
      e.update(lanes: straightLanes(offset: 0.3), ego: testEgo(),
          dtSeconds: 0.05);

      final LateralState s = e.update(
        lanes: noLanes(),
        ego: testEgo(),
        dtSeconds: 0.2,
      );
      expect(s.source, LateralSource.deadReckoned);
      expect(s.isUsable, isTrue);
      expect(s.offsetInLaneMeters, closeTo(-0.3, 0.05));
    });

    test('confidence falls off quadratically, not linearly', () {
      final LateralEstimator e = LateralEstimator(maxBridgeSeconds: 2);
      e.update(lanes: straightLanes(), ego: testEgo(), dtSeconds: 0.05);

      double at(double seconds) {
        final LateralEstimator f = LateralEstimator(maxBridgeSeconds: 2);
        f.update(lanes: straightLanes(), ego: testEgo(), dtSeconds: 0.05);
        double t = 0;
        LateralState s = f.state;
        while (t < seconds) {
          s = f.update(lanes: noLanes(), ego: testEgo(), dtSeconds: 0.1);
          t += 0.1;
        }
        return s.offsetConfidence.value;
      }

      // Half the bridge time must cost much less than half the confidence:
      // a double integration's error grows with the square of the gap.
      final double half = at(1.0);
      final double full = at(1.9);
      expect(half, greaterThan(0.6 * at(0.1)));
      expect(full, lessThan(half * 0.5));
    });

    test('the bridge is abandoned rather than stretched', () {
      final LateralEstimator e = LateralEstimator(maxBridgeSeconds: 1.0);
      e.update(lanes: straightLanes(offset: 0.3), ego: testEgo(),
          dtSeconds: 0.05);
      LateralState s = e.state;
      for (int i = 0; i < 20; i++) {
        s = e.update(lanes: noLanes(), ego: testEgo(), dtSeconds: 0.1);
      }
      expect(s.source, LateralSource.none);
      expect(s.isUsable, isFalse);
    });

    test('a yaw during the dropout moves the estimate', () {
      final LateralEstimator e = LateralEstimator();
      e.update(lanes: straightLanes(), ego: testEgo(), dtSeconds: 0.05);
      LateralState s = e.state;
      for (int i = 0; i < 8; i++) {
        s = e.update(
          lanes: noLanes(),
          ego: testEgo(speedMps: 14, yawRate: 0.12),
          dtSeconds: 0.1,
        );
      }
      expect(s.offsetInLaneMeters, greaterThan(0.2),
          reason: 'turning right while blind moves us right of centre');
    });

    test('lane membership is never carried across a dropout', () {
      // Whether we changed lane is exactly what we could not see.
      final LateralEstimator e = LateralEstimator();
      e.update(
        lanes: lanesWithEdge(outerType: LineType.curb),
        ego: testEgo(),
        dtSeconds: 0.05,
      );
      expect(e.state.isLaneLevel, isTrue);

      final LateralState s =
          e.update(lanes: noLanes(), ego: testEgo(), dtSeconds: 0.2);
      expect(s.laneIndexFromEdge, isNull);
      expect(s.isLaneLevel, isFalse);
      expect(s.isUsable, isTrue, reason: 'the offset still survives');
    });
  });

  group('which lane are we in', () {
    test('a kerb on the outer side means the outermost lane', () {
      final LateralEstimator e = LateralEstimator();
      final LateralState s = e.update(
        lanes: lanesWithEdge(outerType: LineType.curb),
        ego: testEgo(),
        dtSeconds: 0.05,
      );
      expect(s.laneIndexFromEdge, 0);
      expect(s.isLaneLevel, isTrue);
    });

    test('ordinary painted lines mean we do not know', () {
      // This is the normal case, and refusing to answer is the point: the
      // decisions built on lane membership are ones you would rather see
      // refused than answered wrongly.
      final LateralEstimator e = LateralEstimator();
      final LateralState s = e.update(
        lanes: lanesWithEdge(outerType: LineType.solid),
        ego: testEgo(),
        dtSeconds: 0.05,
      );
      expect(s.laneIndexFromEdge, isNull);
      expect(s.isLaneLevel, isFalse);
      expect(s.isUsable, isTrue);
      expect(s.description, contains('lane number unknown'));
    });

    test('the lane count is never invented', () {
      final LateralEstimator e = LateralEstimator();
      final LateralState s = e.update(
        lanes: lanesWithEdge(outerType: LineType.curb),
        ego: testEgo(),
        dtSeconds: 0.05,
      );
      expect(s.laneCount, isNull,
          reason: 'no lane-level map data exists in this build');
    });

    test('left-hand traffic looks at the other boundary', () {
      final LateralEstimator e =
          LateralEstimator(drivingSide: DrivingSide.left);
      // A kerb on the *right* is now the inner side, so it proves nothing.
      final LateralState s = e.update(
        lanes: lanesWithEdge(outerType: LineType.curb),
        ego: testEgo(),
        dtSeconds: 0.05,
      );
      expect(s.laneIndexFromEdge, isNull);
    });
  });

  group('map matching as a cross-check', () {
    test('a heading matching the road is reported as agreement', () {
      final LateralEstimator e = LateralEstimator();
      final LateralState s = e.update(
        lanes: straightLanes(),
        ego: testEgo(),
        dtSeconds: 0.05,
        route: testRouteProgress(
          intent: ManeuverIntent.straight,
          distanceToManeuverMeters: 200,
        ),
      );
      expect(s.headingAgreesWithRoad, isTrue);
    });

    test('driving the road the other way is still agreement', () {
      // A polyline has one direction; a road has two.
      final LateralEstimator e = LateralEstimator();
      final LateralState s = e.update(
        lanes: straightLanes(),
        ego: testEgo().copyWith(headingDegrees: 180),
        dtSeconds: 0.05,
        route: testRouteProgress(
          intent: ManeuverIntent.straight,
          distanceToManeuverMeters: 200,
        ),
      );
      expect(s.headingAgreesWithRoad, isTrue);
    });

    test('pointing across the road flags the match as wrong', () {
      final LateralEstimator e = LateralEstimator();
      final LateralState s = e.update(
        lanes: straightLanes(),
        ego: testEgo().copyWith(headingDegrees: 90),
        dtSeconds: 0.05,
        route: testRouteProgress(
          intent: ManeuverIntent.straight,
          distanceToManeuverMeters: 200,
        ),
      );
      expect(s.headingAgreesWithRoad, isFalse);
    });

    test('with no route there is nothing to cross-check', () {
      final LateralEstimator e = LateralEstimator();
      final LateralState s = e.update(
        lanes: straightLanes(),
        ego: testEgo(),
        dtSeconds: 0.05,
      );
      expect(s.headingAgreesWithRoad, isNull);
    });
  });

  test('it survives a recording round trip', () {
    final LateralEstimator e = LateralEstimator();
    final LateralState s = e.update(
      lanes: lanesWithEdge(outerType: LineType.curb, offset: 0.2),
      ego: testEgo(),
      dtSeconds: 0.05,
    );
    final LateralState back = LateralState.fromJson(s.toJson());
    expect(back.offsetInLaneMeters, closeTo(s.offsetInLaneMeters, 0.01));
    expect(back.laneIndexFromEdge, s.laneIndexFromEdge);
    expect(back.source, s.source);
  });
}
