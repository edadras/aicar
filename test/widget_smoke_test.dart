import 'package:aicar/core/confidence.dart';
import 'package:aicar/core/geometry.dart';
import 'package:aicar/decision/driving_decision.dart';
import 'package:aicar/perception/object_class.dart';
import 'package:aicar/planning/local_path_planner.dart';
import 'package:aicar/planning/planned_path.dart';
import 'package:aicar/simulation/simulated_control.dart';
import 'package:aicar/simulation/vehicle_state.dart';
import 'package:aicar/tracking/object_track.dart';
import 'package:aicar/pipeline/pipeline_result.dart';
import 'package:aicar/ui/hud/hud_panels.dart';
import 'package:aicar/ui/hud/perception_overlay.dart';
import 'package:aicar/ui/hud/steering_wheel.dart';
import 'package:aicar/ui/theme.dart';
import 'package:aicar/world_model/hazard.dart';
import 'package:aicar/world_model/world_state.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'support/world_builder.dart';

/// The HUD always lays its panels out inside a bounded region (a Positioned
/// with left/right, or a Row), so the test harness provides the same.
Widget wrap(Widget child, {double width = 880}) => MaterialApp(
      theme: HudTheme.theme(),
      home: Scaffold(
        body: Center(
          child: SizedBox(width: width, child: child),
        ),
      ),
    );

PipelineResult resultFor(WorldState world) {
  final PlannedPath path = LocalPathPlanner().plan(world);
  return PipelineResult(
    world: world,
    path: path,
    decision: DrivingDecision(
      state: DrivingState.cruise,
      reason: 'Path clear for 40 m',
      confidence: 0.86,
      timestampMicros: world.timestampMicros,
      frameId: world.frameId,
      targetSpeedMps: 13.9,
    ),
    command: SimulatedControlCommand(
      steeringAngleDegrees: -7,
      throttlePercent: 18,
      brakePercent: 0,
      timestampMicros: world.timestampMicros,
      frameId: world.frameId,
    ),
    vehicle: VehicleState.stationary().copyWith(speedMps: 11.9),
    collisions: const <dynamic>[].cast(),
    totalLatencyMicros: 42000,
    stageTimings: const <String, double>{'Total Pipeline': 42},
  );
}

void main() {
  group('HUD widgets render', () {
    testWidgets('control strip shows speed, steering, throttle and brake',
        (WidgetTester tester) async {
      await tester.binding.setSurfaceSize(const Size(900, 200));
      await tester.pumpWidget(wrap(ControlStrip(
        result: resultFor(testWorld()),
        steeringRatio: 15,
        steeringLimitDegrees: 35,
      )));

      expect(find.text('SPEED'), findsOneWidget);
      expect(find.text('STEERING'), findsOneWidget);
      expect(find.text('THROTTLE'), findsOneWidget);
      expect(find.text('BRAKE'), findsOneWidget);
      // 13.9 m/s = 50 km/h
      expect(find.text('50'), findsOneWidget);
      expect(find.text('-7°'), findsOneWidget);
      expect(find.text('18%'), findsOneWidget);
      await tester.binding.setSurfaceSize(null);
    });

    testWidgets('steering readout is blank when there is no usable path',
        (WidgetTester tester) async {
      await tester.binding.setSurfaceSize(const Size(900, 200));
      final PipelineResult base = resultFor(testWorld());
      final PipelineResult noPath = PipelineResult(
        world: base.world,
        path: PlannedPath.none(frameId: 1, timestampMicros: 0),
        decision: base.decision,
        command: base.command,
        vehicle: base.vehicle,
        collisions: base.collisions,
        totalLatencyMicros: base.totalLatencyMicros,
        stageTimings: base.stageTimings,
      );

      await tester.pumpWidget(wrap(ControlStrip(
        result: noPath,
        steeringRatio: 15,
        steeringLimitDegrees: 35,
      )));

      // "I do not know" must look different from "straight ahead".
      expect(find.text('—'), findsOneWidget);
      expect(find.text('-7°'), findsNothing);
      await tester.binding.setSurfaceSize(null);
    });

    testWidgets('decision panel shows the reason and confidence',
        (WidgetTester tester) async {
      await tester.pumpWidget(wrap(DecisionPanel(
        decision: const DrivingDecision(
          state: DrivingState.slowDown,
          reason: 'Motorcycle entering predicted path',
          confidence: 0.89,
          timestampMicros: 0,
          frameId: 0,
        ),
      )));

      expect(find.text('SLOW_DOWN'), findsOneWidget);
      expect(find.text('Motorcycle entering predicted path'), findsOneWidget);
      expect(find.text('89%'), findsOneWidget);
    });

    testWidgets('hazard banner shows a critical hazard with its TTC',
        (WidgetTester tester) async {
      await tester.binding.setSurfaceSize(const Size(900, 200));
      await tester.pumpWidget(wrap(HazardBanner(
        world: testWorld(
          hazards: const <Hazard>[
            Hazard(
              type: HazardType.collisionImminent,
              severity: HazardSeverity.critical,
              description: 'PEDESTRIAN #3 at 12.3 m, crossing',
              confidence: 0.9,
              distanceMeters: 12.3,
              timeToCollisionSeconds: 2.1,
            ),
          ],
        ),
      )));

      expect(find.text('COLLISION RISK'), findsOneWidget);
      expect(find.text('2.1'), findsOneWidget);
      await tester.binding.setSurfaceSize(null);
    });

    testWidgets('banner shows AUTONOMY CONFIDENCE LOW when confidence drops',
        (WidgetTester tester) async {
      await tester.binding.setSurfaceSize(const Size(900, 200));
      await tester.pumpWidget(wrap(HazardBanner(
        world: testWorld(
          autonomy: AutonomyConfidence.compute(
            perception: 0.05,
            lanes: 0.2,
            depth: 0.2,
            egoMotion: 0.4,
            planning: 0.1,
          ),
        ),
      )));

      expect(find.text('AUTONOMY CONFIDENCE LOW'), findsOneWidget);
      await tester.binding.setSurfaceSize(null);
    });

    testWidgets('the simulation-only badge is always available',
        (WidgetTester tester) async {
      await tester.pumpWidget(
          wrap(const SimulationOnlyBadge(compact: true), width: 300));
      expect(find.text('SIMULATION ONLY'), findsOneWidget);
    });

    testWidgets('steering wheel renders at any angle',
        (WidgetTester tester) async {
      for (final double angle in <double>[-35, -12, 0, 22, 35]) {
        await tester.pumpWidget(wrap(
          SteeringWheelIndicator(roadWheelDegrees: angle),
          width: 120,
        ));
        expect(tester.takeException(), isNull);
      }
    });

    testWidgets('confidence panel lists every subsystem',
        (WidgetTester tester) async {
      await tester.binding.setSurfaceSize(const Size(500, 400));
      await tester.pumpWidget(wrap(AutonomyConfidencePanel(
        autonomy: AutonomyConfidence.compute(
          perception: 0.8,
          lanes: 0.7,
          depth: 0.6,
          egoMotion: 0.9,
          planning: 0.75,
        ),
      )));

      for (final String label in <String>[
        'Perception',
        'Lanes',
        'Depth',
        'Ego',
        'Plan',
      ]) {
        expect(find.text(label), findsOneWidget);
      }
      await tester.binding.setSurfaceSize(null);
    });
  });

  group('perception overlay painter', () {
    testWidgets('paints a populated scene without throwing',
        (WidgetTester tester) async {
      final WorldState world = testWorld(
        tracks: <ObjectTrack>[
          testTrack(
            id: 12,
            objectClass: ObjectClass.car,
            position: const Vec2(0.3, 18),
            risk: CollisionRisk.high,
            ttc: 2.4,
          ),
          testTrack(
            id: 18,
            objectClass: ObjectClass.motorcycle,
            position: const Vec2(2.2, 14),
            relativeVelocity: const Vec2(-1.8, -2),
            direction: MotionDirection.crossingRightToLeft,
            laneRelation: LaneRelation.crossing,
            risk: CollisionRisk.critical,
            ttc: 1.2,
          ),
        ],
      );

      await tester.binding.setSurfaceSize(const Size(800, 450));
      await tester.pumpWidget(wrap(
        SizedBox(
          width: 800,
          height: 450,
          child: CustomPaint(
            painter: PerceptionOverlayPainter(
              world: world,
              path: LocalPathPlanner().plan(world),
              options: const OverlayOptions(
                roadEdges: true,
                predictedPaths: true,
                horizonLine: true,
              ),
              pulse: 0.5,
            ),
          ),
        ),
      ));

      expect(tester.takeException(), isNull);
      await tester.binding.setSurfaceSize(null);
    });

    testWidgets('paints an empty scene without throwing',
        (WidgetTester tester) async {
      await tester.binding.setSurfaceSize(const Size(800, 450));
      await tester.pumpWidget(wrap(
        SizedBox(
          width: 800,
          height: 450,
          child: CustomPaint(
            painter: PerceptionOverlayPainter(
              world: WorldState.initial(),
              path: null,
              options: const OverlayOptions(),
            ),
          ),
        ),
      ));
      expect(tester.takeException(), isNull);
      await tester.binding.setSurfaceSize(null);
    });
  });
}
