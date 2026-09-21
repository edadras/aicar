import 'package:aicar/ai/interfaces/object_tracker.dart';
import 'package:aicar/camera/camera_frame.dart';
import 'package:aicar/decision/decision_engine_impl.dart';
import 'package:aicar/decision/driving_decision.dart';
import 'package:aicar/depth/neural_depth_estimator.dart';
import 'package:aicar/perception/neural_object_detector.dart';
import 'package:aicar/perception/traffic_light_recognizer.dart';
import 'package:aicar/perception/traffic_sign_recognizer.dart';
import 'package:aicar/pipeline/perception_pipeline.dart';
import 'package:aicar/pipeline/pipeline_config.dart';
import 'package:aicar/pipeline/pipeline_result.dart';
import 'package:aicar/planning/local_path_planner.dart';
import 'package:aicar/planning/planned_path.dart';
import 'package:aicar/road/classical_lane_detector.dart';
import 'package:aicar/road/heuristic_road_segmenter.dart';
import 'package:aicar/tracking/multi_object_tracker.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/synthetic_road.dart';
import '../support/world_builder.dart';

/// Builds the pipeline exactly as PipelineFactory would with no models
/// installed: classical lanes, heuristic segmentation, geometric depth, and
/// an explicitly-unavailable detector.
PerceptionPipeline buildModelFreePipeline() {
  return PerceptionPipeline(
    objectDetector: UnavailableObjectDetector(),
    tracker: MultiObjectTracker(calibration: testCalibration),
    laneDetector: ClassicalLaneDetector(calibration: testCalibration),
    roadSegmenter: HeuristicRoadSegmenter(),
    depthEstimator: UnavailableDepthEstimator(),
    trafficSignDetector: TrafficSignRecognizer(),
    trafficLightDetector: TrafficLightRecognizer(),
    planner: LocalPathPlanner(),
    decisionEngine: RuleBasedDecisionEngine(),
    calibration: testCalibration,
    config: const PipelineConfig(
      // Run every stage every frame so the test exercises all of them.
      cadence: StageCadence(
        segmentationEveryNFrames: 1,
        depthEveryNFrames: 1,
        laneEveryNFrames: 1,
        signsEveryNFrames: 1,
      ),
    ),
  );
}

void main() {
  group('full pipeline with no models installed', () {
    late PerceptionPipeline pipeline;

    setUp(() => pipeline = buildModelFreePipeline());

    test('processes a synthetic road frame end to end', () async {
      const SyntheticRoad road = SyntheticRoad(calibration: testCalibration);
      final CameraFrame frame = road.renderFrame();

      final PipelineResult result = await pipeline.process(
        frame: frame,
        ego: testEgo(speedMps: 13.9),
      );

      // Lanes are found by the classical detector with no weights at all.
      expect(result.world.lanes.boundaries, isNotEmpty);
      expect(result.world.lanes.left, isNotNull);
      expect(result.world.lanes.right, isNotNull);

      // A path is planned from them.
      expect(result.path.isUsable, isTrue);
      expect(result.path.source, PathSource.laneCenterline);

      // A decision is made, with a reason.
      expect(result.decision.reason, isNotEmpty);

      // And a simulated command is produced.
      expect(result.command.throttlePercent, inInclusiveRange(0, 100));
      expect(result.command.brakePercent, inInclusiveRange(0, 100));
      expect(result.command.isSimulationOnly, isTrue);

      expect(result.totalLatencyMicros, greaterThan(0));
    });

    test('reports the missing detector rather than an empty road', () async {
      const SyntheticRoad road = SyntheticRoad(calibration: testCalibration);
      final PipelineResult result = await pipeline.process(
        frame: road.renderFrame(),
        ego: testEgo(),
      );

      expect(result.world.tracks, isEmpty);
      expect(
        result.world.degradedSubsystems.join(' '),
        contains('no object detection model installed'),
      );
      expect(result.world.autonomy.perception, 0);
      expect(result.world.autonomy.isLow, isTrue);
      expect(result.decision.state, DrivingState.uncertain);
      // UNCERTAIN must never command acceleration.
      expect(result.command.throttlePercent, 0);
    });

    test('runs a sequence of frames without drifting or throwing', () async {
      const double dt = 0.05;
      PipelineResult? last;

      for (int i = 0; i < 12; i++) {
        final SyntheticRoad road = SyntheticRoad(
          calibration: testCalibration,
          // A gentle curve developing over the sequence.
          curvature: 0.0001 * i,
        );
        last = await pipeline.process(
          frame: road.renderFrame(
            id: i,
            timestampMicros: (i * dt * 1e6).round(),
          ),
          ego: testEgo(speedMps: 13.9, ts: (i * dt * 1e6).round()),
        );
      }

      expect(last, isNotNull);
      expect(last!.world.frameId, 11);
      expect(last.path.isUsable, isTrue);
      // The simulated vehicle has integrated forward.
      expect(last.vehicle.distanceTravelledMeters, greaterThan(0));
      // Timings were collected for every stage that ran.
      expect(last.stageTimings.keys, contains('Lane Detection'));
      expect(last.stageTimings.keys, contains('Total Pipeline'));
    });

    test('reset clears all cross-frame state', () async {
      const SyntheticRoad road = SyntheticRoad(calibration: testCalibration);
      for (int i = 0; i < 4; i++) {
        await pipeline.process(
          frame: road.renderFrame(id: i, timestampMicros: i * 50000),
          ego: testEgo(ts: i * 50000),
        );
      }
      expect(pipeline.lastPath, isNotNull);
      expect(pipeline.vehicleState.distanceTravelledMeters,
          greaterThanOrEqualTo(0));

      pipeline.reset();

      expect(pipeline.lastPath, isNull);
      expect(pipeline.vehicleState.speedMps, 0);
      expect(pipeline.vehicleState.distanceTravelledMeters, 0);
      expect((pipeline.tracker as MultiObjectTracker).allTracks, isEmpty);
    });

    test('an unmarked road does not produce a fabricated lane', () async {
      const SyntheticRoad blank = SyntheticRoad(
        calibration: testCalibration,
        laneCenters: <double>[],
      );
      final PipelineResult result = await pipeline.process(
        frame: blank.renderFrame(),
        ego: testEgo(),
      );

      expect(result.world.lanes.boundaries, isEmpty);
      // Either a corridor backed by real evidence, or no path at all —
      // never an invented lane.
      if (result.path.isUsable) {
        expect(result.path.source, PathSource.corridorCenterline);
        expect(result.world.corridor, isNotNull);
        expect(result.world.corridor!.evidence, isNotEmpty);
      } else {
        expect(result.decision.state, DrivingState.uncertain);
      }
    });

    test('a long time gap does not corrupt the next result', () async {
      const SyntheticRoad road = SyntheticRoad(calibration: testCalibration);
      await pipeline.process(
        frame: road.renderFrame(id: 0, timestampMicros: 0),
        ego: testEgo(ts: 0),
      );

      // The app was backgrounded for ten seconds.
      final PipelineResult after = await pipeline.process(
        frame: road.renderFrame(id: 1, timestampMicros: 10000000),
        ego: testEgo(ts: 10000000),
      );

      expect(after.world.frameId, 1);
      expect(after.command.steeringAngleDegrees.isFinite, isTrue);
      expect(after.vehicle.speedMps.isFinite, isTrue);
    });
  });

  group('tracker interface', () {
    test('the pipeline exposes its tracker through the interface', () {
      final PerceptionPipeline pipeline = buildModelFreePipeline();
      expect(pipeline.tracker, isA<ObjectTracker>());
      expect(pipeline.tracker.trackerName, isNotEmpty);
    });
  });
}
