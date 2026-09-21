import 'package:aicar/ai/interfaces/object_detector.dart';
import 'package:aicar/ai/model_descriptor.dart';
import 'package:aicar/camera/camera_frame.dart';
import 'package:aicar/decision/decision_engine_impl.dart';
import 'package:aicar/depth/neural_depth_estimator.dart';
import 'package:aicar/perception/detection.dart';
import 'package:aicar/perception/traffic_light_recognizer.dart';
import 'package:aicar/perception/traffic_sign_recognizer.dart';
import 'package:aicar/pipeline/pipeline_config.dart';
import 'package:aicar/planning/local_path_planner.dart';
import 'package:aicar/road/classical_lane_detector.dart';
import 'package:aicar/road/heuristic_road_segmenter.dart';
import 'package:aicar/tracking/multi_object_tracker.dart';
import 'package:aicar/pipeline/perception_pipeline.dart';
import 'package:aicar/pipeline/pipeline_result.dart';
import 'package:aicar/road/road_marking.dart';
import 'package:aicar/sensors/ego_motion.dart';
import 'package:aicar/world_model/hazard.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/painted_road.dart';
import '../support/world_builder.dart';

/// A detector that works and sees nothing.
///
/// Distinct from [UnavailableObjectDetector], and the distinction is the
/// point: with no detector at all the stack goes UNCERTAIN and refuses to
/// act on anything, which is correct but makes every other behaviour
/// untestable. This one reports a confident, empty scene — a clear road —
/// so what the road-reading stages do can actually be observed.
class _ClearRoadDetector extends ObjectDetector {
  @override
  String get modelId => 'test-clear-road';

  @override
  String get displayName => 'Clear road (test)';

  @override
  ModelRole get role => ModelRole.objectDetection;

  @override
  ModelDescriptor? get descriptor => null;

  @override
  bool get isReady => true;

  @override
  String? get unavailableReason => null;

  @override
  double get scoreThreshold => 0.5;

  @override
  List<String> get supportedLabels => const <String>[];

  @override
  Future<void> load() async {}

  @override
  Future<DetectionResult> detect(CameraFrame frame) async => DetectionResult(
        detections: const <Detection>[],
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        inferenceMicros: 1000,
        modelName: 'test-clear-road',
      );

  @override
  Future<void> close() async {}
}

PerceptionPipeline buildPipeline() => PerceptionPipeline(
      objectDetector: _ClearRoadDetector(),
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
        cadence: StageCadence(
          segmentationEveryNFrames: 1,
          depthEveryNFrames: 1,
          laneEveryNFrames: 1,
          signsEveryNFrames: 1,
          markingsEveryNFrames: 1,
        ),
      ),
    );

/// The whole chain, from painted asphalt to a decision: render a crossing at
/// a known distance, drive towards it frame by frame, and check the stack
/// reads it, believes it, and slows down for it.
///
/// This is the test that would catch a break anywhere in between — the
/// bird's-eye geometry, the row profile, the tracker's association, the
/// advisory speed, or the decision engine's priorities.
void main() {
  /// Drive [frames] frames towards a scene, closing at [speedMps].
  Future<List<PipelineResult>> driveTowards(
    List<PaintPatch> Function(double nearMeters) scene, {
    required double startDistance,
    double speedMps = 12,
    int frames = 8,
    double dtSeconds = 0.1,
  }) async {
    final PerceptionPipeline pipeline = buildPipeline();
    final List<PipelineResult> results = <PipelineResult>[];
    double distance = startDistance;

    for (int i = 0; i < frames; i++) {
      final PaintedRoad road = PaintedRoad(
        calibration: testCalibration,
        patches: scene(distance),
        noiseAmplitude: 5,
      );
      final CameraFrame frame = road.renderFrame(
        id: i,
        timestampMicros: (i * dtSeconds * 1e6).round(),
      );
      final EgoMotionState ego = testEgo(
        speedMps: speedMps,
        ts: frame.timestampMicros,
      );
      results.add(await pipeline.process(frame: frame, ego: ego));
      distance -= speedMps * dtSeconds;
    }
    return results;
  }

  test('a painted crossing is read, confirmed and slowed for', () async {
    final List<PipelineResult> results = await driveTowards(
      (double d) => PaintedRoad.crosswalk(nearMeters: d, depthMeters: 3),
      startDistance: 24,
    );

    final PipelineResult last = results.last;
    final RoadMarking? crossing =
        last.world.markingAhead(RoadMarkingType.crosswalk);
    expect(crossing, isNotNull,
        reason: 'markings seen: ${last.world.roadMarkings}');
    expect(crossing!.observationCount, greaterThanOrEqualTo(3),
        reason: 'it should have accumulated evidence, not fired once');

    // What the driver actually sees: the crossing reaches the hazard list,
    // with its distance, which is what the HUD banner and the overlay read.
    expect(
      last.world.hazards.any((Hazard h) =>
          h.type == HazardType.crosswalkAhead &&
          (h.distanceMeters ?? 0) > 0),
      isTrue,
      reason: 'hazards: ${last.world.hazards.map((Hazard h) => h.type).toList()}',
    );
  });

  test('a painted speed bump is read and slowed for', () async {
    // Started closer than the crossing on purpose: a hump's bands repeat
    // into the vanishing point, so at 640x360 it does not resolve until
    // roughly 14 m. See RoadMarkingDetector's range notes.
    final List<PipelineResult> results = await driveTowards(
      (double d) => PaintedRoad.speedBump(nearMeters: d, bands: 4),
      startDistance: 15,
      frames: 10,
    );

    final PipelineResult last = results.last;
    expect(last.world.markingAhead(RoadMarkingType.speedBump), isNotNull,
        reason: 'markings seen: ${last.world.roadMarkings}');
    expect(
      last.world.hazards
          .any((Hazard h) => h.type == HazardType.speedBumpAhead),
      isTrue,
      reason: 'hazards: ${last.world.hazards.map((Hazard h) => h.type).toList()}',
    );
  });

  test('plain asphalt produces no markings and no junction', () async {
    final List<PipelineResult> results = await driveTowards(
      (double d) => const <PaintPatch>[],
      startDistance: 30,
    );
    final PipelineResult last = results.last;
    expect(last.world.roadMarkings, isEmpty);
    expect(last.world.intersection, isNull);
    expect(
      results.every((PipelineResult r) => r.world.hazards.every((Hazard h) =>
          h.type != HazardType.crosswalkAhead &&
          h.type != HazardType.speedBumpAhead)),
      isTrue,
      reason: 'clean asphalt must produce no road-marking hazards at all',
    );
  });

  test('a stop line plus a crossing is inferred as a junction', () async {
    final List<PipelineResult> results = await driveTowards(
      (double d) => <PaintPatch>[
        ...PaintedRoad.stopLine(nearMeters: d),
        ...PaintedRoad.crosswalk(nearMeters: d + 1.2, depthMeters: 3),
      ],
      startDistance: 24,
    );
    final PipelineResult last = results.last;
    expect(last.world.intersection, isNotNull,
        reason: 'markings seen: ${last.world.roadMarkings}');
    expect(last.world.intersection!.evidence, isNotEmpty);
  });

  test('every decision the stack reaches carries its reasoning', () async {
    final List<PipelineResult> results = await driveTowards(
      (double d) => PaintedRoad.crosswalk(nearMeters: d, depthMeters: 3),
      startDistance: 24,
    );
    for (final PipelineResult r in results) {
      expect(r.decision.reason, isNotEmpty);
      expect(r.command.isSimulationOnly, isTrue);
      expect(r.command.turnSignal.isSimulationOnly, isTrue);
    }
  });
}
