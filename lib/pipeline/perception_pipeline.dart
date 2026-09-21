import 'dart:math' as math;

import '../ai/interfaces/decision_engine.dart';
import '../ai/interfaces/depth_estimator.dart';
import '../ai/interfaces/lane_detector.dart';
import '../ai/interfaces/local_planner.dart';
import '../ai/interfaces/object_detector.dart';
import '../ai/interfaces/object_tracker.dart';
import '../ai/interfaces/road_segmenter.dart';
import '../ai/interfaces/traffic_light_detector.dart';
import '../ai/interfaces/traffic_sign_detector.dart';
import '../camera/camera_calibration.dart';
import '../camera/camera_frame.dart';
import '../camera/image_preprocessing.dart';
import '../core/logging.dart';
import '../core/profiling.dart';
import '../debug/system_monitor.dart';
import '../decision/driving_decision.dart';
import '../depth/depth_fusion.dart';
import '../depth/depth_map.dart';
import '../navigation/maneuver.dart';
import '../navigation/route.dart';
import '../perception/detection.dart';
import '../perception/regulatory_context_tracker.dart';
import '../perception/traffic_light.dart';
import '../perception/traffic_light_recognizer.dart';
import '../perception/traffic_sign.dart';
import '../perception/traffic_sign_recognizer.dart';
import '../planning/collision_predictor.dart';
import '../planning/planned_path.dart';
import '../road/birds_eye_view.dart';
import '../road/drivable_area_builder.dart';
import '../road/lane.dart';
import '../road/intersection_detector.dart';
import '../road/no_lane_corridor.dart';
import '../road/road_edge_detector.dart';
import '../road/road_marking.dart';
import '../road/road_marking_detector.dart';
import '../road/road_marking_tracker.dart';
import '../road/road_segmentation.dart';
import '../sensors/ego_motion.dart';
import '../simulation/bicycle_model.dart';
import '../simulation/simulated_control.dart';
import '../simulation/simulated_vehicle_controller.dart';
import '../simulation/turn_signal_planner.dart';
import '../simulation/vehicle_state.dart';
import '../tracking/multi_object_tracker.dart';
import '../tracking/object_track.dart';
import '../world_model/world_model_builder.dart';
import '../world_model/world_state.dart';
import 'pipeline_config.dart';
import 'pipeline_result.dart';
import 'thermal_governor.dart';

/// Runs the full perception → planning → decision → simulation cycle for one
/// camera frame.
///
/// Every stage is injected, so this class contains only the *order* of the
/// pipeline and the data flowing between stages — not the algorithms. That is
/// what makes a model swap a construction-site change rather than a surgery.
///
/// Stages that fail degrade the frame and are named in
/// [WorldState.degradedSubsystems]; they never throw out of [process],
/// because a pipeline that dies on one bad frame is useless in a car.
class PerceptionPipeline {
  PerceptionPipeline({
    required this.objectDetector,
    required this.tracker,
    required this.laneDetector,
    required this.roadSegmenter,
    required this.depthEstimator,
    required this.trafficSignDetector,
    required this.trafficLightDetector,
    required this.planner,
    required this.decisionEngine,
    required this.calibration,
    PipelineConfig? config,
    PipelineProfiler? profiler,
    DepthFusion? depthFusion,
    DrivableAreaBuilder? drivableAreaBuilder,
    RoadEdgeDetector? roadEdgeDetector,
    NoLaneCorridorEstimator? corridorEstimator,
    CollisionPredictor? collisionPredictor,
    WorldModelBuilder? worldModelBuilder,
    RegulatoryContextTracker? regulatoryTracker,
    RoadMarkingDetector? roadMarkingDetector,
    RoadMarkingTracker? roadMarkingTracker,
    IntersectionDetector? intersectionDetector,
    TurnSignalPlanner? turnSignalPlanner,
    ThermalGovernor? governor,
    SimulatedVehicleController? controller,
    KinematicBicycleModel? vehicleModel,
  })  : config = config ?? const PipelineConfig(),
        profiler = profiler ?? PipelineProfiler(),
        depthFusion = depthFusion ?? const DepthFusion(),
        drivableAreaBuilder =
            drivableAreaBuilder ?? const DrivableAreaBuilder(),
        roadEdgeDetector = roadEdgeDetector ?? const RoadEdgeDetector(),
        corridorEstimator =
            corridorEstimator ?? const NoLaneCorridorEstimator(),
        collisionPredictor =
            collisionPredictor ?? const CollisionPredictor(),
        worldModelBuilder = worldModelBuilder ?? const WorldModelBuilder(),
        regulatoryTracker =
            regulatoryTracker ?? RegulatoryContextTracker(),
        roadMarkingDetector =
            roadMarkingDetector ?? RoadMarkingDetector(),
        roadMarkingTracker = roadMarkingTracker ?? RoadMarkingTracker(),
        intersectionDetector =
            intersectionDetector ?? const IntersectionDetector(),
        turnSignalPlanner = turnSignalPlanner ?? TurnSignalPlanner(),
        governor = governor ??
            ThermalGovernor(
              baseTargetFps:
                  (config ?? const PipelineConfig()).camera.targetInferenceFps,
            ),
        vehicleModel = vehicleModel ??
            KinematicBicycleModel(
              parameters: (config ?? const PipelineConfig()).vehicle,
            ),
        controller = controller ??
            SimulatedVehicleController(
              parameters: (config ?? const PipelineConfig()).vehicle,
              steeringLimitDegrees:
                  (config ?? const PipelineConfig()).steeringLimitDegrees,
            );

  static const String _tag = 'Pipeline';

  ObjectDetector objectDetector;
  ObjectTracker tracker;
  LaneDetector laneDetector;
  RoadSegmenter roadSegmenter;
  DepthEstimator depthEstimator;
  TrafficSignDetector trafficSignDetector;
  TrafficLightDetector trafficLightDetector;
  LocalPlanner planner;
  DecisionEngine decisionEngine;

  CameraCalibration calibration;
  PipelineConfig config;

  final PipelineProfiler profiler;
  final DepthFusion depthFusion;
  final DrivableAreaBuilder drivableAreaBuilder;
  final RoadEdgeDetector roadEdgeDetector;
  final NoLaneCorridorEstimator corridorEstimator;
  final CollisionPredictor collisionPredictor;
  final WorldModelBuilder worldModelBuilder;
  final RegulatoryContextTracker regulatoryTracker;
  final RoadMarkingDetector roadMarkingDetector;
  final RoadMarkingTracker roadMarkingTracker;
  final IntersectionDetector intersectionDetector;
  final TurnSignalPlanner turnSignalPlanner;

  /// Decides how hard the pipeline may work given heat, battery and latency.
  final ThermalGovernor governor;

  final SimulatedVehicleController controller;
  final KinematicBicycleModel vehicleModel;

  // --- Cross-frame state --------------------------------------------------

  BirdsEyeView? _bev;
  int _frameIndex = 0;
  int _lastTimestampMicros = -1;

  RoadSegmentation? _lastSegmentation;
  DepthMap? _lastDepth;
  LaneDetectionResult? _lastLanes;
  LaneDetectionResult? _lastGoodLanes;
  int? _lastGoodLanesMicros;
  List<TrafficSign> _lastSigns = const <TrafficSign>[];
  List<TrafficLight> _lastLights = const <TrafficLight>[];
  double _lastMarkingUpdateMeters = 0;

  PlannedPath? _lastPath;
  VehicleState _vehicleState = VehicleState.stationary();
  final Map<int, _TrackDistanceMemory> _distanceMemory =
      <int, _TrackDistanceMemory>{};

  VehicleState get vehicleState => _vehicleState;
  PlannedPath? get lastPath => _lastPath;

  /// Process one frame end to end.
  Future<PipelineResult> process({
    required CameraFrame frame,
    required EgoMotionState ego,
    RouteProgress? routeProgress,
    SystemSample? systemSample,
  }) async {
    final int cycleStart = DateTime.now().microsecondsSinceEpoch;
    final List<String> degraded = <String>[];
    final Map<String, double> timings = <String, double>{};

    final double dt = _lastTimestampMicros < 0
        ? 0.05
        : math.max(
            0.001,
            (frame.timestampMicros - _lastTimestampMicros) / 1e6,
          );
    _lastTimestampMicros = frame.timestampMicros;
    _frameIndex++;

    regulatoryTracker.advance(ego.speedMps, dt);
    _syncCalibration(frame);

    // --- 0. how hard are we allowed to work this cycle? -------------------
    //
    // Evaluated before anything expensive runs, so the plan applies to this
    // frame rather than the next one. The governor never switches object
    // detection off; what it sheds is the slow-changing stages, and when even
    // that is not enough it says the frame rate is insufficient rather than
    // letting the stack report a confident view of a road it is barely
    // looking at.
    final PerformancePlan plan = governor.update(
      system: systemSample,
      achievedFps: profiler.processingFps,
      speedMps: ego.speedMps,
      pipelineP95Ms: profiler.stage(PipelineStageNames.total).p95Ms,
      dtSeconds: dt,
      baseCadence: config.cadence,
      baseToggles: config.toggles,
    );
    final StageCadence cadence = plan.cadence;
    final PipelineStageToggles toggles = plan.toggles;
    // Only an insufficient *rate* counts as degraded perception. Running
    // depth every sixth frame instead of every third costs confidence, which
    // AutonomyConfidence already accounts for; it is not the stack saying it
    // cannot see. Conflating the two would push the decision engine to
    // UNCERTAIN the moment the phone warmed up, which would make the
    // governor worse than useless.
    if (plan.isFrameRateInsufficient) {
      degraded.add(
        'frame rate ${plan.achievedFps.toStringAsFixed(1)} FPS below the '
        '${plan.requiredFps.toStringAsFixed(1)} FPS this speed needs',
      );
    }

    // --- 1. preprocessing metadata ---------------------------------------
    final double ambientLuminance = profiler.measure(
      PipelineStageNames.preprocess,
      () => _ambientLuminance(frame),
    );

    // --- 2. object detection ---------------------------------------------
    DetectionResult detections = DetectionResult.noModel(
      frameId: frame.id,
      timestampMicros: frame.timestampMicros,
      reason: 'object detection disabled',
    );
    if (toggles.objectDetection) {
      detections = await profiler.measureAsync(
        PipelineStageNames.objectDetection,
        () => objectDetector.detect(frame),
      );
      if (detections.isDegraded) {
        degraded.add('detection: ${detections.degradedReason}');
      }
    } else {
      degraded.add('object detection disabled');
    }

    // --- 3. tracking -----------------------------------------------------
    List<ObjectTrack> tracks = const <ObjectTrack>[];
    if (toggles.tracking) {
      tracks = profiler.measure(
        PipelineStageNames.tracking,
        () => tracker.update(
          detections: detections,
          egoMotion: ego,
          timestampMicros: frame.timestampMicros,
        ),
      );
    }

    // --- 4. segmentation --------------------------------------------------
    RoadSegmentation? segmentation = _lastSegmentation;
    if (toggles.segmentation &&
        cadence.shouldRun(
          cadence.segmentationEveryNFrames,
          _frameIndex,
        )) {
      final RoadSegmentation fresh = await profiler.measureAsync(
        PipelineStageNames.segmentation,
        () => roadSegmenter.segment(frame),
      );
      segmentation = fresh;
      _lastSegmentation = fresh;
      if (fresh.isDegraded) {
        degraded.add('segmentation: ${fresh.degradedReason}');
      }
    }

    // --- 5. lanes ---------------------------------------------------------
    LaneDetectionResult lanes = _lastLanes ??
        LaneDetectionResult.empty(
          frameId: frame.id,
          timestampMicros: frame.timestampMicros,
        );
    if (toggles.laneDetection &&
        cadence.shouldRun(
          cadence.laneEveryNFrames,
          _frameIndex,
        )) {
      lanes = await profiler.measureAsync(
        PipelineStageNames.laneDetection,
        () => laneDetector.detectLanes(
          frame,
          segmentation: segmentation,
          previous: _lastLanes,
        ),
      );
      _lastLanes = lanes;
      if (lanes.overallConfidence > 0.5) {
        _lastGoodLanes = lanes;
        _lastGoodLanesMicros = frame.timestampMicros;
      }
    }

    // --- 6. drivable area and road edges ---------------------------------
    DrivableArea drivableArea = DrivableArea.empty(
      frameId: frame.id,
      timestampMicros: frame.timestampMicros,
    );
    final RoadSegmentation? seg = segmentation;
    if (seg != null && seg.isUsable) {
      drivableArea = profiler.measure(
        PipelineStageNames.segmentation,
        () => drivableAreaBuilder.build(
          segmentation: seg,
          calibration: frame.calibration,
          obstacles: tracks,
          egoLateralOffset: -lanes.egoLateralOffsetMeters,
        ),
      );
    }

    List<RoadEdge> roadEdges = const <RoadEdge>[];
    if (toggles.roadEdges) {
      roadEdges = profiler.measure(
        PipelineStageNames.roadEdge,
        () => roadEdgeDetector.detect(
          frame: frame,
          bev: _ensureBev(frame),
          segmentation: segmentation,
          drivableArea: drivableArea,
        ),
      );
    }

    // --- 7. NO_LANE_MODE corridor ----------------------------------------
    CorridorEstimate? corridor;
    final bool lanesUsable = lanes.overallConfidence >= 0.45 &&
        lanes.mode != LaneMode.noLane &&
        lanes.mode != LaneMode.none;
    if (!lanesUsable) {
      corridor = corridorEstimator.estimate(
        drivableArea: drivableArea,
        roadEdges: roadEdges,
        tracks: tracks,
        egoSpeedMps: ego.speedMps,
        lastGoodLanes: _lastGoodLanes,
        lastGoodLanesAgeSeconds: _lastGoodLanesMicros == null
            ? null
            : (frame.timestampMicros - _lastGoodLanesMicros!) / 1e6,
        intent: routeProgress?.intent ?? ManeuverIntent.unknown,
      );
      if (corridor == null) {
        degraded.add('no lane model and no corridor evidence');
      }
    }

    // --- 8. depth ---------------------------------------------------------
    DepthMap? depth = _lastDepth;
    if (toggles.depth &&
        cadence.shouldRun(
          cadence.depthEveryNFrames,
          _frameIndex,
        )) {
      DepthMap fresh = await profiler.measureAsync(
        PipelineStageNames.depth,
        () => depthEstimator.estimate(frame),
      );
      if (fresh.hasData) {
        fresh = profiler.measure(
          PipelineStageNames.depthFusion,
          () => depthFusion.fitToGroundPlane(
            depth: fresh,
            calibration: frame.calibration,
          ),
        );
      }
      depth = fresh;
      _lastDepth = fresh;
    }

    // --- 9. depth fusion onto tracks -------------------------------------
    if (tracks.isNotEmpty) {
      profiler.measure(PipelineStageNames.depthFusion, () {
        _fuseDistances(tracks, frame, depth, ego);
      });
      // Re-read the tracks so the refined ranges are visible downstream.
      tracks = tracker.activeTracks;
    }

    // --- 9b. road markings ------------------------------------------------
    //
    // Stop lines, crossings and speed bumps. Run after the lanes because the
    // marking tracker wants the same ego motion the rest of the frame used,
    // and before the signs so the junction inference can weigh both.
    RoadMarkingResult markingResult = RoadMarkingResult.none(
      frameId: frame.id,
      timestampMicros: frame.timestampMicros,
    );
    if (toggles.roadMarkings) {
      if (cadence.shouldRun(
        cadence.markingsEveryNFrames,
        _frameIndex,
      )) {
        markingResult = profiler.measure(
          PipelineStageNames.roadMarkings,
          () => roadMarkingDetector.detect(frame, lanes: lanes),
        );
        if (markingResult.isDegraded) {
          degraded.add('road markings: ${markingResult.degradedReason}');
        }
      }
      // The tracker runs every frame regardless: it is what carries a
      // marking towards us between detections, and skipping it would make
      // the distances stale exactly when they matter most.
      final double travelled =
          regulatoryTracker.odometerMeters - _lastMarkingUpdateMeters;
      _lastMarkingUpdateMeters = regulatoryTracker.odometerMeters;
      roadMarkingTracker.update(
        result: markingResult,
        travelledMeters: travelled,
        verticalAccelMps2: ego.verticalAccelMps2,
      );
    }
    final List<RoadMarking> markings = roadMarkingTracker.upcoming;

    // --- 10. signs and lights --------------------------------------------
    List<TrafficSign> signs = _lastSigns;
    if (toggles.trafficSigns &&
        cadence.shouldRun(
          cadence.signsEveryNFrames,
          _frameIndex,
        )) {
      signs = await profiler.measureAsync(
        PipelineStageNames.trafficSign,
        () => trafficSignDetector.detectSigns(
          frame,
          candidates: detections.detections,
        ),
      );
      _lastSigns = signs;
    }

    List<TrafficLight> lights = _lastLights;
    if (toggles.trafficLights) {
      lights = await profiler.measureAsync(
        PipelineStageNames.trafficLight,
        () => trafficLightDetector.detectLights(
          frame,
          candidates: detections.detections,
        ),
      );
      _lastLights = lights;
    }

    regulatoryTracker.observeSigns(signs);
    regulatoryTracker.observeLights(lights);

    // --- 10b. junction inference -------------------------------------------
    final IntersectionEstimate? intersection = toggles.roadMarkings
        ? intersectionDetector.detect(
            lanes: lanes,
            drivableArea: drivableArea,
            markings: markings,
            lights: lights,
            regulatory: regulatoryTracker.context,
            tracks: tracks,
            egoSpeedMps: ego.speedMps,
          )
        : null;

    // --- 11. world model (first pass) ------------------------------------
    WorldState world = profiler.measure(
      PipelineStageNames.worldModel,
      () => worldModelBuilder.build(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        ego: ego,
        calibration: frame.calibration,
        lanes: lanes,
        drivableArea: drivableArea,
        roadEdges: roadEdges,
        tracks: tracks,
        signs: signs,
        lights: lights,
        regulatory: regulatoryTracker.context,
        roadMarkings: markings,
        intersection: intersection,
        corridor: corridor,
        routeProgress: routeProgress,
        depth: depth,
        segmentation: segmentation,
        detections: detections,
        ambientLuminance: ambientLuminance,
        planningConfidence: _lastPath?.confidence ?? 0,
        degradedSubsystems: degraded,
      ),
    );

    // Give the sign and light recognisers the path geometry they need to
    // judge relevance on the *next* frame.
    _shareEgoPath(world);

    // --- 12. planning -----------------------------------------------------
    PlannedPath path = PlannedPath.none(
      frameId: frame.id,
      timestampMicros: frame.timestampMicros,
      reason: 'planning disabled',
    );
    if (toggles.planning) {
      path = profiler.measure(
        PipelineStageNames.planning,
        () => planner.plan(world, previous: _lastPath),
      );
      _lastPath = path;
    }

    // --- 13. collision prediction ----------------------------------------
    List<CollisionAssessment> collisions = const <CollisionAssessment>[];
    if (tracks.isNotEmpty) {
      collisions = profiler.measure(
        PipelineStageNames.collision,
        () => collisionPredictor.assess(world, path),
      );
      _applyAssessments(collisions);
      tracks = tracker.activeTracks;

      // Rebuild the world with risk-annotated tracks and the real planning
      // confidence. The second pass is cheap (no inference) and is what makes
      // the published world model internally consistent.
      world = profiler.measure(
        PipelineStageNames.worldModel,
        () => worldModelBuilder.build(
          frameId: frame.id,
          timestampMicros: frame.timestampMicros,
          ego: ego,
          calibration: frame.calibration,
          lanes: lanes,
          drivableArea: drivableArea,
          roadEdges: roadEdges,
          tracks: tracks,
          signs: signs,
          lights: lights,
          regulatory: regulatoryTracker.context,
          roadMarkings: markings,
          intersection: intersection,
          corridor: corridor,
          routeProgress: routeProgress,
          depth: depth,
          segmentation: segmentation,
          detections: detections,
          ambientLuminance: ambientLuminance,
          planningConfidence: path.confidence,
          degradedSubsystems: degraded,
        ),
      );
    }

    // --- 14. decision -----------------------------------------------------
    DrivingDecision decision = DrivingDecision(
      state: DrivingState.uncertain,
      reason: 'decision engine disabled',
      confidence: 0,
      timestampMicros: frame.timestampMicros,
      frameId: frame.id,
    );
    if (toggles.decision) {
      decision = profiler.measure(
        PipelineStageNames.decision,
        () => decisionEngine.decide(
          world: world,
          path: path,
          collisions: collisions,
        ),
      );
    }

    // --- 15. simulated control and vehicle dynamics -----------------------
    SimulatedControlCommand command = SimulatedControlCommand.neutral(
      timestampMicros: frame.timestampMicros,
      frameId: frame.id,
    );
    if (toggles.vehicleSimulation) {
      profiler.measure(PipelineStageNames.vehicleSim, () {
        command = controller.compute(
          world: world,
          path: path,
          decision: decision,
          simulated: _vehicleState,
        );
        // Indicating is decided from the manoeuvre, not from the steering
        // angle: the lamp has to come on before the wheel moves.
        command = command.copyWith(
          turnSignal: turnSignalPlanner.update(
            world: world,
            decision: decision,
            path: path,
            dtSeconds: dt,
          ),
        );
        _vehicleState = controller.advanceSimulation(
          model: vehicleModel,
          state: _vehicleState,
          command: command,
          world: world,
          dtSeconds: dt,
        );
      });
    }

    final int latency =
        DateTime.now().microsecondsSinceEpoch - cycleStart;
    profiler.onFrameProcessed(latency);

    for (final StageProfile s in profiler.stages) {
      timings[s.name] = s.lastMs;
    }

    return PipelineResult(
      world: world,
      path: path,
      decision: decision,
      command: command,
      vehicle: _vehicleState,
      collisions: collisions,
      totalLatencyMicros: latency,
      stageTimings: timings,
      performance: plan,
    );
  }

  // --- helpers ------------------------------------------------------------

  void _syncCalibration(CameraFrame frame) {
    if (frame.calibration.imageWidth == calibration.imageWidth &&
        frame.calibration.imageHeight == calibration.imageHeight) {
      return;
    }
    calibration = frame.calibration;
    if (tracker is MultiObjectTracker) {
      (tracker as MultiObjectTracker).calibration = frame.calibration;
    }
    _bev = null;
    roadMarkingDetector.invalidate();
  }

  BirdsEyeView _ensureBev(CameraFrame frame) {
    final BirdsEyeView? existing = _bev;
    if (existing != null && existing.matches(frame.calibration)) {
      return existing;
    }
    final BirdsEyeView built =
        BirdsEyeView.build(calibration: frame.calibration);
    _bev = built;
    Log.debug(_tag, 'rebuilt IPM table for ${frame.width}x${frame.height}');
    return built;
  }

  double _ambientLuminance(CameraFrame frame) {
    if (frame.format == PixelFormat.gray8) {
      return ImagePreprocessing.meanLuminance(
        frame.bytes,
        frame.width,
        frame.height,
        stride: 8,
      );
    }
    // Sample the RGB frame sparsely; exact luminance is not needed, only
    // enough to tell day from night from a blown-out tunnel exit.
    int sum = 0;
    int count = 0;
    for (int i = 0; i < frame.bytes.length - 2; i += 3 * 64) {
      sum += (frame.bytes[i] * 77 +
              frame.bytes[i + 1] * 150 +
              frame.bytes[i + 2] * 29) >>
          8;
      count++;
    }
    return count == 0 ? 128 : sum / count;
  }

  void _fuseDistances(
    List<ObjectTrack> tracks,
    CameraFrame frame,
    DepthMap? depth,
    EgoMotionState ego,
  ) {
    final DepthMap usableDepth = depth ??
        DepthMap.unavailable(
          frameId: frame.id,
          timestampMicros: frame.timestampMicros,
        );
    final double now = frame.timestampMicros / 1e6;

    for (final ObjectTrack track in tracks) {
      final _TrackDistanceMemory? memory = _distanceMemory[track.id];

      final FusedDistance fused = depthFusion.fuse(
        track: track,
        calibration: frame.calibration,
        depth: usableDepth,
        egoMotion: ego,
        previousDistanceMeters: memory?.distanceMeters,
        previousTimestampSeconds: memory?.timestampSeconds,
        currentTimestampSeconds: now,
      );

      if (fused.confidence.value > 0.05 && tracker is MultiObjectTracker) {
        (tracker as MultiObjectTracker).refineDistance(
          track.id,
          fused.distanceMeters,
          fused.confidence.value,
        );
      }

      _distanceMemory[track.id] = _TrackDistanceMemory(
        distanceMeters: fused.distanceMeters,
        timestampSeconds: now,
      );
    }

    final Set<int> live = tracks.map((ObjectTrack t) => t.id).toSet();
    _distanceMemory.removeWhere((int id, _) => !live.contains(id));
  }

  void _applyAssessments(List<CollisionAssessment> collisions) {
    if (tracker is! MultiObjectTracker) return;
    final MultiObjectTracker t = tracker as MultiObjectTracker;
    for (final CollisionAssessment c in collisions) {
      t.applyRiskAssessment(
        c.trackId,
        laneRelation: c.laneRelation,
        laneRelationConfidence: c.laneRelationConfidence,
        risk: c.risk,
        timeToCollisionSeconds: c.timeToCollisionSeconds,
        predictedPath: c.predictedPath,
        direction: c.willEnterPath ? MotionDirection.enteringPath : null,
      );
    }
  }

  void _shareEgoPath(WorldState world) {
    final path = _lastPath;
    final centerline = path != null && path.isUsable
        ? path.curve
        : world.referenceCenterline;

    if (trafficLightDetector is TrafficLightRecognizer) {
      final TrafficLightRecognizer r =
          trafficLightDetector as TrafficLightRecognizer;
      r.egoPathCenterline = centerline;
      r.egoLaneHalfWidth = world.lanes.laneWidthMeters / 2;
    }
    if (trafficSignDetector is TrafficSignRecognizer) {
      (trafficSignDetector as TrafficSignRecognizer).egoPathCenterline =
          centerline;
    }
  }

  /// Reset all cross-frame state. Used when starting a new drive or when
  /// switching between live and replay.
  void reset() {
    tracker.reset();
    planner.reset();
    decisionEngine.reset();
    controller.reset();
    regulatoryTracker.reset();
    governor.reset();
    roadMarkingTracker.reset();
    roadMarkingDetector.invalidate();
    turnSignalPlanner.reset();
    profiler.reset();
    _bev = null;
    _frameIndex = 0;
    _lastTimestampMicros = -1;
    _lastSegmentation = null;
    _lastDepth = null;
    _lastLanes = null;
    _lastGoodLanes = null;
    _lastGoodLanesMicros = null;
    _lastSigns = const <TrafficSign>[];
    _lastLights = const <TrafficLight>[];
    _lastMarkingUpdateMeters = 0;
    _lastPath = null;
    _vehicleState = VehicleState.stationary();
    _distanceMemory.clear();
    Log.info(_tag, 'pipeline reset');
  }

  Future<void> dispose() async {
    await objectDetector.close();
    await laneDetector.close();
    await roadSegmenter.close();
    await depthEstimator.close();
    await trafficSignDetector.close();
    await trafficLightDetector.close();
  }
}

class _TrackDistanceMemory {
  const _TrackDistanceMemory({
    required this.distanceMeters,
    required this.timestampSeconds,
  });

  final double distanceMeters;
  final double timestampSeconds;
}
