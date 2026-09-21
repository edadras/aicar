import '../camera/camera_service.dart';
import '../decision/decision_engine_impl.dart';
import '../planning/local_path_planner.dart';
import '../simulation/vehicle_state.dart';

/// Which optional stages run.
///
/// Every stage can be switched off independently, which is what makes the
/// performance screen useful: you can measure exactly what depth estimation
/// costs by turning it off and watching the frame rate.
class PipelineStageToggles {
  const PipelineStageToggles({
    this.objectDetection = true,
    this.tracking = true,
    this.segmentation = true,
    this.laneDetection = true,
    this.roadEdges = true,
    this.roadMarkings = true,
    this.depth = true,
    this.trafficSigns = true,
    this.trafficLights = true,
    this.planning = true,
    this.decision = true,
    this.vehicleSimulation = true,
  });

  final bool objectDetection;
  final bool tracking;
  final bool segmentation;
  final bool laneDetection;
  final bool roadEdges;

  /// Stop lines, crossings and speed bumps, plus the junction inference they
  /// feed. Cheap — it reuses the bird's-eye machinery the lane detector
  /// already needs — but switchable like every other stage.
  final bool roadMarkings;

  final bool depth;
  final bool trafficSigns;
  final bool trafficLights;
  final bool planning;
  final bool decision;
  final bool vehicleSimulation;

  PipelineStageToggles copyWith({
    bool? objectDetection,
    bool? tracking,
    bool? segmentation,
    bool? laneDetection,
    bool? roadEdges,
    bool? roadMarkings,
    bool? depth,
    bool? trafficSigns,
    bool? trafficLights,
    bool? planning,
    bool? decision,
    bool? vehicleSimulation,
  }) =>
      PipelineStageToggles(
        objectDetection: objectDetection ?? this.objectDetection,
        tracking: tracking ?? this.tracking,
        segmentation: segmentation ?? this.segmentation,
        laneDetection: laneDetection ?? this.laneDetection,
        roadEdges: roadEdges ?? this.roadEdges,
        roadMarkings: roadMarkings ?? this.roadMarkings,
        depth: depth ?? this.depth,
        trafficSigns: trafficSigns ?? this.trafficSigns,
        trafficLights: trafficLights ?? this.trafficLights,
        planning: planning ?? this.planning,
        decision: decision ?? this.decision,
        vehicleSimulation: vehicleSimulation ?? this.vehicleSimulation,
      );

  Map<String, bool> toJson() => <String, bool>{
        'objectDetection': objectDetection,
        'tracking': tracking,
        'segmentation': segmentation,
        'laneDetection': laneDetection,
        'roadEdges': roadEdges,
        'roadMarkings': roadMarkings,
        'depth': depth,
        'trafficSigns': trafficSigns,
        'trafficLights': trafficLights,
        'planning': planning,
        'decision': decision,
        'vehicleSimulation': vehicleSimulation,
      };
}

/// How often the slower stages run, relative to the detector.
///
/// Segmentation and depth are the two most expensive stages and the two that
/// change most slowly — the road does not move between frames. Running them
/// every second or third cycle roughly doubles the achievable detection rate
/// on a Galaxy S23 with no meaningful loss in the corridor or the distances.
class StageCadence {
  const StageCadence({
    this.segmentationEveryNFrames = 2,
    this.depthEveryNFrames = 3,
    this.laneEveryNFrames = 1,
    this.signsEveryNFrames = 2,
    this.markingsEveryNFrames = 2,
  });

  final int segmentationEveryNFrames;
  final int depthEveryNFrames;
  final int laneEveryNFrames;
  final int signsEveryNFrames;
  final int markingsEveryNFrames;

  bool shouldRun(int cadence, int frameIndex) =>
      cadence <= 1 || frameIndex % cadence == 0;
}

/// Full pipeline configuration.
class PipelineConfig {
  const PipelineConfig({
    this.camera = const CameraConfig(),
    this.toggles = const PipelineStageToggles(),
    this.cadence = const StageCadence(),
    this.planner = const PlannerConfig(),
    this.decisionConfig = const DecisionConfig(),
    this.vehicle = const VehicleParameters(),
    this.steeringLimitDegrees = 35.0,
    this.recordEveryNFrames = 1,
  });

  final CameraConfig camera;
  final PipelineStageToggles toggles;
  final StageCadence cadence;
  final PlannerConfig planner;
  final DecisionConfig decisionConfig;
  final VehicleParameters vehicle;
  final double steeringLimitDegrees;
  final int recordEveryNFrames;

  PipelineConfig copyWith({
    CameraConfig? camera,
    PipelineStageToggles? toggles,
    StageCadence? cadence,
    VehicleParameters? vehicle,
    double? steeringLimitDegrees,
    int? recordEveryNFrames,
  }) =>
      PipelineConfig(
        camera: camera ?? this.camera,
        toggles: toggles ?? this.toggles,
        cadence: cadence ?? this.cadence,
        planner: planner,
        decisionConfig: decisionConfig,
        vehicle: vehicle ?? this.vehicle,
        steeringLimitDegrees:
            steeringLimitDegrees ?? this.steeringLimitDegrees,
        recordEveryNFrames: recordEveryNFrames ?? this.recordEveryNFrames,
      );
}
