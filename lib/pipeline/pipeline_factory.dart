import '../ai/inference_backend.dart';
import '../ai/interfaces/depth_estimator.dart';
import '../ai/interfaces/lane_detector.dart';
import '../ai/interfaces/object_detector.dart';
import '../ai/interfaces/road_segmenter.dart';
import '../ai/interfaces/traffic_light_detector.dart';
import '../ai/interfaces/traffic_sign_detector.dart';
import '../ai/model_descriptor.dart';
import '../ai/model_registry.dart';
import '../camera/camera_calibration.dart';
import '../core/logging.dart';
import '../decision/decision_engine_impl.dart';
import '../depth/neural_depth_estimator.dart';
import '../perception/neural_object_detector.dart';
import '../perception/traffic_light_recognizer.dart';
import '../perception/traffic_sign_recognizer.dart';
import '../planning/local_path_planner.dart';
import '../road/classical_lane_detector.dart';
import '../road/heuristic_road_segmenter.dart';
import '../road/neural_lane_detector.dart';
import '../road/neural_road_segmenter.dart';
import '../tracking/multi_object_tracker.dart';
import 'perception_pipeline.dart';
import 'pipeline_config.dart';

/// Which implementation ended up filling each role, and why.
///
/// Surfaced on the AI Models screen so the user can see at a glance that, for
/// example, lanes are being found by the classical detector because no lane
/// model is installed — rather than wondering why detection quality differs
/// from what they expected.
class PipelineComposition {
  const PipelineComposition(this.roles);

  final Map<String, RoleBinding> roles;

  List<RoleBinding> get degraded =>
      roles.values.where((RoleBinding b) => b.isFallback).toList();

  bool get hasNeuralDetector =>
      roles['objectDetection']?.isFallback == false;
}

class RoleBinding {
  const RoleBinding({
    required this.role,
    required this.implementation,
    required this.isFallback,
    required this.explanation,
  });

  final String role;
  final String implementation;

  /// True when this is not a neural model — either a classical algorithm
  /// standing in, or nothing at all.
  final bool isFallback;

  final String explanation;
}

/// Builds a [PerceptionPipeline] from whatever models are installed.
///
/// The whole point of the model abstraction is realised here: each role is
/// filled by a neural model when one is available, by a classical algorithm
/// when one exists and is genuinely useful, and by an explicitly-degraded
/// stub when neither is possible. Nothing silently pretends to work.
class PipelineFactory {
  const PipelineFactory({
    required this.registry,
    required this.backend,
  });

  static const String _tag = 'PipelineFactory';

  final ModelRegistry registry;
  final InferenceBackend backend;

  Future<(PerceptionPipeline, PipelineComposition)> build({
    required CameraCalibration calibration,
    PipelineConfig config = const PipelineConfig(),
  }) async {
    final Map<String, RoleBinding> roles = <String, RoleBinding>{};

    // --- object detection -------------------------------------------------
    final ModelDescriptor? detectorModel =
        registry.selectedFor(ModelRole.objectDetection);
    ObjectDetector detector;
    if (detectorModel != null) {
      final NeuralObjectDetector neural = NeuralObjectDetector(
        descriptor: detectorModel,
        backend: backend,
      );
      await neural.load();
      if (neural.isReady) {
        detector = neural;
        roles['objectDetection'] = RoleBinding(
          role: 'Object detection',
          implementation: detectorModel.name,
          isFallback: false,
          explanation: 'Running on '
              '${detectorModel.delegate.name.toUpperCase()}',
        );
      } else {
        detector = UnavailableObjectDetector(
          neural.unavailableReason ?? 'model failed to load',
        );
        roles['objectDetection'] = RoleBinding(
          role: 'Object detection',
          implementation: 'none',
          isFallback: true,
          explanation: neural.unavailableReason ?? 'model failed to load',
        );
      }
    } else {
      detector = UnavailableObjectDetector();
      roles['objectDetection'] = const RoleBinding(
        role: 'Object detection',
        implementation: 'none',
        isFallback: true,
        // There is no classical substitute for object detection, and
        // pretending the road is empty would be the most dangerous possible
        // failure mode. The stack reports this as degraded instead.
        explanation: 'No model installed — vehicles, pedestrians and '
            'obstacles will NOT be detected',
      );
    }

    // --- lanes ------------------------------------------------------------
    final ModelDescriptor? laneModel =
        registry.selectedFor(ModelRole.laneDetection);
    LaneDetector laneDetector;
    if (laneModel != null) {
      final NeuralLaneDetector neural =
          NeuralLaneDetector(descriptor: laneModel, backend: backend);
      await neural.load();
      if (neural.isReady) {
        laneDetector = neural;
        roles['laneDetection'] = RoleBinding(
          role: 'Lane detection',
          implementation: laneModel.name,
          isFallback: false,
          explanation: 'Row-anchor model',
        );
      } else {
        laneDetector = ClassicalLaneDetector(calibration: calibration);
        roles['laneDetection'] = RoleBinding(
          role: 'Lane detection',
          implementation: 'Classical (IPM + matched filter)',
          isFallback: true,
          explanation: 'Model failed to load: '
              '${neural.unavailableReason}',
        );
      }
    } else {
      laneDetector = ClassicalLaneDetector(calibration: calibration);
      roles['laneDetection'] = const RoleBinding(
        role: 'Lane detection',
        implementation: 'Classical (IPM + matched filter)',
        isFallback: true,
        explanation: 'No model installed. Works well on clear markings; '
            'weaker at night and on worn paint',
      );
    }

    // --- segmentation -----------------------------------------------------
    final ModelDescriptor? segModel =
        registry.selectedFor(ModelRole.roadSegmentation);
    RoadSegmenter segmenter;
    if (segModel != null) {
      final NeuralRoadSegmenter neural =
          NeuralRoadSegmenter(descriptor: segModel, backend: backend);
      await neural.load();
      if (neural.isReady) {
        segmenter = neural;
        roles['roadSegmentation'] = RoleBinding(
          role: 'Road segmentation',
          implementation: segModel.name,
          isFallback: false,
          explanation: 'Semantic segmentation',
        );
      } else {
        segmenter = HeuristicRoadSegmenter();
        roles['roadSegmentation'] = RoleBinding(
          role: 'Road segmentation',
          implementation: 'Heuristic (seeded region growing)',
          isFallback: true,
          explanation: 'Model failed to load: '
              '${neural.unavailableReason}',
        );
      }
    } else {
      segmenter = HeuristicRoadSegmenter();
      roles['roadSegmentation'] = const RoleBinding(
        role: 'Road segmentation',
        implementation: 'Heuristic (seeded region growing)',
        isFallback: true,
        explanation: 'No model installed. Cannot distinguish asphalt from '
            'similarly-coloured pavement',
      );
    }

    // --- depth ------------------------------------------------------------
    final ModelDescriptor? depthModel =
        registry.selectedFor(ModelRole.depthEstimation);
    DepthEstimator depthEstimator;
    if (depthModel != null) {
      final NeuralDepthEstimator neural =
          NeuralDepthEstimator(descriptor: depthModel, backend: backend);
      await neural.load();
      if (neural.isReady) {
        depthEstimator = neural;
        roles['depthEstimation'] = RoleBinding(
          role: 'Depth estimation',
          implementation: depthModel.name,
          isFallback: false,
          explanation: depthModel.outputFormat ==
                  ModelOutputFormat.metricDepthMap
              ? 'Metric depth'
              : 'Relative depth, fitted to the ground plane each frame',
        );
      } else {
        depthEstimator = UnavailableDepthEstimator(
          neural.unavailableReason ?? 'model failed to load',
        );
        roles['depthEstimation'] = RoleBinding(
          role: 'Depth estimation',
          implementation: 'Geometric only',
          isFallback: true,
          explanation: 'Model failed to load: '
              '${neural.unavailableReason}',
        );
      }
    } else {
      depthEstimator = UnavailableDepthEstimator();
      roles['depthEstimation'] = const RoleBinding(
        role: 'Depth estimation',
        implementation: 'Geometric only',
        isFallback: true,
        // Distance still works without a depth model — ground-plane geometry,
        // size priors and motion parallax carry it — just with lower
        // confidence, which the fusion reports honestly.
        explanation: 'No model installed. Distances come from ground-plane '
            'geometry, size priors and motion parallax',
      );
    }

    // --- signs and lights -------------------------------------------------
    final ModelDescriptor? signModel =
        registry.selectedFor(ModelRole.trafficSignClassification);
    final TrafficSignDetector signDetector = TrafficSignRecognizer(
      classifierDescriptor: signModel,
      backend: signModel == null ? null : backend,
    );
    await signDetector.load();
    roles['trafficSigns'] = RoleBinding(
      role: 'Traffic signs',
      implementation: signModel?.name ?? 'Shape + colour + template digits',
      isFallback: signModel == null,
      explanation: signModel == null
          ? 'No classifier installed. Coarse categories only; speed-limit '
              'digits read by template matching'
          : 'Classifier running on detector crops',
    );

    final TrafficLightDetector lightDetector = TrafficLightRecognizer();
    roles['trafficLights'] = const RoleBinding(
      role: 'Traffic lights',
      implementation: 'Hue + aspect position',
      isFallback: false,
      explanation: 'Classical by design: a saturated point light against a '
          'dark housing is close to the ideal case for colour analysis',
    );

    final PerceptionPipeline pipeline = PerceptionPipeline(
      objectDetector: detector,
      tracker: MultiObjectTracker(calibration: calibration),
      laneDetector: laneDetector,
      roadSegmenter: segmenter,
      depthEstimator: depthEstimator,
      trafficSignDetector: signDetector,
      trafficLightDetector: lightDetector,
      planner: LocalPathPlanner(config: config.planner),
      decisionEngine:
          RuleBasedDecisionEngine(config: config.decisionConfig),
      calibration: calibration,
      config: config,
    );

    final PipelineComposition composition = PipelineComposition(roles);
    for (final RoleBinding b in composition.degraded) {
      Log.warn(_tag, '${b.role}: ${b.implementation} — ${b.explanation}');
    }

    return (pipeline, composition);
  }
}
