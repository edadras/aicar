import 'model_descriptor.dart';

/// Labels for the 80 COCO classes, in the order every standard export uses.
const List<String> cocoLabels = <String>[
  'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
  'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
  'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
  'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag',
  'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
  'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
  'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon',
  'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
  'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant',
  'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
  'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
  'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
  'hair drier', 'toothbrush',
];

/// The COCO **90-class** label map, as the TensorFlow Object Detection API
/// exports it.
///
/// Distinct from [cocoLabels] and not interchangeable with it: this map keeps
/// the gaps in the original COCO category ids as `???` placeholders, so a
/// class index from an SSD or EfficientDet head lands on the right name.
/// Indexing an 80-entry list with a 90-map index silently mislabels
/// everything after the first gap — a motorcycle becomes an airplane.
///
/// The `???` entries have no mapping in [ObjectClassMapping], so detections
/// on them are dropped rather than becoming `unknown` obstacles.
const List<String> coco90Labels = <String>[
  'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
  'truck', 'boat', 'traffic light', 'fire hydrant', '???', 'stop sign',
  'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
  'elephant', 'bear', 'zebra', 'giraffe', '???', 'backpack', 'umbrella',
  '???', '???', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis',
  'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
  'skateboard', 'surfboard', 'tennis racket', 'bottle', '???',
  'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
  'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut',
  'cake', 'chair', 'couch', 'potted plant', 'bed', '???', 'dining table',
  '???', '???', 'toilet', '???', 'tv', 'laptop', 'mouse', 'remote',
  'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
  'refrigerator', '???', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
  'hair drier', 'toothbrush',
];

/// Surface classes for a Cityscapes-style segmenter, reduced to the ten this
/// project cares about. Index order matches `docs/MODELS.md`.
const List<String> drivingSurfaceLabels = <String>[
  'road', 'drivable road', 'sidewalk', 'curb', 'grass', 'building',
  'vehicle', 'pedestrian', 'obstacle', 'unknown',
];

/// GTSRB-style traffic-sign classes used by the bundled sign classifier
/// descriptor.
const List<String> trafficSignLabels = <String>[
  'speed_limit_20', 'speed_limit_30', 'speed_limit_50', 'speed_limit_60',
  'speed_limit_70', 'speed_limit_80', 'end_speed_limit_80', 'speed_limit_100',
  'speed_limit_120', 'no_overtaking', 'no_overtaking_trucks',
  'priority_next_intersection', 'priority_road', 'give_way', 'stop',
  'no_vehicles', 'no_trucks', 'no_entry', 'general_warning', 'curve_left',
  'curve_right', 'double_curve', 'bumpy_road', 'slippery_road',
  'road_narrows', 'road_work', 'traffic_signals', 'pedestrian_crossing',
  'school_zone', 'bicycle_crossing', 'ice_snow', 'animal_crossing',
  'end_restrictions', 'mandatory_right', 'mandatory_left',
  'mandatory_straight', 'mandatory_straight_right', 'mandatory_straight_left',
  'keep_right', 'keep_left', 'roundabout', 'end_no_overtaking',
  'end_no_overtaking_trucks',
];

/// Models this build knows how to drive.
///
/// One detector ships inside the APK ([efficientDetLite0]) so perception works
/// on first launch. Everything else is installed at runtime into the app's
/// model directory (see `docs/MODELS.md`): model files are large,
/// licence-encumbered and upgradable independently of the app. The catalog is
/// what makes that possible without the user writing a descriptor by hand — it
/// records the input size, normalisation, output layout and label set each
/// known architecture needs.
class ModelCatalog {
  const ModelCatalog._();

  static const String yolov8nId = 'yolov8n-640-fp16';
  static const String yolov8sId = 'yolov8s-640-fp16';
  static const String yolo11nId = 'yolo11n-640-fp16';
  static const String ssdMobileNetId = 'ssd-mobilenet-v2-int8';
  static const String efficientDetLite0Id = 'efficientdet-lite0-320-int8';
  static const String midasSmallId = 'midas-v2-small-256';
  static const String depthAnythingId = 'depth-anything-v2-small-518';
  static const String ufldId = 'ufld-v2-culane-800';
  static const String segformerId = 'segformer-b0-driving-512';
  static const String signClassifierId = 'gtsrb-classifier-48';

  /// All descriptors the app recognises, keyed by id.
  static Map<String, ModelDescriptor> get all => <String, ModelDescriptor>{
        for (final ModelDescriptor d in <ModelDescriptor>[
          yolov8n,
          yolov8s,
          yolo11n,
          efficientDetLite0,
          ssdMobileNet,
          midasSmall,
          depthAnythingSmall,
          ultraFastLaneV2,
          segformerDriving,
          signClassifier,
        ])
          d.id: d,
      };

  static List<ModelDescriptor> forRole(ModelRole role) =>
      all.values.where((ModelDescriptor d) => d.role == role).toList();

  static ModelDescriptor? byId(String id) => all[id];

  // --- Object detection ---------------------------------------------------

  /// The recommended default on a Galaxy S23: ~10 ms on the Adreno GPU at
  /// 640x640 fp16, which leaves room for depth and segmentation in the same
  /// frame budget.
  static const ModelDescriptor yolov8n = ModelDescriptor(
    id: yolov8nId,
    name: 'YOLOv8n (COCO, 640, fp16)',
    role: ModelRole.objectDetection,
    assetOrFilePath: 'yolov8n_float16.tflite',
    inputWidth: 640,
    inputHeight: 640,
    outputFormat: ModelOutputFormat.yoloV8,
    labels: cocoLabels,
    scoreThreshold: 0.35,
    iouThreshold: 0.45,
    delegate: InferenceDelegate.gpu,
    labelVocabulary: 'coco',
    notes: 'Fast and accurate enough for vehicles and pedestrians. Does not '
        'know traffic cones, barriers or road debris — install a '
        'driving-specific model for those.',
  );

  /// Noticeably better on small, distant objects; roughly 2.5x the cost.
  static const ModelDescriptor yolov8s = ModelDescriptor(
    id: yolov8sId,
    name: 'YOLOv8s (COCO, 640, fp16)',
    role: ModelRole.objectDetection,
    assetOrFilePath: 'yolov8s_float16.tflite',
    inputWidth: 640,
    inputHeight: 640,
    outputFormat: ModelOutputFormat.yoloV8,
    labels: cocoLabels,
    scoreThreshold: 0.35,
    delegate: InferenceDelegate.gpu,
    labelVocabulary: 'coco',
    notes: 'Use when inference FPS is comfortably above target and distant '
        'motorcycles are being missed.',
  );

  static const ModelDescriptor yolo11n = ModelDescriptor(
    id: yolo11nId,
    name: 'YOLO11n (COCO, 640, fp16)',
    role: ModelRole.objectDetection,
    assetOrFilePath: 'yolo11n_float16.tflite',
    inputWidth: 640,
    inputHeight: 640,
    outputFormat: ModelOutputFormat.yoloV8,
    labels: cocoLabels,
    scoreThreshold: 0.35,
    delegate: InferenceDelegate.gpu,
    labelVocabulary: 'coco',
  );

  /// The detector **shipped inside the APK**, so the stack has real object
  /// detection on first launch with nothing to download.
  ///
  /// EfficientDet-Lite0 is Apache-2.0 (Google / TensorFlow Hub), which is why
  /// it is the one bundled: YOLOv8 and YOLO11 are excellent but AGPL-3.0, and
  /// redistributing their weights inside the APK would put the whole app
  /// under the AGPL. Those stay installable at runtime, as the user's choice
  /// about their own licensing.
  ///
  /// Its exported graph emits at most 25 boxes per frame. That is a property
  /// of the export, not a threshold that can be raised: a dense intersection
  /// can genuinely fill every slot, and when it does the detector marks the
  /// frame saturated rather than letting the stack believe the rest of the
  /// scene is empty.
  static const ModelDescriptor efficientDetLite0 = ModelDescriptor(
    id: efficientDetLite0Id,
    name: 'EfficientDet-Lite0 (COCO, 320, int8) — bundled',
    role: ModelRole.objectDetection,
    assetOrFilePath: bundledDetectorAsset,
    inputWidth: 320,
    inputHeight: 320,
    outputFormat: ModelOutputFormat.ssdMobileNet,
    labels: coco90Labels,
    quantized: true,
    scoreThreshold: 0.40,
    delegate: InferenceDelegate.nnapi,
    labelVocabulary: 'coco',
    isBundledAsset: true,
    sizeBytes: 4563519,
    notes: 'Ships with the app (Apache-2.0). ~20 ms per frame on a Galaxy '
        'S23 via NNAPI. Caps at 25 detections per frame and, at 320x320, '
        'loses small objects beyond roughly 60 m — install a larger detector '
        'for longer range.',
  );

  /// Flutter asset key of the bundled detector. Passed straight to the native
  /// runtime, which turns it into an APK lookup key.
  static const String bundledDetectorAsset =
      'assets/models/efficientdet_lite0.tflite';

  /// Int8 and NNAPI-friendly: the coolest-running option for long drives,
  /// at a real cost in small-object recall.
  static const ModelDescriptor ssdMobileNet = ModelDescriptor(
    id: ssdMobileNetId,
    name: 'SSD MobileNet V2 (COCO, 300, int8)',
    role: ModelRole.objectDetection,
    assetOrFilePath: 'ssd_mobilenet_v2_int8.tflite',
    inputWidth: 300,
    inputHeight: 300,
    outputFormat: ModelOutputFormat.ssdMobileNet,
    labels: coco90Labels,
    quantized: true,
    scoreThreshold: 0.45,
    delegate: InferenceDelegate.nnapi,
    labelVocabulary: 'coco',
    notes: 'Lowest power draw. Misses small/distant objects — expect shorter '
        'effective detection range.',
  );

  // --- Depth --------------------------------------------------------------

  static const ModelDescriptor midasSmall = ModelDescriptor(
    id: midasSmallId,
    name: 'MiDaS v2.1 small (256)',
    role: ModelRole.depthEstimation,
    assetOrFilePath: 'midas_v21_small_256.tflite',
    inputWidth: 256,
    inputHeight: 256,
    outputFormat: ModelOutputFormat.relativeDepthMap,
    inputMean: <double>[123.675, 116.28, 103.53],
    inputStd: <double>[58.395, 57.12, 57.375],
    delegate: InferenceDelegate.gpu,
    extra: <String, dynamic>{'outputWidth': 256, 'outputHeight': 256},
    notes: 'Relative inverse depth. Requires ground-plane anchors to recover '
        'metric scale — see DepthFusion.',
  );

  static const ModelDescriptor depthAnythingSmall = ModelDescriptor(
    id: depthAnythingId,
    name: 'Depth Anything V2 small (518)',
    role: ModelRole.depthEstimation,
    assetOrFilePath: 'depth_anything_v2_small.tflite',
    inputWidth: 518,
    inputHeight: 518,
    outputFormat: ModelOutputFormat.relativeDepthMap,
    inputMean: <double>[123.675, 116.28, 103.53],
    inputStd: <double>[58.395, 57.12, 57.375],
    delegate: InferenceDelegate.gpu,
    extra: <String, dynamic>{'outputWidth': 518, 'outputHeight': 518},
    notes: 'Much better geometry than MiDaS, noticeably slower. Budget ~35 ms '
        'on the S23 GPU.',
  );

  // --- Lanes and surfaces -------------------------------------------------

  static const ModelDescriptor ultraFastLaneV2 = ModelDescriptor(
    id: ufldId,
    name: 'Ultra-Fast-Lane-Detection v2 (CULane, 800x320)',
    role: ModelRole.laneDetection,
    assetOrFilePath: 'ufld_v2_culane.tflite',
    inputWidth: 800,
    inputHeight: 320,
    outputFormat: ModelOutputFormat.laneRowAnchor,
    inputMean: <double>[123.675, 116.28, 103.53],
    inputStd: <double>[58.395, 57.12, 57.375],
    delegate: InferenceDelegate.gpu,
    extra: <String, dynamic>{
      'griddingNum': 200,
      'rowAnchorCount': 72,
      'laneCount': 4,
      'rowAnchorStart': 0.42,
      'rowAnchorEnd': 1.0,
    },
    notes: 'Row-anchor lane model. Far more robust than the classical '
        'detector at night and on worn markings.',
  );

  static const ModelDescriptor segformerDriving = ModelDescriptor(
    id: segformerId,
    name: 'SegFormer-B0 driving surfaces (512x288)',
    role: ModelRole.roadSegmentation,
    assetOrFilePath: 'segformer_b0_driving.tflite',
    inputWidth: 512,
    inputHeight: 288,
    outputFormat: ModelOutputFormat.semanticSegmentation,
    labels: drivingSurfaceLabels,
    inputMean: <double>[123.675, 116.28, 103.53],
    inputStd: <double>[58.395, 57.12, 57.375],
    delegate: InferenceDelegate.gpu,
    extra: <String, dynamic>{'outputWidth': 128, 'outputHeight': 72},
    notes: 'Provides the drivable-area overlay and NO_LANE_MODE corridor.',
  );

  // --- Signs --------------------------------------------------------------

  static const ModelDescriptor signClassifier = ModelDescriptor(
    id: signClassifierId,
    name: 'GTSRB sign classifier (48x48)',
    role: ModelRole.trafficSignClassification,
    assetOrFilePath: 'gtsrb_classifier_48.tflite',
    inputWidth: 48,
    inputHeight: 48,
    outputFormat: ModelOutputFormat.classification,
    labels: trafficSignLabels,
    delegate: InferenceDelegate.cpu,
    numThreads: 2,
    scoreThreshold: 0.60,
    notes: 'Runs on crops from the object detector, so it costs ~1 ms per '
        'candidate rather than a full frame.',
  );
}
