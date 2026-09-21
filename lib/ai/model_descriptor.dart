import 'dart:convert';

/// Which part of the pipeline a model plugs into.
enum ModelRole {
  objectDetection('Object Detection'),
  depthEstimation('Depth Estimation'),
  laneDetection('Lane Detection'),
  roadSegmentation('Road Segmentation'),
  trafficSignDetection('Traffic Sign Detection'),
  trafficSignClassification('Traffic Sign Classification'),
  trafficLightClassification('Traffic Light Classification');

  const ModelRole(this.label);
  final String label;
}

/// Output decoding convention. The decoder is chosen from this rather than
/// guessed from tensor shapes, because several architectures share shapes and
/// differ only in layout.
enum ModelOutputFormat {
  /// `[1, 84, N]` or `[1, N, 84]`: cx, cy, w, h then per-class scores.
  yoloV8,

  /// `[1, N, 85]`: cx, cy, w, h, objectness then per-class scores.
  yoloV5,

  /// TFLite Object Detection API: 4 tensors (boxes, classes, scores, count).
  ssdMobileNet,

  /// Single-channel inverse-depth map, e.g. MiDaS / DepthAnything.
  relativeDepthMap,

  /// Single-channel metric depth in metres.
  metricDepthMap,

  /// `[1, H, W, C]` per-pixel class logits.
  semanticSegmentation,

  /// UFLD-style row anchors: `[1, griddingNum+1, rowAnchors, lanes]`.
  laneRowAnchor,

  /// `[1, C]` class logits.
  classification,
}

/// How a frame is fitted to the model's input tensor.
///
/// This is not cosmetic. A detector's boxes come back normalised to the
/// *input tensor*, so the fit has to be undone in exactly the way it was
/// applied. Getting it wrong does not throw — it silently shifts and squashes
/// every box, which looks like a badly-trained model rather than a bug.
enum InputFit {
  /// Preserve aspect ratio, pad the remainder. What YOLO expects, because
  /// that is how it was trained.
  letterbox,

  /// Resize straight to the input dimensions, distorting aspect ratio. What
  /// the TensorFlow Object Detection API's `fixed_shape_resizer` does, so it
  /// is what SSD and EfficientDet heads expect.
  stretch,
}

/// Which compute unit the native runtime should try to use.
enum InferenceDelegate {
  /// Multi-threaded XNNPACK on the CPU. Always available; the fallback.
  cpu,

  /// Adreno GPU via the TFLite GPU delegate. Usually the best choice on a
  /// Galaxy S23 for float models.
  gpu,

  /// NNAPI, which on Snapdragon routes to the Hexagon DSP for quantised
  /// models. Fastest and coolest when the model is fully int8-quantised.
  nnapi,
}

/// Everything needed to load and run one model, independent of the model file
/// itself. Descriptors are plain data so they can live in a JSON sidecar next
/// to the weights and be edited without rebuilding the app.
class ModelDescriptor {
  const ModelDescriptor({
    required this.id,
    required this.name,
    required this.role,
    required this.assetOrFilePath,
    required this.inputWidth,
    required this.inputHeight,
    required this.outputFormat,
    this.inputChannels = 3,
    this.labelsPath,
    this.labels = const <String>[],
    this.inputMean = const <double>[0, 0, 0],
    this.inputStd = const <double>[255, 255, 255],
    this.channelsFirst = false,
    this.quantized = false,
    this.delegate = InferenceDelegate.gpu,
    this.numThreads = 4,
    this.scoreThreshold = 0.35,
    this.iouThreshold = 0.45,
    this.labelVocabulary = 'coco',
    InputFit? inputFit,
    this.isBundledAsset = false,
    this.sizeBytes,
    this.downloadUrl,
    this.downloadSha256,
    this.licence,
    this.notes,
    this.extra = const <String, dynamic>{},
  }) : _inputFit = inputFit;

  final String id;
  final String name;
  final ModelRole role;

  /// Either a Flutter asset key (when [isBundledAsset]) or an absolute path in
  /// the app's model directory.
  final String assetOrFilePath;

  final int inputWidth;
  final int inputHeight;
  final int inputChannels;
  final ModelOutputFormat outputFormat;

  final String? labelsPath;
  final List<String> labels;

  final List<double> inputMean;
  final List<double> inputStd;
  final bool channelsFirst;
  final bool quantized;

  final InferenceDelegate delegate;
  final int numThreads;

  final double scoreThreshold;
  final double iouThreshold;

  /// Selects the [ObjectClassMapping] used to translate [labels].
  final String labelVocabulary;

  final InputFit? _inputFit;

  /// How the frame is fitted to the input tensor, and therefore how detector
  /// boxes are mapped back onto it.
  ///
  /// Each head has exactly one correct answer, so it is derived from
  /// [outputFormat] by default rather than left as a knob a descriptor can
  /// set wrong. An unusual export can still override it.
  InputFit get inputFit =>
      _inputFit ??
      switch (outputFormat) {
        ModelOutputFormat.yoloV5 || ModelOutputFormat.yoloV8 =>
          InputFit.letterbox,
        _ => InputFit.stretch,
      };

  final bool isBundledAsset;
  final int? sizeBytes;

  /// Where the weights can be fetched from, for models too large to ship in
  /// the APK.
  ///
  /// A download is always the user's explicit choice: it costs their data,
  /// their storage and — once running — their battery and their thermal
  /// budget. Nothing here is fetched in the background.
  final String? downloadUrl;

  /// Expected SHA-256 of the downloaded file. A model that does not match is
  /// discarded rather than run: silently running the wrong weights produces
  /// plausible-looking output, which is the worst possible failure here.
  final String? downloadSha256;

  /// Licence of the weights, shown before any download starts.
  final String? licence;

  /// True when this model can be fetched but is not present.
  bool get isDownloadable => downloadUrl != null;

  final String? notes;

  /// Format-specific knobs (row anchors, gridding number, depth scale ...).
  final Map<String, dynamic> extra;

  double extraDouble(String key, [double fallback = 0]) =>
      (extra[key] as num?)?.toDouble() ?? fallback;

  int extraInt(String key, [int fallback = 0]) =>
      (extra[key] as num?)?.toInt() ?? fallback;

  ModelDescriptor copyWith({
    String? assetOrFilePath,
    List<String>? labels,
    InferenceDelegate? delegate,
    int? numThreads,
    double? scoreThreshold,
    double? iouThreshold,
    bool? isBundledAsset,
    int? sizeBytes,
  }) =>
      ModelDescriptor(
        id: id,
        name: name,
        role: role,
        assetOrFilePath: assetOrFilePath ?? this.assetOrFilePath,
        inputWidth: inputWidth,
        inputHeight: inputHeight,
        inputChannels: inputChannels,
        outputFormat: outputFormat,
        labelsPath: labelsPath,
        labels: labels ?? this.labels,
        inputMean: inputMean,
        inputStd: inputStd,
        channelsFirst: channelsFirst,
        quantized: quantized,
        delegate: delegate ?? this.delegate,
        numThreads: numThreads ?? this.numThreads,
        scoreThreshold: scoreThreshold ?? this.scoreThreshold,
        iouThreshold: iouThreshold ?? this.iouThreshold,
        labelVocabulary: labelVocabulary,
        inputFit: _inputFit,
        isBundledAsset: isBundledAsset ?? this.isBundledAsset,
        sizeBytes: sizeBytes ?? this.sizeBytes,
        downloadUrl: downloadUrl,
        downloadSha256: downloadSha256,
        licence: licence,
        notes: notes,
        extra: extra,
      );

  Map<String, dynamic> toJson() => <String, dynamic>{
        'id': id,
        'name': name,
        'role': role.name,
        'path': assetOrFilePath,
        'inputWidth': inputWidth,
        'inputHeight': inputHeight,
        'inputChannels': inputChannels,
        'outputFormat': outputFormat.name,
        if (labelsPath != null) 'labelsPath': labelsPath,
        if (labels.isNotEmpty) 'labels': labels,
        'inputMean': inputMean,
        'inputStd': inputStd,
        'channelsFirst': channelsFirst,
        'quantized': quantized,
        'delegate': delegate.name,
        'numThreads': numThreads,
        'scoreThreshold': scoreThreshold,
        'iouThreshold': iouThreshold,
        'labelVocabulary': labelVocabulary,
        'inputFit': inputFit.name,
        'isBundledAsset': isBundledAsset,
        if (sizeBytes != null) 'sizeBytes': sizeBytes,
        if (downloadUrl != null) 'downloadUrl': downloadUrl,
        if (downloadSha256 != null) 'downloadSha256': downloadSha256,
        if (licence != null) 'licence': licence,
        if (notes != null) 'notes': notes,
        if (extra.isNotEmpty) 'extra': extra,
      };

  static ModelDescriptor fromJson(Map<String, dynamic> j) => ModelDescriptor(
        id: j['id'] as String,
        name: j['name'] as String,
        role: ModelRole.values.firstWhere(
          (ModelRole r) => r.name == j['role'],
          orElse: () => ModelRole.objectDetection,
        ),
        assetOrFilePath: j['path'] as String,
        inputWidth: (j['inputWidth'] as num).toInt(),
        inputHeight: (j['inputHeight'] as num).toInt(),
        inputChannels: (j['inputChannels'] as num?)?.toInt() ?? 3,
        outputFormat: ModelOutputFormat.values.firstWhere(
          (ModelOutputFormat f) => f.name == j['outputFormat'],
          orElse: () => ModelOutputFormat.yoloV8,
        ),
        labelsPath: j['labelsPath'] as String?,
        labels: (j['labels'] as List<dynamic>?)
                ?.map((dynamic e) => e as String)
                .toList() ??
            const <String>[],
        inputMean: _doubles(j['inputMean']) ?? const <double>[0, 0, 0],
        inputStd: _doubles(j['inputStd']) ?? const <double>[255, 255, 255],
        channelsFirst: j['channelsFirst'] as bool? ?? false,
        quantized: j['quantized'] as bool? ?? false,
        delegate: InferenceDelegate.values.firstWhere(
          (InferenceDelegate d) => d.name == j['delegate'],
          orElse: () => InferenceDelegate.gpu,
        ),
        numThreads: (j['numThreads'] as num?)?.toInt() ?? 4,
        scoreThreshold: (j['scoreThreshold'] as num?)?.toDouble() ?? 0.35,
        iouThreshold: (j['iouThreshold'] as num?)?.toDouble() ?? 0.45,
        labelVocabulary: j['labelVocabulary'] as String? ?? 'coco',
        inputFit: j['inputFit'] == null
            ? null
            : InputFit.values.firstWhere(
                (InputFit f) => f.name == j['inputFit'],
                orElse: () => InputFit.letterbox,
              ),
        isBundledAsset: j['isBundledAsset'] as bool? ?? false,
        sizeBytes: (j['sizeBytes'] as num?)?.toInt(),
        downloadUrl: j['downloadUrl'] as String?,
        downloadSha256: j['downloadSha256'] as String?,
        licence: j['licence'] as String?,
        notes: j['notes'] as String?,
        extra: (j['extra'] as Map<String, dynamic>?) ?? const <String, dynamic>{},
      );

  static List<double>? _doubles(dynamic v) => v == null
      ? null
      : (v as List<dynamic>).map((dynamic e) => (e as num).toDouble()).toList();

  static ModelDescriptor fromJsonString(String s) =>
      fromJson(jsonDecode(s) as Map<String, dynamic>);

  @override
  String toString() =>
      'ModelDescriptor($id, ${role.label}, ${inputWidth}x$inputHeight, '
      '${delegate.name})';
}
