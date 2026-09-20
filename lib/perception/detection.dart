import '../core/confidence.dart';
import '../core/geometry.dart';
import 'object_class.dart';

/// A single-frame detection, before tracking assigns it a stable identity.
///
/// Detections are cheap value objects: the tracker consumes them and produces
/// [ObjectTrack]s, which are what the rest of the stack actually reasons
/// about.
class Detection {
  const Detection({
    required this.objectClass,
    required this.box,
    required this.score,
    required this.frameId,
    required this.timestampMicros,
    this.rawLabel,
    this.classScores,
  });

  final ObjectClass objectClass;

  /// Normalised image coordinates (0..1) so the box is resolution-independent.
  final BoundingBox box;

  final double score;
  final int frameId;
  final int timestampMicros;

  /// The detector's own label, kept for the debug overlay and for datasets.
  final String? rawLabel;

  /// Full class distribution when the backend provides one. The tracker uses
  /// it to keep a running class belief instead of flip-flopping between
  /// `car` and `van` frame to frame.
  final Map<ObjectClass, double>? classScores;

  Confidence get confidence =>
      Confidence(score, source: 'detector${rawLabel == null ? '' : ':$rawLabel'}');

  Detection copyWith({
    ObjectClass? objectClass,
    BoundingBox? box,
    double? score,
  }) =>
      Detection(
        objectClass: objectClass ?? this.objectClass,
        box: box ?? this.box,
        score: score ?? this.score,
        frameId: frameId,
        timestampMicros: timestampMicros,
        rawLabel: rawLabel,
        classScores: classScores,
      );

  Map<String, dynamic> toJson() => <String, dynamic>{
        'cls': objectClass.name,
        'box': box.toJson(),
        'score': double.parse(score.toStringAsFixed(4)),
        if (rawLabel != null) 'raw': rawLabel,
      };

  static Detection fromJson(
    Map<String, dynamic> j, {
    required int frameId,
    required int timestampMicros,
  }) =>
      Detection(
        objectClass: ObjectClass.fromName(j['cls'] as String),
        box: BoundingBox.fromJson(j['box'] as Map<String, dynamic>),
        score: (j['score'] as num).toDouble(),
        frameId: frameId,
        timestampMicros: timestampMicros,
        rawLabel: j['raw'] as String?,
      );

  @override
  String toString() =>
      '${objectClass.label} ${(score * 100).round()}% $box';
}

/// Everything a detector produced for one frame, plus the metadata needed to
/// decide how much of it to believe.
class DetectionResult {
  const DetectionResult({
    required this.detections,
    required this.frameId,
    required this.timestampMicros,
    required this.inferenceMicros,
    required this.modelName,
    this.isDegraded = false,
    this.degradedReason,
  });

  /// The honest empty result used when no detector model is installed. It is
  /// explicitly marked degraded so the autonomy confidence collapses rather
  /// than the stack silently believing the road is empty.
  factory DetectionResult.noModel({
    required int frameId,
    required int timestampMicros,
    String reason = 'no object detection model installed',
  }) =>
      DetectionResult(
        detections: const <Detection>[],
        frameId: frameId,
        timestampMicros: timestampMicros,
        inferenceMicros: 0,
        modelName: 'none',
        isDegraded: true,
        degradedReason: reason,
      );

  final List<Detection> detections;
  final int frameId;
  final int timestampMicros;
  final int inferenceMicros;
  final String modelName;

  /// True when the result cannot be trusted as a complete view of the scene.
  final bool isDegraded;
  final String? degradedReason;

  bool get isEmpty => detections.isEmpty;
  int get count => detections.length;

  /// Aggregate perception confidence for this frame.
  ///
  /// An empty *non-degraded* result is a legitimate, confident observation
  /// ("the road ahead is clear"), so it scores high. An empty *degraded*
  /// result means we simply cannot see, and scores zero.
  double get frameConfidence {
    if (isDegraded) return 0;
    if (detections.isEmpty) return 0.85;
    double sum = 0;
    for (final Detection d in detections) {
      sum += d.score;
    }
    return clampDouble(sum / detections.length, 0, 1);
  }

  List<Detection> ofClass(ObjectClass c) =>
      detections.where((Detection d) => d.objectClass == c).toList();

  @override
  String toString() => 'DetectionResult(${detections.length} dets, '
      '${(inferenceMicros / 1000).toStringAsFixed(1)}ms, $modelName'
      '${isDegraded ? ', DEGRADED: $degradedReason' : ''})';
}
