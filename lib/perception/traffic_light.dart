import '../core/confidence.dart';
import '../core/geometry.dart';

enum TrafficLightColor {
  red('RED'),
  yellow('YELLOW'),
  green('GREEN'),
  redYellow('RED+YELLOW'),
  flashingYellow('FLASHING YELLOW'),
  off('OFF'),
  unknown('UNKNOWN');

  const TrafficLightColor(this.label);
  final String label;

  /// Whether the ego vehicle must stop for this indication.
  bool get requiresStop =>
      this == TrafficLightColor.red || this == TrafficLightColor.redYellow;

  /// Whether the indication permits proceeding.
  bool get permitsGo => this == TrafficLightColor.green;
}

/// Which movement the aspect governs.
enum TrafficLightArrow {
  none('CIRCULAR'),
  straight('STRAIGHT'),
  left('LEFT ARROW'),
  right('RIGHT ARROW'),
  leftStraight('LEFT + STRAIGHT'),
  rightStraight('RIGHT + STRAIGHT');

  const TrafficLightArrow(this.label);
  final String label;
}

/// Whether a detected light governs *our* movement.
///
/// This is the hard part of traffic-light perception and the part most likely
/// to be wrong, so it is an explicit, confidence-carrying field rather than an
/// assumption. A light for the cross street, or for the bus lane, must not
/// stop us.
enum TrafficLightRelevance {
  /// Governs the ego vehicle's movement.
  egoPath('EGO PATH'),

  /// Governs a different movement (cross traffic, another lane, a side road).
  otherPath('OTHER PATH'),

  /// Cannot be determined from the available evidence.
  unknown('RELEVANCE UNKNOWN');

  const TrafficLightRelevance(this.label);
  final String label;
}

class TrafficLight {
  const TrafficLight({
    required this.id,
    required this.color,
    required this.arrow,
    required this.box,
    required this.confidence,
    required this.colorConfidence,
    required this.relevance,
    required this.relevanceConfidence,
    required this.frameId,
    required this.timestampMicros,
    this.distanceMeters,
    this.lateralOffsetMeters,
    this.observationCount = 1,
    this.stableColorFrames = 1,
  });

  final int id;
  final TrafficLightColor color;
  final TrafficLightArrow arrow;
  final BoundingBox box;

  /// Confidence that there is a traffic light here at all.
  final Confidence confidence;

  /// Confidence in the *colour*, which is a separate question and degrades
  /// badly against a low sun or a bright sky.
  final Confidence colorConfidence;

  final TrafficLightRelevance relevance;
  final Confidence relevanceConfidence;

  final int frameId;
  final int timestampMicros;
  final double? distanceMeters;
  final double? lateralOffsetMeters;
  final int observationCount;

  /// Consecutive frames showing the same colour. A single red frame in a
  /// sequence of greens is a reflection or a motion-blur artefact, not a
  /// signal change, so acting on it would produce phantom braking.
  final int stableColorFrames;

  /// The light is only allowed to change the driving decision once it is
  /// stable, relevant and confidently coloured.
  bool get isActionable =>
      relevance == TrafficLightRelevance.egoPath &&
      relevanceConfidence.value >= 0.55 &&
      colorConfidence.value >= 0.60 &&
      stableColorFrames >= 3;

  String get displayText => arrow == TrafficLightArrow.none
      ? color.label
      : '${color.label}\n${arrow.label}';

  TrafficLight copyWith({
    TrafficLightColor? color,
    TrafficLightArrow? arrow,
    BoundingBox? box,
    Confidence? confidence,
    Confidence? colorConfidence,
    TrafficLightRelevance? relevance,
    Confidence? relevanceConfidence,
    double? distanceMeters,
    double? lateralOffsetMeters,
    int? observationCount,
    int? stableColorFrames,
    int? frameId,
    int? timestampMicros,
  }) =>
      TrafficLight(
        id: id,
        color: color ?? this.color,
        arrow: arrow ?? this.arrow,
        box: box ?? this.box,
        confidence: confidence ?? this.confidence,
        colorConfidence: colorConfidence ?? this.colorConfidence,
        relevance: relevance ?? this.relevance,
        relevanceConfidence: relevanceConfidence ?? this.relevanceConfidence,
        frameId: frameId ?? this.frameId,
        timestampMicros: timestampMicros ?? this.timestampMicros,
        distanceMeters: distanceMeters ?? this.distanceMeters,
        lateralOffsetMeters: lateralOffsetMeters ?? this.lateralOffsetMeters,
        observationCount: observationCount ?? this.observationCount,
        stableColorFrames: stableColorFrames ?? this.stableColorFrames,
      );

  Map<String, dynamic> toJson() => <String, dynamic>{
        'id': id,
        'color': color.name,
        'arrow': arrow.name,
        'box': box.toJson(),
        'conf': confidence.toJson(),
        'colorConf': colorConfidence.toJson(),
        'relevance': relevance.name,
        'relevanceConf': relevanceConfidence.toJson(),
        if (distanceMeters != null)
          'dist': double.parse(distanceMeters!.toStringAsFixed(2)),
        'count': observationCount,
        'stable': stableColorFrames,
      };

  static TrafficLight fromJson(
    Map<String, dynamic> j, {
    required int frameId,
    required int timestampMicros,
  }) =>
      TrafficLight(
        id: (j['id'] as num).toInt(),
        color: TrafficLightColor.values.firstWhere(
          (TrafficLightColor c) => c.name == j['color'],
          orElse: () => TrafficLightColor.unknown,
        ),
        arrow: TrafficLightArrow.values.firstWhere(
          (TrafficLightArrow a) => a.name == j['arrow'],
          orElse: () => TrafficLightArrow.none,
        ),
        box: BoundingBox.fromJson(j['box'] as Map<String, dynamic>),
        confidence: Confidence.fromJson(j['conf'] as Map<String, dynamic>),
        colorConfidence:
            Confidence.fromJson(j['colorConf'] as Map<String, dynamic>),
        relevance: TrafficLightRelevance.values.firstWhere(
          (TrafficLightRelevance r) => r.name == j['relevance'],
          orElse: () => TrafficLightRelevance.unknown,
        ),
        relevanceConfidence:
            Confidence.fromJson(j['relevanceConf'] as Map<String, dynamic>),
        frameId: frameId,
        timestampMicros: timestampMicros,
        distanceMeters: (j['dist'] as num?)?.toDouble(),
        observationCount: (j['count'] as num?)?.toInt() ?? 1,
        stableColorFrames: (j['stable'] as num?)?.toInt() ?? 1,
      );

  @override
  String toString() => 'Light#$id ${color.label}'
      '${arrow == TrafficLightArrow.none ? '' : ' ${arrow.label}'} '
      '${relevance.label} ${colorConfidence.percent}%';
}
