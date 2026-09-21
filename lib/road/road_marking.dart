import '../core/confidence.dart';

/// Markings painted on the road surface that change how we should drive.
///
/// Only markings with a behavioural consequence are modelled, for the same
/// reason [TrafficSignType] is selective: a class, an icon and a confidence
/// number are only worth carrying if some decision reads them.
enum RoadMarkingType {
  /// The transverse bar at a junction mouth. On its own it is **not** an
  /// obligation — a stop line at a green light means keep going. It is strong
  /// evidence that a junction is there, which is what the intersection
  /// detector uses it for.
  stopLine('STOP LINE', approachMeters: 45, advisorySpeedKph: 0),

  /// Zebra or ladder pedestrian crossing.
  crosswalk('CROSSWALK', approachMeters: 60, advisorySpeedKph: 25),

  /// A painted speed hump or cushion.
  speedBump('SPEED BUMP', approachMeters: 50, advisorySpeedKph: 20);

  const RoadMarkingType(
    this.label, {
    required this.approachMeters,
    required this.advisorySpeedKph,
  });

  final String label;

  /// How far ahead the marking starts influencing the decision.
  final double approachMeters;

  /// Speed to be travelling at when the marking is crossed, km/h.
  final double advisorySpeedKph;

  double get advisorySpeedMps => advisorySpeedKph / 3.6;

  /// Whether the marking itself compels a speed reduction.
  ///
  /// A speed bump does: it is a physical feature of the road and crossing it
  /// fast is uncomfortable at best. A crosswalk does: you must be able to
  /// stop for someone stepping onto it. A stop line does not, because what
  /// governs a junction is the sign or the signal, not the paint.
  bool get compelsSlowing => this != RoadMarkingType.stopLine;
}

/// One marking found on the road ahead, in the vehicle frame.
///
/// Distances come from the bird's-eye mapping, so they inherit the
/// calibration's accuracy. [confidence] already accounts for that: it falls
/// off with distance because the same grid row covers more ground the further
/// away it is.
class RoadMarking {
  const RoadMarking({
    required this.type,
    required this.distanceMeters,
    required this.depthMeters,
    required this.lateralCenterMeters,
    required this.widthMeters,
    required this.confidence,
    required this.frameId,
    required this.timestampMicros,
    this.observationCount = 1,
    this.firstSeenMicros,
    this.confirmedByMotion = false,
  });

  final RoadMarkingType type;

  /// Distance to the **near** edge of the marking, metres ahead.
  final double distanceMeters;

  /// Extent along the direction of travel.
  final double depthMeters;

  /// Centre of the marking across the road; 0 is straight ahead.
  final double lateralCenterMeters;

  /// Extent across the road.
  final double widthMeters;

  final Confidence confidence;
  final int frameId;
  final int timestampMicros;

  /// Frames this marking has been seen in. One sighting of a crosswalk is a
  /// pattern in the noise; six is a crosswalk.
  final int observationCount;

  final int? firstSeenMicros;

  /// Set once the vehicle has physically crossed a predicted speed bump and
  /// the IMU saw the jolt. Nothing downstream acts on it — it exists so the
  /// detector's claims can be scored against what actually happened.
  final bool confirmedByMotion;

  /// Far edge of the marking.
  double get farEdgeMeters => distanceMeters + depthMeters;

  /// Seconds until the near edge is reached at [speedMps].
  double? timeToReach(double speedMps) {
    if (speedMps <= 0.3) return null;
    return distanceMeters / speedMps;
  }

  /// Whether the marking lies across the path we are actually driving.
  ///
  /// A crosswalk on a side street is visible but irrelevant, and treating it
  /// as ours would brake for nothing.
  bool coversLateral(double egoLateral, double halfWidth) {
    final double left = lateralCenterMeters - widthMeters / 2;
    final double right = lateralCenterMeters + widthMeters / 2;
    return right >= egoLateral - halfWidth && left <= egoLateral + halfWidth;
  }

  RoadMarking copyWith({
    double? distanceMeters,
    Confidence? confidence,
    int? observationCount,
    int? frameId,
    int? timestampMicros,
    bool? confirmedByMotion,
  }) =>
      RoadMarking(
        type: type,
        distanceMeters: distanceMeters ?? this.distanceMeters,
        depthMeters: depthMeters,
        lateralCenterMeters: lateralCenterMeters,
        widthMeters: widthMeters,
        confidence: confidence ?? this.confidence,
        frameId: frameId ?? this.frameId,
        timestampMicros: timestampMicros ?? this.timestampMicros,
        observationCount: observationCount ?? this.observationCount,
        firstSeenMicros: firstSeenMicros,
        confirmedByMotion: confirmedByMotion ?? this.confirmedByMotion,
      );

  Map<String, dynamic> toJson() => <String, dynamic>{
        'type': type.name,
        'dist': double.parse(distanceMeters.toStringAsFixed(2)),
        'depth': double.parse(depthMeters.toStringAsFixed(2)),
        'lat': double.parse(lateralCenterMeters.toStringAsFixed(2)),
        'width': double.parse(widthMeters.toStringAsFixed(2)),
        'conf': double.parse(confidence.value.toStringAsFixed(3)),
        'n': observationCount,
        if (confirmedByMotion) 'imuConfirmed': true,
      };

  static RoadMarking fromJson(
    Map<String, dynamic> j, {
    required int frameId,
    required int timestampMicros,
  }) =>
      RoadMarking(
        type: RoadMarkingType.values.firstWhere(
          (RoadMarkingType t) => t.name == j['type'],
          orElse: () => RoadMarkingType.stopLine,
        ),
        distanceMeters: (j['dist'] as num?)?.toDouble() ?? 0,
        depthMeters: (j['depth'] as num?)?.toDouble() ?? 0,
        lateralCenterMeters: (j['lat'] as num?)?.toDouble() ?? 0,
        widthMeters: (j['width'] as num?)?.toDouble() ?? 0,
        confidence: Confidence((j['conf'] as num?)?.toDouble() ?? 0,
            source: 'recorded'),
        frameId: frameId,
        timestampMicros: timestampMicros,
        observationCount: (j['n'] as num?)?.toInt() ?? 1,
        confirmedByMotion: j['imuConfirmed'] as bool? ?? false,
      );

  @override
  String toString() => '${type.label} at '
      '${distanceMeters.toStringAsFixed(1)} m '
      '(${confidence.percent}%, x$observationCount)';
}

/// One frame's worth of road-marking detection.
class RoadMarkingResult {
  const RoadMarkingResult({
    required this.markings,
    required this.frameId,
    required this.timestampMicros,
    this.isDegraded = false,
    this.degradedReason,
    this.searchRangeMeters = 0,
  });

  /// The honest empty result: we looked and saw nothing.
  factory RoadMarkingResult.none({
    required int frameId,
    required int timestampMicros,
    double searchRangeMeters = 0,
  }) =>
      RoadMarkingResult(
        markings: const <RoadMarking>[],
        frameId: frameId,
        timestampMicros: timestampMicros,
        searchRangeMeters: searchRangeMeters,
      );

  /// We could not look — not the same thing as seeing nothing.
  factory RoadMarkingResult.unavailable({
    required int frameId,
    required int timestampMicros,
    required String reason,
  }) =>
      RoadMarkingResult(
        markings: const <RoadMarking>[],
        frameId: frameId,
        timestampMicros: timestampMicros,
        isDegraded: true,
        degradedReason: reason,
      );

  final List<RoadMarking> markings;
  final int frameId;
  final int timestampMicros;
  final bool isDegraded;
  final String? degradedReason;

  /// How far ahead the bird's-eye grid could actually see this frame. A
  /// crosswalk at 50 m is not "not detected" if we only looked to 45 m.
  final double searchRangeMeters;

  bool get isEmpty => markings.isEmpty;

  RoadMarking? nearestOf(RoadMarkingType type) {
    RoadMarking? best;
    for (final RoadMarking m in markings) {
      if (m.type != type) continue;
      if (best == null || m.distanceMeters < best.distanceMeters) best = m;
    }
    return best;
  }

  List<Map<String, dynamic>> toJson() => <Map<String, dynamic>>[
        for (final RoadMarking m in markings) m.toJson(),
      ];

  @override
  String toString() => 'RoadMarkings(${markings.length}'
      '${isDegraded ? ', DEGRADED: $degradedReason' : ''})';
}
