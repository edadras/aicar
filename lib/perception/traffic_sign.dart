import '../core/confidence.dart';
import '../core/geometry.dart';

/// Regulatory and warning signs the stack reacts to.
///
/// Only signs with a behavioural consequence are modelled. A sign that would
/// not change any decision is not worth a class, an icon and a confidence.
enum TrafficSignType {
  stop('STOP', isRegulatory: true, requiresStop: true),
  giveWay('GIVE WAY', isRegulatory: true),
  noEntry('NO ENTRY', isRegulatory: true),
  speedLimit('SPEED LIMIT', isRegulatory: true, carriesValue: true),
  endOfSpeedLimit('END OF LIMIT', isRegulatory: true, carriesValue: true),
  noOvertaking('NO OVERTAKING', isRegulatory: true),
  endOfNoOvertaking('END OF NO OVERTAKING', isRegulatory: true),
  noLeftTurn('NO LEFT TURN', isRegulatory: true),
  noRightTurn('NO RIGHT TURN', isRegulatory: true),
  noUTurn('NO U-TURN', isRegulatory: true),
  mandatoryStraight('STRAIGHT ONLY', isRegulatory: true),
  mandatoryLeft('LEFT ONLY', isRegulatory: true),
  mandatoryRight('RIGHT ONLY', isRegulatory: true),
  pedestrianCrossing('PEDESTRIAN CROSSING', isWarning: true),
  schoolZone('SCHOOL ZONE', isWarning: true, carriesValue: true),
  roadWork('ROAD WORK', isWarning: true),
  slipperyRoad('SLIPPERY ROAD', isWarning: true),
  bumpyRoad('UNEVEN ROAD', isWarning: true),
  narrowRoad('ROAD NARROWS', isWarning: true),
  curveLeft('CURVE LEFT', isWarning: true),
  curveRight('CURVE RIGHT', isWarning: true),
  roundabout('ROUNDABOUT', isWarning: true),
  trafficSignalAhead('SIGNALS AHEAD', isWarning: true),
  animalCrossing('ANIMAL CROSSING', isWarning: true),
  generalWarning('WARNING', isWarning: true),
  unknown('UNKNOWN SIGN');

  const TrafficSignType(
    this.label, {
    this.isRegulatory = false,
    this.isWarning = false,
    this.requiresStop = false,
    this.carriesValue = false,
  });

  final String label;
  final bool isRegulatory;
  final bool isWarning;

  /// The decision engine must produce a full stop for this sign.
  final bool requiresStop;

  /// The sign has a numeric value (a speed limit, a zone limit).
  final bool carriesValue;

  /// Signs that constrain the simulated lane-change manoeuvre.
  bool get forbidsOvertaking => this == TrafficSignType.noOvertaking;

  /// How far ahead the sign stays relevant after it leaves the frame, metres.
  double get persistenceMeters => switch (this) {
        TrafficSignType.speedLimit => 800,
        TrafficSignType.endOfSpeedLimit => 50,
        TrafficSignType.noOvertaking => 500,
        TrafficSignType.schoolZone => 300,
        TrafficSignType.roadWork => 300,
        TrafficSignType.stop => 60,
        TrafficSignType.giveWay => 60,
        TrafficSignType.pedestrianCrossing => 80,
        _ => 120,
      };
}

/// A recognised sign with its distance and (where applicable) its value.
class TrafficSign {
  const TrafficSign({
    required this.id,
    required this.type,
    required this.box,
    required this.confidence,
    required this.frameId,
    required this.timestampMicros,
    this.speedLimitKph,
    this.valueConfidence,
    this.distanceMeters,
    this.appliesToEgoLane = true,
    this.firstSeenMicros,
    this.observationCount = 1,
  });

  final int id;
  final TrafficSignType type;
  final BoundingBox box;
  final Confidence confidence;
  final int frameId;
  final int timestampMicros;

  /// Extracted numeric value for [TrafficSignType.speedLimit] and friends.
  final int? speedLimitKph;

  /// Confidence in the *number*, which is usually lower than confidence in
  /// the sign's presence: recognising a round red-rimmed sign is easy,
  /// reading "80" versus "30" at 60 m is not.
  final Confidence? valueConfidence;

  final double? distanceMeters;

  /// Signs on a slip road or a side street do not apply to us. Resolved from
  /// the sign's lateral position relative to the drivable corridor.
  final bool appliesToEgoLane;

  final int? firstSeenMicros;

  /// How many frames this sign has been confirmed in. A speed limit seen once
  /// is a guess; seen eight times it is a fact.
  final int observationCount;

  /// Only accept a speed limit once it has been seen repeatedly and read
  /// confidently. Acting on a misread limit is worse than not reading it.
  bool get isSpeedLimitTrustworthy =>
      type == TrafficSignType.speedLimit &&
      speedLimitKph != null &&
      observationCount >= 3 &&
      (valueConfidence?.value ?? 0) >= 0.70;

  String get displayText {
    if (type.carriesValue && speedLimitKph != null) {
      return '${type.label}\n$speedLimitKph km/h';
    }
    return type.label;
  }

  TrafficSign copyWith({
    BoundingBox? box,
    Confidence? confidence,
    int? speedLimitKph,
    Confidence? valueConfidence,
    double? distanceMeters,
    bool? appliesToEgoLane,
    int? observationCount,
    int? frameId,
    int? timestampMicros,
  }) =>
      TrafficSign(
        id: id,
        type: type,
        box: box ?? this.box,
        confidence: confidence ?? this.confidence,
        frameId: frameId ?? this.frameId,
        timestampMicros: timestampMicros ?? this.timestampMicros,
        speedLimitKph: speedLimitKph ?? this.speedLimitKph,
        valueConfidence: valueConfidence ?? this.valueConfidence,
        distanceMeters: distanceMeters ?? this.distanceMeters,
        appliesToEgoLane: appliesToEgoLane ?? this.appliesToEgoLane,
        firstSeenMicros: firstSeenMicros,
        observationCount: observationCount ?? this.observationCount,
      );

  Map<String, dynamic> toJson() => <String, dynamic>{
        'id': id,
        'type': type.name,
        'box': box.toJson(),
        'conf': confidence.toJson(),
        if (speedLimitKph != null) 'value': speedLimitKph,
        if (valueConfidence != null) 'valueConf': valueConfidence!.toJson(),
        if (distanceMeters != null)
          'dist': double.parse(distanceMeters!.toStringAsFixed(2)),
        'egoLane': appliesToEgoLane,
        'count': observationCount,
      };

  static TrafficSign fromJson(
    Map<String, dynamic> j, {
    required int frameId,
    required int timestampMicros,
  }) =>
      TrafficSign(
        id: (j['id'] as num).toInt(),
        type: TrafficSignType.values.firstWhere(
          (TrafficSignType t) => t.name == j['type'],
          orElse: () => TrafficSignType.unknown,
        ),
        box: BoundingBox.fromJson(j['box'] as Map<String, dynamic>),
        confidence: Confidence.fromJson(j['conf'] as Map<String, dynamic>),
        frameId: frameId,
        timestampMicros: timestampMicros,
        speedLimitKph: (j['value'] as num?)?.toInt(),
        valueConfidence: j['valueConf'] == null
            ? null
            : Confidence.fromJson(j['valueConf'] as Map<String, dynamic>),
        distanceMeters: (j['dist'] as num?)?.toDouble(),
        appliesToEgoLane: j['egoLane'] as bool? ?? true,
        observationCount: (j['count'] as num?)?.toInt() ?? 1,
      );

  @override
  String toString() => '${type.label}'
      '${speedLimitKph != null ? ' $speedLimitKph' : ''} '
      '${confidence.percent}%'
      '${distanceMeters != null ? ' @${distanceMeters!.toStringAsFixed(0)}m' : ''}';
}

/// The regulatory context currently in force, assembled from signs seen over
/// time rather than from the current frame alone.
///
/// A speed limit sign is visible for maybe a second and then governs the next
/// kilometre; without this accumulator the stack would forget it instantly.
class RegulatoryContext {
  const RegulatoryContext({
    this.speedLimitKph,
    this.speedLimitConfidence = 0,
    this.speedLimitSetAtMeters,
    this.overtakingProhibited = false,
    this.inSchoolZone = false,
    this.inRoadWorks = false,
    this.pendingStop = false,
    this.pendingGiveWay = false,
    this.distanceTravelledMeters = 0,
  });

  final int? speedLimitKph;
  final double speedLimitConfidence;

  /// Odometer reading when the current limit was adopted, so it can expire
  /// after [TrafficSignType.persistenceMeters].
  final double? speedLimitSetAtMeters;

  final bool overtakingProhibited;
  final bool inSchoolZone;
  final bool inRoadWorks;
  final bool pendingStop;
  final bool pendingGiveWay;
  final double distanceTravelledMeters;

  bool get hasSpeedLimit => speedLimitKph != null && speedLimitConfidence > 0.6;

  /// Target speed the simulated controller aims for, m/s.
  ///
  /// Road works and school zones tighten the limit on top of the posted value
  /// because the posted value assumes normal conditions.
  double? get targetSpeedMps {
    if (!hasSpeedLimit) return null;
    double kph = speedLimitKph!.toDouble();
    if (inSchoolZone) kph = kph.clamp(0, 30);
    if (inRoadWorks) kph = kph.clamp(0, 50);
    return kph / 3.6;
  }

  RegulatoryContext copyWith({
    int? speedLimitKph,
    double? speedLimitConfidence,
    double? speedLimitSetAtMeters,
    bool? overtakingProhibited,
    bool? inSchoolZone,
    bool? inRoadWorks,
    bool? pendingStop,
    bool? pendingGiveWay,
    double? distanceTravelledMeters,
  }) =>
      RegulatoryContext(
        speedLimitKph: speedLimitKph ?? this.speedLimitKph,
        speedLimitConfidence:
            speedLimitConfidence ?? this.speedLimitConfidence,
        speedLimitSetAtMeters:
            speedLimitSetAtMeters ?? this.speedLimitSetAtMeters,
        overtakingProhibited:
            overtakingProhibited ?? this.overtakingProhibited,
        inSchoolZone: inSchoolZone ?? this.inSchoolZone,
        inRoadWorks: inRoadWorks ?? this.inRoadWorks,
        pendingStop: pendingStop ?? this.pendingStop,
        pendingGiveWay: pendingGiveWay ?? this.pendingGiveWay,
        distanceTravelledMeters:
            distanceTravelledMeters ?? this.distanceTravelledMeters,
      );

  Map<String, dynamic> toJson() => <String, dynamic>{
        if (speedLimitKph != null) 'speedLimit': speedLimitKph,
        'speedLimitConf':
            double.parse(speedLimitConfidence.toStringAsFixed(3)),
        'noOvertaking': overtakingProhibited,
        'schoolZone': inSchoolZone,
        'roadWorks': inRoadWorks,
        'pendingStop': pendingStop,
        'pendingGiveWay': pendingGiveWay,
      };

  @override
  String toString() => 'Regulatory('
      '${speedLimitKph ?? '-'} km/h'
      '${overtakingProhibited ? ', no overtaking' : ''}'
      '${inSchoolZone ? ', school' : ''}'
      '${inRoadWorks ? ', works' : ''})';
}
