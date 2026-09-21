import '../core/confidence.dart';

/// Which side of the carriageway traffic drives on.
///
/// Needed because "the lane markings on my left are solid" means opposite
/// things in Tehran and in London.
enum DrivingSide {
  right('Right-hand traffic'),
  left('Left-hand traffic');

  const DrivingSide(this.label);
  final String label;

  /// The side the carriageway's outer edge is on.
  bool get outerEdgeIsRight => this == DrivingSide.right;
}

/// Where the vehicle is across the road, and how much of that we actually
/// know.
///
/// Two very different questions live here, and keeping them apart is the
/// point of the class:
///
///  * **Where in the lane am I?** Answered by the camera, to a few
///    centimetres, whenever markings are visible.
///  * **Which lane am I in?** Almost never answered. It needs a lane-level
///    map or an unambiguous set of boundary types, and most of the time
///    neither exists — so [laneIndexFromEdge] is usually null and
///    [isLaneLevel] is usually false.
///
/// Nothing here comes from GPS. A phone's GNSS fix is good to a handful of
/// metres at best, which is wider than a lane; using it to decide which lane
/// we are in would produce an answer that is confident and wrong about half
/// the time. GPS contributes the *road*, through map matching, and nothing
/// finer.
class LateralState {
  const LateralState({
    required this.offsetInLaneMeters,
    required this.offsetConfidence,
    required this.source,
    this.laneWidthMeters,
    this.laneIndexFromEdge,
    this.laneCount,
    this.laneIndexConfidence = 0,
    this.secondsSinceObservation = 0,
    this.headingAgreesWithRoad,
  });

  static final LateralState unknownState = LateralState(
    offsetInLaneMeters: 0,
    offsetConfidence: Confidence.zero,
    source: LateralSource.none,
  );

  /// Signed offset from the lane centre, metres. Positive is right.
  final double offsetInLaneMeters;

  final Confidence offsetConfidence;

  /// What produced the current offset.
  final LateralSource source;

  final double? laneWidthMeters;

  /// Lane index counted from the outer edge of the carriageway, 0-based, or
  /// null when it cannot be established — which is the normal case.
  final int? laneIndexFromEdge;

  /// Total lanes in our direction, when the map or the markings say so.
  final int? laneCount;

  final double laneIndexConfidence;

  /// How long since the camera last saw markings. The dead-reckoned bridge
  /// is only good for a second or two.
  final double secondsSinceObservation;

  /// Whether the fused heading agrees with the matched road's bearing.
  ///
  /// A cross-check, not a position: disagreement means the map match is on
  /// the wrong road, which invalidates anything derived from it.
  final bool? headingAgreesWithRoad;

  /// True only when the lane we are in is actually known, rather than the
  /// position within whatever lane we are in.
  bool get isLaneLevel =>
      laneIndexFromEdge != null && laneIndexConfidence >= 0.6;

  bool get isUsable => offsetConfidence.value >= 0.35;

  /// Plain-language summary for the HUD and the log.
  String get description {
    if (!isUsable) return 'Lateral position unknown';
    final String where = offsetInLaneMeters.abs() < 0.15
        ? 'centred in lane'
        : '${offsetInLaneMeters.abs().toStringAsFixed(2)} m '
            '${offsetInLaneMeters < 0 ? 'left' : 'right'} of centre';
    final String which = isLaneLevel
        ? ', lane ${laneIndexFromEdge! + 1}'
            '${laneCount == null ? '' : ' of $laneCount'}'
        : ', lane number unknown';
    return '$where$which (${source.label})';
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'offset': double.parse(offsetInLaneMeters.toStringAsFixed(3)),
        'conf': double.parse(offsetConfidence.value.toStringAsFixed(3)),
        'source': source.name,
        if (laneWidthMeters != null)
          'laneWidth': double.parse(laneWidthMeters!.toStringAsFixed(2)),
        if (laneIndexFromEdge != null) 'laneIndex': laneIndexFromEdge,
        if (laneCount != null) 'laneCount': laneCount,
        'laneIndexConf': double.parse(laneIndexConfidence.toStringAsFixed(3)),
        'sinceObs': double.parse(secondsSinceObservation.toStringAsFixed(2)),
        if (headingAgreesWithRoad != null)
          'headingAgrees': headingAgreesWithRoad,
      };

  static LateralState fromJson(Map<String, dynamic>? j) {
    if (j == null) return unknownState;
    return LateralState(
      offsetInLaneMeters: (j['offset'] as num?)?.toDouble() ?? 0,
      offsetConfidence:
          Confidence((j['conf'] as num?)?.toDouble() ?? 0, source: 'recorded'),
      source: LateralSource.values.firstWhere(
        (LateralSource s) => s.name == j['source'],
        orElse: () => LateralSource.none,
      ),
      laneWidthMeters: (j['laneWidth'] as num?)?.toDouble(),
      laneIndexFromEdge: (j['laneIndex'] as num?)?.toInt(),
      laneCount: (j['laneCount'] as num?)?.toInt(),
      laneIndexConfidence: (j['laneIndexConf'] as num?)?.toDouble() ?? 0,
      secondsSinceObservation: (j['sinceObs'] as num?)?.toDouble() ?? 0,
      headingAgreesWithRoad: j['headingAgrees'] as bool?,
    );
  }

  @override
  String toString() => 'LateralState($description)';
}

/// Where the current lateral estimate came from.
enum LateralSource {
  /// The camera saw both lane boundaries this frame.
  laneObservation('camera'),

  /// One boundary plus the learned lane width.
  singleBoundary('camera, one edge'),

  /// No markings this frame; carried forward on yaw rate and speed.
  deadReckoned('IMU dead reckoning'),

  /// Nothing usable.
  none('none');

  const LateralSource(this.label);
  final String label;
}
