import '../core/geometry.dart';

/// Where a signal head or a sign sits relative to us, and what the stack
/// knows about the junction it might belong to.
class RelevanceEvidence {
  const RelevanceEvidence({
    required this.lateralOffsetMeters,
    required this.distanceMeters,
    required this.pathLateralAtDistance,
    required this.egoLaneHalfWidth,
    this.boxHeightPixels = 20,
    this.aspectRatio,
    this.lateralDriftPerMetre,
    this.junctionDistanceMeters,
    this.junctionConfidence = 0,
  });

  /// Lateral position of the object in the vehicle frame, metres.
  final double lateralOffsetMeters;

  final double distanceMeters;

  /// Where our own path will be at [distanceMeters].
  final double pathLateralAtDistance;

  final double egoLaneHalfWidth;

  final double boxHeightPixels;

  /// Width divided by height of the housing, where known.
  final double? aspectRatio;

  /// How fast the object slides across our path per metre we close on it.
  final double? lateralDriftPerMetre;

  final double? junctionDistanceMeters;
  final double junctionConfidence;

  double get offsetFromPath => lateralOffsetMeters - pathLateralAtDistance;

  /// How far off our path an object may sit and still plausibly govern us.
  ///
  /// Widens with distance because both the range estimate and the path
  /// prediction get looser the further out they reach.
  double get toleranceMeters =>
      egoLaneHalfWidth + 2.0 + clampDouble(distanceMeters / 25, 0, 3.0);
}

/// The verdict, with the evidence that produced it.
class RelevanceVerdict {
  const RelevanceVerdict({
    required this.governsEgo,
    required this.confidence,
    required this.reasons,
  });

  /// True for "ours", false for "someone else's", null for "cannot tell".
  ///
  /// Null is a real answer and the right one surprisingly often. A system
  /// that confidently attributes the cross street's red light to its own
  /// lane will brake in the middle of a junction.
  final bool? governsEgo;

  final double confidence;

  /// What each cue contributed, for the debug overlay and the recording.
  final List<String> reasons;

  @override
  String toString() => 'Relevance('
      '${governsEgo == null ? 'unknown' : (governsEgo! ? 'ours' : 'other')}, '
      '${(confidence * 100).round()}%)';
}

/// Decides whether a signal head or sign governs *our* lane.
///
/// Lateral offset alone is not enough, and relying on it is how a stack ends
/// up braking for the cross street's red light: a head on a gantry over the
/// cross street can sit almost directly over our path. Three further cues,
/// none sufficient alone, make the difference:
///
///  * **Agreement with the junction.** A signal head governs an approach to a
///    junction. One whose range matches the junction ahead is far more likely
///    to be ours than one 40 m short of it. This is the strongest cue and is
///    why the junction detector exists upstream of it.
///  * **Foreshortening.** A vertical housing facing us is about a third as
///    wide as it is tall. Facing the cross street it is seen edge-on and is
///    far narrower. Only applied where the housing is recognisably vertical —
///    guessing at foreshortening without knowing the orientation would
///    misread every horizontally-mounted head.
///  * **Lateral drift.** As we approach, a head that governs us stays over
///    our lane; one over the cross street sweeps sideways. Measured across
///    frames, this separates the two cases geometry alone cannot.
class SignalRelevanceResolver {
  const SignalRelevanceResolver({
    this.farOffPathMultiple = 2.2,
    this.driftThresholdPerMetre = 0.09,
    this.junctionToleranceMeters = 30,
    this.governsThreshold = 0.42,
    this.rejectThreshold = 0.22,
  });

  /// Beyond this multiple of the tolerance, the object is not ours and no
  /// other cue is needed.
  final double farOffPathMultiple;

  final double driftThresholdPerMetre;

  /// How far a head may be from the junction estimate and still be counted
  /// as part of it. Generous, because the stop line, the near head and the
  /// far head of one junction can span 30 m, and the junction estimate is
  /// itself only good to a few metres.
  final double junctionToleranceMeters;

  /// Score at or above which the object is taken to govern us.
  final double governsThreshold;

  /// Score at or below which it is taken to govern something else. Between
  /// the two, the answer is that we do not know — which is a real answer.
  final double rejectThreshold;

  RelevanceVerdict resolve(RelevanceEvidence e) {
    final List<String> reasons = <String>[];
    final double offset = e.offsetFromPath.abs();
    final double tolerance = e.toleranceMeters;

    if (offset > tolerance * farOffPathMultiple) {
      return RelevanceVerdict(
        governsEgo: false,
        confidence: 0.7,
        reasons: <String>[
          '${offset.toStringAsFixed(1)} m off our path at '
              '${e.distanceMeters.toStringAsFixed(0)} m',
        ],
      );
    }

    final double? aspect = e.aspectRatio;
    final bool foreshortened =
        aspect != null && aspect > 0.05 && aspect < 0.18;
    if (foreshortened) reasons.add('housing seen edge-on');

    final double? drift = e.lateralDriftPerMetre;
    final bool driftingAway =
        drift != null && drift.abs() > driftThresholdPerMetre;
    if (driftingAway) {
      reasons.add('sliding across our path as we approach '
          '(${drift.abs().toStringAsFixed(2)} m per metre closed)');
    }

    if (offset > tolerance) {
      if (foreshortened || driftingAway) {
        return RelevanceVerdict(
          governsEgo: false,
          confidence: 0.6,
          reasons: reasons,
        );
      }
      return RelevanceVerdict(
        governsEgo: null,
        confidence: 0.3,
        reasons: <String>[
          ...reasons,
          'off our path but not far enough to rule out',
        ],
      );
    }

    // Position gives a starting likelihood; the other cues multiply it.
    //
    // Multiplicative rather than additive on purpose. "The housing is seen
    // edge-on" is not a small deduction from an otherwise good case — it is
    // strong evidence the head is aimed at a different approach, and it has
    // to be able to overturn a confident position. An additive model cannot
    // do that without penalties so large they misbehave everywhere else.
    final double sizeQuality = clampDouble((e.boxHeightPixels - 8) / 20, 0, 1);
    final double centrality = clampDouble(1 - offset / tolerance, 0, 1);
    double score = 0.25 + 0.3 * centrality + 0.2 * sizeQuality;
    reasons.add('${offset.toStringAsFixed(1)} m from our path '
        '(tolerance ${tolerance.toStringAsFixed(1)} m)');

    final double? junction = e.junctionDistanceMeters;
    if (junction != null && e.junctionConfidence > 0.4) {
      final double mismatch = (e.distanceMeters - junction).abs();
      if (mismatch <= junctionToleranceMeters) {
        final double fit = clampDouble(1 - mismatch / junctionToleranceMeters,
            0, 1);
        score *= 1 + 0.4 * e.junctionConfidence * fit;
        reasons.add('matches the junction '
            '${junction.toStringAsFixed(0)} m ahead');
      } else {
        score *= 1 - 0.75 * e.junctionConfidence;
        reasons.add('${mismatch.toStringAsFixed(0)} m from the junction '
            'ahead — governs something else');
      }
    }

    if (foreshortened) score *= 0.35;
    if (driftingAway) score *= 0.4;

    if (score <= rejectThreshold) {
      return RelevanceVerdict(
        governsEgo: false,
        confidence: 0.6,
        reasons: reasons,
      );
    }
    if (score < governsThreshold) {
      return RelevanceVerdict(
        governsEgo: null,
        confidence: 0.3,
        reasons: reasons,
      );
    }
    return RelevanceVerdict(
      governsEgo: true,
      confidence: clampDouble(score, 0, 0.95),
      reasons: reasons,
    );
  }
}
