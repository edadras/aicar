import 'dart:math' as math;

import 'geometry.dart';

/// Every perception output in this project carries a confidence. Nothing the
/// AI produces is treated as ground truth — see `docs/CONFIDENCE.md`.
///
/// A [Confidence] is a value in `[0, 1]` plus the reason it ended up there,
/// which is what makes the debug overlay and the recorded dataset useful.
class Confidence implements Comparable<Confidence> {
  Confidence(double value, {this.source = ''})
      : value = clampDouble(value, 0.0, 1.0);

  static final Confidence zero = Confidence(0, source: 'none');
  static final Confidence certain = Confidence(1, source: 'exact');

  final double value;
  final String source;

  bool get isUsable => value >= ConfidenceThresholds.usable;
  bool get isHigh => value >= ConfidenceThresholds.high;
  bool get isLow => value < ConfidenceThresholds.usable;

  int get percent => (value * 100).round();

  Confidence scaled(double k, {String? source}) =>
      Confidence(value * k, source: source ?? this.source);

  /// Independent-evidence combination (noisy-OR). Use when two *different*
  /// cues both support the same hypothesis.
  Confidence combineIndependent(Confidence other) => Confidence(
        1 - (1 - value) * (1 - other.value),
        source: '$source+${other.source}',
      );

  /// Conjunctive combination. Use when a result is only as good as its weakest
  /// input (e.g. a distance that needs both a detection *and* a depth map).
  Confidence combineConjunctive(Confidence other) => Confidence(
        value * other.value,
        source: '$source&${other.source}',
      );

  @override
  int compareTo(Confidence other) => value.compareTo(other.value);

  Map<String, dynamic> toJson() =>
      <String, dynamic>{'v': double.parse(value.toStringAsFixed(4)), 's': source};

  static Confidence fromJson(Map<String, dynamic> j) =>
      Confidence((j['v'] as num).toDouble(), source: (j['s'] as String?) ?? '');

  @override
  String toString() => '$percent%${source.isEmpty ? '' : ' ($source)'}';
}

/// Shared thresholds so that "low confidence" means the same thing in the
/// decision engine, the HUD and the recorder.
class ConfidenceThresholds {
  const ConfidenceThresholds._();

  /// Below this, a perception result is not allowed to influence control.
  static const double usable = 0.45;

  /// Above this, a result may be acted on without corroboration.
  static const double high = 0.75;

  /// Below this aggregate score the whole stack declares AUTONOMY CONFIDENCE
  /// LOW and the decision engine drops to `UNCERTAIN`.
  static const double autonomyFloor = 0.40;

  /// A lane is only used for path generation above this.
  static const double laneUsable = 0.50;

  /// Detections weaker than this are dropped before tracking.
  static const double detectionFloor = 0.30;
}

/// Weighted roll-up of the per-subsystem confidences into a single number
/// that drives the AUTONOMY CONFIDENCE LOW banner.
///
/// The weights encode which subsystems the control decision actually depends
/// on: if we cannot see where the road is, nothing else matters much.
class AutonomyConfidence {
  const AutonomyConfidence({
    required this.perception,
    required this.lanes,
    required this.depth,
    required this.egoMotion,
    required this.planning,
    required this.overall,
    required this.weakestSubsystem,
  });

  factory AutonomyConfidence.compute({
    required double perception,
    required double lanes,
    required double depth,
    required double egoMotion,
    required double planning,
  }) {
    const Map<String, double> weights = <String, double>{
      'perception': 0.28,
      'lanes': 0.22,
      'depth': 0.18,
      'egoMotion': 0.12,
      'planning': 0.20,
    };
    final Map<String, double> values = <String, double>{
      'perception': clampDouble(perception, 0, 1),
      'lanes': clampDouble(lanes, 0, 1),
      'depth': clampDouble(depth, 0, 1),
      'egoMotion': clampDouble(egoMotion, 0, 1),
      'planning': clampDouble(planning, 0, 1),
    };

    double weighted = 0;
    for (final MapEntry<String, double> e in values.entries) {
      weighted += e.value * weights[e.key]!;
    }

    // A weighted mean alone hides a single catastrophic subsystem, so pull the
    // score towards the weakest input. The stack is only as trustworthy as the
    // thing it is worst at.
    double worst = 1;
    String worstName = 'none';
    for (final MapEntry<String, double> e in values.entries) {
      if (e.value < worst) {
        worst = e.value;
        worstName = e.key;
      }
    }
    final double overall = math.min(weighted, 0.5 * weighted + 0.5 * worst);

    return AutonomyConfidence(
      perception: values['perception']!,
      lanes: values['lanes']!,
      depth: values['depth']!,
      egoMotion: values['egoMotion']!,
      planning: values['planning']!,
      overall: clampDouble(overall, 0, 1),
      weakestSubsystem: worstName,
    );
  }

  static const AutonomyConfidence unknown = AutonomyConfidence(
    perception: 0,
    lanes: 0,
    depth: 0,
    egoMotion: 0,
    planning: 0,
    overall: 0,
    weakestSubsystem: 'uninitialised',
  );

  final double perception;
  final double lanes;
  final double depth;
  final double egoMotion;
  final double planning;
  final double overall;
  final String weakestSubsystem;

  bool get isLow => overall < ConfidenceThresholds.autonomyFloor;

  Map<String, dynamic> toJson() => <String, dynamic>{
        'perception': _r(perception),
        'lanes': _r(lanes),
        'depth': _r(depth),
        'egoMotion': _r(egoMotion),
        'planning': _r(planning),
        'overall': _r(overall),
        'weakest': weakestSubsystem,
      };

  static double _r(double v) => double.parse(v.toStringAsFixed(3));

  @override
  String toString() =>
      'Autonomy ${(overall * 100).round()}% (weakest: $weakestSubsystem)';
}
