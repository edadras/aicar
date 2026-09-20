import 'dart:math' as math;

import '../core/confidence.dart';
import '../core/geometry.dart';

/// Where a boundary sits relative to the ego vehicle.
enum LanePosition {
  leftAdjacentOuter('Left adjacent outer'),
  leftAdjacent('Left adjacent'),
  egoLeft('Left lane boundary'),
  egoRight('Right lane boundary'),
  rightAdjacent('Right adjacent'),
  rightAdjacentOuter('Right adjacent outer');

  const LanePosition(this.label);
  final String label;

  bool get isEgoBoundary =>
      this == LanePosition.egoLeft || this == LanePosition.egoRight;
}

/// Marking style. This carries real legal meaning — a solid line means the
/// simulated lane change must not be proposed — so it is modelled explicitly
/// rather than lumped into a generic "lane line".
enum LineType {
  solid('Solid'),
  dashed('Dashed'),
  doubleSolid('Double solid'),
  solidDashed('Solid + dashed'),
  botsDots('Raised markers'),
  curb('Curb'),
  roadEdge('Road edge'),
  unknown('Unknown');

  const LineType(this.label);
  final String label;

  /// Whether crossing this marking is permitted.
  bool get isCrossable => switch (this) {
        LineType.dashed || LineType.botsDots => true,
        LineType.solidDashed => true, // depends on side; resolved by caller
        _ => false,
      };
}

enum LineColor { white, yellow, blue, unknown }

/// One detected lane boundary.
///
/// Geometry is stored in the **vehicle frame** as `x = f(y)`: lateral offset
/// as a function of distance ahead. That parameterisation is single-valued for
/// any real road (unlike `y = f(x)`) and feeds the planner directly.
class LaneBoundary {
  const LaneBoundary({
    required this.position,
    required this.curve,
    required this.confidence,
    required this.lineType,
    required this.color,
    required this.minRangeMeters,
    required this.maxRangeMeters,
    this.imagePoints = const <PixelPoint>[],
    this.supportPointCount = 0,
  });

  final LanePosition position;

  /// `x = c0 + c1*y + c2*y²` in metres, vehicle frame.
  final Polynomial curve;

  final Confidence confidence;
  final LineType lineType;
  final LineColor color;

  /// Longitudinal extent over which the fit is supported by evidence.
  /// Extrapolating a lane beyond its observed range is a classic way to invent
  /// a road that is not there, so consumers must respect this.
  final double minRangeMeters;
  final double maxRangeMeters;

  /// Pixels that supported the fit, kept for the debug overlay.
  final List<PixelPoint> imagePoints;
  final int supportPointCount;

  double get rangeMeters => maxRangeMeters - minRangeMeters;

  /// Lateral offset at [distanceAhead] metres, or `null` outside the
  /// supported range.
  double? lateralAt(double distanceAhead) {
    if (distanceAhead < minRangeMeters - 1 ||
        distanceAhead > maxRangeMeters + 2) {
      return null;
    }
    return curve.evaluate(distanceAhead);
  }

  /// Lateral offset with extrapolation allowed, for drawing only.
  double lateralAtUnchecked(double distanceAhead) =>
      curve.evaluate(distanceAhead);

  /// Heading of the boundary at [distanceAhead], radians, positive to the
  /// right.
  double headingAt(double distanceAhead) =>
      math.atan(curve.derivative(distanceAhead));

  double curvatureAt(double distanceAhead) =>
      curve.curvatureAt(distanceAhead);

  bool get isUsable => confidence.value >= ConfidenceThresholds.laneUsable;

  LaneBoundary copyWith({
    LanePosition? position,
    Polynomial? curve,
    Confidence? confidence,
    LineType? lineType,
    LineColor? color,
    double? minRangeMeters,
    double? maxRangeMeters,
  }) =>
      LaneBoundary(
        position: position ?? this.position,
        curve: curve ?? this.curve,
        confidence: confidence ?? this.confidence,
        lineType: lineType ?? this.lineType,
        color: color ?? this.color,
        minRangeMeters: minRangeMeters ?? this.minRangeMeters,
        maxRangeMeters: maxRangeMeters ?? this.maxRangeMeters,
        imagePoints: imagePoints,
        supportPointCount: supportPointCount,
      );

  Map<String, dynamic> toJson() => <String, dynamic>{
        'pos': position.name,
        'curve': curve.toJson(),
        'conf': confidence.toJson(),
        'type': lineType.name,
        'color': color.name,
        'minRange': double.parse(minRangeMeters.toStringAsFixed(2)),
        'maxRange': double.parse(maxRangeMeters.toStringAsFixed(2)),
        'support': supportPointCount,
      };

  static LaneBoundary fromJson(Map<String, dynamic> j) => LaneBoundary(
        position: LanePosition.values.firstWhere(
          (LanePosition p) => p.name == j['pos'],
          orElse: () => LanePosition.egoLeft,
        ),
        curve: Polynomial.fromJson(j['curve'] as List<dynamic>),
        confidence: Confidence.fromJson(j['conf'] as Map<String, dynamic>),
        lineType: LineType.values.firstWhere(
          (LineType t) => t.name == j['type'],
          orElse: () => LineType.unknown,
        ),
        color: LineColor.values.firstWhere(
          (LineColor c) => c.name == j['color'],
          orElse: () => LineColor.unknown,
        ),
        minRangeMeters: (j['minRange'] as num).toDouble(),
        maxRangeMeters: (j['maxRange'] as num).toDouble(),
        supportPointCount: (j['support'] as num?)?.toInt() ?? 0,
      );

  @override
  String toString() => '${position.label} ${confidence.percent}% '
      '${lineType.label}/${color.name} (${minRangeMeters.toStringAsFixed(0)}–'
      '${maxRangeMeters.toStringAsFixed(0)}m)';
}

/// How the lane/corridor geometry for this frame was obtained.
enum LaneMode {
  /// Both ego boundaries detected with usable confidence.
  bothBoundaries('LANE'),

  /// One boundary detected; the other inferred from the learned lane width.
  singleBoundary('SINGLE LANE EDGE'),

  /// No usable markings. The corridor comes from drivable area, road edges,
  /// curbs, lead-vehicle trajectories and navigation intent instead.
  noLane('NO_LANE_MODE'),

  /// Nothing usable at all — not even a drivable region.
  none('NO ROAD MODEL');

  const LaneMode(this.badge);
  final String badge;
}

/// The lane model for one frame.
class LaneDetectionResult {
  const LaneDetectionResult({
    required this.boundaries,
    required this.mode,
    required this.frameId,
    required this.timestampMicros,
    required this.laneWidthMeters,
    required this.laneWidthConfidence,
    required this.egoLateralOffsetMeters,
    required this.egoHeadingErrorRadians,
    this.modelName = 'classical-cv',
    this.isDegraded = false,
    this.degradedReason,
  });

  factory LaneDetectionResult.empty({
    required int frameId,
    required int timestampMicros,
    String reason = 'no lane evidence',
  }) =>
      LaneDetectionResult(
        boundaries: const <LaneBoundary>[],
        mode: LaneMode.none,
        frameId: frameId,
        timestampMicros: timestampMicros,
        laneWidthMeters: 3.5,
        laneWidthConfidence: 0,
        egoLateralOffsetMeters: 0,
        egoHeadingErrorRadians: 0,
        isDegraded: true,
        degradedReason: reason,
      );

  final List<LaneBoundary> boundaries;
  final LaneMode mode;
  final int frameId;
  final int timestampMicros;

  /// Estimated width of the ego lane. Tracked over time because it is a much
  /// better prior than a hard-coded 3.5 m once a few good frames have passed.
  final double laneWidthMeters;
  final double laneWidthConfidence;

  /// Signed lateral offset of the vehicle from the lane centre, metres.
  /// Positive = the vehicle is right of centre.
  final double egoLateralOffsetMeters;

  /// Angle between the vehicle heading and the lane direction, radians.
  /// Positive = the vehicle is pointing right of the lane.
  final double egoHeadingErrorRadians;

  final String modelName;
  final bool isDegraded;
  final String? degradedReason;

  LaneBoundary? boundaryAt(LanePosition p) {
    for (final LaneBoundary b in boundaries) {
      if (b.position == p) return b;
    }
    return null;
  }

  LaneBoundary? get left => boundaryAt(LanePosition.egoLeft);
  LaneBoundary? get right => boundaryAt(LanePosition.egoRight);

  List<LaneBoundary> get adjacent => boundaries
      .where((LaneBoundary b) => !b.position.isEgoBoundary)
      .toList();

  bool get hasLeftAdjacentLane =>
      boundaryAt(LanePosition.leftAdjacent) != null;
  bool get hasRightAdjacentLane =>
      boundaryAt(LanePosition.rightAdjacent) != null;

  /// Centreline of the ego lane as `x = f(y)`, or `null` when the lane model
  /// is not good enough to define one.
  Polynomial? get centerline {
    final LaneBoundary? l = left;
    final LaneBoundary? r = right;
    if (l != null && r != null) {
      return Polynomial(<double>[
        for (int i = 0; i < math.max(l.curve.coefficients.length,
                r.curve.coefficients.length); i++)
          ((i < l.curve.coefficients.length ? l.curve.coefficients[i] : 0) +
                  (i < r.curve.coefficients.length
                      ? r.curve.coefficients[i]
                      : 0)) /
              2,
      ]);
    }
    if (l != null) {
      return Polynomial(<double>[
        l.curve.coefficients[0] + laneWidthMeters / 2,
        ...l.curve.coefficients.skip(1),
      ]);
    }
    if (r != null) {
      return Polynomial(<double>[
        r.curve.coefficients[0] - laneWidthMeters / 2,
        ...r.curve.coefficients.skip(1),
      ]);
    }
    return null;
  }

  /// Aggregate lane confidence used by the autonomy roll-up.
  double get overallConfidence {
    if (isDegraded) return 0;
    final LaneBoundary? l = left;
    final LaneBoundary? r = right;
    return switch (mode) {
      LaneMode.bothBoundaries =>
        math.min(l?.confidence.value ?? 0, r?.confidence.value ?? 0) * 0.5 +
            ((l?.confidence.value ?? 0) + (r?.confidence.value ?? 0)) / 4,
      LaneMode.singleBoundary =>
        (l?.confidence.value ?? r?.confidence.value ?? 0) * 0.6,
      LaneMode.noLane => 0.25,
      LaneMode.none => 0.0,
    };
  }

  /// Furthest distance ahead over which the lane model is supported.
  double get usableRangeMeters {
    double best = 0;
    for (final LaneBoundary b in boundaries) {
      if (b.position.isEgoBoundary && b.maxRangeMeters > best) {
        best = b.maxRangeMeters;
      }
    }
    return best;
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'mode': mode.name,
        'boundaries': <Map<String, dynamic>>[
          for (final LaneBoundary b in boundaries) b.toJson(),
        ],
        'laneWidth': double.parse(laneWidthMeters.toStringAsFixed(2)),
        'laneWidthConf': double.parse(laneWidthConfidence.toStringAsFixed(3)),
        'egoOffset': double.parse(egoLateralOffsetMeters.toStringAsFixed(3)),
        'egoHeadingError':
            double.parse(egoHeadingErrorRadians.toStringAsFixed(4)),
        'model': modelName,
        if (isDegraded) 'degraded': degradedReason,
      };

  static LaneDetectionResult fromJson(
    Map<String, dynamic> j, {
    required int frameId,
    required int timestampMicros,
  }) =>
      LaneDetectionResult(
        boundaries: <LaneBoundary>[
          for (final dynamic b in (j['boundaries'] as List<dynamic>))
            LaneBoundary.fromJson(b as Map<String, dynamic>),
        ],
        mode: LaneMode.values.firstWhere(
          (LaneMode m) => m.name == j['mode'],
          orElse: () => LaneMode.none,
        ),
        frameId: frameId,
        timestampMicros: timestampMicros,
        laneWidthMeters: (j['laneWidth'] as num).toDouble(),
        laneWidthConfidence: (j['laneWidthConf'] as num).toDouble(),
        egoLateralOffsetMeters: (j['egoOffset'] as num).toDouble(),
        egoHeadingErrorRadians: (j['egoHeadingError'] as num).toDouble(),
        modelName: j['model'] as String? ?? 'replay',
        isDegraded: j['degraded'] != null,
        degradedReason: j['degraded'] as String?,
      );

  @override
  String toString() => 'Lanes(${mode.badge}, ${boundaries.length} boundaries, '
      'width ${laneWidthMeters.toStringAsFixed(2)}m, '
      'offset ${egoLateralOffsetMeters.toStringAsFixed(2)}m)';
}
