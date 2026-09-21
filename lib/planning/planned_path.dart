import 'dart:math' as math;

import '../camera/camera_calibration.dart';
import '../core/geometry.dart';

/// One point on the planned path.
class PathPoint {
  const PathPoint({
    required this.position,
    required this.distanceAlong,
    required this.headingRadians,
    required this.curvature,
    required this.targetSpeedMps,
    required this.lateralClearance,
  });

  /// Vehicle-frame position, metres.
  final Vec2 position;

  /// Arc length from the vehicle, metres.
  final double distanceAlong;

  /// Path heading, radians, positive to the right.
  final double headingRadians;

  /// Signed curvature, 1/m. Positive curves right.
  final double curvature;

  /// Speed the vehicle should be doing here, m/s.
  final double targetSpeedMps;

  /// Distance to the nearest obstacle edge at this point, metres.
  final double lateralClearance;

  Map<String, dynamic> toJson() => <String, dynamic>{
        'x': double.parse(position.x.toStringAsFixed(2)),
        'y': double.parse(position.y.toStringAsFixed(2)),
        'k': double.parse(curvature.toStringAsFixed(5)),
        'v': double.parse(targetSpeedMps.toStringAsFixed(2)),
      };
}

/// Where the path's geometry came from. Recorded because "we followed the
/// lane" and "we followed the car in front because we could not see the lane"
/// are very different claims about the same trajectory.
enum PathSource {
  laneCenterline('Lane centreline'),
  corridorCenterline('NO_LANE_MODE corridor'),
  obstacleAvoidance('Obstacle avoidance'),
  laneChangeSimulation('Simulated lane change'),
  previousPathHold('Holding previous path'),
  none('No path');

  const PathSource(this.label);
  final String label;
}

/// The trajectory the stack would follow if it were driving.
///
/// Purely a *proposal*: it is drawn on the HUD and fed to the simulated
/// controller, and it reaches no actuator anywhere. See [SafetyMode].
class PlannedPath {
  const PlannedPath({
    required this.points,
    required this.curve,
    required this.source,
    required this.confidence,
    required this.lateralOffsetFromReference,
    required this.maxRangeMeters,
    required this.frameId,
    required this.timestampMicros,
    this.isBlocked = false,
    this.blockedAtMeters,
    this.blockReason,
    this.corridorHalfWidth = 1.75,
  });

  factory PlannedPath.none({
    required int frameId,
    required int timestampMicros,
    String reason = 'no usable road model',
  }) =>
      PlannedPath(
        points: const <PathPoint>[],
        curve: const Polynomial.zero(),
        source: PathSource.none,
        confidence: 0,
        lateralOffsetFromReference: 0,
        maxRangeMeters: 0,
        frameId: frameId,
        timestampMicros: timestampMicros,
        isBlocked: true,
        blockedAtMeters: 0,
        blockReason: reason,
      );

  final List<PathPoint> points;

  /// `x = f(y)` in the vehicle frame, the form the steering controller wants.
  final Polynomial curve;

  final PathSource source;
  final double confidence;

  /// How far the chosen path sits from the lane/corridor centre, metres.
  /// Non-zero means the planner moved over for something.
  final double lateralOffsetFromReference;

  final double maxRangeMeters;
  final int frameId;
  final int timestampMicros;

  /// True when something stops the path short of its intended range.
  final bool isBlocked;
  final double? blockedAtMeters;
  final String? blockReason;

  /// Half-width of the corridor the path sits in, for drawing and for the
  /// collision predictor's swept-area test.
  final double corridorHalfWidth;

  bool get isEmpty => points.isEmpty;
  bool get isUsable => points.length >= 3 && confidence > 0.15;

  /// Lateral position at [distanceAhead], or `null` beyond the planned range.
  double? lateralAt(double distanceAhead) {
    if (distanceAhead < 0 || distanceAhead > maxRangeMeters + 1) return null;
    return curve.evaluate(distanceAhead);
  }

  double headingAt(double distanceAhead) =>
      math.atan(curve.derivative(distanceAhead));

  double curvatureAt(double distanceAhead) =>
      curve.curvatureAt(distanceAhead);

  /// Target speed at [distanceAhead], interpolated between path points.
  double targetSpeedAt(double distanceAhead) {
    if (points.isEmpty) return 0;
    if (distanceAhead <= points.first.distanceAlong) {
      return points.first.targetSpeedMps;
    }
    for (int i = 1; i < points.length; i++) {
      if (points[i].distanceAlong >= distanceAhead) {
        final PathPoint a = points[i - 1];
        final PathPoint b = points[i];
        final double span = b.distanceAlong - a.distanceAlong;
        final double t =
            span <= 0 ? 0 : (distanceAhead - a.distanceAlong) / span;
        return lerpDouble(a.targetSpeedMps, b.targetSpeedMps, t);
      }
    }
    return points.last.targetSpeedMps;
  }

  /// Speed the path as a whole allows: the minimum over its length.
  double get limitingSpeedMps {
    if (points.isEmpty) return 0;
    double minimum = double.infinity;
    for (final PathPoint p in points) {
      if (p.targetSpeedMps < minimum) minimum = p.targetSpeedMps;
    }
    return minimum.isFinite ? minimum : 0;
  }

  /// Project the path's centreline into image space for the HUD overlay.
  List<PixelPoint> toImagePolyline(CameraCalibration calibration) {
    final List<PixelPoint> out = <PixelPoint>[];
    for (final PathPoint p in points) {
      final PixelPoint? pixel = calibration.projectGroundToImage(p.position);
      if (pixel != null) out.add(pixel);
    }
    return out;
  }

  /// Left and right edges of the corridor in image space, for the band
  /// rendering the HUD uses.
  (List<PixelPoint>, List<PixelPoint>) toImageCorridor(
    CameraCalibration calibration,
  ) {
    final List<PixelPoint> left = <PixelPoint>[];
    final List<PixelPoint> right = <PixelPoint>[];
    for (final PathPoint p in points) {
      // Offset perpendicular to the local heading, not laterally, so the
      // corridor stays a constant width through a curve.
      final double heading = p.headingRadians;
      final Vec2 normal = Vec2(math.cos(heading), -math.sin(heading));
      final PixelPoint? l = calibration
          .projectGroundToImage(p.position - normal * corridorHalfWidth);
      final PixelPoint? r = calibration
          .projectGroundToImage(p.position + normal * corridorHalfWidth);
      if (l != null) left.add(l);
      if (r != null) right.add(r);
    }
    return (left, right);
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'source': source.name,
        'curve': curve.toJson(),
        'conf': double.parse(confidence.toStringAsFixed(3)),
        'offset':
            double.parse(lateralOffsetFromReference.toStringAsFixed(2)),
        'maxRange': double.parse(maxRangeMeters.toStringAsFixed(1)),
        'halfWidth': double.parse(corridorHalfWidth.toStringAsFixed(2)),
        if (isBlocked) 'blocked': blockReason,
        if (blockedAtMeters != null)
          'blockedAt': double.parse(blockedAtMeters!.toStringAsFixed(1)),
        'points': <Map<String, dynamic>>[
          for (final PathPoint p in points) p.toJson(),
        ],
      };

  /// Reconstruct a path from a recording.
  ///
  /// The per-point speed profile and clearances are re-derived from the
  /// stored points rather than recomputed, so a replayed path is exactly the
  /// path that was drawn at the time.
  static PlannedPath fromJson(
    Map<String, dynamic> j, {
    required int frameId,
    required int timestampMicros,
  }) {
    final Polynomial curve = Polynomial.fromJson(j['curve'] as List<dynamic>);
    final List<PathPoint> points = <PathPoint>[];
    double arcLength = 0;
    Vec2? previous;

    for (final dynamic raw in (j['points'] as List<dynamic>? ?? const <dynamic>[])) {
      final Map<String, dynamic> pt = raw as Map<String, dynamic>;
      final Vec2 position = Vec2(
        (pt['x'] as num).toDouble(),
        (pt['y'] as num).toDouble(),
      );
      if (previous != null) arcLength += position.distanceTo(previous);
      previous = position;
      points.add(PathPoint(
        position: position,
        distanceAlong: arcLength,
        headingRadians: math.atan(curve.derivative(position.y)),
        curvature: (pt['k'] as num?)?.toDouble() ?? 0,
        targetSpeedMps: (pt['v'] as num?)?.toDouble() ?? 0,
        lateralClearance: 0,
      ));
    }

    return PlannedPath(
      points: points,
      curve: curve,
      source: PathSource.values.firstWhere(
        (PathSource s) => s.name == j['source'],
        orElse: () => PathSource.none,
      ),
      confidence: (j['conf'] as num?)?.toDouble() ?? 0,
      lateralOffsetFromReference: (j['offset'] as num?)?.toDouble() ?? 0,
      maxRangeMeters: (j['maxRange'] as num?)?.toDouble() ?? 0,
      frameId: frameId,
      timestampMicros: timestampMicros,
      isBlocked: j['blocked'] != null,
      blockedAtMeters: (j['blockedAt'] as num?)?.toDouble(),
      blockReason: j['blocked'] as String?,
      corridorHalfWidth: (j['halfWidth'] as num?)?.toDouble() ?? 1.75,
    );
  }

  @override
  String toString() => 'PlannedPath(${source.label}, '
      '${maxRangeMeters.toStringAsFixed(0)}m, '
      '${(confidence * 100).round()}%'
      '${isBlocked ? ', BLOCKED: $blockReason' : ''})';
}
