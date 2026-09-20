import 'dart:math' as math;

import '../core/geometry.dart';
import '../navigation/maneuver.dart';
import '../perception/object_class.dart';
import '../tracking/object_track.dart';
import 'lane.dart';
import 'road_segmentation.dart';

/// Where a piece of corridor evidence came from. Recorded per-estimate so the
/// debug overlay (and a dataset consumer) can see *why* the stack thought the
/// road went where it did.
enum CorridorEvidence {
  drivableArea('Drivable area'),
  roadEdges('Road edges'),
  leadVehicles('Vehicle trajectories'),
  laneMemory('Recent lane geometry'),
  navigationIntent('Navigation intent');

  const CorridorEvidence(this.label);
  final String label;
}

/// A drivable corridor estimated without lane markings.
class CorridorEstimate {
  const CorridorEstimate({
    required this.centerline,
    required this.halfWidthMeters,
    required this.minRangeMeters,
    required this.maxRangeMeters,
    required this.confidence,
    required this.evidence,
  });

  /// `x = f(y)` in the vehicle frame.
  final Polynomial centerline;

  /// Half the usable corridor width at the vehicle.
  final double halfWidthMeters;

  final double minRangeMeters;
  final double maxRangeMeters;
  final double confidence;
  final Set<CorridorEvidence> evidence;

  double get rangeMeters => maxRangeMeters - minRangeMeters;

  double? lateralAt(double distanceAhead) {
    if (distanceAhead > maxRangeMeters + 2) return null;
    return centerline.evaluate(distanceAhead);
  }

  String get evidenceLabel =>
      evidence.map((CorridorEvidence e) => e.label).join(' + ');

  Map<String, dynamic> toJson() => <String, dynamic>{
        'centerline': centerline.toJson(),
        'halfWidth': double.parse(halfWidthMeters.toStringAsFixed(2)),
        'minRange': double.parse(minRangeMeters.toStringAsFixed(1)),
        'maxRange': double.parse(maxRangeMeters.toStringAsFixed(1)),
        'conf': double.parse(confidence.toStringAsFixed(3)),
        'evidence': evidence.map((CorridorEvidence e) => e.name).toList(),
      };
}

/// Builds a drivable corridor when lane markings are missing or unreliable.
///
/// The rule this class exists to enforce: **a corridor is only produced when
/// there is positive evidence for one.** Absence of lane markings is not, by
/// itself, permission to invent a path down the middle of the frame — that
/// would be the single most dangerous behaviour a system like this could have,
/// because it looks confident and is completely unfounded.
///
/// Evidence is combined as weighted centre-point observations along the road,
/// then fitted. Each source has a weight reflecting how directly it constrains
/// where the drivable surface is:
///  * segmentation corridor centre — direct, dense, medium accuracy;
///  * road edges — direct, sparse, high accuracy where present;
///  * the paths other vehicles are taking — indirect but extremely reliable,
///    because a car ahead is empirical proof that the surface is drivable;
///  * recent lane geometry — a short-lived memory for markings that have just
///    faded out;
///  * navigation intent — the weakest, and only a bias.
class NoLaneCorridorEstimator {
  const NoLaneCorridorEstimator({
    this.minEvidencePoints = 4,
    this.minConfidence = 0.22,
    this.maxRangeMeters = 50,
    this.defaultHalfWidthMeters = 1.75,
    this.laneMemorySeconds = 2.5,
  });

  final int minEvidencePoints;

  /// Below this the corridor is not reported at all. The decision engine then
  /// sees no path and goes to `UNCERTAIN`, which is the correct outcome.
  final double minConfidence;

  final double maxRangeMeters;
  final double defaultHalfWidthMeters;
  final double laneMemorySeconds;

  /// Returns `null` when the available evidence does not support a corridor.
  CorridorEstimate? estimate({
    required DrivableArea drivableArea,
    required List<RoadEdge> roadEdges,
    required List<ObjectTrack> tracks,
    required double egoSpeedMps,
    LaneDetectionResult? lastGoodLanes,
    double? lastGoodLanesAgeSeconds,
    ManeuverIntent intent = ManeuverIntent.unknown,
  }) {
    final List<_CentrePoint> points = <_CentrePoint>[];
    final Set<CorridorEvidence> evidence = <CorridorEvidence>{};

    _addDrivableAreaEvidence(drivableArea, points, evidence);
    _addRoadEdgeEvidence(roadEdges, points, evidence);
    _addLeadVehicleEvidence(tracks, egoSpeedMps, points, evidence);
    _addLaneMemoryEvidence(
        lastGoodLanes, lastGoodLanesAgeSeconds, points, evidence);

    if (points.length < minEvidencePoints) return null;

    // Navigation only nudges: a small lateral bias that grows with distance,
    // so it can bend the far end of the corridor towards an upcoming turn
    // without moving the near field where we actually are.
    if (intent.lateralBiasSign != 0) {
      bool biased = false;
      for (int i = 0; i < points.length; i++) {
        final _CentrePoint p = points[i];
        if (p.forward < 15) continue;
        final double bias = intent.lateralBiasSign *
            0.012 *
            (p.forward - 15) *
            (p.forward - 15) /
            10;
        points[i] = _CentrePoint(
          forward: p.forward,
          lateral: p.lateral + clampDouble(bias, -2.0, 2.0),
          weight: p.weight * 0.9,
          source: p.source,
        );
        biased = true;
      }
      if (biased) evidence.add(CorridorEvidence.navigationIntent);
    }

    points.sort((_CentrePoint a, _CentrePoint b) =>
        a.forward.compareTo(b.forward));

    final List<double> xs =
        points.map((_CentrePoint p) => p.forward).toList();
    final List<double> ys =
        points.map((_CentrePoint p) => p.lateral).toList();
    final List<double> ws = points.map((_CentrePoint p) => p.weight).toList();

    // Quadratic needs at least three distinct ranges; fall back to linear when
    // the evidence is clustered, rather than fabricating a curvature.
    final Set<int> distinctRanges =
        xs.map((double x) => (x / 5).round()).toSet();
    final int degree = distinctRanges.length >= 3 ? 2 : 1;

    Polynomial? fit =
        fitPolynomial(xs, ys, degree: degree, weights: ws);
    fit ??= fitPolynomial(xs, ys, degree: 1, weights: ws);
    if (fit == null) return null;

    // Reject geometry no road has.
    final double midRange = (xs.first + xs.last) / 2;
    if (fit.curvatureAt(midRange).abs() > 0.03) return null;
    if (fit.evaluate(math.max(xs.first, 5)).abs() > 5.0) return null;

    double residual = 0;
    double weightSum = 0;
    for (int i = 0; i < xs.length; i++) {
      residual += ws[i] * (fit.evaluate(xs[i]) - ys[i]).abs();
      weightSum += ws[i];
    }
    residual = weightSum <= 0 ? 999 : residual / weightSum;

    final double halfWidth = _estimateHalfWidth(drivableArea, roadEdges);
    final double confidence = _confidence(
      points: points,
      residual: residual,
      evidence: evidence,
      rangeMeters: xs.last - xs.first,
    );
    if (confidence < minConfidence) return null;

    return CorridorEstimate(
      centerline: fit,
      halfWidthMeters: halfWidth,
      minRangeMeters: xs.first,
      maxRangeMeters: math.min(xs.last, maxRangeMeters),
      confidence: confidence,
      evidence: evidence,
    );
  }

  void _addDrivableAreaEvidence(
    DrivableArea area,
    List<_CentrePoint> points,
    Set<CorridorEvidence> evidence,
  ) {
    if (area.isEmpty) return;
    for (final DrivableSample s in area.samples) {
      if (!s.isPlausible) continue;
      // A very wide corridor (a junction, a car park) says little about where
      // the *lane* is, so its centre is weighted down.
      final double widthPenalty =
          s.width > 8 ? clampDouble(8 / s.width, 0.3, 1) : 1.0;
      points.add(_CentrePoint(
        forward: s.distanceAhead,
        lateral: s.center,
        weight: s.confidence * widthPenalty * 1.0,
        source: CorridorEvidence.drivableArea,
      ));
      evidence.add(CorridorEvidence.drivableArea);
    }
  }

  void _addRoadEdgeEvidence(
    List<RoadEdge> edges,
    List<_CentrePoint> points,
    Set<CorridorEvidence> evidence,
  ) {
    if (edges.isEmpty) return;
    final RoadEdge? left = edges.cast<RoadEdge?>().firstWhere(
        (RoadEdge? e) => e!.isLeft,
        orElse: () => null);
    final RoadEdge? right = edges.cast<RoadEdge?>().firstWhere(
        (RoadEdge? e) => !e!.isLeft,
        orElse: () => null);

    for (double d = 6; d <= maxRangeMeters; d += 4) {
      final double? l = left?.lateralAt(d);
      final double? r = right?.lateralAt(d);
      double? centre;
      double weight = 0;

      if (l != null && r != null) {
        centre = (l + r) / 2;
        weight = 1.6 * math.min(left!.confidence, right!.confidence);
      } else if (l != null) {
        // One edge plus the assumed half-width. Weaker, and marked as such.
        centre = l + defaultHalfWidthMeters;
        weight = 0.55 * left!.confidence;
      } else if (r != null) {
        centre = r - defaultHalfWidthMeters;
        weight = 0.55 * right!.confidence;
      }

      if (centre == null || weight <= 0.05) continue;
      points.add(_CentrePoint(
        forward: d,
        lateral: centre,
        weight: weight,
        source: CorridorEvidence.roadEdges,
      ));
      evidence.add(CorridorEvidence.roadEdges);
    }
  }

  /// A vehicle travelling ahead of us in the same direction is empirical proof
  /// that the surface it is on is drivable, and its recent track is a sampled
  /// version of the road's shape. This is the strongest cue available on a
  /// completely unmarked road.
  void _addLeadVehicleEvidence(
    List<ObjectTrack> tracks,
    double egoSpeedMps,
    List<_CentrePoint> points,
    Set<CorridorEvidence> evidence,
  ) {
    for (final ObjectTrack t in tracks) {
      if (!t.isConfirmed) continue;
      if (!t.objectClass.isVehicle) continue;
      if (t.objectClass == ObjectClass.motorcycle) continue; // filters between lanes
      if (t.position.y < 6 || t.position.y > maxRangeMeters) continue;
      if (t.position.x.abs() > 6) continue;

      // Must be going the same way as us, not oncoming.
      final bool sameDirection = t.velocityWorld.y > 0.5 ||
          (egoSpeedMps < 1.0 && t.velocityWorld.length < 1.0);
      if (!sameDirection) continue;

      final double weight =
          1.3 * t.confidence.value * clampDouble(t.distanceConfidence.value, 0.2, 1);
      points.add(_CentrePoint(
        forward: t.position.y,
        lateral: t.position.x,
        weight: weight,
        source: CorridorEvidence.leadVehicles,
      ));

      // The track's own history traces the road behind that vehicle.
      final List<TrackObservation> history = t.history.toList();
      for (int i = math.max(0, history.length - 12);
          i < history.length;
          i += 4) {
        final TrackObservation o = history[i];
        if (o.position.y < 6 || o.position.y > maxRangeMeters) continue;
        points.add(_CentrePoint(
          forward: o.position.y,
          lateral: o.position.x,
          weight: weight * 0.4,
          source: CorridorEvidence.leadVehicles,
        ));
      }
      evidence.add(CorridorEvidence.leadVehicles);
    }
  }

  /// Markings that have just faded out still constrain where the lane is for
  /// a second or two — but only for a second or two, and with a weight that
  /// decays to nothing.
  void _addLaneMemoryEvidence(
    LaneDetectionResult? lastGood,
    double? ageSeconds,
    List<_CentrePoint> points,
    Set<CorridorEvidence> evidence,
  ) {
    if (lastGood == null || ageSeconds == null) return;
    if (ageSeconds > laneMemorySeconds) return;
    final Polynomial? centre = lastGood.centerline;
    if (centre == null) return;

    final double decay = 1.0 - ageSeconds / laneMemorySeconds;
    final double weight = 0.7 * decay * lastGood.overallConfidence;
    if (weight < 0.05) return;

    for (double d = 6; d <= math.min(lastGood.usableRangeMeters, 30); d += 5) {
      points.add(_CentrePoint(
        forward: d,
        lateral: centre.evaluate(d),
        weight: weight,
        source: CorridorEvidence.laneMemory,
      ));
    }
    evidence.add(CorridorEvidence.laneMemory);
  }

  double _estimateHalfWidth(DrivableArea area, List<RoadEdge> edges) {
    final List<double> widths = <double>[];
    for (final DrivableSample s in area.samples) {
      if (s.distanceAhead > 25) break;
      if (s.isPlausible) widths.add(s.width);
    }
    if (widths.isEmpty) return defaultHalfWidthMeters;
    widths.sort();
    final double median = widths[widths.length ~/ 2];
    // A corridor is not a lane: cap the half-width at something a vehicle
    // would actually use, so the planner does not treat a whole junction as
    // free space.
    return clampDouble(median / 2, 1.4, 2.6);
  }

  double _confidence({
    required List<_CentrePoint> points,
    required double residual,
    required Set<CorridorEvidence> evidence,
    required double rangeMeters,
  }) {
    final double residualScore = clampDouble(1 - residual / 1.2, 0, 1);
    final double rangeScore = clampDouble(rangeMeters / 30, 0, 1);
    final double densityScore =
        clampDouble(points.length / (minEvidencePoints * 3), 0, 1);

    // Independent evidence sources are worth much more than more samples from
    // one source: segmentation agreeing with a lead vehicle's path is strong,
    // 40 segmentation rows agreeing with each other is not.
    final double diversityScore =
        clampDouble((evidence.length - 1) / 2.0, 0, 1);

    double c = 0.30 * residualScore +
        0.22 * rangeScore +
        0.18 * densityScore +
        0.30 * diversityScore;

    // NO_LANE_MODE is inherently less certain than a marked lane, and the
    // number reported downstream must say so.
    return clampDouble(c * 0.85, 0, 1);
  }
}

class _CentrePoint {
  const _CentrePoint({
    required this.forward,
    required this.lateral,
    required this.weight,
    required this.source,
  });

  final double forward;
  final double lateral;
  final double weight;
  final CorridorEvidence source;
}
