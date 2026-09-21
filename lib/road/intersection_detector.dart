import 'dart:math' as math;

import '../core/confidence.dart';
import '../core/geometry.dart';
import '../perception/traffic_light.dart';
import '../perception/traffic_sign.dart';
import '../tracking/object_track.dart';
import 'lane.dart';
import 'road_marking.dart';
import 'road_segmentation.dart';

/// What governs the junction ahead, as far as we can tell.
enum IntersectionControl {
  /// A traffic light we have resolved as ours.
  signalised('SIGNALLED'),

  /// A stop sign applies.
  stopSign('STOP'),

  /// A give-way sign or a roundabout applies.
  giveWay('GIVE WAY'),

  /// A junction with no control we can see. This is the dangerous one: it
  /// does not mean we have priority, it means we do not know who does.
  uncontrolled('UNCONTROLLED');

  const IntersectionControl(this.label);
  final String label;

  /// Whether we must be prepared to give way to traffic already there.
  bool get requiresYield => this != IntersectionControl.signalised;
}

/// A junction inferred ahead of the vehicle.
///
/// This is an *inference*, never an observation: no single cue proves a
/// junction, and [evidence] lists exactly which cues contributed so a human
/// reviewing a recording can see what the stack was reasoning from.
class IntersectionEstimate {
  const IntersectionEstimate({
    required this.distanceMeters,
    required this.confidence,
    required this.control,
    required this.evidence,
    this.crossingTrafficTrackIds = const <int>[],
  });

  final double distanceMeters;
  final Confidence confidence;
  final IntersectionControl control;

  /// Human-readable cues, in the order they were weighed.
  final List<String> evidence;

  /// Tracks moving across our path near the junction mouth.
  final List<int> crossingTrafficTrackIds;

  bool get hasCrossingTraffic => crossingTrafficTrackIds.isNotEmpty;

  /// Speed to approach at, m/s.
  ///
  /// A green light does not mean full speed: it means the *signal* is not
  /// what limits us. Crossing traffic still does, which is why traffic
  /// tightens the figure regardless of the control.
  double approachSpeedMps({required double postedLimitMps}) {
    double cap = switch (control) {
      IntersectionControl.signalised => postedLimitMps,
      IntersectionControl.giveWay => 5.5,
      IntersectionControl.stopSign => 3.0,
      IntersectionControl.uncontrolled => 7.0,
    };
    if (hasCrossingTraffic) cap = math.min(cap, 4.5);
    return cap;
  }

  /// Distance at which the approach should already have begun, given a
  /// comfortable deceleration from [speedMps] down to the approach speed.
  double cautionOnsetMeters({
    required double speedMps,
    required double postedLimitMps,
  }) {
    final double target = approachSpeedMps(postedLimitMps: postedLimitMps);
    if (speedMps <= target) return 12;
    const double comfortDecel = 1.8;
    final double braking =
        (speedMps * speedMps - target * target) / (2 * comfortDecel);
    return clampDouble(braking + 8, 12, 90);
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'dist': double.parse(distanceMeters.toStringAsFixed(1)),
        'conf': double.parse(confidence.value.toStringAsFixed(3)),
        'control': control.name,
        'evidence': evidence,
        if (crossingTrafficTrackIds.isNotEmpty)
          'crossing': crossingTrafficTrackIds,
      };

  static IntersectionEstimate fromJson(Map<String, dynamic> j) =>
      IntersectionEstimate(
        distanceMeters: (j['dist'] as num?)?.toDouble() ?? 0,
        confidence: Confidence((j['conf'] as num?)?.toDouble() ?? 0,
            source: 'recorded'),
        control: IntersectionControl.values.firstWhere(
          (IntersectionControl c) => c.name == j['control'],
          orElse: () => IntersectionControl.uncontrolled,
        ),
        evidence: <String>[
          for (final dynamic e
              in (j['evidence'] as List<dynamic>? ?? const <dynamic>[]))
            '$e',
        ],
        crossingTrafficTrackIds: <int>[
          for (final dynamic t
              in (j['crossing'] as List<dynamic>? ?? const <dynamic>[]))
            (t as num).toInt(),
        ],
      );

  @override
  String toString() => 'Intersection(${control.label} at '
      '${distanceMeters.toStringAsFixed(0)} m, ${confidence.percent}%)';
}

/// Infers a junction ahead from cues that are individually weak.
///
/// There is no "intersection" class in any detector, and there does not need
/// to be: a junction announces itself in several ways at once, and the
/// combination is far more reliable than any one of them. A stop line, a
/// crossing, lane markings that simply stop while the asphalt continues, a
/// road that widens, a signal head, a give-way sign, cars crossing our path —
/// each is ordinary on its own and unmistakable together.
///
/// The cues are combined as a noisy-OR, which has the property this needs:
/// several independent weak cues reinforce each other, but no amount of one
/// cue reaches certainty.
class IntersectionDetector {
  const IntersectionDetector({
    this.minConfidence = 0.35,
    this.maxDistanceMeters = 70,
    this.crossingSpeedMps = 1.5,
  });

  /// Below this the inference is not reported at all. A junction we are 40 %
  /// sure of, announced every few seconds on a straight road, would train the
  /// driver to ignore the panel.
  final double minConfidence;

  final double maxDistanceMeters;

  /// Lateral speed above which a track counts as crossing rather than drifting.
  final double crossingSpeedMps;

  IntersectionEstimate? detect({
    required LaneDetectionResult lanes,
    required DrivableArea drivableArea,
    required List<RoadMarking> markings,
    required List<TrafficLight> lights,
    required RegulatoryContext regulatory,
    required List<ObjectTrack> tracks,
    required double egoSpeedMps,
  }) {
    final List<_Cue> cues = <_Cue>[];

    // --- Paint on the road ------------------------------------------------
    final RoadMarking? stopLine = _nearest(markings, RoadMarkingType.stopLine);
    if (stopLine != null) {
      cues.add(_Cue(
        weight: 0.55 * stopLine.confidence.value,
        distance: stopLine.distanceMeters,
        text: 'stop line at ${stopLine.distanceMeters.toStringAsFixed(0)} m',
      ));
    }

    final RoadMarking? crossing =
        _nearest(markings, RoadMarkingType.crosswalk);
    if (crossing != null) {
      cues.add(_Cue(
        weight: 0.40 * crossing.confidence.value,
        distance: crossing.distanceMeters,
        text: 'crosswalk at ${crossing.distanceMeters.toStringAsFixed(0)} m',
      ));
    }

    // --- Signals and signs ------------------------------------------------
    TrafficLight? governing;
    for (final TrafficLight l in lights) {
      if (!l.isActionable) continue;
      if (governing == null ||
          (l.distanceMeters ?? 1e9) < (governing.distanceMeters ?? 1e9)) {
        governing = l;
      }
    }
    if (governing != null) {
      cues.add(_Cue(
        weight: 0.70 * governing.relevanceConfidence.value,
        distance: governing.distanceMeters ?? 30,
        text: 'signal head ahead (${governing.color.label})',
      ));
    }

    if (regulatory.pendingStop) {
      cues.add(const _Cue(
        weight: 0.65,
        distance: 25,
        text: 'stop sign in force',
      ));
    } else if (regulatory.pendingGiveWay) {
      cues.add(const _Cue(
        weight: 0.60,
        distance: 25,
        text: 'give-way sign in force',
      ));
    }

    // --- Road geometry ----------------------------------------------------
    //
    // Lane markings stop at a junction mouth while the asphalt carries on.
    // That gap between "how far the lanes go" and "how far the road goes" is
    // one of the most reliable junction cues there is, and it costs nothing:
    // both numbers are already computed.
    //
    // The test is deliberately proportional rather than a fixed gap. Lane
    // fits routinely run a little shorter than the segmented surface on an
    // ordinary road — the marking response just gets too weak to fit — and a
    // fixed "8 m shorter" rule would call that a junction on every straight.
    // Markings stopping *dead* while the asphalt runs on twice as far is a
    // different thing, and that is what a junction mouth looks like.
    final double laneRange = lanes.usableRangeMeters;
    final double roadRange = drivableArea.maxRangeMeters;
    if (laneRange > 6 &&
        roadRange > laneRange * 1.6 &&
        roadRange - laneRange > 12 &&
        lanes.overallConfidence > 0.45) {
      cues.add(_Cue(
        weight: 0.35,
        distance: laneRange,
        text: 'lane markings end at ${laneRange.toStringAsFixed(0)} m '
            'while the road continues to ${roadRange.toStringAsFixed(0)} m',
      ));
    }

    final double? widening = _wideningDistance(drivableArea);
    if (widening != null) {
      cues.add(_Cue(
        weight: 0.30,
        distance: widening,
        text: 'road opens out at ${widening.toStringAsFixed(0)} m',
      ));
    }

    if (cues.isEmpty) return null;

    // --- Combine ----------------------------------------------------------
    double miss = 1;
    for (final _Cue c in cues) {
      miss *= 1 - clampDouble(c.weight, 0, 0.92);
    }
    final double combined = 1 - miss;
    if (combined < minConfidence) return null;

    // The junction is where the *nearest* cue puts it: braking for the far
    // edge of a junction we are about to enter would be too late.
    double distance = maxDistanceMeters;
    for (final _Cue c in cues) {
      distance = math.min(distance, c.distance);
    }
    if (distance > maxDistanceMeters) return null;

    // --- Crossing traffic -------------------------------------------------
    final List<int> crossers = <int>[];
    for (final ObjectTrack t in tracks) {
      if (!t.objectClass.isVehicle && !t.objectClass.isVulnerable) continue;
      if (t.position.y <= 0 || t.position.y > distance + 18) continue;
      if (t.velocityWorld.x.abs() < crossingSpeedMps) continue;
      if (!t.direction.isLateral &&
          t.direction != MotionDirection.enteringPath) {
        continue;
      }
      crossers.add(t.id);
    }

    final IntersectionControl control;
    if (governing != null && governing.relevanceConfidence.value > 0.5) {
      control = IntersectionControl.signalised;
    } else if (regulatory.pendingStop) {
      control = IntersectionControl.stopSign;
    } else if (regulatory.pendingGiveWay) {
      control = IntersectionControl.giveWay;
    } else {
      control = IntersectionControl.uncontrolled;
    }

    final List<String> evidence = <String>[
      for (final _Cue c in (cues.toList()
        ..sort((_Cue a, _Cue b) => b.weight.compareTo(a.weight))))
        c.text,
      if (crossers.isNotEmpty)
        '${crossers.length} vehicle(s) crossing our path',
    ];

    return IntersectionEstimate(
      distanceMeters: distance,
      confidence: Confidence(combined, source: 'junction-cues'),
      control: control,
      evidence: evidence,
      crossingTrafficTrackIds: crossers,
    );
  }

  RoadMarking? _nearest(List<RoadMarking> markings, RoadMarkingType type) {
    RoadMarking? best;
    for (final RoadMarking m in markings) {
      if (m.type != type) continue;
      if (best == null || m.distanceMeters < best.distanceMeters) best = m;
    }
    return best;
  }

  /// Distance at which the drivable corridor first widens sharply.
  ///
  /// Side roads joining make the segmented road area flare outwards well
  /// before the junction itself is visible as anything else.
  double? _wideningDistance(DrivableArea area) {
    if (area.samples.length < 4) return null;
    final double? near = area.widthAt(area.samples.first.distanceAhead + 2);
    if (near == null || near <= 0) return null;

    for (final DrivableSample s in area.samples) {
      if (s.distanceAhead < 8) continue;
      final double width = s.rightEdge - s.leftEdge;
      if (width > near * 1.5 && width - near > 3.0) {
        return s.distanceAhead;
      }
    }
    return null;
  }
}

class _Cue {
  const _Cue({
    required this.weight,
    required this.distance,
    required this.text,
  });

  final double weight;
  final double distance;
  final String text;
}
