import 'dart:math' as math;

import '../core/geometry.dart';
import '../sensors/ego_motion.dart';
import 'maneuver.dart';

/// One leg of a route, ending in a manoeuvre.
class RouteStep {
  const RouteStep({
    required this.maneuver,
    required this.polyline,
    required this.distanceMeters,
    required this.durationSeconds,
    this.roadName,
    this.speedLimitKph,
    this.exitNumber,
  });

  final ManeuverType maneuver;

  /// Geographic shape of this leg, ordered from its start to its end.
  final List<GeoPosition> polyline;

  final double distanceMeters;
  final double durationSeconds;
  final String? roadName;

  /// Posted limit from the map data, when available. Treated as a *prior*
  /// only — a sign observed on the road always wins, because map data is
  /// routinely out of date and road works change limits daily.
  final int? speedLimitKph;

  final String? exitNumber;

  /// The point at which the manoeuvre happens: the end of this leg.
  GeoPosition? get maneuverPoint =>
      polyline.isEmpty ? null : polyline.last;

  Map<String, dynamic> toJson() => <String, dynamic>{
        'maneuver': maneuver.name,
        'distance': double.parse(distanceMeters.toStringAsFixed(1)),
        'duration': double.parse(durationSeconds.toStringAsFixed(1)),
        if (roadName != null) 'road': roadName,
        if (speedLimitKph != null) 'speedLimit': speedLimitKph,
        if (exitNumber != null) 'exit': exitNumber,
        'polyline': <List<double>>[
          for (final GeoPosition p in polyline)
            <double>[
              double.parse(p.latitude.toStringAsFixed(6)),
              double.parse(p.longitude.toStringAsFixed(6)),
            ],
        ],
      };

  @override
  String toString() =>
      '${maneuver.label}${roadName == null ? '' : ' onto $roadName'} '
      'in ${distanceMeters.round()} m';
}

/// A computed route from the current position to a destination.
class NavigationRoute {
  const NavigationRoute({
    required this.origin,
    required this.destination,
    required this.steps,
    required this.totalDistanceMeters,
    required this.totalDurationSeconds,
    required this.computedAt,
    this.source = 'unknown',
    this.destinationName,
  });

  final GeoPosition origin;
  final GeoPosition destination;
  final List<RouteStep> steps;
  final double totalDistanceMeters;
  final double totalDurationSeconds;
  final DateTime computedAt;
  final String source;
  final String? destinationName;

  bool get isEmpty => steps.isEmpty;

  /// Whole-route polyline, for drawing on the map.
  List<GeoPosition> get polyline => <GeoPosition>[
        for (final RouteStep s in steps) ...s.polyline,
      ];

  Map<String, dynamic> toJson() => <String, dynamic>{
        'origin': origin.toJson(),
        'destination': destination.toJson(),
        if (destinationName != null) 'destinationName': destinationName,
        'totalDistance':
            double.parse(totalDistanceMeters.toStringAsFixed(1)),
        'totalDuration':
            double.parse(totalDurationSeconds.toStringAsFixed(1)),
        'computedAt': computedAt.toIso8601String(),
        'source': source,
        'steps': <Map<String, dynamic>>[
          for (final RouteStep s in steps) s.toJson(),
        ],
      };

  @override
  String toString() =>
      'Route(${(totalDistanceMeters / 1000).toStringAsFixed(1)} km, '
      '${steps.length} steps, via $source)';
}

/// Where we are on the route right now, and what is coming next.
///
/// This is the object the driving stack consumes. Note what it does *not*
/// contain: any geometry expressed in the vehicle frame, any steering angle,
/// any path. Navigation's entire contribution to control is
/// [nextManeuverIntent] plus [distanceToManeuverMeters].
class RouteProgress {
  const RouteProgress({
    required this.route,
    required this.currentStepIndex,
    required this.distanceToManeuverMeters,
    required this.distanceRemainingMeters,
    required this.durationRemainingSeconds,
    required this.nextManeuver,
    required this.roadBearingDegrees,
    required this.isOffRoute,
    required this.matchQuality,
    this.currentRoadName,
    this.mapSpeedLimitKph,
  });

  final NavigationRoute route;
  final int currentStepIndex;

  /// Metres to the point where the next manoeuvre happens.
  final double distanceToManeuverMeters;

  final double distanceRemainingMeters;
  final double durationRemainingSeconds;
  final ManeuverType nextManeuver;

  /// Bearing of the road we are on, degrees from true north. Used as a
  /// cross-check on the fused heading, not as a source of steering.
  final double roadBearingDegrees;

  final bool isOffRoute;

  /// How well the GNSS position matched the route polyline, 0..1. Low quality
  /// in an urban canyon means the intent should carry less weight.
  final double matchQuality;

  final String? currentRoadName;
  final int? mapSpeedLimitKph;

  /// The reduced intent handed to the planner.
  ///
  /// Only becomes a turn intent once the manoeuvre is close enough to matter.
  /// Announcing TURN_LEFT two kilometres early would bias the corridor
  /// estimate down a road that is not there yet.
  ManeuverIntent get intent {
    if (isOffRoute || matchQuality < 0.3) return ManeuverIntent.unknown;
    if (distanceToManeuverMeters > _intentHorizonFor(nextManeuver)) {
      return ManeuverIntent.straight;
    }
    return nextManeuver.intent;
  }

  static double _intentHorizonFor(ManeuverType m) => switch (m) {
        ManeuverType.offRamp || ManeuverType.onRamp => 400,
        ManeuverType.merge || ManeuverType.fork => 300,
        ManeuverType.keepLeft || ManeuverType.keepRight => 250,
        ManeuverType.roundaboutEnter => 120,
        _ => 90,
      };

  String get displayInstruction {
    final String distance = distanceToManeuverMeters >= 1000
        ? '${(distanceToManeuverMeters / 1000).toStringAsFixed(1)} km'
        : '${distanceToManeuverMeters.round()} m';
    return '${nextManeuver.label} in $distance';
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'step': currentStepIndex,
        'toManeuver':
            double.parse(distanceToManeuverMeters.toStringAsFixed(1)),
        'remaining':
            double.parse(distanceRemainingMeters.toStringAsFixed(1)),
        'etaSeconds':
            double.parse(durationRemainingSeconds.toStringAsFixed(1)),
        'maneuver': nextManeuver.name,
        'intent': intent.name,
        'roadBearing': double.parse(roadBearingDegrees.toStringAsFixed(1)),
        'offRoute': isOffRoute,
        'matchQuality': double.parse(matchQuality.toStringAsFixed(3)),
        if (currentRoadName != null) 'road': currentRoadName,
        if (mapSpeedLimitKph != null) 'mapSpeedLimit': mapSpeedLimitKph,
      };

  @override
  String toString() => '$displayInstruction '
      '(${(distanceRemainingMeters / 1000).toStringAsFixed(1)} km left'
      '${isOffRoute ? ', OFF ROUTE' : ''})';
}

/// Projects a GNSS fix onto a route polyline.
///
/// Snapping matters more than it looks: a raw fix can sit 15 m off the road in
/// a city, and using it directly would report the wrong step, the wrong
/// distance-to-turn and eventually a spurious off-route. The match quality
/// this returns is what stops a bad snap being trusted.
class RouteMatcher {
  const RouteMatcher({
    this.offRouteThresholdMeters = 45,
    this.offRouteConfirmations = 3,
  });

  final double offRouteThresholdMeters;

  /// Consecutive bad fixes before declaring off-route. A single 60 m outlier
  /// under a bridge is not a wrong turn.
  final int offRouteConfirmations;

  RouteProgress? match({
    required NavigationRoute route,
    required GeoPosition position,
    required int consecutiveOffRouteFixes,
  }) {
    if (route.steps.isEmpty) return null;

    _Snap? best;
    for (int stepIndex = 0; stepIndex < route.steps.length; stepIndex++) {
      final RouteStep step = route.steps[stepIndex];
      for (int i = 0; i + 1 < step.polyline.length; i++) {
        final _Snap snap = _snapToSegment(
          position,
          step.polyline[i],
          step.polyline[i + 1],
          stepIndex,
          i,
        );
        if (best == null || snap.distanceMeters < best.distanceMeters) {
          best = snap;
        }
      }
    }
    if (best == null) return null;

    final RouteStep step = route.steps[best.stepIndex];

    // Distance along the remainder of this step, then the manoeuvre.
    double toManeuver = best.distanceToSegmentEnd;
    for (int i = best.segmentIndex + 1; i + 1 < step.polyline.length; i++) {
      toManeuver += step.polyline[i].distanceTo(step.polyline[i + 1]);
    }

    double remaining = toManeuver;
    for (int s = best.stepIndex + 1; s < route.steps.length; s++) {
      remaining += route.steps[s].distanceMeters;
    }

    // Proportional ETA: better than nothing and does not pretend to model
    // traffic we cannot see.
    final double durationRemaining = route.totalDistanceMeters <= 0
        ? 0
        : route.totalDurationSeconds *
            (remaining / route.totalDistanceMeters);

    final bool offRoute = best.distanceMeters > offRouteThresholdMeters &&
        consecutiveOffRouteFixes >= offRouteConfirmations;

    // Match quality combines how close the snap was with how good the fix was.
    final double proximity = clampDouble(
      1 - best.distanceMeters / offRouteThresholdMeters,
      0,
      1,
    );
    final double matchQuality =
        clampDouble(proximity * position.confidence.value, 0, 1);

    final ManeuverType next = best.stepIndex + 1 < route.steps.length
        ? route.steps[best.stepIndex + 1].maneuver
        : ManeuverType.arrive;

    return RouteProgress(
      route: route,
      currentStepIndex: best.stepIndex,
      distanceToManeuverMeters: toManeuver,
      distanceRemainingMeters: remaining,
      durationRemainingSeconds: durationRemaining,
      nextManeuver: next,
      roadBearingDegrees: best.bearingDegrees,
      isOffRoute: offRoute,
      matchQuality: matchQuality,
      currentRoadName: step.roadName,
      mapSpeedLimitKph: step.speedLimitKph,
    );
  }

  /// Perpendicular projection of [p] onto the segment `a-b`.
  _Snap _snapToSegment(
    GeoPosition p,
    GeoPosition a,
    GeoPosition b,
    int stepIndex,
    int segmentIndex,
  ) {
    // Work in a local east-north frame: over a segment of a few hundred
    // metres this is accurate to centimetres and avoids spherical trig in a
    // loop that runs over every segment of the route.
    final Vec2 ao = a.localOffsetFrom(a);
    final Vec2 bo = b.localOffsetFrom(a);
    final Vec2 po = p.localOffsetFrom(a);

    final Vec2 ab = bo - ao;
    final double lengthSquared = ab.lengthSquared;
    double t = lengthSquared < 1e-9 ? 0 : (po - ao).dot(ab) / lengthSquared;
    t = clampDouble(t, 0, 1);

    final Vec2 closest = ao + ab * t;
    final double distance = (po - closest).length;
    final double segmentLength = a.distanceTo(b);

    return _Snap(
      stepIndex: stepIndex,
      segmentIndex: segmentIndex,
      distanceMeters: distance,
      distanceToSegmentEnd: segmentLength * (1 - t),
      bearingDegrees: a.bearingTo(b),
    );
  }
}

class _Snap {
  const _Snap({
    required this.stepIndex,
    required this.segmentIndex,
    required this.distanceMeters,
    required this.distanceToSegmentEnd,
    required this.bearingDegrees,
  });

  final int stepIndex;
  final int segmentIndex;
  final double distanceMeters;
  final double distanceToSegmentEnd;
  final double bearingDegrees;
}

/// Decode an encoded polyline (the format OSRM and most routing APIs use).
List<GeoPosition> decodePolyline(String encoded, {int precision = 5}) {
  final List<GeoPosition> points = <GeoPosition>[];
  final double factor = math.pow(10, precision).toDouble();
  int index = 0;
  int lat = 0;
  int lng = 0;

  while (index < encoded.length) {
    int shift = 0;
    int result = 0;
    int byte;
    do {
      if (index >= encoded.length) return points;
      byte = encoded.codeUnitAt(index++) - 63;
      result |= (byte & 0x1f) << shift;
      shift += 5;
    } while (byte >= 0x20);
    lat += (result & 1) != 0 ? ~(result >> 1) : (result >> 1);

    shift = 0;
    result = 0;
    do {
      if (index >= encoded.length) return points;
      byte = encoded.codeUnitAt(index++) - 63;
      result |= (byte & 0x1f) << shift;
      shift += 5;
    } while (byte >= 0x20);
    lng += (result & 1) != 0 ? ~(result >> 1) : (result >> 1);

    points.add(GeoPosition(
      latitude: lat / factor,
      longitude: lng / factor,
      accuracyMeters: 0,
      timestampMicros: 0,
    ));
  }
  return points;
}
