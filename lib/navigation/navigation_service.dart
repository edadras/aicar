import 'dart:async';

import '../core/logging.dart';
import '../sensors/ego_motion.dart';
import 'maneuver.dart';
import 'route.dart';
import 'route_provider.dart';

/// Owns the active route and keeps it matched against the live position.
///
/// The contract with the rest of the stack is narrow on purpose: the only
/// thing that reaches the planner is a [ManeuverIntent] and a distance. There
/// is no code path by which navigation can produce a steering angle, and the
/// whole driving stack runs unchanged when navigation is unavailable.
class NavigationService {
  NavigationService({
    RouteProvider? provider,
    this.matcher = const RouteMatcher(),
    this.rerouteCooldown = const Duration(seconds: 20),
  }) : provider = provider ??
            FallbackRouteProvider(<RouteProvider>[
              OsrmRouteProvider(),
              const DirectLineRouteProvider(),
            ]);

  static const String _tag = 'Navigation';

  /// Swappable at runtime so Settings can force the offline provider.
  RouteProvider provider;
  final RouteMatcher matcher;
  final Duration rerouteCooldown;

  final StreamController<RouteProgress?> _progressController =
      StreamController<RouteProgress?>.broadcast();

  NavigationRoute? _route;
  RouteProgress? _progress;
  GeoPosition? _destination;
  String? _destinationName;
  int _consecutiveOffRouteFixes = 0;
  DateTime? _lastRerouteAt;
  bool _routing = false;
  String? _lastError;

  NavigationRoute? get route => _route;
  RouteProgress? get progress => _progress;
  GeoPosition? get destination => _destination;
  String? get destinationName => _destinationName;
  bool get hasRoute => _route != null;
  bool get isRouting => _routing;
  String? get lastError => _lastError;

  Stream<RouteProgress?> get progressStream => _progressController.stream;

  /// Current intent, or [ManeuverIntent.unknown] when there is no usable
  /// route. The planner treats unknown as "no navigation input", not as an
  /// instruction to go straight.
  ManeuverIntent get currentIntent =>
      _progress?.intent ?? ManeuverIntent.unknown;

  double? get distanceToManeuverMeters =>
      _progress?.distanceToManeuverMeters;

  /// Compute a route to [destination] from [origin].
  Future<bool> setDestination({
    required GeoPosition origin,
    required GeoPosition destination,
    String? name,
  }) async {
    _destination = destination;
    _destinationName = name;
    return _computeRoute(origin);
  }

  Future<bool> _computeRoute(GeoPosition origin) async {
    final GeoPosition? dest = _destination;
    if (dest == null) return false;
    if (_routing) return false;

    _routing = true;
    _lastError = null;
    try {
      final NavigationRoute? r = await provider.route(
        origin: origin,
        destination: dest,
        destinationName: _destinationName,
      );
      if (r == null) {
        _lastError = 'No route found (offline, or no road connection)';
        Log.warn(_tag, _lastError!);
        return false;
      }
      _route = r;
      _consecutiveOffRouteFixes = 0;
      _lastRerouteAt = DateTime.now();
      Log.info(_tag, 'route set: $r');
      return true;
    } finally {
      _routing = false;
    }
  }

  /// Feed a new position; returns the updated progress.
  ///
  /// Reroutes automatically when the match has been bad for several fixes in
  /// a row, subject to a cooldown so a tunnel does not trigger a reroute storm.
  Future<RouteProgress?> updatePosition(GeoPosition position) async {
    final NavigationRoute? r = _route;
    if (r == null) return null;

    final RouteProgress? p = matcher.match(
      route: r,
      position: position,
      consecutiveOffRouteFixes: _consecutiveOffRouteFixes,
    );

    if (p == null) {
      _progress = null;
      _emit(null);
      return null;
    }

    if (p.matchQuality < 0.15) {
      _consecutiveOffRouteFixes++;
    } else {
      _consecutiveOffRouteFixes = 0;
    }

    _progress = p;
    _emit(p);

    if (p.isOffRoute && _canReroute()) {
      Log.info(_tag, 'off route — recomputing');
      await _computeRoute(position);
    }

    return p;
  }

  bool _canReroute() {
    if (_routing) return false;
    final DateTime? last = _lastRerouteAt;
    if (last == null) return true;
    return DateTime.now().difference(last) > rerouteCooldown;
  }

  void _emit(RouteProgress? p) {
    if (!_progressController.isClosed) _progressController.add(p);
  }

  void clearRoute() {
    _route = null;
    _progress = null;
    _destination = null;
    _destinationName = null;
    _consecutiveOffRouteFixes = 0;
    _emit(null);
    Log.info(_tag, 'route cleared');
  }

  Future<void> dispose() async {
    await _progressController.close();
  }
}
