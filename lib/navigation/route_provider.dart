import 'dart:async';
import 'dart:convert';

import 'package:http/http.dart' as http;

import '../core/logging.dart';
import '../sensors/ego_motion.dart';
import 'maneuver.dart';
import 'route.dart';

/// Computes routes. Online and offline implementations both satisfy this, and
/// the navigation service degrades from one to the other without the driving
/// stack noticing — perception, planning and simulation never depend on
/// navigation being available at all.
abstract class RouteProvider {
  String get name;

  /// Whether this provider can be used right now (network, keys, data).
  Future<bool> isAvailable();

  Future<NavigationRoute?> route({
    required GeoPosition origin,
    required GeoPosition destination,
    String? destinationName,
  });
}

/// Routing through a public OSRM instance.
///
/// This is the **only** part of the application that requires the internet,
/// and it is deliberately isolated behind [RouteProvider] so that losing
/// connectivity costs turn-by-turn guidance and nothing else.
class OsrmRouteProvider implements RouteProvider {
  OsrmRouteProvider({
    this.baseUrl = 'https://router.project-osrm.org',
    http.Client? client,
    this.timeout = const Duration(seconds: 12),
  }) : _client = client ?? http.Client();

  static const String _tag = 'OsrmRouteProvider';

  final String baseUrl;
  final http.Client _client;
  final Duration timeout;

  @override
  String get name => 'OSRM ($baseUrl)';

  @override
  Future<bool> isAvailable() async {
    try {
      final Uri uri = Uri.parse('$baseUrl/route/v1/driving/0,0;0.001,0.001'
          '?overview=false');
      final http.Response response =
          await _client.get(uri).timeout(const Duration(seconds: 5));
      return response.statusCode < 500;
    } catch (_) {
      return false;
    }
  }

  @override
  Future<NavigationRoute?> route({
    required GeoPosition origin,
    required GeoPosition destination,
    String? destinationName,
  }) async {
    final Uri uri = Uri.parse(
      '$baseUrl/route/v1/driving/'
      '${origin.longitude},${origin.latitude};'
      '${destination.longitude},${destination.latitude}'
      '?overview=full&geometries=polyline&steps=true&annotations=false',
    );

    try {
      final http.Response response = await _client.get(uri).timeout(timeout);
      if (response.statusCode != 200) {
        Log.warn(_tag, 'routing failed: HTTP ${response.statusCode}');
        return null;
      }

      final Map<String, dynamic> body =
          jsonDecode(response.body) as Map<String, dynamic>;
      if (body['code'] != 'Ok') {
        Log.warn(_tag, 'routing failed: ${body['code']}');
        return null;
      }

      final List<dynamic> routes = body['routes'] as List<dynamic>;
      if (routes.isEmpty) return null;
      final Map<String, dynamic> first = routes.first as Map<String, dynamic>;

      final List<RouteStep> steps = <RouteStep>[];
      final List<dynamic> legs = first['legs'] as List<dynamic>;
      for (final dynamic legRaw in legs) {
        final Map<String, dynamic> leg = legRaw as Map<String, dynamic>;
        for (final dynamic stepRaw in (leg['steps'] as List<dynamic>)) {
          final Map<String, dynamic> step = stepRaw as Map<String, dynamic>;
          final Map<String, dynamic> maneuver =
              step['maneuver'] as Map<String, dynamic>;

          steps.add(RouteStep(
            maneuver: ManeuverType.fromOsrm(
              maneuver['type'] as String? ?? '',
              maneuver['modifier'] as String?,
            ),
            polyline: decodePolyline(step['geometry'] as String? ?? ''),
            distanceMeters: (step['distance'] as num?)?.toDouble() ?? 0,
            durationSeconds: (step['duration'] as num?)?.toDouble() ?? 0,
            roadName: (step['name'] as String?)?.isEmpty ?? true
                ? null
                : step['name'] as String,
          ));
        }
      }

      if (steps.isEmpty) return null;

      return NavigationRoute(
        origin: origin,
        destination: destination,
        destinationName: destinationName,
        steps: steps,
        totalDistanceMeters: (first['distance'] as num?)?.toDouble() ?? 0,
        totalDurationSeconds: (first['duration'] as num?)?.toDouble() ?? 0,
        computedAt: DateTime.now(),
        source: 'osrm',
      );
    } on TimeoutException {
      Log.warn(_tag, 'routing timed out after ${timeout.inSeconds}s');
      return null;
    } catch (e) {
      Log.warn(_tag, 'routing error: $e');
      return null;
    }
  }

  void dispose() => _client.close();
}

/// Offline fallback: a straight line to the destination.
///
/// This is not a route and does not pretend to be one — [name] and the route's
/// `source` both say so, and the UI labels it clearly. It exists because a
/// bearing and a distance to the destination are genuinely useful when there
/// is no connectivity, and because the rest of the stack should not have to
/// branch on whether navigation exists.
class DirectLineRouteProvider implements RouteProvider {
  const DirectLineRouteProvider({this.assumedSpeedMps = 13.9});

  /// 50 km/h, used only for the ETA estimate.
  final double assumedSpeedMps;

  @override
  String get name => 'Direct line (offline — not a road route)';

  @override
  Future<bool> isAvailable() async => true;

  @override
  Future<NavigationRoute?> route({
    required GeoPosition origin,
    required GeoPosition destination,
    String? destinationName,
  }) async {
    final double distance = origin.distanceTo(destination);
    if (distance < 1) return null;

    // Interpolate so the matcher has segments to snap to.
    const int segments = 24;
    final List<GeoPosition> polyline = <GeoPosition>[
      for (int i = 0; i <= segments; i++)
        GeoPosition(
          latitude: origin.latitude +
              (destination.latitude - origin.latitude) * i / segments,
          longitude: origin.longitude +
              (destination.longitude - origin.longitude) * i / segments,
          accuracyMeters: 0,
          timestampMicros: 0,
        ),
    ];

    return NavigationRoute(
      origin: origin,
      destination: destination,
      destinationName: destinationName,
      steps: <RouteStep>[
        RouteStep(
          maneuver: ManeuverType.depart,
          polyline: polyline,
          distanceMeters: distance,
          durationSeconds: distance / assumedSpeedMps,
          roadName: 'Direct line',
        ),
        RouteStep(
          maneuver: ManeuverType.arrive,
          polyline: <GeoPosition>[destination, destination],
          distanceMeters: 0,
          durationSeconds: 0,
        ),
      ],
      totalDistanceMeters: distance,
      totalDurationSeconds: distance / assumedSpeedMps,
      computedAt: DateTime.now(),
      source: 'direct-line',
    );
  }
}

/// Tries providers in order and uses the first that succeeds.
class FallbackRouteProvider implements RouteProvider {
  const FallbackRouteProvider(this.providers);

  final List<RouteProvider> providers;

  @override
  String get name => providers.map((RouteProvider p) => p.name).join(' → ');

  @override
  Future<bool> isAvailable() async {
    for (final RouteProvider p in providers) {
      if (await p.isAvailable()) return true;
    }
    return false;
  }

  @override
  Future<NavigationRoute?> route({
    required GeoPosition origin,
    required GeoPosition destination,
    String? destinationName,
  }) async {
    for (final RouteProvider p in providers) {
      final NavigationRoute? r = await p.route(
        origin: origin,
        destination: destination,
        destinationName: destinationName,
      );
      if (r != null) return r;
    }
    return null;
  }
}
