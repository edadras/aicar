import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:provider/provider.dart';

import '../../navigation/maneuver.dart';
import '../../navigation/route.dart';
import '../../navigation/route_provider.dart';
import '../../sensors/ego_motion.dart';
import '../driving_session.dart';
import '../theme.dart';

/// Pick a destination and compute a route.
///
/// The screen is explicit that navigation contributes intent only. It is the
/// one place in the app that touches the network, and the one feature that
/// simply does not work offline — everything else keeps running.
class NavigationScreen extends StatefulWidget {
  const NavigationScreen({super.key});

  @override
  State<NavigationScreen> createState() => _NavigationScreenState();
}

class _NavigationScreenState extends State<NavigationScreen> {
  final MapController _map = MapController();
  LatLng? _destination;
  bool _routing = false;
  String? _message;

  @override
  Widget build(BuildContext context) {
    final DrivingSession session = context.watch<DrivingSession>();
    final GeoPosition? here = session.sensors.lastFix;
    final NavigationRoute? route = session.navigation.route;
    final RouteProgress? progress = session.navigation.progress;

    final LatLng centre = here != null
        ? LatLng(here.latitude, here.longitude)
        : const LatLng(35.6892, 51.3890);

    return Scaffold(
      appBar: AppBar(
        title: const Text('Navigation'),
        actions: <Widget>[
          if (route != null)
            IconButton(
              icon: const Icon(Icons.clear),
              tooltip: 'Clear route',
              onPressed: () {
                session.navigation.clearRoute();
                setState(() {
                  _destination = null;
                  _message = null;
                });
              },
            ),
        ],
      ),
      body: Column(
        children: <Widget>[
          Expanded(
            child: FlutterMap(
              mapController: _map,
              options: MapOptions(
                initialCenter: centre,
                initialZoom: 14,
                onTap: (TapPosition _, LatLng point) =>
                    setState(() => _destination = point),
              ),
              children: <Widget>[
                TileLayer(
                  urlTemplate:
                      'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
                  userAgentPackageName: 'com.aicar.aicar',
                  // Tiles are the only network dependency, and losing them
                  // costs the map picture, not the routing or the driving.
                  errorTileCallback: (_, __, ___) {},
                ),
                if (route != null)
                  PolylineLayer<Object>(
                    polylines: <Polyline<Object>>[
                      Polyline<Object>(
                        points: <LatLng>[
                          for (final GeoPosition p in route.polyline)
                            LatLng(p.latitude, p.longitude),
                        ],
                        strokeWidth: 5,
                        color: HudTheme.info,
                      ),
                    ],
                  ),
                MarkerLayer(
                  markers: <Marker>[
                    if (here != null)
                      Marker(
                        point: LatLng(here.latitude, here.longitude),
                        width: 22,
                        height: 22,
                        child: Container(
                          decoration: BoxDecoration(
                            color: HudTheme.accent,
                            shape: BoxShape.circle,
                            border: Border.all(
                                color: Colors.white, width: 2),
                          ),
                        ),
                      ),
                    if (_destination != null)
                      Marker(
                        point: _destination!,
                        width: 30,
                        height: 30,
                        child: const Icon(Icons.place,
                            color: HudTheme.critical, size: 28),
                      ),
                  ],
                ),
              ],
            ),
          ),
          Container(
            padding: const EdgeInsets.all(14),
            decoration: const BoxDecoration(
              border: Border(top: BorderSide(color: HudTheme.outline)),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                if (progress != null) ...<Widget>[
                  Row(
                    children: <Widget>[
                      HudBadge(
                        text: progress.intent.label,
                        color: HudTheme.info,
                        filled: true,
                      ),
                      const SizedBox(width: 10),
                      Expanded(
                        child: Text(progress.displayInstruction,
                            style: HudTheme.body),
                      ),
                    ],
                  ),
                  const SizedBox(height: 8),
                  Row(
                    children: <Widget>[
                      _stat('Remaining',
                          '${(progress.distanceRemainingMeters / 1000)
                              .toStringAsFixed(1)} km'),
                      const SizedBox(width: 18),
                      _stat('ETA',
                          '${(progress.durationRemainingSeconds / 60)
                              .round()} min'),
                      const SizedBox(width: 18),
                      _stat('Match',
                          '${(progress.matchQuality * 100).round()}%'),
                      if (progress.isOffRoute) ...<Widget>[
                        const SizedBox(width: 18),
                        const HudBadge(
                            text: 'OFF ROUTE', color: HudTheme.caution),
                      ],
                    ],
                  ),
                  const SizedBox(height: 12),
                ],
                if (_message != null) ...<Widget>[
                  Text(_message!,
                      style: HudTheme.caption
                          .copyWith(color: HudTheme.caution)),
                  const SizedBox(height: 8),
                ],
                Row(
                  children: <Widget>[
                    Expanded(
                      child: FilledButton.icon(
                        onPressed: _destination == null || _routing
                            ? null
                            : () => _route(session, here),
                        icon: _routing
                            ? const SizedBox(
                                width: 16,
                                height: 16,
                                child: CircularProgressIndicator(
                                    strokeWidth: 2),
                              )
                            : const Icon(Icons.directions, size: 18),
                        label: Text(here == null
                            ? 'Waiting for a GPS fix'
                            : 'Route to the marker'),
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 10),
                const Text(
                  'Navigation contributes a high-level intent only — '
                  'STRAIGHT, TURN_LEFT, KEEP_RIGHT and so on. It never '
                  'produces a steering angle: the local planner decides the '
                  'path from the road it can actually see, and the whole '
                  'driving stack works unchanged with no route at all.',
                  style: HudTheme.caption,
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _stat(String label, String value) => Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Text(label.toUpperCase(),
              style: HudTheme.hudLabel.copyWith(fontSize: 10)),
          Text(value,
              style: HudTheme.body.copyWith(
                  fontFamily: HudTheme.monoFamily, fontSize: 14)),
        ],
      );

  Future<void> _route(DrivingSession session, GeoPosition? here) async {
    if (here == null || _destination == null) return;
    setState(() {
      _routing = true;
      _message = null;
    });

    final bool ok = await session.navigation.setDestination(
      origin: here,
      destination: GeoPosition(
        latitude: _destination!.latitude,
        longitude: _destination!.longitude,
        accuracyMeters: 0,
        timestampMicros: 0,
      ),
    );

    if (!mounted) return;
    setState(() {
      _routing = false;
      if (!ok) {
        _message = session.navigation.lastError ??
            'Could not compute a route.';
      } else if (session.navigation.route?.source == 'direct-line') {
        _message = 'Offline: showing a straight line to the destination, '
            'not a road route.';
      }
    });
  }
}

/// Exposed so Settings can force the offline provider.
RouteProvider offlineOnlyProvider() => const DirectLineRouteProvider();

/// Intent labels, for documentation and the settings screen.
const List<ManeuverIntent> supportedIntents = ManeuverIntent.values;
