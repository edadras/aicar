import 'dart:async';

import 'package:geolocator/geolocator.dart';

import '../core/logging.dart';
import '../core/time_sync.dart';
import 'ego_motion.dart';

/// Wraps the platform location provider and emits [GeoPosition] fixes on the
/// project's monotonic clock.
///
/// The OS timestamp on a fix is wall-clock and can jump (NTP sync, timezone
/// changes); it is kept for the record but the value used for time alignment
/// is the monotonic receive time, adjusted by the fix's own age when the
/// platform reports one.
class GpsService {
  GpsService({required MonotonicClock clock}) : _clock = clock;

  static const String _tag = 'GpsService';

  final MonotonicClock _clock;
  final StreamController<GeoPosition> _fixes =
      StreamController<GeoPosition>.broadcast();

  StreamSubscription<Position>? _sub;
  GeoPosition? _lastFix;
  bool _running = false;
  int _fixCount = 0;

  Stream<GeoPosition> get fixes => _fixes.stream;
  GeoPosition? get lastFix => _lastFix;
  bool get isRunning => _running;
  int get fixCount => _fixCount;

  /// Request permission and confirm the location service is enabled.
  /// Returns a human-readable reason on failure, `null` on success.
  Future<String?> ensurePermission() async {
    if (!await Geolocator.isLocationServiceEnabled()) {
      return 'Location services are turned off on this device.';
    }
    LocationPermission permission = await Geolocator.checkPermission();
    if (permission == LocationPermission.denied) {
      permission = await Geolocator.requestPermission();
    }
    if (permission == LocationPermission.denied) {
      return 'Location permission denied.';
    }
    if (permission == LocationPermission.deniedForever) {
      return 'Location permission permanently denied — enable it in Settings.';
    }
    return null;
  }

  Future<void> start() async {
    if (_running) return;
    final String? error = await ensurePermission();
    if (error != null) {
      Log.warn(_tag, 'not starting: $error');
      throw StateError(error);
    }

    _running = true;
    _sub = Geolocator.getPositionStream(
      locationSettings: const LocationSettings(
        accuracy: LocationAccuracy.bestForNavigation,
        // Distance filter 0: at motorway speed a 5 m filter would throttle the
        // stream to ~4 Hz in a way that depends on speed, which complicates
        // the fusion timing for no benefit.
        distanceFilter: 0,
        timeLimit: null,
      ),
    ).listen(
      _onPosition,
      onError: (Object e) => Log.warn(_tag, 'position stream error: $e'),
      cancelOnError: false,
    );
    Log.info(_tag, 'position stream started');
  }

  void _onPosition(Position p) {
    _fixCount++;
    // `Position.timestamp` is wall-clock; convert to our monotonic base by
    // subtracting the fix's age at the moment it reached us.
    final int nowMicros = _clock.micros;
    final int ageMicros = DateTime.now()
        .difference(p.timestamp)
        .inMicroseconds
        .clamp(0, 5000000);

    final GeoPosition fix = GeoPosition(
      latitude: p.latitude,
      longitude: p.longitude,
      accuracyMeters: p.accuracy,
      altitudeMeters: p.altitude,
      speedMps: p.speed >= 0 ? p.speed : null,
      speedAccuracyMps: p.speedAccuracy > 0 ? p.speedAccuracy : null,
      // Android reports 0° when it has no course fix, and a stationary device
      // reports noise; the estimator gates on speed anyway.
      headingDegrees: p.heading >= 0 ? p.heading : null,
      headingAccuracyDegrees:
          p.headingAccuracy > 0 ? p.headingAccuracy : null,
      timestampMicros: nowMicros - ageMicros,
    );
    _lastFix = fix;
    if (!_fixes.isClosed) _fixes.add(fix);
  }

  Future<void> stop() async {
    _running = false;
    await _sub?.cancel();
    _sub = null;
    Log.info(_tag, 'position stream stopped');
  }

  Future<void> dispose() async {
    await stop();
    await _fixes.close();
  }
}
