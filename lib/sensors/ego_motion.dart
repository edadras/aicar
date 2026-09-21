import 'dart:math' as math;

import '../core/confidence.dart';
import '../core/geometry.dart';

/// A GNSS fix with its reported accuracy. Accuracy is carried everywhere
/// because a 30 m urban-canyon fix and a 3 m open-road fix must not be treated
/// the same by the navigation matcher.
class GeoPosition {
  const GeoPosition({
    required this.latitude,
    required this.longitude,
    required this.accuracyMeters,
    this.altitudeMeters,
    this.speedMps,
    this.speedAccuracyMps,
    this.headingDegrees,
    this.headingAccuracyDegrees,
    required this.timestampMicros,
  });

  final double latitude;
  final double longitude;
  final double accuracyMeters;
  final double? altitudeMeters;
  final double? speedMps;
  final double? speedAccuracyMps;
  final double? headingDegrees;
  final double? headingAccuracyDegrees;
  final int timestampMicros;

  static const double earthRadiusMeters = 6378137.0;

  /// Great-circle distance via the haversine formula.
  double distanceTo(GeoPosition other) {
    final double lat1 = degToRad(latitude);
    final double lat2 = degToRad(other.latitude);
    final double dLat = lat2 - lat1;
    final double dLon = degToRad(other.longitude - longitude);
    final double a = math.sin(dLat / 2) * math.sin(dLat / 2) +
        math.cos(lat1) * math.cos(lat2) * math.sin(dLon / 2) * math.sin(dLon / 2);
    return 2 * earthRadiusMeters * math.atan2(math.sqrt(a), math.sqrt(1 - a));
  }

  /// Initial bearing to [other], degrees clockwise from true north.
  double bearingTo(GeoPosition other) {
    final double lat1 = degToRad(latitude);
    final double lat2 = degToRad(other.latitude);
    final double dLon = degToRad(other.longitude - longitude);
    final double y = math.sin(dLon) * math.cos(lat2);
    final double x = math.cos(lat1) * math.sin(lat2) -
        math.sin(lat1) * math.cos(lat2) * math.cos(dLon);
    return (radToDeg(math.atan2(y, x)) + 360) % 360;
  }

  /// Local east-north offset in metres, valid over the few kilometres a route
  /// leg spans. Cheaper and more stable than a full projection.
  Vec2 localOffsetFrom(GeoPosition origin) {
    final double mPerDegLat = 111320.0;
    final double mPerDegLon =
        111320.0 * math.cos(degToRad((latitude + origin.latitude) / 2));
    return Vec2(
      (longitude - origin.longitude) * mPerDegLon, // east
      (latitude - origin.latitude) * mPerDegLat, // north
    );
  }

  /// Confidence derived from the reported horizontal accuracy. 5 m or better
  /// is fully trusted for route matching; beyond 40 m it is close to useless.
  Confidence get confidence {
    if (accuracyMeters <= 0) return Confidence(0.2, source: 'gps:no-accuracy');
    final double c = clampDouble(1.0 - (accuracyMeters - 5) / 35.0, 0.05, 1.0);
    return Confidence(c, source: 'gps:${accuracyMeters.toStringAsFixed(0)}m');
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'lat': latitude,
        'lon': longitude,
        'acc': accuracyMeters,
        if (altitudeMeters != null) 'alt': altitudeMeters,
        if (speedMps != null) 'spd': speedMps,
        if (headingDegrees != null) 'hdg': headingDegrees,
        'ts': timestampMicros,
      };

  static GeoPosition fromJson(Map<String, dynamic> j) => GeoPosition(
        latitude: (j['lat'] as num).toDouble(),
        longitude: (j['lon'] as num).toDouble(),
        accuracyMeters: (j['acc'] as num).toDouble(),
        altitudeMeters: (j['alt'] as num?)?.toDouble(),
        speedMps: (j['spd'] as num?)?.toDouble(),
        headingDegrees: (j['hdg'] as num?)?.toDouble(),
        timestampMicros: (j['ts'] as num).toInt(),
      );

  @override
  String toString() => '(${latitude.toStringAsFixed(6)}, '
      '${longitude.toStringAsFixed(6)}) ±${accuracyMeters.toStringAsFixed(0)}m';
}

/// One IMU sample set, already expressed in the **vehicle** frame
/// (x right, y forward, z up) rather than the phone's own frame.
class ImuSample {
  const ImuSample({
    required this.accelerationMps2,
    required this.angularRateRadPerS,
    required this.timestampMicros,
    this.magneticHeadingDegrees,
  });

  /// Linear acceleration with gravity removed, m/s².
  final Vec3 accelerationMps2;

  /// Gyroscope rates, rad/s. `z` is the yaw rate — the component that matters
  /// for detecting turns.
  final Vec3 angularRateRadPerS;

  final int timestampMicros;
  final double? magneticHeadingDegrees;

  double get longitudinalAcceleration => accelerationMps2.y;
  double get lateralAcceleration => accelerationMps2.x;

  /// Positive = pushed upwards. Gravity has already been removed, so on
  /// smooth tarmac this sits near zero and a speed bump is unmistakable.
  double get verticalAcceleration => accelerationMps2.z;

  /// Positive = turning right, matching the steering sign convention.
  double get yawRate => -angularRateRadPerS.z;

  static ImuSample lerp(ImuSample a, ImuSample b, double t) => ImuSample(
        accelerationMps2: Vec3.lerp(a.accelerationMps2, b.accelerationMps2, t),
        angularRateRadPerS:
            Vec3.lerp(a.angularRateRadPerS, b.angularRateRadPerS, t),
        timestampMicros: a.timestampMicros +
            ((b.timestampMicros - a.timestampMicros) * t).round(),
        magneticHeadingDegrees: a.magneticHeadingDegrees == null ||
                b.magneticHeadingDegrees == null
            ? (a.magneticHeadingDegrees ?? b.magneticHeadingDegrees)
            : _lerpAngle(a.magneticHeadingDegrees!, b.magneticHeadingDegrees!, t),
      );

  static double _lerpAngle(double a, double b, double t) {
    double d = (b - a) % 360;
    if (d > 180) d -= 360;
    if (d < -180) d += 360;
    return (a + d * t) % 360;
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'ax': _r(accelerationMps2.x),
        'ay': _r(accelerationMps2.y),
        'az': _r(accelerationMps2.z),
        'gx': _r(angularRateRadPerS.x),
        'gy': _r(angularRateRadPerS.y),
        'gz': _r(angularRateRadPerS.z),
        if (magneticHeadingDegrees != null) 'mh': _r(magneticHeadingDegrees!),
        'ts': timestampMicros,
      };

  static double _r(double v) => double.parse(v.toStringAsFixed(4));

  static ImuSample fromJson(Map<String, dynamic> j) => ImuSample(
        accelerationMps2: Vec3(
          (j['ax'] as num).toDouble(),
          (j['ay'] as num).toDouble(),
          (j['az'] as num).toDouble(),
        ),
        angularRateRadPerS: Vec3(
          (j['gx'] as num).toDouble(),
          (j['gy'] as num).toDouble(),
          (j['gz'] as num).toDouble(),
        ),
        magneticHeadingDegrees: (j['mh'] as num?)?.toDouble(),
        timestampMicros: (j['ts'] as num).toInt(),
      );
}

/// Simple 3-vector for IMU data. Kept minimal on purpose.
class Vec3 {
  const Vec3(this.x, this.y, this.z);
  const Vec3.zero() : x = 0, y = 0, z = 0;

  final double x;
  final double y;
  final double z;

  double get length => math.sqrt(x * x + y * y + z * z);

  Vec3 operator +(Vec3 o) => Vec3(x + o.x, y + o.y, z + o.z);
  Vec3 operator -(Vec3 o) => Vec3(x - o.x, y - o.y, z - o.z);
  Vec3 operator *(double s) => Vec3(x * s, y * s, z * s);

  static Vec3 lerp(Vec3 a, Vec3 b, double t) => Vec3(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
      );

  @override
  String toString() => '(${x.toStringAsFixed(2)}, ${y.toStringAsFixed(2)}, '
      '${z.toStringAsFixed(2)})';
}

/// The fused motion state of the ego vehicle at one instant.
///
/// Produced by [EgoMotionEstimator] from GPS + IMU + (optionally) visual
/// odometry, and consumed by essentially every downstream stage: the tracker
/// needs it to separate object motion from ego motion, the planner needs speed
/// to choose a look-ahead, and the decision engine needs it for TTC.
class EgoMotionState {
  const EgoMotionState({
    required this.speedMps,
    required this.headingDegrees,
    required this.yawRateRadPerS,
    required this.longitudinalAccelMps2,
    required this.lateralAccelMps2,
    required this.position,
    this.verticalAccelMps2 = 0,
    required this.timestampMicros,
    required this.speedConfidence,
    required this.headingConfidence,
    this.isStationary = false,
    this.source = 'fused',
  });

  static const EgoMotionState unknown = EgoMotionState(
    speedMps: 0,
    headingDegrees: 0,
    yawRateRadPerS: 0,
    longitudinalAccelMps2: 0,
    lateralAccelMps2: 0,
    position: null,
    timestampMicros: 0,
    speedConfidence: 0,
    headingConfidence: 0,
    isStationary: true,
    source: 'unknown',
  );

  final double speedMps;

  /// Degrees clockwise from true north.
  final double headingDegrees;

  /// Positive = turning right.
  final double yawRateRadPerS;

  final double longitudinalAccelMps2;
  final double lateralAccelMps2;

  /// Vertical acceleration in the vehicle frame, gravity removed.
  ///
  /// Nothing steers by this. It is here so the stack can check a prediction
  /// against reality: when a speed bump the camera claimed to see is crossed,
  /// the jolt either happened or it did not.
  final double verticalAccelMps2;

  final GeoPosition? position;
  final int timestampMicros;
  final double speedConfidence;
  final double headingConfidence;
  final bool isStationary;
  final String source;

  double get speedKph => speedMps * 3.6;
  double get yawRateDegPerS => radToDeg(yawRateRadPerS);

  /// Radius of the circle the vehicle is currently tracing, metres. Infinite
  /// when going straight; used as a sanity bound on planned-path curvature.
  double get turnRadiusMeters {
    if (yawRateRadPerS.abs() < 1e-4) return double.infinity;
    return speedMps / yawRateRadPerS.abs();
  }

  double get overallConfidence => math.min(speedConfidence, headingConfidence);

  EgoMotionState copyWith({
    double? speedMps,
    double? headingDegrees,
    double? yawRateRadPerS,
    double? longitudinalAccelMps2,
    double? lateralAccelMps2,
    double? verticalAccelMps2,
    GeoPosition? position,
    int? timestampMicros,
    double? speedConfidence,
    double? headingConfidence,
    bool? isStationary,
    String? source,
  }) =>
      EgoMotionState(
        speedMps: speedMps ?? this.speedMps,
        headingDegrees: headingDegrees ?? this.headingDegrees,
        yawRateRadPerS: yawRateRadPerS ?? this.yawRateRadPerS,
        longitudinalAccelMps2:
            longitudinalAccelMps2 ?? this.longitudinalAccelMps2,
        lateralAccelMps2: lateralAccelMps2 ?? this.lateralAccelMps2,
        verticalAccelMps2: verticalAccelMps2 ?? this.verticalAccelMps2,
        position: position ?? this.position,
        timestampMicros: timestampMicros ?? this.timestampMicros,
        speedConfidence: speedConfidence ?? this.speedConfidence,
        headingConfidence: headingConfidence ?? this.headingConfidence,
        isStationary: isStationary ?? this.isStationary,
        source: source ?? this.source,
      );

  Map<String, dynamic> toJson() => <String, dynamic>{
        'speed': _r(speedMps),
        'heading': _r(headingDegrees),
        'yawRate': _r(yawRateRadPerS),
        'accLong': _r(longitudinalAccelMps2),
        'accLat': _r(lateralAccelMps2),
        'accVert': _r(verticalAccelMps2),
        'pos': position?.toJson(),
        'ts': timestampMicros,
        'speedConf': _r(speedConfidence),
        'headingConf': _r(headingConfidence),
        'stationary': isStationary,
        'source': source,
      };

  static double _r(double v) => double.parse(v.toStringAsFixed(3));

  static EgoMotionState fromJson(Map<String, dynamic> j) => EgoMotionState(
        speedMps: (j['speed'] as num).toDouble(),
        headingDegrees: (j['heading'] as num).toDouble(),
        yawRateRadPerS: (j['yawRate'] as num).toDouble(),
        longitudinalAccelMps2: (j['accLong'] as num).toDouble(),
        lateralAccelMps2: (j['accLat'] as num).toDouble(),
        verticalAccelMps2: (j['accVert'] as num?)?.toDouble() ?? 0,
        position: j['pos'] == null
            ? null
            : GeoPosition.fromJson(j['pos'] as Map<String, dynamic>),
        timestampMicros: (j['ts'] as num).toInt(),
        speedConfidence: (j['speedConf'] as num).toDouble(),
        headingConfidence: (j['headingConf'] as num).toDouble(),
        isStationary: j['stationary'] as bool? ?? false,
        source: (j['source'] as String?) ?? 'replay',
      );

  @override
  String toString() => 'Ego(${speedKph.toStringAsFixed(1)} km/h, '
      'hdg ${headingDegrees.toStringAsFixed(0)}°, '
      'yaw ${yawRateDegPerS.toStringAsFixed(1)}°/s)';
}
