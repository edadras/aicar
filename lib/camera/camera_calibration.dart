import 'dart:math' as math;

import '../core/geometry.dart';

/// Pinhole camera model plus the rigid mounting transform between the phone
/// and the vehicle.
///
/// Frames used here:
///  * **Image**: `(u, v)` pixels, origin top-left.
///  * **Camera**: `x` right, `y` down, `z` along the optical axis.
///  * **World/vehicle**: `x` right of the vehicle centreline, `y` forward,
///    ground plane at the camera height below the lens.
///
/// Everything that converts pixels to metres — ground-plane depth, inverse
/// perspective mapping for lanes, the planned-path overlay — goes through this
/// class, so a bad calibration degrades gracefully and visibly instead of
/// silently biasing one subsystem.
class CameraCalibration {
  const CameraCalibration({
    required this.imageWidth,
    required this.imageHeight,
    required this.horizontalFovDegrees,
    required this.cameraHeightMeters,
    required this.pitchDegrees,
    required this.rollDegrees,
    required this.yawDegrees,
    required this.lateralOffsetMeters,
    required this.longitudinalOffsetMeters,
    this.principalPointX,
    this.principalPointY,
    this.isCalibrated = false,
    this.calibratedAt,
  });

  /// Defaults tuned for a Galaxy S23 main camera in a typical windscreen /
  /// dashboard cradle. Marked `isCalibrated: false` so the HUD can tell the
  /// user the numbers are assumptions, not measurements.
  factory CameraCalibration.galaxyS23Default({
    int imageWidth = 1280,
    int imageHeight = 720,
  }) =>
      CameraCalibration(
        imageWidth: imageWidth,
        imageHeight: imageHeight,
        // S23 main camera: 24 mm equivalent ≈ 73.7° horizontal FOV on 4:3;
        // a 16:9 stream crops vertically, not horizontally, so the horizontal
        // figure carries over.
        horizontalFovDegrees: 73.7,
        cameraHeightMeters: 1.20,
        pitchDegrees: 3.0,
        rollDegrees: 0.0,
        yawDegrees: 0.0,
        lateralOffsetMeters: 0.35,
        longitudinalOffsetMeters: 2.10,
        isCalibrated: false,
      );

  final int imageWidth;
  final int imageHeight;

  /// Horizontal field of view of the *full* stream, in degrees.
  final double horizontalFovDegrees;

  /// Lens height above the road surface, metres.
  final double cameraHeightMeters;

  /// Downward tilt of the optical axis, degrees. Positive = pointing down.
  final double pitchDegrees;

  /// Rotation about the optical axis, degrees. Positive = image rotates
  /// clockwise (phone leaning right in the cradle).
  final double rollDegrees;

  /// Rotation about the vertical axis, degrees. Positive = pointing right of
  /// the vehicle's forward direction.
  final double yawDegrees;

  /// Lateral offset of the lens from the vehicle centreline, metres.
  /// Positive = mounted to the right of the centreline.
  final double lateralOffsetMeters;

  /// Distance from the front axle to the lens, metres. Used to express
  /// distances relative to the vehicle front rather than the phone.
  final double longitudinalOffsetMeters;

  final double? principalPointX;
  final double? principalPointY;

  final bool isCalibrated;
  final DateTime? calibratedAt;

  // --- Intrinsics ---------------------------------------------------------

  double get fx =>
      (imageWidth / 2) / math.tan(degToRad(horizontalFovDegrees) / 2);

  /// Square pixels are a very good assumption on modern phone sensors, so the
  /// vertical focal length equals the horizontal one.
  double get fy => fx;

  double get cx => principalPointX ?? imageWidth / 2;
  double get cy => principalPointY ?? imageHeight / 2;

  double get verticalFovDegrees =>
      radToDeg(2 * math.atan((imageHeight / 2) / fy));

  double get pitchRadians => degToRad(pitchDegrees);
  double get rollRadians => degToRad(rollDegrees);
  double get yawRadians => degToRad(yawDegrees);

  /// Image row of the horizon. Anything above it cannot be on the ground
  /// plane, which is the primary sanity check for geometric depth.
  double get horizonY => cy - fy * math.tan(pitchRadians);

  /// Normalised horizon position, handy for drawing and for clipping the
  /// drivable-area search region.
  double get horizonYNormalized => horizonY / imageHeight;

  // --- Projection ---------------------------------------------------------

  /// Undo roll so the rest of the maths can assume a level camera.
  PixelPoint _deroll(PixelPoint p) {
    if (rollDegrees.abs() < 1e-6) return p;
    final double c = math.cos(-rollRadians);
    final double s = math.sin(-rollRadians);
    final double du = p.u - cx;
    final double dv = p.v - cy;
    return PixelPoint(cx + du * c - dv * s, cy + du * s + dv * c);
  }

  PixelPoint _reroll(PixelPoint p) {
    if (rollDegrees.abs() < 1e-6) return p;
    final double c = math.cos(rollRadians);
    final double s = math.sin(rollRadians);
    final double du = p.u - cx;
    final double dv = p.v - cy;
    return PixelPoint(cx + du * c - dv * s, cy + du * s + dv * c);
  }

  /// Project an image point onto the road plane.
  ///
  /// Returns `null` for rays at or above the horizon — those never meet the
  /// ground, and inventing a distance for them is exactly the kind of silent
  /// fabrication this project avoids.
  ///
  /// The result is in **vehicle** coordinates: `x` right of the centreline,
  /// `y` forward from the lens.
  Vec2? projectToGround(PixelPoint pixel) {
    final PixelPoint p = _deroll(pixel);
    final double dx = (p.u - cx) / fx;
    final double dy = (p.v - cy) / fy;
    const double dz = 1.0;

    final double cp = math.cos(pitchRadians);
    final double sp = math.sin(pitchRadians);

    // Camera -> world (y still points down).
    final double wy = dy * cp + dz * sp;
    if (wy <= 1e-6) return null; // at or above the horizon

    final double wz = -dy * sp + dz * cp;
    if (wz <= 1e-6) return null; // behind the camera after the tilt

    final double t = cameraHeightMeters / wy;
    double forward = t * wz;
    double lateral = t * dx;

    if (yawDegrees.abs() > 1e-6) {
      final Vec2 rotated = Vec2(lateral, forward).rotated(-yawRadians);
      lateral = rotated.x;
      forward = rotated.y;
    }

    return Vec2(lateral + lateralOffsetMeters, forward);
  }

  /// Inverse of [projectToGround]: where does a point on the road appear?
  /// Returns `null` for points behind the image plane.
  PixelPoint? projectGroundToImage(Vec2 groundPoint) {
    double lateral = groundPoint.x - lateralOffsetMeters;
    double forward = groundPoint.y;

    if (yawDegrees.abs() > 1e-6) {
      final Vec2 rotated = Vec2(lateral, forward).rotated(yawRadians);
      lateral = rotated.x;
      forward = rotated.y;
    }

    final double cp = math.cos(pitchRadians);
    final double sp = math.sin(pitchRadians);

    // World (x right, y down = camera height, z forward) -> camera.
    final double h = cameraHeightMeters;
    final double xc = lateral;
    final double yc = h * cp - forward * sp;
    final double zc = h * sp + forward * cp;
    if (zc <= 1e-6) return null;

    return _reroll(PixelPoint(cx + fx * xc / zc, cy + fy * yc / zc));
  }

  /// Distance to an object standing on the road, from the image row of its
  /// contact patch. This is the single most reliable monocular distance cue
  /// available without a depth network.
  ///
  /// Returns `null` above the horizon.
  double? groundDistanceForImageRow(double v, {double u = double.nan}) {
    final double uu = u.isNaN ? cx : u;
    final Vec2? g = projectToGround(PixelPoint(uu, v));
    if (g == null) return null;
    return g.length;
  }

  /// Distance implied by an object's apparent height, given a prior on its
  /// true height. Complements the ground-plane cue: it still works when the
  /// contact patch is occluded or the road is not flat.
  double? distanceFromApparentHeight({
    required double boxHeightPixels,
    required double realHeightMeters,
  }) {
    if (boxHeightPixels <= 1) return null;
    return fy * realHeightMeters / boxHeightPixels;
  }

  /// Distance implied by apparent width, used for vehicles seen from behind
  /// where width is a tighter prior than height.
  double? distanceFromApparentWidth({
    required double boxWidthPixels,
    required double realWidthMeters,
  }) {
    if (boxWidthPixels <= 1) return null;
    return fx * realWidthMeters / boxWidthPixels;
  }

  /// Metres per pixel on the ground at image row [v]. Grows without bound near
  /// the horizon, which is why far-field geometric depth is down-weighted.
  double? groundResolutionAtRow(double v) {
    final Vec2? a = projectToGround(PixelPoint(cx, v));
    final Vec2? b = projectToGround(PixelPoint(cx, v - 1));
    if (a == null || b == null) return null;
    return (b.y - a.y).abs();
  }

  /// How much to trust a ground-plane distance at row [v]. Falls off sharply
  /// as the contact point approaches the horizon, where one pixel of error is
  /// worth many metres.
  double groundDepthConfidenceAtRow(double v) {
    final double below = v - horizonY;
    if (below <= 2) return 0;
    final double span = imageHeight - horizonY;
    if (span <= 1) return 0;
    final double ratio = clampDouble(below / span, 0, 1);
    // sqrt keeps mid-field rows usefully confident instead of collapsing.
    double base = math.sqrt(ratio);
    if (!isCalibrated) base *= 0.75; // defaults are a guess, say so
    return clampDouble(base, 0, 1);
  }

  // --- Derived helpers ----------------------------------------------------

  /// Rescale the model to a different processing resolution. Intrinsics scale
  /// linearly; the mounting geometry does not change.
  CameraCalibration scaledTo(int width, int height) {
    if (width == imageWidth && height == imageHeight) return this;
    final double sx = width / imageWidth;
    final double sy = height / imageHeight;
    return copyWith(
      imageWidth: width,
      imageHeight: height,
      principalPointX: cx * sx,
      principalPointY: cy * sy,
    );
  }

  CameraCalibration copyWith({
    int? imageWidth,
    int? imageHeight,
    double? horizontalFovDegrees,
    double? cameraHeightMeters,
    double? pitchDegrees,
    double? rollDegrees,
    double? yawDegrees,
    double? lateralOffsetMeters,
    double? longitudinalOffsetMeters,
    double? principalPointX,
    double? principalPointY,
    bool? isCalibrated,
    DateTime? calibratedAt,
  }) =>
      CameraCalibration(
        imageWidth: imageWidth ?? this.imageWidth,
        imageHeight: imageHeight ?? this.imageHeight,
        horizontalFovDegrees: horizontalFovDegrees ?? this.horizontalFovDegrees,
        cameraHeightMeters: cameraHeightMeters ?? this.cameraHeightMeters,
        pitchDegrees: pitchDegrees ?? this.pitchDegrees,
        rollDegrees: rollDegrees ?? this.rollDegrees,
        yawDegrees: yawDegrees ?? this.yawDegrees,
        lateralOffsetMeters: lateralOffsetMeters ?? this.lateralOffsetMeters,
        longitudinalOffsetMeters:
            longitudinalOffsetMeters ?? this.longitudinalOffsetMeters,
        principalPointX: principalPointX ?? this.principalPointX,
        principalPointY: principalPointY ?? this.principalPointY,
        isCalibrated: isCalibrated ?? this.isCalibrated,
        calibratedAt: calibratedAt ?? this.calibratedAt,
      );

  Map<String, dynamic> toJson() => <String, dynamic>{
        'imageWidth': imageWidth,
        'imageHeight': imageHeight,
        'hfov': horizontalFovDegrees,
        'height': cameraHeightMeters,
        'pitch': pitchDegrees,
        'roll': rollDegrees,
        'yaw': yawDegrees,
        'lateralOffset': lateralOffsetMeters,
        'longitudinalOffset': longitudinalOffsetMeters,
        if (principalPointX != null) 'ppx': principalPointX,
        if (principalPointY != null) 'ppy': principalPointY,
        'isCalibrated': isCalibrated,
        'calibratedAt': calibratedAt?.toIso8601String(),
      };

  static CameraCalibration fromJson(Map<String, dynamic> j) => CameraCalibration(
        imageWidth: (j['imageWidth'] as num).toInt(),
        imageHeight: (j['imageHeight'] as num).toInt(),
        horizontalFovDegrees: (j['hfov'] as num).toDouble(),
        cameraHeightMeters: (j['height'] as num).toDouble(),
        pitchDegrees: (j['pitch'] as num).toDouble(),
        rollDegrees: (j['roll'] as num).toDouble(),
        yawDegrees: (j['yaw'] as num?)?.toDouble() ?? 0,
        lateralOffsetMeters: (j['lateralOffset'] as num).toDouble(),
        longitudinalOffsetMeters:
            (j['longitudinalOffset'] as num?)?.toDouble() ?? 2.0,
        principalPointX: (j['ppx'] as num?)?.toDouble(),
        principalPointY: (j['ppy'] as num?)?.toDouble(),
        isCalibrated: j['isCalibrated'] as bool? ?? false,
        calibratedAt: j['calibratedAt'] == null
            ? null
            : DateTime.tryParse(j['calibratedAt'] as String),
      );

  @override
  String toString() => 'CameraCalibration(${imageWidth}x$imageHeight, '
      'fov ${horizontalFovDegrees.toStringAsFixed(1)}°, '
      'h ${cameraHeightMeters.toStringAsFixed(2)}m, '
      'pitch ${pitchDegrees.toStringAsFixed(1)}°'
      '${isCalibrated ? '' : ', UNCALIBRATED'})';
}
