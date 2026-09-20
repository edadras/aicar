import 'dart:math' as math;
import 'dart:typed_data';

import '../camera/camera_calibration.dart';
import '../core/geometry.dart';

/// Semantic classes the segmenter distinguishes.
///
/// The split between [road] and [drivableRoad] is deliberate and is what the
/// planner actually uses: asphalt occupied by a parked lorry is still road,
/// but it is not drivable.
enum SurfaceClass {
  road('Road'),
  drivableRoad('Drivable road'),
  sidewalk('Sidewalk'),
  curb('Curb'),
  grass('Grass'),
  building('Building'),
  vehicle('Vehicle'),
  pedestrian('Pedestrian'),
  obstacle('Obstacle'),
  unknown('Unknown');

  const SurfaceClass(this.label);
  final String label;

  bool get isDrivable =>
      this == SurfaceClass.road || this == SurfaceClass.drivableRoad;

  /// Surfaces a vehicle could physically cross but must not plan through.
  bool get isSoftBoundary =>
      this == SurfaceClass.curb || this == SurfaceClass.grass;

  static SurfaceClass fromIndex(int i) => i >= 0 && i < SurfaceClass.values.length
      ? SurfaceClass.values[i]
      : SurfaceClass.unknown;
}

/// Per-pixel semantic map at a reduced resolution.
///
/// Segmentation runs at a coarse grid (typically 160x96) because the planner
/// needs "where can I drive" at metre resolution, not pixel resolution, and a
/// coarse grid costs a fraction of the inference time.
class RoadSegmentation {
  RoadSegmentation({
    required this.width,
    required this.height,
    required this.classIndices,
    required this.classConfidence,
    required this.frameId,
    required this.timestampMicros,
    this.modelName = 'classical-cv',
    this.isDegraded = false,
    this.degradedReason,
  }) : assert(classIndices.length == width * height);

  factory RoadSegmentation.unavailable({
    required int frameId,
    required int timestampMicros,
    String reason = 'no segmentation model installed',
  }) =>
      RoadSegmentation(
        width: 1,
        height: 1,
        classIndices: Uint8List(1)..[0] = SurfaceClass.unknown.index,
        classConfidence: Float32List(1),
        frameId: frameId,
        timestampMicros: timestampMicros,
        modelName: 'none',
        isDegraded: true,
        degradedReason: reason,
      );

  final int width;
  final int height;
  final Uint8List classIndices;
  final Float32List classConfidence;
  final int frameId;
  final int timestampMicros;
  final String modelName;
  final bool isDegraded;
  final String? degradedReason;

  bool get isUsable => !isDegraded && width > 1 && height > 1;

  SurfaceClass classAt(int x, int y) {
    if (x < 0 || y < 0 || x >= width || y >= height) {
      return SurfaceClass.unknown;
    }
    return SurfaceClass.fromIndex(classIndices[y * width + x]);
  }

  double confidenceAt(int x, int y) {
    if (x < 0 || y < 0 || x >= width || y >= height) return 0;
    return classConfidence[y * width + x];
  }

  SurfaceClass classAtNormalized(double nx, double ny) => classAt(
        (nx * width).round().clamp(0, width - 1),
        (ny * height).round().clamp(0, height - 1),
      );

  bool isDrivableAtNormalized(double nx, double ny) =>
      classAtNormalized(nx, ny).isDrivable;

  /// Fraction of the lower half of the frame classified as drivable. A useful
  /// one-number health check: near zero means the model has lost the road.
  double get drivableFraction {
    if (!isUsable) return 0;
    int drivable = 0;
    int total = 0;
    for (int y = height ~/ 2; y < height; y++) {
      for (int x = 0; x < width; x++) {
        total++;
        if (SurfaceClass.fromIndex(classIndices[y * width + x]).isDrivable) {
          drivable++;
        }
      }
    }
    return total == 0 ? 0 : drivable / total;
  }

  double get overallConfidence {
    if (!isUsable) return 0;
    double sum = 0;
    for (final double c in classConfidence) {
      sum += c;
    }
    final double mean = sum / classConfidence.length;
    // A model that says "road" everywhere, including the sky, is confidently
    // wrong; penalise implausible drivable fractions.
    final double frac = drivableFraction;
    final double plausibility = frac < 0.05
        ? frac / 0.05
        : (frac > 0.92 ? clampDouble((1 - frac) / 0.08, 0, 1) : 1.0);
    return clampDouble(mean * plausibility, 0, 1);
  }

  /// Scan upward from the bottom of each column and return the image row at
  /// which the drivable surface ends. This is the raw material for both the
  /// drivable corridor and the road-edge detector.
  Float32List drivableTopRowPerColumn() {
    final Float32List out = Float32List(width);
    for (int x = 0; x < width; x++) {
      int topRow = height;
      int consecutiveNonDrivable = 0;
      for (int y = height - 1; y >= 0; y--) {
        final bool drivable =
            SurfaceClass.fromIndex(classIndices[y * width + x]).isDrivable;
        if (drivable) {
          topRow = y;
          consecutiveNonDrivable = 0;
        } else {
          consecutiveNonDrivable++;
          // Tolerate a couple of rows of noise (lane markings, shadows,
          // a manhole cover) before declaring the road finished.
          if (consecutiveNonDrivable > 2) break;
        }
      }
      out[x] = topRow.toDouble();
    }
    return out;
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'w': width,
        'h': height,
        'model': modelName,
        'drivableFraction': double.parse(drivableFraction.toStringAsFixed(3)),
        'conf': double.parse(overallConfidence.toStringAsFixed(3)),
        if (isDegraded) 'degraded': degradedReason,
      };
}

/// The drivable corridor in **metric vehicle coordinates**.
///
/// This is the form the planner consumes: for each distance ahead, how far
/// left and right can the vehicle go. Converting once, here, keeps the
/// planner free of any image-space reasoning.
class DrivableArea {
  const DrivableArea({
    required this.samples,
    required this.confidence,
    required this.frameId,
    required this.timestampMicros,
    required this.source,
  });

  factory DrivableArea.empty({
    required int frameId,
    required int timestampMicros,
  }) =>
      DrivableArea(
        samples: const <DrivableSample>[],
        confidence: 0,
        frameId: frameId,
        timestampMicros: timestampMicros,
        source: 'none',
      );

  /// Ordered by increasing distance ahead.
  final List<DrivableSample> samples;
  final double confidence;
  final int frameId;
  final int timestampMicros;
  final String source;

  bool get isEmpty => samples.isEmpty;

  double get maxRangeMeters => samples.isEmpty ? 0 : samples.last.distanceAhead;

  /// Left and right metric limits at [distanceAhead], interpolated.
  (double, double)? limitsAt(double distanceAhead) {
    if (samples.isEmpty) return null;
    if (distanceAhead <= samples.first.distanceAhead) {
      return (samples.first.leftEdge, samples.first.rightEdge);
    }
    if (distanceAhead >= samples.last.distanceAhead) return null;

    for (int i = 1; i < samples.length; i++) {
      if (samples[i].distanceAhead >= distanceAhead) {
        final DrivableSample a = samples[i - 1];
        final DrivableSample b = samples[i];
        final double span = b.distanceAhead - a.distanceAhead;
        final double t = span <= 0 ? 0 : (distanceAhead - a.distanceAhead) / span;
        return (
          lerpDouble(a.leftEdge, b.leftEdge, t),
          lerpDouble(a.rightEdge, b.rightEdge, t),
        );
      }
    }
    return null;
  }

  /// Is a metric point inside the corridor?
  bool contains(Vec2 point) {
    final (double, double)? lim = limitsAt(point.y);
    if (lim == null) return false;
    return point.x >= lim.$1 && point.x <= lim.$2;
  }

  /// Centre of the corridor at [distanceAhead] — the fallback path when there
  /// are no lane markings at all.
  double? centerAt(double distanceAhead) {
    final (double, double)? lim = limitsAt(distanceAhead);
    return lim == null ? null : (lim.$1 + lim.$2) / 2;
  }

  double? widthAt(double distanceAhead) {
    final (double, double)? lim = limitsAt(distanceAhead);
    return lim == null ? null : lim.$2 - lim.$1;
  }

  /// Project the corridor back into image space for the translucent overlay.
  List<(PixelPoint, PixelPoint)> toImageBand(CameraCalibration calibration) {
    final List<(PixelPoint, PixelPoint)> band = <(PixelPoint, PixelPoint)>[];
    for (final DrivableSample s in samples) {
      final PixelPoint? l = calibration
          .projectGroundToImage(Vec2(s.leftEdge, s.distanceAhead));
      final PixelPoint? r = calibration
          .projectGroundToImage(Vec2(s.rightEdge, s.distanceAhead));
      if (l != null && r != null) band.add((l, r));
    }
    return band;
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'source': source,
        'conf': double.parse(confidence.toStringAsFixed(3)),
        'samples': <List<double>>[
          for (final DrivableSample s in samples)
            <double>[
              double.parse(s.distanceAhead.toStringAsFixed(2)),
              double.parse(s.leftEdge.toStringAsFixed(2)),
              double.parse(s.rightEdge.toStringAsFixed(2)),
              double.parse(s.confidence.toStringAsFixed(3)),
            ],
        ],
      };

  static DrivableArea fromJson(
    Map<String, dynamic> j, {
    required int frameId,
    required int timestampMicros,
  }) =>
      DrivableArea(
        samples: <DrivableSample>[
          for (final dynamic s in (j['samples'] as List<dynamic>))
            DrivableSample(
              distanceAhead: ((s as List<dynamic>)[0] as num).toDouble(),
              leftEdge: (s[1] as num).toDouble(),
              rightEdge: (s[2] as num).toDouble(),
              confidence: (s[3] as num).toDouble(),
            ),
        ],
        confidence: (j['conf'] as num).toDouble(),
        frameId: frameId,
        timestampMicros: timestampMicros,
        source: j['source'] as String? ?? 'replay',
      );

  @override
  String toString() => 'DrivableArea(${samples.length} samples, '
      '${maxRangeMeters.toStringAsFixed(0)}m, '
      '${(confidence * 100).round()}%, $source)';
}

/// The drivable corridor at one distance ahead.
class DrivableSample {
  const DrivableSample({
    required this.distanceAhead,
    required this.leftEdge,
    required this.rightEdge,
    required this.confidence,
  });

  /// Metres ahead of the camera.
  final double distanceAhead;

  /// Metres right of the vehicle centreline (so normally negative).
  final double leftEdge;

  /// Metres right of the vehicle centreline.
  final double rightEdge;

  final double confidence;

  double get width => rightEdge - leftEdge;
  double get center => (leftEdge + rightEdge) / 2;

  bool get isPlausible =>
      width > 1.8 && width < 25 && leftEdge < 1.0 && rightEdge > -1.0;

  @override
  String toString() => '${distanceAhead.toStringAsFixed(0)}m: '
      '[${leftEdge.toStringAsFixed(1)}, ${rightEdge.toStringAsFixed(1)}]';
}

/// A detected road edge (curb, gutter, verge, barrier line) in metric space.
class RoadEdge {
  const RoadEdge({
    required this.isLeft,
    required this.curve,
    required this.confidence,
    required this.kind,
    required this.minRangeMeters,
    required this.maxRangeMeters,
  });

  final bool isLeft;
  final Polynomial curve;
  final double confidence;
  final RoadEdgeKind kind;
  final double minRangeMeters;
  final double maxRangeMeters;

  double? lateralAt(double distanceAhead) {
    if (distanceAhead < minRangeMeters - 1 ||
        distanceAhead > maxRangeMeters + 2) {
      return null;
    }
    return curve.evaluate(distanceAhead);
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'side': isLeft ? 'L' : 'R',
        'curve': curve.toJson(),
        'conf': double.parse(confidence.toStringAsFixed(3)),
        'kind': kind.name,
        'minRange': double.parse(minRangeMeters.toStringAsFixed(2)),
        'maxRange': double.parse(maxRangeMeters.toStringAsFixed(2)),
      };

  static RoadEdge fromJson(Map<String, dynamic> j) => RoadEdge(
        isLeft: j['side'] == 'L',
        curve: Polynomial.fromJson(j['curve'] as List<dynamic>),
        confidence: (j['conf'] as num).toDouble(),
        kind: RoadEdgeKind.values.firstWhere(
          (RoadEdgeKind k) => k.name == j['kind'],
          orElse: () => RoadEdgeKind.unknown,
        ),
        minRangeMeters: (j['minRange'] as num).toDouble(),
        maxRangeMeters: (j['maxRange'] as num).toDouble(),
      );

  @override
  String toString() => '${isLeft ? 'Left' : 'Right'} ${kind.name} edge '
      '${(confidence * 100).round()}%';
}

enum RoadEdgeKind {
  curb,
  barrier,
  verge,
  parkedVehicles,
  unknown;
}

/// Shared helper: how far ahead segmentation can be trusted given the
/// calibration. Beyond ~60 m a 160-wide segmentation grid resolves less than
/// one pixel per metre, so claims about the corridor there are meaningless.
double segmentationUsefulRangeMeters(
  CameraCalibration calibration,
  int segHeight,
) {
  final double rowsBelowHorizon =
      segHeight * (1 - calibration.horizonYNormalized);
  if (rowsBelowHorizon < 4) return 0;
  // Distance at which one segmentation row spans more than 2 m.
  for (double d = 8; d < 120; d += 2) {
    final PixelPoint? p =
        calibration.projectGroundToImage(Vec2(0, d));
    final PixelPoint? p2 =
        calibration.projectGroundToImage(Vec2(0, d + 2));
    if (p == null || p2 == null) return d;
    final double rowsPerTwoMetres =
        (p.v - p2.v).abs() * segHeight / calibration.imageHeight;
    if (rowsPerTwoMetres < 1.0) return d;
  }
  return 120;
}

/// Numerically stable softmax over a slice, used when decoding segmentation
/// logits.
void softmaxInPlace(Float32List values, int offset, int count) {
  double maxV = -double.infinity;
  for (int i = 0; i < count; i++) {
    final double v = values[offset + i];
    if (v > maxV) maxV = v;
  }
  double sum = 0;
  for (int i = 0; i < count; i++) {
    final double e = math.exp(values[offset + i] - maxV);
    values[offset + i] = e;
    sum += e;
  }
  if (sum <= 0) return;
  for (int i = 0; i < count; i++) {
    values[offset + i] /= sum;
  }
}
