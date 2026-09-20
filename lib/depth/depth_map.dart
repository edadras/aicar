import 'dart:math' as math;
import 'dart:typed_data';

import '../core/geometry.dart';

/// What the numbers in a [DepthMap] actually mean.
enum DepthScale {
  /// Metres. Only a metric-depth network or the geometric estimator produces
  /// this directly.
  metric,

  /// Monotonic in inverse distance, arbitrary scale (MiDaS-style). Must be
  /// fitted to metric scale before it means anything — see
  /// [DepthMap.fitToMetric].
  relativeInverse,
}

/// A dense per-pixel depth image.
///
/// The Galaxy S23 has no LiDAR and no usable stereo baseline for road-scale
/// distances, so every dense depth value here is a *monocular estimate*. The
/// class therefore refuses to pretend: a relative map cannot answer
/// [distanceAt] until it has been fitted against a metric reference, and
/// [confidenceAt] falls off in exactly the regions where monocular depth is
/// known to be unreliable.
class DepthMap {
  DepthMap({
    required this.width,
    required this.height,
    required this.values,
    required this.scale,
    required this.frameId,
    required this.timestampMicros,
    this.globalConfidence = 0.5,
    this.metricGain = 1.0,
    this.metricBias = 0.0,
    this.isFitted = false,
  }) : assert(values.length == width * height);

  /// A placeholder map used when no depth network is installed. Every query
  /// returns `null`, which forces the fusion layer onto its geometric cues.
  factory DepthMap.unavailable({
    required int frameId,
    required int timestampMicros,
  }) =>
      DepthMap(
        width: 1,
        height: 1,
        values: Float32List(1),
        scale: DepthScale.relativeInverse,
        frameId: frameId,
        timestampMicros: timestampMicros,
        globalConfidence: 0,
        isFitted: false,
      );

  final int width;
  final int height;
  final Float32List values;
  final DepthScale scale;
  final int frameId;
  final int timestampMicros;

  /// How much the map as a whole is trusted, before per-pixel modulation.
  final double globalConfidence;

  /// Affine fit `metric = gain / (raw + bias)` for relative maps, or
  /// `metric = gain * raw + bias` for metric maps.
  final double metricGain;
  final double metricBias;

  /// Whether [metricGain]/[metricBias] were actually estimated from data.
  final bool isFitted;

  bool get isUsable => globalConfidence > 0 && width > 1 && height > 1;

  double rawAt(double nx, double ny) {
    final int x = (nx * width).round().clamp(0, width - 1);
    final int y = (ny * height).round().clamp(0, height - 1);
    return values[y * width + x];
  }

  /// Median raw value over a normalised region. The median rejects the
  /// halo of background pixels that a bounding box inevitably includes.
  double? rawMedianInBox(BoundingBox box, {int maxSamples = 256}) {
    final int x0 = (box.left * width).floor().clamp(0, width - 1);
    final int x1 = (box.right * width).ceil().clamp(x0 + 1, width);
    final int y0 = (box.top * height).floor().clamp(0, height - 1);
    final int y1 = (box.bottom * height).ceil().clamp(y0 + 1, height);

    final int w = x1 - x0;
    final int h = y1 - y0;
    if (w <= 0 || h <= 0) return null;

    // Sample the central half of the box: the outer band is mostly
    // background, whatever the detector's box quality.
    final int ix0 = x0 + w ~/ 4;
    final int ix1 = x1 - w ~/ 4;
    final int iy0 = y0 + h ~/ 4;
    final int iy1 = y1 - h ~/ 4;

    final int area = math.max(1, (ix1 - ix0) * (iy1 - iy0));
    final int stride = math.max(1, math.sqrt(area / maxSamples).ceil());

    final List<double> samples = <double>[];
    for (int y = iy0; y < iy1; y += stride) {
      for (int x = ix0; x < ix1; x += stride) {
        final double v = values[y * width + x];
        if (v.isFinite) samples.add(v);
      }
    }
    if (samples.isEmpty) return null;
    samples.sort();
    return samples[samples.length ~/ 2];
  }

  /// Metric distance at a normalised image point, or `null` when the map
  /// cannot honestly be converted.
  double? distanceAt(double nx, double ny) {
    if (!isUsable) return null;
    final double raw = rawAt(nx, ny);
    return _toMetric(raw);
  }

  double? distanceInBox(BoundingBox box) {
    if (!isUsable) return null;
    final double? raw = rawMedianInBox(box);
    return raw == null ? null : _toMetric(raw);
  }

  double? _toMetric(double raw) {
    if (!raw.isFinite) return null;
    switch (scale) {
      case DepthScale.metric:
        final double d = metricGain * raw + metricBias;
        return d.isFinite && d > 0 ? d : null;
      case DepthScale.relativeInverse:
        if (!isFitted) return null; // scale unknown — say so
        final double denom = raw + metricBias;
        if (denom.abs() < 1e-6) return null;
        final double d = metricGain / denom;
        return d.isFinite && d > 0 ? d : null;
    }
  }

  /// Per-pixel confidence modifier.
  ///
  /// Monocular depth is least reliable near the top of the frame (sky and
  /// far field, where inverse depth saturates) and at the image border (where
  /// the receptive field is truncated). Both are encoded here rather than
  /// being re-derived by every consumer.
  double confidenceAt(double nx, double ny) {
    if (!isUsable) return 0;
    double c = globalConfidence;

    // Vertical: the far field compresses into a few quantisation levels.
    if (ny < 0.35) {
      c *= clampDouble(0.3 + ny / 0.35 * 0.7, 0.2, 1.0);
    }
    // Border falloff.
    final double edge = math.min(
      math.min(nx, 1 - nx),
      math.min(ny, 1 - ny),
    );
    if (edge < 0.06) c *= clampDouble(0.5 + edge / 0.12, 0.4, 1.0);

    if (!isFitted && scale == DepthScale.relativeInverse) c *= 0.0;
    return clampDouble(c, 0, 1);
  }

  /// Re-scale a relative map using metric reference points.
  ///
  /// Monocular networks are accurate *up to an unknown affine transform of
  /// inverse depth*. The ground plane gives us reliable metric anchors for the
  /// road surface, so a least-squares fit of `1/metric = a*raw + b` recovers
  /// the missing scale. This is what turns a pretty picture into a usable
  /// distance, and refusing to guess when there are too few anchors is what
  /// keeps it honest.
  DepthMap fitToMetric(List<DepthMetricAnchor> anchors) {
    if (scale == DepthScale.metric) return this;
    final List<DepthMetricAnchor> usable = anchors
        .where((DepthMetricAnchor a) =>
            a.metricDistance > 1.0 &&
            a.metricDistance < 120 &&
            a.rawValue.isFinite)
        .toList();
    if (usable.length < 3) return this;

    // Fit 1/d = a*raw + b by weighted least squares.
    final List<double> xs = <double>[];
    final List<double> ys = <double>[];
    final List<double> ws = <double>[];
    for (final DepthMetricAnchor a in usable) {
      xs.add(a.rawValue);
      ys.add(1.0 / a.metricDistance);
      ws.add(a.weight);
    }
    final Polynomial? fit = fitPolynomial(xs, ys, degree: 1, weights: ws);
    if (fit == null) return this;

    final double a = fit.coefficients.length > 1 ? fit.coefficients[1] : 0;
    final double b = fit.coefficients[0];
    if (a.abs() < 1e-9) return this;

    // metric = 1/(a*raw + b) = (1/a) / (raw + b/a)
    final double gain = 1.0 / a;
    final double bias = b / a;

    // Residuals tell us how well the network agrees with the geometry; that
    // is the most informative confidence signal available for a depth map.
    double sse = 0;
    double sst = 0;
    final double meanY = ys.reduce((double p, double q) => p + q) / ys.length;
    for (int i = 0; i < xs.length; i++) {
      final double pred = a * xs[i] + b;
      sse += (pred - ys[i]) * (pred - ys[i]);
      sst += (ys[i] - meanY) * (ys[i] - meanY);
    }
    final double r2 = sst < 1e-12 ? 0 : clampDouble(1 - sse / sst, 0, 1);

    return DepthMap(
      width: width,
      height: height,
      values: values,
      scale: scale,
      frameId: frameId,
      timestampMicros: timestampMicros,
      globalConfidence: clampDouble(0.25 + 0.65 * r2, 0, 0.9),
      metricGain: gain,
      metricBias: bias,
      isFitted: true,
    );
  }

  /// Downsample for the debug overlay. Full-resolution depth is far more data
  /// than a 200-pixel-wide preview needs.
  Uint8List toPreviewBytes(int outWidth, int outHeight) {
    final Uint8List out = Uint8List(outWidth * outHeight);
    if (!isUsable) return out;

    double minV = double.infinity;
    double maxV = -double.infinity;
    for (final double v in values) {
      if (!v.isFinite) continue;
      if (v < minV) minV = v;
      if (v > maxV) maxV = v;
    }
    final double span = (maxV - minV).abs() < 1e-9 ? 1 : maxV - minV;

    for (int y = 0; y < outHeight; y++) {
      final int sy = (y * height ~/ outHeight).clamp(0, height - 1);
      for (int x = 0; x < outWidth; x++) {
        final int sx = (x * width ~/ outWidth).clamp(0, width - 1);
        final double v = values[sy * width + sx];
        out[y * outWidth + x] =
            v.isFinite ? (((v - minV) / span) * 255).round().clamp(0, 255) : 0;
      }
    }
    return out;
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'w': width,
        'h': height,
        'scale': scale.name,
        'conf': double.parse(globalConfidence.toStringAsFixed(3)),
        'gain': metricGain,
        'bias': metricBias,
        'fitted': isFitted,
      };
}

/// One (raw depth value, known metric distance) pair used to fit a relative
/// depth map onto metric scale.
class DepthMetricAnchor {
  const DepthMetricAnchor({
    required this.rawValue,
    required this.metricDistance,
    this.weight = 1.0,
    this.source = 'ground-plane',
  });

  final double rawValue;
  final double metricDistance;
  final double weight;
  final String source;
}
