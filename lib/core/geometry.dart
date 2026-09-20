import 'dart:math' as math;
import 'dart:typed_data';

/// Minimal 2-D vector used across perception, planning and simulation.
///
/// Vehicle frame convention used everywhere in this project:
///   * `x` — metres to the **right** of the ego vehicle centreline.
///   * `y` — metres **forward** along the ego heading.
/// Angles are radians, positive = clockwise seen from above (i.e. to the
/// right), matching the steering-angle sign convention (`+` = right).
class Vec2 {
  const Vec2(this.x, this.y);
  const Vec2.zero() : x = 0, y = 0;

  final double x;
  final double y;

  double get length => math.sqrt(x * x + y * y);
  double get lengthSquared => x * x + y * y;

  /// Heading of the vector, 0 = straight ahead (+y), positive to the right.
  double get heading => math.atan2(x, y);

  Vec2 operator +(Vec2 o) => Vec2(x + o.x, y + o.y);
  Vec2 operator -(Vec2 o) => Vec2(x - o.x, y - o.y);
  Vec2 operator *(double s) => Vec2(x * s, y * s);
  Vec2 operator /(double s) => Vec2(x / s, y / s);
  Vec2 operator -() => Vec2(-x, -y);

  double dot(Vec2 o) => x * o.x + y * o.y;
  double cross(Vec2 o) => x * o.y - y * o.x;
  double distanceTo(Vec2 o) => (this - o).length;

  Vec2 normalized() {
    final double l = length;
    return l < 1e-9 ? const Vec2.zero() : Vec2(x / l, y / l);
  }

  /// Rotate by [radians] using the project convention (positive = to the
  /// right, i.e. clockwise in the (x-right, y-forward) plane).
  Vec2 rotated(double radians) {
    final double c = math.cos(radians);
    final double s = math.sin(radians);
    return Vec2(x * c + y * s, -x * s + y * c);
  }

  Vec2 lerp(Vec2 o, double t) => Vec2(x + (o.x - x) * t, y + (o.y - y) * t);

  Map<String, double> toJson() => <String, double>{'x': x, 'y': y};

  static Vec2 fromJson(Map<String, dynamic> j) =>
      Vec2((j['x'] as num).toDouble(), (j['y'] as num).toDouble());

  @override
  String toString() => 'Vec2(${x.toStringAsFixed(2)}, ${y.toStringAsFixed(2)})';

  @override
  bool operator ==(Object other) =>
      other is Vec2 && other.x == x && other.y == y;

  @override
  int get hashCode => Object.hash(x, y);
}

/// Pixel-space point. Kept separate from [Vec2] so that image coordinates and
/// metric vehicle coordinates can never be mixed up by accident.
class PixelPoint {
  const PixelPoint(this.u, this.v);
  final double u;
  final double v;

  PixelPoint operator +(PixelPoint o) => PixelPoint(u + o.u, v + o.v);
  PixelPoint operator -(PixelPoint o) => PixelPoint(u - o.u, v - o.v);
  PixelPoint operator *(double s) => PixelPoint(u * s, v * s);

  double distanceTo(PixelPoint o) {
    final double du = u - o.u;
    final double dv = v - o.v;
    return math.sqrt(du * du + dv * dv);
  }

  Map<String, double> toJson() => <String, double>{'u': u, 'v': v};

  static PixelPoint fromJson(Map<String, dynamic> j) =>
      PixelPoint((j['u'] as num).toDouble(), (j['v'] as num).toDouble());

  @override
  String toString() => 'PixelPoint(${u.toStringAsFixed(1)}, ${v.toStringAsFixed(1)})';
}

/// Axis-aligned bounding box in **normalised** image coordinates (0..1), so a
/// box survives a change of inference resolution unchanged.
class BoundingBox {
  const BoundingBox({
    required this.left,
    required this.top,
    required this.width,
    required this.height,
  });

  factory BoundingBox.fromLTRB(double l, double t, double r, double b) =>
      BoundingBox(left: l, top: t, width: r - l, height: b - t);

  /// Build from absolute pixel coordinates and normalise by the image size.
  factory BoundingBox.fromPixels({
    required double left,
    required double top,
    required double right,
    required double bottom,
    required int imageWidth,
    required int imageHeight,
  }) {
    return BoundingBox(
      left: left / imageWidth,
      top: top / imageHeight,
      width: (right - left) / imageWidth,
      height: (bottom - top) / imageHeight,
    );
  }

  final double left;
  final double top;
  final double width;
  final double height;

  double get right => left + width;
  double get bottom => top + height;
  double get centerX => left + width / 2;
  double get centerY => top + height / 2;
  double get area => width <= 0 || height <= 0 ? 0 : width * height;
  double get aspectRatio => height <= 1e-9 ? 0 : width / height;

  /// Mid-point of the bottom edge — the contact patch with the road surface,
  /// which is what the ground-plane depth model needs.
  PixelPoint get groundContactPoint => PixelPoint(centerX, bottom);

  BoundingBox intersect(BoundingBox o) {
    final double l = math.max(left, o.left);
    final double t = math.max(top, o.top);
    final double r = math.min(right, o.right);
    final double b = math.min(bottom, o.bottom);
    if (r <= l || b <= t) {
      return const BoundingBox(left: 0, top: 0, width: 0, height: 0);
    }
    return BoundingBox.fromLTRB(l, t, r, b);
  }

  /// Intersection over union, the standard detection/tracking overlap metric.
  double iou(BoundingBox o) {
    final double inter = intersect(o).area;
    final double union = area + o.area - inter;
    return union <= 1e-12 ? 0 : inter / union;
  }

  BoundingBox clampToUnit() {
    final double l = _clamp01(left);
    final double t = _clamp01(top);
    final double r = _clamp01(right);
    final double b = _clamp01(bottom);
    return BoundingBox.fromLTRB(l, t, math.max(l, r), math.max(t, b));
  }

  BoundingBox lerp(BoundingBox o, double t) => BoundingBox(
        left: left + (o.left - left) * t,
        top: top + (o.top - top) * t,
        width: width + (o.width - width) * t,
        height: height + (o.height - height) * t,
      );

  Map<String, double> toJson() => <String, double>{
        'l': left,
        't': top,
        'w': width,
        'h': height,
      };

  static BoundingBox fromJson(Map<String, dynamic> j) => BoundingBox(
        left: (j['l'] as num).toDouble(),
        top: (j['t'] as num).toDouble(),
        width: (j['w'] as num).toDouble(),
        height: (j['h'] as num).toDouble(),
      );

  @override
  String toString() =>
      'BBox(${left.toStringAsFixed(3)},${top.toStringAsFixed(3)} '
      '${width.toStringAsFixed(3)}x${height.toStringAsFixed(3)})';
}

double _clamp01(double v) => v < 0 ? 0 : (v > 1 ? 1 : v);

/// Clamp helper that is `double`-typed (avoids the `num` return of
/// `num.clamp`, which needs a cast at every call site).
double clampDouble(double v, double lo, double hi) =>
    v < lo ? lo : (v > hi ? hi : v);

double lerpDouble(double a, double b, double t) => a + (b - a) * t;

/// Wrap an angle into `(-pi, pi]`.
double normalizeAngle(double radians) {
  double a = radians;
  while (a <= -math.pi) {
    a += 2 * math.pi;
  }
  while (a > math.pi) {
    a -= 2 * math.pi;
  }
  return a;
}

double degToRad(double deg) => deg * math.pi / 180.0;
double radToDeg(double rad) => rad * 180.0 / math.pi;

/// A polynomial `y = c0 + c1*x + c2*x^2 + ...` used to model lane boundaries
/// and the planned path. Degree is whatever [coefficients] implies.
class Polynomial {
  const Polynomial(this.coefficients);

  const Polynomial.zero() : coefficients = const <double>[0.0];

  final List<double> coefficients;

  int get degree => coefficients.length - 1;

  double evaluate(double x) {
    // Horner's scheme.
    double acc = 0;
    for (int i = coefficients.length - 1; i >= 0; i--) {
      acc = acc * x + coefficients[i];
    }
    return acc;
  }

  double derivative(double x) {
    double acc = 0;
    for (int i = coefficients.length - 1; i >= 1; i--) {
      acc = acc * x + coefficients[i] * i;
    }
    return acc;
  }

  double secondDerivative(double x) {
    double acc = 0;
    for (int i = coefficients.length - 1; i >= 2; i--) {
      acc = acc * x + coefficients[i] * i * (i - 1);
    }
    return acc;
  }

  /// Signed curvature of the curve `y = f(x)` at [x], in 1/m when the
  /// polynomial is expressed in metres.
  double curvatureAt(double x) {
    final double d1 = derivative(x);
    final double d2 = secondDerivative(x);
    final double denom = math.pow(1 + d1 * d1, 1.5).toDouble();
    return denom < 1e-9 ? 0 : d2 / denom;
  }

  Polynomial scaled(double k) =>
      Polynomial(coefficients.map((double c) => c * k).toList());

  Polynomial lerp(Polynomial o, double t) {
    final int n = math.max(coefficients.length, o.coefficients.length);
    return Polynomial(<double>[
      for (int i = 0; i < n; i++)
        lerpDouble(
          i < coefficients.length ? coefficients[i] : 0,
          i < o.coefficients.length ? o.coefficients[i] : 0,
          t,
        ),
    ]);
  }

  List<double> toJson() => coefficients;

  static Polynomial fromJson(List<dynamic> j) =>
      Polynomial(j.map((dynamic e) => (e as num).toDouble()).toList());

  @override
  String toString() =>
      'Poly[${coefficients.map((double c) => c.toStringAsExponential(2)).join(', ')}]';
}

/// Weighted least-squares polynomial fit solved with Gaussian elimination on
/// the normal equations. Returns `null` when the system is rank deficient,
/// which is the honest answer when there are not enough distinct samples.
///
/// [degree] 2 (quadratic) is what the lane and path models use: it captures
/// the constant offset, the heading and the curvature of a road.
Polynomial? fitPolynomial(
  List<double> xs,
  List<double> ys, {
  int degree = 2,
  List<double>? weights,
}) {
  assert(xs.length == ys.length);
  final int n = xs.length;
  final int m = degree + 1;
  if (n < m) return null;

  // Normal equations A^T W A c = A^T W y, accumulated directly.
  final Float64List ata = Float64List(m * m);
  final Float64List atb = Float64List(m);
  final Float64List powers = Float64List(2 * degree + 1);

  for (int k = 0; k < n; k++) {
    final double x = xs[k];
    final double y = ys[k];
    final double w = weights == null ? 1.0 : weights[k];
    if (w <= 0) continue;
    powers[0] = 1;
    for (int p = 1; p <= 2 * degree; p++) {
      powers[p] = powers[p - 1] * x;
    }
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < m; j++) {
        ata[i * m + j] += w * powers[i + j];
      }
      atb[i] += w * powers[i] * y;
    }
  }

  final List<double>? sol = _solveLinearSystem(ata, atb, m);
  if (sol == null) return null;
  return Polynomial(sol);
}

/// Gauss–Jordan with partial pivoting. `a` is row-major `n x n`.
List<double>? _solveLinearSystem(Float64List a, Float64List b, int n) {
  final Float64List m = Float64List.fromList(a);
  final Float64List v = Float64List.fromList(b);
  for (int col = 0; col < n; col++) {
    int pivot = col;
    double best = m[col * n + col].abs();
    for (int r = col + 1; r < n; r++) {
      final double val = m[r * n + col].abs();
      if (val > best) {
        best = val;
        pivot = r;
      }
    }
    if (best < 1e-12) return null; // singular / rank deficient
    if (pivot != col) {
      for (int c = 0; c < n; c++) {
        final double t = m[col * n + c];
        m[col * n + c] = m[pivot * n + c];
        m[pivot * n + c] = t;
      }
      final double t = v[col];
      v[col] = v[pivot];
      v[pivot] = t;
    }
    final double d = m[col * n + col];
    for (int c = col; c < n; c++) {
      m[col * n + c] /= d;
    }
    v[col] /= d;
    for (int r = 0; r < n; r++) {
      if (r == col) continue;
      final double f = m[r * n + col];
      if (f == 0) continue;
      for (int c = col; c < n; c++) {
        m[r * n + c] -= f * m[col * n + c];
      }
      v[r] -= f * v[col];
    }
  }
  return v.toList();
}

/// RANSAC wrapper around [fitPolynomial]. Lane pixels are noisy and contain
/// outliers from shadows, patches and other cars, so a plain least-squares fit
/// is not good enough on real footage.
Polynomial? fitPolynomialRansac(
  List<double> xs,
  List<double> ys, {
  int degree = 2,
  int iterations = 40,
  double inlierThreshold = 0.05,
  int? seed,
}) {
  final int n = xs.length;
  final int minSamples = degree + 1;
  if (n < minSamples) return null;
  if (n <= minSamples + 1) return fitPolynomial(xs, ys, degree: degree);

  final math.Random rng = math.Random(seed ?? 0xA1CA8);
  Polynomial? best;
  int bestInliers = -1;

  final List<double> sx = List<double>.filled(minSamples, 0);
  final List<double> sy = List<double>.filled(minSamples, 0);

  for (int it = 0; it < iterations; it++) {
    for (int s = 0; s < minSamples; s++) {
      final int idx = rng.nextInt(n);
      sx[s] = xs[idx];
      sy[s] = ys[idx];
    }
    final Polynomial? candidate = fitPolynomial(sx, sy, degree: degree);
    if (candidate == null) continue;
    int inliers = 0;
    for (int k = 0; k < n; k++) {
      if ((candidate.evaluate(xs[k]) - ys[k]).abs() <= inlierThreshold) {
        inliers++;
      }
    }
    if (inliers > bestInliers) {
      bestInliers = inliers;
      best = candidate;
    }
  }

  if (best == null) return fitPolynomial(xs, ys, degree: degree);

  // Refit on the consensus set for a lower-variance estimate.
  final List<double> ix = <double>[];
  final List<double> iy = <double>[];
  for (int k = 0; k < n; k++) {
    if ((best.evaluate(xs[k]) - ys[k]).abs() <= inlierThreshold) {
      ix.add(xs[k]);
      iy.add(ys[k]);
    }
  }
  if (ix.length < minSamples) return best;
  return fitPolynomial(ix, iy, degree: degree) ?? best;
}
