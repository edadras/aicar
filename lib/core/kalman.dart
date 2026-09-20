import 'dart:math' as math;
import 'dart:typed_data';

/// Small dense linear-algebra helpers. A full matrix library would be overkill
/// for the 4x4 filters this project runs, and keeping it here avoids a
/// dependency in the hot tracking loop.
class Matrix {
  Matrix(this.rows, this.cols) : data = Float64List(rows * cols);

  Matrix.fromList(this.rows, this.cols, List<double> values)
      : data = Float64List.fromList(values) {
    assert(values.length == rows * cols);
  }

  factory Matrix.identity(int n) {
    final Matrix m = Matrix(n, n);
    for (int i = 0; i < n; i++) {
      m.set(i, i, 1);
    }
    return m;
  }

  factory Matrix.diagonal(List<double> d) {
    final Matrix m = Matrix(d.length, d.length);
    for (int i = 0; i < d.length; i++) {
      m.set(i, i, d[i]);
    }
    return m;
  }

  final int rows;
  final int cols;
  final Float64List data;

  double get(int r, int c) => data[r * cols + c];
  void set(int r, int c, double v) => data[r * cols + c] = v;

  Matrix copy() => Matrix.fromList(rows, cols, data.toList());

  Matrix multiply(Matrix o) {
    assert(cols == o.rows);
    final Matrix out = Matrix(rows, o.cols);
    for (int i = 0; i < rows; i++) {
      for (int k = 0; k < cols; k++) {
        final double a = get(i, k);
        if (a == 0) continue;
        for (int j = 0; j < o.cols; j++) {
          out.data[i * o.cols + j] += a * o.get(k, j);
        }
      }
    }
    return out;
  }

  Matrix transpose() {
    final Matrix out = Matrix(cols, rows);
    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        out.set(j, i, get(i, j));
      }
    }
    return out;
  }

  Matrix operator +(Matrix o) {
    assert(rows == o.rows && cols == o.cols);
    final Matrix out = Matrix(rows, cols);
    for (int i = 0; i < data.length; i++) {
      out.data[i] = data[i] + o.data[i];
    }
    return out;
  }

  Matrix operator -(Matrix o) {
    assert(rows == o.rows && cols == o.cols);
    final Matrix out = Matrix(rows, cols);
    for (int i = 0; i < data.length; i++) {
      out.data[i] = data[i] - o.data[i];
    }
    return out;
  }

  /// Gauss–Jordan inverse. Returns `null` for a singular matrix; callers treat
  /// that as "skip this update" rather than crashing the pipeline.
  Matrix? inverse() {
    assert(rows == cols);
    final int n = rows;
    final Matrix a = copy();
    final Matrix inv = Matrix.identity(n);
    for (int col = 0; col < n; col++) {
      int pivot = col;
      double best = a.get(col, col).abs();
      for (int r = col + 1; r < n; r++) {
        final double v = a.get(r, col).abs();
        if (v > best) {
          best = v;
          pivot = r;
        }
      }
      if (best < 1e-12) return null;
      if (pivot != col) {
        a._swapRows(col, pivot);
        inv._swapRows(col, pivot);
      }
      final double d = a.get(col, col);
      for (int c = 0; c < n; c++) {
        a.set(col, c, a.get(col, c) / d);
        inv.set(col, c, inv.get(col, c) / d);
      }
      for (int r = 0; r < n; r++) {
        if (r == col) continue;
        final double f = a.get(r, col);
        if (f == 0) continue;
        for (int c = 0; c < n; c++) {
          a.set(r, c, a.get(r, c) - f * a.get(col, c));
          inv.set(r, c, inv.get(r, c) - f * inv.get(col, c));
        }
      }
    }
    return inv;
  }

  void _swapRows(int r1, int r2) {
    for (int c = 0; c < cols; c++) {
      final double t = get(r1, c);
      set(r1, c, get(r2, c));
      set(r2, c, t);
    }
  }

  @override
  String toString() {
    final StringBuffer b = StringBuffer();
    for (int i = 0; i < rows; i++) {
      b.write('[');
      for (int j = 0; j < cols; j++) {
        b.write(get(i, j).toStringAsFixed(3));
        if (j < cols - 1) b.write(', ');
      }
      b.writeln(']');
    }
    return b.toString();
  }
}

/// Generic discrete linear Kalman filter.
///
/// Used by the object tracker (constant-velocity box model) and by the ego
/// motion estimator (speed/heading fusion of GPS and IMU).
class KalmanFilter {
  KalmanFilter({
    required this.stateSize,
    required this.measurementSize,
    required Matrix initialState,
    required Matrix initialCovariance,
  })  : x = initialState.copy(),
        p = initialCovariance.copy() {
    assert(initialState.rows == stateSize && initialState.cols == 1);
    assert(initialCovariance.rows == stateSize);
  }

  final int stateSize;
  final int measurementSize;

  /// State estimate, `stateSize x 1`.
  Matrix x;

  /// Estimate covariance, `stateSize x stateSize`.
  Matrix p;

  /// Predict with transition [f] and process noise [q].
  void predict(Matrix f, Matrix q) {
    x = f.multiply(x);
    p = f.multiply(p).multiply(f.transpose()) + q;
  }

  /// Correct with measurement [z], observation matrix [h] and noise [r].
  /// Returns `false` when the innovation covariance is singular, leaving the
  /// state untouched.
  bool update(Matrix z, Matrix h, Matrix r) {
    final Matrix y = z - h.multiply(x); // innovation
    final Matrix ht = h.transpose();
    final Matrix s = h.multiply(p).multiply(ht) + r;
    final Matrix? sInv = s.inverse();
    if (sInv == null) return false;
    final Matrix k = p.multiply(ht).multiply(sInv);
    x = x + k.multiply(y);
    final Matrix i = Matrix.identity(stateSize);
    final Matrix ikh = i - k.multiply(h);
    // Joseph form keeps P symmetric positive-definite over long runs.
    p = ikh.multiply(p).multiply(ikh.transpose()) +
        k.multiply(r).multiply(k.transpose());
    return true;
  }

  /// Squared Mahalanobis distance of a measurement — the gating statistic that
  /// stops the tracker from associating a detection that is far too far away.
  double mahalanobisSquared(Matrix z, Matrix h, Matrix r) {
    final Matrix y = z - h.multiply(x);
    final Matrix s = h.multiply(p).multiply(h.transpose()) + r;
    final Matrix? sInv = s.inverse();
    if (sInv == null) return double.infinity;
    final Matrix d = y.transpose().multiply(sInv).multiply(y);
    return d.get(0, 0);
  }

  double state(int i) => x.get(i, 0);

  double uncertainty(int i) => math.sqrt(math.max(0, p.get(i, i)));
}

/// First-order low-pass used to smooth noisy scalar signals (steering command,
/// throttle, depth confidence) without the latency of a long moving average.
class ExponentialFilter {
  ExponentialFilter({required this.timeConstantSeconds, double? initial})
      : _value = initial;

  final double timeConstantSeconds;
  double? _value;

  double? get value => _value;
  bool get hasValue => _value != null;

  double update(double sample, double dtSeconds) {
    if (_value == null || dtSeconds <= 0) {
      _value = sample;
      return sample;
    }
    final double alpha = 1 - math.exp(-dtSeconds / timeConstantSeconds);
    _value = _value! + alpha * (sample - _value!);
    return _value!;
  }

  void reset([double? initial]) => _value = initial;
}
