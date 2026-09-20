import 'dart:math' as math;

/// All pipeline stages timestamp their inputs and outputs from one monotonic
/// clock so that camera frames, IMU samples and GPS fixes can be aligned even
/// though they arrive from three different Android subsystems at three
/// different rates.
///
/// Wall-clock time is recorded separately, once per session, so that a
/// recording can still be related to real time without every sample paying for
/// a non-monotonic clock read.
class MonotonicClock {
  MonotonicClock() : _stopwatch = Stopwatch()..start() {
    _epochWallMicros = DateTime.now().microsecondsSinceEpoch;
  }

  /// Test seam: a clock that only advances when [advance] is called.
  MonotonicClock.manual()
      : _stopwatch = null,
        _manualMicros = 0 {
    _epochWallMicros = 0;
  }

  final Stopwatch? _stopwatch;
  int _manualMicros = 0;
  late final int _epochWallMicros;

  /// Microseconds since the clock was created.
  int get micros =>
      _stopwatch != null ? _stopwatch.elapsedMicroseconds : _manualMicros;

  double get seconds => micros / 1e6;

  /// Wall-clock time corresponding to a monotonic [micros] reading.
  DateTime wallClockFor(int monotonicMicros) => DateTime.fromMicrosecondsSinceEpoch(
        _epochWallMicros + monotonicMicros,
      );

  void advance(Duration d) {
    if (_stopwatch != null) {
      throw StateError('advance() is only valid on a manual clock');
    }
    _manualMicros += d.inMicroseconds;
  }
}

/// A value paired with the monotonic timestamp at which it was *measured*
/// (not the time it was received, which can lag by a frame or more).
class Stamped<T> {
  const Stamped(this.value, this.timestampMicros);

  final T value;
  final int timestampMicros;

  double get timestampSeconds => timestampMicros / 1e6;

  Stamped<R> map<R>(R Function(T) f) => Stamped<R>(f(value), timestampMicros);

  @override
  String toString() => 'Stamped($value @ ${timestampSeconds.toStringAsFixed(3)}s)';
}

/// Keeps a short history of stamped samples and interpolates between them.
///
/// This is how the pipeline answers "what was the vehicle's yaw rate at the
/// exact instant this camera frame was exposed?" — the IMU sample that is
/// closest in time is rarely the one that arrived alongside the frame.
class TimeAlignedBuffer<T> {
  TimeAlignedBuffer({
    required this.capacity,
    required T Function(T a, T b, double t) interpolate,
  }) : _interpolate = interpolate;

  final int capacity;
  final T Function(T a, T b, double t) _interpolate;
  final List<Stamped<T>> _samples = <Stamped<T>>[];

  int get length => _samples.length;
  bool get isEmpty => _samples.isEmpty;
  Stamped<T>? get latest => _samples.isEmpty ? null : _samples.last;
  List<Stamped<T>> get samples => List<Stamped<T>>.unmodifiable(_samples);

  void add(Stamped<T> sample) {
    // Samples can arrive slightly out of order across sensor threads; keep the
    // buffer sorted so the binary search below stays valid.
    if (_samples.isNotEmpty &&
        sample.timestampMicros < _samples.last.timestampMicros) {
      int i = _samples.length;
      while (i > 0 && _samples[i - 1].timestampMicros > sample.timestampMicros) {
        i--;
      }
      _samples.insert(i, sample);
    } else {
      _samples.add(sample);
    }
    while (_samples.length > capacity) {
      _samples.removeAt(0);
    }
  }

  /// Linear interpolation at [timestampMicros]. Returns `null` if the buffer
  /// is empty; clamps to the endpoints outside the covered interval rather
  /// than extrapolating, because extrapolated IMU data is worse than stale.
  T? sampleAt(int timestampMicros) {
    if (_samples.isEmpty) return null;
    if (_samples.length == 1) return _samples.first.value;
    if (timestampMicros <= _samples.first.timestampMicros) {
      return _samples.first.value;
    }
    if (timestampMicros >= _samples.last.timestampMicros) {
      return _samples.last.value;
    }

    int lo = 0;
    int hi = _samples.length - 1;
    while (hi - lo > 1) {
      final int mid = (lo + hi) >> 1;
      if (_samples[mid].timestampMicros <= timestampMicros) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    final Stamped<T> a = _samples[lo];
    final Stamped<T> b = _samples[hi];
    final int span = b.timestampMicros - a.timestampMicros;
    final double t = span <= 0
        ? 0.0
        : (timestampMicros - a.timestampMicros) / span;
    return _interpolate(a.value, b.value, t);
  }

  /// How far the newest sample is from [timestampMicros], in seconds. Used to
  /// discount confidence when a sensor has gone quiet.
  double stalenessSeconds(int timestampMicros) {
    if (_samples.isEmpty) return double.infinity;
    return math.max(0, timestampMicros - _samples.last.timestampMicros) / 1e6;
  }

  void clear() => _samples.clear();
}
