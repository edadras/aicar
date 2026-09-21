import '../core/logging.dart';
import '../sensors/ego_motion.dart';
import '../tracking/object_track.dart';

/// One measurement of how fast a stationary object approached us, against how
/// fast we were actually travelling.
class ClosureSample {
  const ClosureSample({
    required this.trackId,
    required this.meanDistanceMeters,
    required this.measuredClosureMps,
    required this.egoSpeedMps,
    required this.dtSeconds,
    required this.timestampMicros,
  });

  final int trackId;

  /// Mid-point of the interval, which is the distance this sample is
  /// evidence *about*.
  final double meanDistanceMeters;

  /// How fast the estimated distance actually shrank.
  final double measuredClosureMps;

  /// How fast the vehicle was moving, from the fused ego motion.
  final double egoSpeedMps;

  final double dtSeconds;
  final int timestampMicros;

  /// Estimated distance divided by true distance.
  ///
  /// The whole method in one line: for a stationary object, the distance to
  /// it must shrink at exactly the speed we are travelling. If the estimator
  /// reports `k` times the true distance, it reports the closure as `k` times
  /// the true closure too — so the ratio *is* the scale error, and it needs
  /// no tape measure, no cones and no surveyed target.
  double get scaleRatio => measuredClosureMps / egoSpeedMps;

  Map<String, dynamic> toJson() => <String, dynamic>{
        'track': trackId,
        'd': double.parse(meanDistanceMeters.toStringAsFixed(2)),
        'closure': double.parse(measuredClosureMps.toStringAsFixed(3)),
        'speed': double.parse(egoSpeedMps.toStringAsFixed(3)),
        'ratio': double.parse(scaleRatio.toStringAsFixed(4)),
        'ts': timestampMicros,
      };
}

/// Error statistics over one distance band.
class DistanceBandResult {
  const DistanceBandResult({
    required this.nearMeters,
    required this.farMeters,
    required this.sampleCount,
    required this.medianScale,
    required this.scaleSpread,
  });

  final double nearMeters;
  final double farMeters;
  final int sampleCount;

  /// Median of [ClosureSample.scaleRatio]. 1.0 means unbiased.
  final double medianScale;

  /// Median absolute deviation of the ratio, scaled to a standard-deviation
  /// equivalent. The random error, as opposed to the bias.
  final double scaleSpread;

  double get centreMeters => (nearMeters + farMeters) / 2;

  /// Systematic error in metres at the centre of this band.
  double get biasMeters => (medianScale - 1) * centreMeters;

  /// Random error in metres at the centre of this band.
  double get noiseMeters => scaleSpread * centreMeters;

  /// What a distance reading in this band is actually worth, ± metres.
  double get totalErrorMeters => biasMeters.abs() + noiseMeters;

  bool get isUsable => sampleCount >= 8;

  Map<String, dynamic> toJson() => <String, dynamic>{
        'near': nearMeters,
        'far': farMeters,
        'n': sampleCount,
        'scale': double.parse(medianScale.toStringAsFixed(4)),
        'spread': double.parse(scaleSpread.toStringAsFixed(4)),
        'biasM': double.parse(biasMeters.toStringAsFixed(2)),
        'noiseM': double.parse(noiseMeters.toStringAsFixed(2)),
      };

  @override
  String toString() => '${nearMeters.round()}-${farMeters.round()} m: '
      'n=$sampleCount scale=${medianScale.toStringAsFixed(3)} '
      '±${totalErrorMeters.toStringAsFixed(2)} m';
}

/// What the whole session says about distance accuracy.
class DistanceAccuracyReport {
  const DistanceAccuracyReport({
    required this.bands,
    required this.totalSamples,
    required this.overallScale,
    required this.overallSpread,
  });

  static const DistanceAccuracyReport empty = DistanceAccuracyReport(
    bands: <DistanceBandResult>[],
    totalSamples: 0,
    overallScale: 1,
    overallSpread: 0,
  );

  final List<DistanceBandResult> bands;
  final int totalSamples;
  final double overallScale;
  final double overallSpread;

  bool get isUsable => totalSamples >= 30;

  /// Error at a given distance, interpolated from the band that covers it.
  double? errorAt(double distanceMeters) {
    for (final DistanceBandResult b in bands) {
      if (!b.isUsable) continue;
      if (distanceMeters >= b.nearMeters && distanceMeters < b.farMeters) {
        return (b.medianScale - 1).abs() * distanceMeters +
            b.scaleSpread * distanceMeters;
      }
    }
    return null;
  }

  /// Does the scale error grow with distance?
  ///
  /// This is the diagnostic that says *which* calibration number is wrong. A
  /// camera-height error scales every distance by the same factor. A pitch
  /// error does not — it grows with distance, because distance from the
  /// ground plane goes as `h / tan(pitch + θ)` and a pitch bias moves the far
  /// field far more than the near field. So a flat profile means fix the
  /// height; a rising one means fix the pitch.
  String get diagnosis {
    final List<DistanceBandResult> usable =
        bands.where((DistanceBandResult b) => b.isUsable).toList();
    if (usable.length < 2) {
      return 'Not enough data yet to separate a height error from a pitch '
          'error.';
    }
    final double near = usable.first.medianScale;
    final double far = usable.last.medianScale;
    final double drift = (far - near).abs();

    if (drift < 0.03 && (near - 1).abs() < 0.03) {
      return 'Scale is flat and close to 1.0 across the range: the '
          'calibration is consistent with the measurements.';
    }
    if (drift < 0.03) {
      return 'Scale is off by a constant '
          '${((near - 1) * 100).toStringAsFixed(1)}% at every distance. That '
          'is the signature of a camera-height error — check the mounting '
          'height in Calibration.';
    }
    return 'Scale drifts from ${near.toStringAsFixed(3)} near to '
        '${far.toStringAsFixed(3)} far. That is the signature of a pitch '
        'error rather than a height error — re-run the horizon step in '
        'Calibration.';
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'n': totalSamples,
        'scale': double.parse(overallScale.toStringAsFixed(4)),
        'spread': double.parse(overallSpread.toStringAsFixed(4)),
        'bands': <Map<String, dynamic>>[
          for (final DistanceBandResult b in bands) b.toJson(),
        ],
      };

  @override
  String toString() => 'DistanceAccuracy(n=$totalSamples, '
      'scale=${overallScale.toStringAsFixed(3)})';
}

/// Measures how accurate the stack's distances actually are, while driving.
///
/// The claim "a car at 25 m reads to about a metre" is not something a
/// simulator can establish, and stating it without evidence is exactly the
/// kind of confident guess this project is built to avoid. This is the
/// instrument that settles it on a real road.
///
/// The method needs no props. For a **stationary** object, the distance to it
/// must shrink at exactly the speed the vehicle is travelling, which GPS and
/// the IMU already measure independently of the camera. So every parked car
/// and every sign post we drive past is a free calibration target, and the
/// ratio of measured closure to measured speed is the distance estimator's
/// scale error directly.
///
/// The guards matter more than the arithmetic:
///
///  * Only tracks the tracker calls stationary, confidently, for long enough.
///    A slow-moving vehicle would read as a scale error.
///  * Only while travelling fast enough for the speed itself to be accurate;
///    GPS speed near walking pace is mostly noise, and dividing by it turns
///    that noise into nonsense.
///  * Only while going roughly straight. Under yaw, the range to an object
///    off to the side changes for reasons that have nothing to do with the
///    distance estimator.
///  * Median and MAD rather than mean and standard deviation, because one
///    mis-associated track produces an outlier that would otherwise move the
///    answer more than a hundred good samples.
class DistanceValidator {
  DistanceValidator({
    this.minEgoSpeedMps = 4.0,
    this.maxYawRateRadPerS = 0.08,
    this.minIntervalSeconds = 0.35,
    this.maxIntervalSeconds = 2.0,
    this.minTrackAge = 6,
    this.minDistanceConfidence = 0.4,
    this.maxSamples = 4000,
    this.bandEdges = const <double>[5, 10, 15, 20, 30, 45, 70],
  });

  static const String _tag = 'DistanceValidator';

  final double minEgoSpeedMps;
  final double maxYawRateRadPerS;
  final double minIntervalSeconds;
  final double maxIntervalSeconds;
  final int minTrackAge;
  final double minDistanceConfidence;
  final int maxSamples;

  /// Band boundaries in metres. Reported per band so a distance-dependent
  /// error is visible as one.
  final List<double> bandEdges;

  final List<ClosureSample> _samples = <ClosureSample>[];
  final Map<int, _TrackObservation> _last = <int, _TrackObservation>{};

  List<ClosureSample> get samples => List<ClosureSample>.unmodifiable(_samples);
  int get sampleCount => _samples.length;

  /// Tracks currently eligible as measurement targets, for the UI.
  int eligibleTargets = 0;

  void reset() {
    _samples.clear();
    _last.clear();
    eligibleTargets = 0;
  }

  /// Feed one frame.
  void observe({
    required List<ObjectTrack> tracks,
    required EgoMotionState ego,
    required int timestampMicros,
  }) {
    final bool conditionsOk = ego.speedMps >= minEgoSpeedMps &&
        ego.speedConfidence >= 0.5 &&
        ego.yawRateRadPerS.abs() <= maxYawRateRadPerS;

    final Set<int> seen = <int>{};
    int eligible = 0;

    for (final ObjectTrack t in tracks) {
      if (!_isUsableTarget(t)) continue;
      eligible++;
      seen.add(t.id);

      final _TrackObservation? prior = _last[t.id];
      final _TrackObservation now = _TrackObservation(
        distanceY: t.position.y,
        timestampMicros: timestampMicros,
        egoSpeedMps: ego.speedMps,
      );

      if (prior == null || !conditionsOk) {
        _last[t.id] = now;
        continue;
      }

      final double dt = (timestampMicros - prior.timestampMicros) / 1e6;
      if (dt < minIntervalSeconds) continue; // keep the older anchor
      if (dt > maxIntervalSeconds) {
        _last[t.id] = now;
        continue;
      }

      final double closure = (prior.distanceY - now.distanceY) / dt;
      final double meanSpeed = (prior.egoSpeedMps + now.egoSpeedMps) / 2;
      _last[t.id] = now;

      if (meanSpeed < minEgoSpeedMps) continue;
      // A stationary object cannot recede, and cannot approach at three times
      // our own speed. Either means the track is not what we think it is.
      final double ratio = closure / meanSpeed;
      if (ratio < 0.2 || ratio > 3.0) continue;

      _samples.add(ClosureSample(
        trackId: t.id,
        meanDistanceMeters: (prior.distanceY + now.distanceY) / 2,
        measuredClosureMps: closure,
        egoSpeedMps: meanSpeed,
        dtSeconds: dt,
        timestampMicros: timestampMicros,
      ));
      if (_samples.length > maxSamples) _samples.removeAt(0);
    }

    eligibleTargets = eligible;
    _last.removeWhere((int id, _) => !seen.contains(id));
  }

  bool _isUsableTarget(ObjectTrack t) =>
      t.isConfirmed &&
      t.age >= minTrackAge &&
      t.position.y > 3 &&
      t.direction == MotionDirection.stationary &&
      t.velocityWorld.length < 0.8 &&
      t.distanceConfidence.value >= minDistanceConfidence;

  DistanceAccuracyReport get report {
    if (_samples.isEmpty) return DistanceAccuracyReport.empty;

    final List<DistanceBandResult> bands = <DistanceBandResult>[];
    for (int i = 0; i + 1 < bandEdges.length; i++) {
      final double near = bandEdges[i];
      final double far = bandEdges[i + 1];
      final List<double> ratios = <double>[
        for (final ClosureSample s in _samples)
          if (s.meanDistanceMeters >= near && s.meanDistanceMeters < far)
            s.scaleRatio,
      ];
      if (ratios.isEmpty) continue;
      final (double median, double spread) = _medianAndMad(ratios);
      bands.add(DistanceBandResult(
        nearMeters: near,
        farMeters: far,
        sampleCount: ratios.length,
        medianScale: median,
        scaleSpread: spread,
      ));
    }

    final (double median, double spread) = _medianAndMad(
      <double>[for (final ClosureSample s in _samples) s.scaleRatio],
    );

    return DistanceAccuracyReport(
      bands: bands,
      totalSamples: _samples.length,
      overallScale: median,
      overallSpread: spread,
    );
  }

  /// Median, and MAD scaled by 1.4826 so it is comparable to a standard
  /// deviation for normally-distributed data.
  static (double, double) _medianAndMad(List<double> values) {
    if (values.isEmpty) return (1, 0);
    final List<double> sorted = List<double>.of(values)..sort();
    final double median = _medianOfSorted(sorted);
    final List<double> deviations = <double>[
      for (final double v in values) (v - median).abs(),
    ]..sort();
    return (median, _medianOfSorted(deviations) * 1.4826);
  }

  static double _medianOfSorted(List<double> sorted) {
    final int n = sorted.length;
    if (n == 0) return 0;
    if (n.isOdd) return sorted[n ~/ 2];
    return (sorted[n ~/ 2 - 1] + sorted[n ~/ 2]) / 2;
  }

  /// CSV of every sample, for analysis off the phone.
  String toCsv() {
    final StringBuffer b = StringBuffer(
      'timestamp_us,track_id,mean_distance_m,measured_closure_mps,'
      'ego_speed_mps,dt_s,scale_ratio\n',
    );
    for (final ClosureSample s in _samples) {
      b.writeln('${s.timestampMicros},${s.trackId},'
          '${s.meanDistanceMeters.toStringAsFixed(3)},'
          '${s.measuredClosureMps.toStringAsFixed(4)},'
          '${s.egoSpeedMps.toStringAsFixed(4)},'
          '${s.dtSeconds.toStringAsFixed(4)},'
          '${s.scaleRatio.toStringAsFixed(5)}');
    }
    return b.toString();
  }

  void logSummary() {
    final DistanceAccuracyReport r = report;
    Log.info(_tag, 'n=${r.totalSamples} '
        'scale=${r.overallScale.toStringAsFixed(3)} '
        '±${r.overallSpread.toStringAsFixed(3)}');
    for (final DistanceBandResult b in r.bands) {
      Log.info(_tag, '  $b');
    }
  }
}

class _TrackObservation {
  const _TrackObservation({
    required this.distanceY,
    required this.timestampMicros,
    required this.egoSpeedMps,
  });

  final double distanceY;
  final int timestampMicros;
  final double egoSpeedMps;
}
