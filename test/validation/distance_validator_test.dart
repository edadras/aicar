import 'dart:math' as math;

import 'package:aicar/core/geometry.dart';
import 'package:aicar/perception/object_class.dart';
import 'package:aicar/tracking/object_track.dart';
import 'package:aicar/validation/distance_validator.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/world_builder.dart';

/// The method in one sentence: for a stationary object, the distance to it
/// must shrink at exactly the speed we are travelling, so the ratio of the
/// two *is* the distance estimator's scale error — no tape measure needed.
///
/// These tests drive a simulated vehicle past simulated parked cars with a
/// deliberately wrong distance estimator, and check the validator recovers
/// the error that was injected.
void main() {
  /// Drive at [speedMps] past stationary objects while the distance
  /// estimator reports `scale x trueDistance`.
  DistanceValidator driveBy({
    required double scale,
    double speedMps = 14,
    double noiseMeters = 0,
    int seconds = 40,
    double yawRate = 0,
    double speedConfidence = 0.9,
    MotionDirection direction = MotionDirection.stationary,
    List<double> startDistances = const <double>[60, 48, 36, 24],
  }) {
    final DistanceValidator v = DistanceValidator();
    final math.Random rng = math.Random(7);
    const double dt = 0.1;

    // Each target starts at its own distance and is replaced once passed, so
    // a long drive produces samples across the whole range.
    final List<double> trueDistance = List<double>.of(startDistances);
    final List<int> ids =
        List<int>.generate(startDistances.length, (int i) => i + 1);
    int nextId = startDistances.length + 1;

    for (int step = 0; step < seconds ~/ dt; step++) {
      final int ts = (step * dt * 1e6).round();
      final List<ObjectTrack> tracks = <ObjectTrack>[];

      for (int i = 0; i < trueDistance.length; i++) {
        trueDistance[i] -= speedMps * dt;
        if (trueDistance[i] < 4) {
          trueDistance[i] = 70;
          ids[i] = nextId++;
        }
        final double reported = trueDistance[i] * scale +
            (noiseMeters == 0 ? 0 : (rng.nextDouble() - 0.5) * 2 * noiseMeters);
        tracks.add(testTrack(
          id: ids[i],
          objectClass: ObjectClass.car,
          position: Vec2(0.2, reported),
          worldVelocity: const Vec2.zero(),
          egoSpeed: speedMps,
          direction: direction,
          distanceConfidence: 0.8,
          age: 30,
        ));
      }

      v.observe(
        tracks: tracks,
        ego: testEgo(
          speedMps: speedMps,
          yawRate: yawRate,
          ts: ts,
          speedConfidence: speedConfidence,
        ),
        timestampMicros: ts,
      );
    }
    return v;
  }

  group('recovering an injected error', () {
    test('a perfect estimator reports a scale of 1', () {
      final DistanceAccuracyReport r = driveBy(scale: 1.0).report;
      expect(r.totalSamples, greaterThan(30));
      expect(r.overallScale, closeTo(1.0, 0.02));
      expect(r.errorAt(25), isNotNull);
      expect(r.errorAt(25)!, lessThan(1.0));
    });

    test('a 12% over-read is measured as a 12% over-read', () {
      final DistanceAccuracyReport r = driveBy(scale: 1.12).report;
      expect(r.overallScale, closeTo(1.12, 0.02));
      // Which at 25 m is a three-metre error, and the report says so.
      expect(r.errorAt(25)!, greaterThan(2.0));
    });

    test('an under-read is measured too', () {
      final DistanceAccuracyReport r = driveBy(scale: 0.88).report;
      expect(r.overallScale, closeTo(0.88, 0.02));
    });

    test('random noise shows up as spread, not as bias', () {
      final DistanceAccuracyReport clean = driveBy(scale: 1.0).report;
      final DistanceAccuracyReport noisy =
          driveBy(scale: 1.0, noiseMeters: 1.5).report;

      expect(noisy.overallScale, closeTo(1.0, 0.05),
          reason: 'noise must not bias the estimate');
      expect(noisy.overallSpread, greaterThan(clean.overallSpread),
          reason: 'but it must show up as spread');
    });
  });

  group('telling a height error from a pitch error', () {
    test('a constant scale error is diagnosed as camera height', () {
      final DistanceAccuracyReport r = driveBy(scale: 1.10).report;
      expect(r.diagnosis.toLowerCase(), contains('height'));
    });

    test('a clean calibration is reported as clean', () {
      final DistanceAccuracyReport r = driveBy(scale: 1.0).report;
      expect(r.diagnosis.toLowerCase(), contains('consistent'));
    });

    test('errors are reported per distance band', () {
      final DistanceAccuracyReport r = driveBy(scale: 1.08).report;
      final List<DistanceBandResult> usable =
          r.bands.where((DistanceBandResult b) => b.isUsable).toList();
      expect(usable.length, greaterThanOrEqualTo(2));
      for (final DistanceBandResult b in usable) {
        expect(b.medianScale, closeTo(1.08, 0.03), reason: '$b');
        // The same 8% is a bigger error in metres further away, which is the
        // whole reason for banding it.
        expect(b.biasMeters.abs(), closeTo(0.08 * b.centreMeters, 0.5));
      }
      expect(usable.last.biasMeters.abs(),
          greaterThan(usable.first.biasMeters.abs()));
    });
  });

  group('the guards', () {
    test('nothing is measured below the minimum speed', () {
      // GPS speed near walking pace is mostly noise, and the method divides
      // by it.
      expect(driveBy(scale: 1.0, speedMps: 2).sampleCount, 0);
    });

    test('nothing is measured while turning', () {
      // Under yaw the range to an object changes for reasons that have
      // nothing to do with the distance estimator.
      expect(driveBy(scale: 1.0, yawRate: 0.4).sampleCount, 0);
    });

    test('a moving object is never used as a target', () {
      expect(
        driveBy(scale: 1.0, direction: MotionDirection.approaching)
            .sampleCount,
        0,
      );
    });

    test('an unreliable speed reading is not trusted', () {
      expect(driveBy(scale: 1.0, speedConfidence: 0.2).sampleCount, 0);
    });

    test('an empty session claims nothing', () {
      final DistanceValidator v = DistanceValidator();
      expect(v.report.totalSamples, 0);
      expect(v.report.isUsable, isFalse);
      expect(v.report.errorAt(25), isNull);
      expect(v.report.diagnosis.toLowerCase(), contains('not enough data'));
    });
  });

  test('samples export as CSV for analysis off the phone', () {
    final DistanceValidator v = driveBy(scale: 1.05);
    final List<String> lines = v.toCsv().trim().split('\n');
    expect(lines.first, startsWith('timestamp_us,track_id,'));
    expect(lines.length, v.sampleCount + 1);
    expect(lines[1].split(',').length, 7);
  });
}
