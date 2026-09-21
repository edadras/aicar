import 'package:aicar/debug/system_monitor.dart';
import 'package:aicar/pipeline/pipeline_config.dart';
import 'package:aicar/pipeline/thermal_governor.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  SystemSample sample({
    ThermalStatus thermal = ThermalStatus.none,
    double? headroom,
    int battery = 80,
    bool charging = false,
  }) =>
      SystemSample(
        timestamp: DateTime.fromMillisecondsSinceEpoch(0),
        batteryPercent: battery,
        isCharging: charging,
        thermal: thermal,
        thermalHeadroom: headroom,
      );

  PerformancePlan run(
    ThermalGovernor g, {
    SystemSample? system,
    double achievedFps = 20,
    double speedMps = 14,
    double p95Ms = 30,
    double dt = 0.05,
  }) =>
      g.update(
        system: system,
        achievedFps: achievedFps,
        speedMps: speedMps,
        pipelineP95Ms: p95Ms,
        dtSeconds: dt,
      );

  group('what makes it step down', () {
    test('a cool phone with headroom runs everything', () {
      final PerformancePlan p =
          run(ThermalGovernor(), system: sample());
      expect(p.level, PerformanceLevel.full);
      expect(p.targetFps, 20);
      expect(p.toggles.depth, isTrue);
      expect(p.toggles.segmentation, isTrue);
    });

    test('moderate thermal sheds the slow-changing stages', () {
      final PerformancePlan p = run(
        ThermalGovernor(),
        system: sample(thermal: ThermalStatus.moderate),
      );
      expect(p.level, PerformanceLevel.reduced);
      expect(p.cadence.depthEveryNFrames,
          greaterThan(const StageCadence().depthEveryNFrames));
      expect(p.reason.toLowerCase(), contains('thermal'));
    });

    test('the forecast is acted on before the clocks come down', () {
      // Status still clear, but the 60 s headroom says throttling is coming.
      // Shedding now avoids it entirely; reacting later does not.
      final PerformancePlan p = run(
        ThermalGovernor(),
        system: sample(thermal: ThermalStatus.none, headroom: 0.92),
      );
      expect(p.level, PerformanceLevel.reduced);
      expect(p.reason, contains('headroom'));
    });

    test('critical thermal switches the optional stages off outright', () {
      final PerformancePlan p = run(
        ThermalGovernor(),
        system: sample(thermal: ThermalStatus.critical),
      );
      expect(p.level, PerformanceLevel.survival);
      expect(p.toggles.depth, isFalse);
      expect(p.toggles.segmentation, isFalse);
    });

    test('a nearly flat battery matters even when the phone is cool', () {
      final PerformancePlan p = run(
        ThermalGovernor(),
        system: sample(battery: 9),
      );
      expect(p.level, PerformanceLevel.conservative);
      expect(p.reason, contains('battery'));
    });

    test('a flat battery on charge does not', () {
      final PerformancePlan p = run(
        ThermalGovernor(),
        system: sample(battery: 9, charging: true),
      );
      expect(p.level, PerformanceLevel.full);
    });

    test('latency over the frame budget is independent evidence', () {
      final PerformancePlan p = run(
        ThermalGovernor(),
        system: sample(),
        p95Ms: 120, // budget at 20 FPS is 50 ms
      );
      expect(p.level, PerformanceLevel.reduced);
      expect(p.reason, contains('p95'));
    });
  });

  group('object detection is never what gets shed', () {
    test('the detector runs every accepted frame at every level', () {
      for (final ThermalStatus t in <ThermalStatus>[
        ThermalStatus.none,
        ThermalStatus.moderate,
        ThermalStatus.severe,
        ThermalStatus.critical,
      ]) {
        final PerformancePlan p =
            run(ThermalGovernor(), system: sample(thermal: t));
        expect(p.toggles.objectDetection, isTrue, reason: t.name);
        expect(p.toggles.tracking, isTrue, reason: t.name);
      }
    });
  });

  group('fall fast, recover slowly', () {
    test('stepping down happens on the first bad sample', () {
      final ThermalGovernor g = ThermalGovernor();
      run(g, system: sample());
      final PerformancePlan p =
          run(g, system: sample(thermal: ThermalStatus.severe));
      expect(p.level, PerformanceLevel.conservative);
    });

    test('stepping up needs a sustained cool period', () {
      final ThermalGovernor g = ThermalGovernor(recoveryDwellSeconds: 10);
      run(g, system: sample(thermal: ThermalStatus.severe));
      expect(g.level, PerformanceLevel.conservative);

      // Cool again, but only briefly.
      for (int i = 0; i < 100; i++) {
        run(g, system: sample(), dt: 0.05);
      }
      expect(g.level, PerformanceLevel.conservative,
          reason: '5 s of cool is not 10 s');

      for (int i = 0; i < 120; i++) {
        run(g, system: sample(), dt: 0.05);
      }
      expect(g.level, PerformanceLevel.reduced,
          reason: 'one step at a time, not straight back to full');
    });

    test('a brief cool spell in a hot drive does not cause oscillation', () {
      final ThermalGovernor g = ThermalGovernor(recoveryDwellSeconds: 45);
      run(g, system: sample(thermal: ThermalStatus.severe));
      for (int i = 0; i < 20; i++) {
        run(g, system: sample(), dt: 0.05);
        run(g, system: sample(thermal: ThermalStatus.severe), dt: 0.05);
      }
      expect(g.level, PerformanceLevel.conservative);
    });
  });

  group('saying when it is not enough', () {
    test('the required frame rate comes from speed, not from taste', () {
      final ThermalGovernor g = ThermalGovernor();
      // One look per 1.5 m travelled.
      expect(g.requiredFpsFor(30 / 3.6), closeTo(30 / 3.6 / 1.5, 0.01));
      expect(g.requiredFpsFor(100 / 3.6), 15, reason: 'clamped at the top');
      expect(g.requiredFpsFor(0), 5, reason: 'and at the bottom');
    });

    test('a rate below what the speed needs is flagged', () {
      final PerformancePlan p = run(
        ThermalGovernor(),
        system: sample(),
        speedMps: 25,
        achievedFps: 6,
      );
      expect(p.isFrameRateInsufficient, isTrue);
      expect(p.requiredFps, greaterThan(p.achievedFps));
    });

    test('the same rate at walking pace is fine', () {
      final PerformancePlan p = run(
        ThermalGovernor(),
        system: sample(),
        speedMps: 4,
        achievedFps: 6,
      );
      expect(p.isFrameRateInsufficient, isFalse);
    });

    test('nothing is judged before a rate has been measured', () {
      final PerformancePlan p = run(
        ThermalGovernor(),
        system: sample(),
        speedMps: 25,
        achievedFps: 0,
      );
      expect(p.isFrameRateInsufficient, isFalse);
    });
  });

  test('a plan serialises everything a reviewer needs', () {
    final PerformancePlan p = run(
      ThermalGovernor(),
      system: sample(thermal: ThermalStatus.severe),
      achievedFps: 7,
      speedMps: 25,
    );
    final Map<String, dynamic> j = p.toJson();
    expect(j['level'], 'conservative');
    expect(j['reason'], isNotEmpty);
    expect(j['achievedFps'], 7);
    expect(j['frameRateInsufficient'], isTrue);
  });

  test('with no system reading at all it does not invent a problem', () {
    final PerformancePlan p = run(ThermalGovernor(), system: null);
    expect(p.level, PerformanceLevel.full);
  });
}
