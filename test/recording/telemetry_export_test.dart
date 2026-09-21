import 'dart:io';

import 'package:aicar/recording/recording_schema.dart';
import 'package:aicar/recording/telemetry_export.dart';
import 'package:flutter_test/flutter_test.dart';

/// Turning a recording back into the numbers a performance question is
/// actually settled with.
void main() {
  late Directory dir;

  setUp(() async {
    dir = await Directory.systemTemp.createTemp('aicar-telemetry-');
  });

  tearDown(() async {
    if (await dir.exists()) await dir.delete(recursive: true);
  });

  /// Write a synthetic session: [seconds] of cycles at [fps], with device
  /// samples every ten seconds.
  Future<File> writeSession({
    double seconds = 60,
    double fps = 20,
    double latencyMs = 30,
    String level = 'full',
    bool insufficientAfterHalfway = false,
    List<String> thermalOverTime = const <String>['none'],
    int batteryStart = 80,
    int batteryEnd = 70,
    bool charging = false,
  }) async {
    final File f = File('${dir.path}/session.jsonl');
    final StringBuffer b = StringBuffer();
    final int cycles = (seconds * fps).round();
    final int stepMicros = (1e6 / fps).round();

    int nextDeviceAt = 0;
    int deviceIndex = 0;

    for (int i = 0; i < cycles; i++) {
      final int ts = i * stepMicros;

      if (ts >= nextDeviceAt) {
        final double progress = cycles <= 1 ? 0 : i / (cycles - 1);
        final String thermal = thermalOverTime[
            (progress * (thermalOverTime.length - 1)).round()];
        b.writeln(SessionRecord(
          type: RecordType.device,
          timestampMicros: ts,
          payload: <String, dynamic>{
            'thermal': thermal,
            'headroom': 0.4 + 0.5 * progress,
            'battery':
                (batteryStart - (batteryStart - batteryEnd) * progress)
                    .round(),
            'charging': charging,
          },
        ).encode());
        nextDeviceAt = ts + 10 * 1000000;
        deviceIndex++;
      }

      final bool insufficient =
          insufficientAfterHalfway && i > cycles / 2;
      b.writeln(SessionRecord(
        type: RecordType.performance,
        timestampMicros: ts,
        payload: <String, dynamic>{
          'frameId': i,
          'latencyMs': latencyMs + (i % 10),
          'stages': <String, double>{
            'Object Detection': 18.0,
            'Depth': 31.0,
            'Lane Detection': 6.0,
          },
          'perf': <String, dynamic>{
            'level': level,
            'targetFps': fps,
            'achievedFps': fps,
            'requiredFps': insufficient ? fps * 2 : fps / 2,
            if (insufficient) 'frameRateInsufficient': true,
          },
        },
      ).encode());
    }
    expect(deviceIndex, greaterThanOrEqualTo(1));
    await f.writeAsString(b.toString());
    return f;
  }

  const TelemetryExporter exporter = TelemetryExporter();

  group('summary', () {
    test('recovers frame rate, latency and duration', () async {
      final TelemetrySummary s =
          await exporter.summarise(await writeSession(seconds: 30, fps: 20));
      expect(s.frames, 600);
      expect(s.durationSeconds, closeTo(30, 0.2));
      expect(s.meanFps, closeTo(20, 0.5));
      expect(s.meanLatencyMs, closeTo(34.5, 1));
      expect(s.p95LatencyMs, greaterThan(s.meanLatencyMs));
    });

    test('per-stage means come back', () async {
      final TelemetrySummary s = await exporter.summarise(await writeSession());
      expect(s.stageMeanMs['Depth'], closeTo(31, 0.01));
      expect(s.stageMeanMs['Object Detection'], closeTo(18, 0.01));
      expect(s.report, contains('Depth'));
    });

    test('time at each governor level is accounted for', () async {
      final TelemetrySummary s = await exporter.summarise(
        await writeSession(seconds: 30, level: 'reduced'),
      );
      expect(s.governorLevelSeconds['reduced'], closeTo(30, 0.5));
    });

    test('time below the needed rate is the headline number', () async {
      // A session that averaged 20 FPS but spent half of it under the
      // requirement is not a session that ran at 20 FPS.
      final TelemetrySummary s = await exporter.summarise(
        await writeSession(seconds: 40, insufficientAfterHalfway: true),
      );
      expect(s.insufficientFrameRateSeconds, closeTo(20, 1));
      expect(s.insufficientFraction, closeTo(0.5, 0.05));
      expect(s.report, contains('Below the rate'));
    });

    test('a clean session says so instead of leaving it blank', () async {
      final TelemetrySummary s = await exporter.summarise(await writeSession());
      expect(s.insufficientFrameRateSeconds, 0);
      expect(s.report, contains('Never below'));
    });
  });

  group('device health', () {
    test('the peak thermal state is the worst one reached', () async {
      final TelemetrySummary s = await exporter.summarise(
        await writeSession(
          seconds: 60,
          thermalOverTime: const <String>['none', 'light', 'severe', 'light'],
        ),
      );
      expect(s.peakThermal, 'severe');
    });

    test('the worst headroom is the highest value, not the lowest', () async {
      // Headroom counts *up* towards 1.0 as the phone heats, so the worst
      // moment of a drive is its maximum. Getting this backwards would
      // report every session as perfectly cool.
      final TelemetrySummary s =
          await exporter.summarise(await writeSession(seconds: 60));
      expect(s.minThermalHeadroom, greaterThan(0.8));
    });

    test('battery drain is extrapolated per hour', () async {
      final TelemetrySummary s = await exporter.summarise(
        await writeSession(
          seconds: 300,
          batteryStart: 90,
          batteryEnd: 80,
        ),
      );
      expect(s.batteryStartPercent, 90);
      expect(s.batteryEndPercent, 80);
      expect(s.batteryDrainPerHour, closeTo(120, 15));
    });

    test('a charging phone reports no drain rather than a wrong one',
        () async {
      final TelemetrySummary s = await exporter.summarise(
        await writeSession(seconds: 300, charging: true),
      );
      expect(s.batteryDrainPerHour, isNull);
    });
  });

  group('csv', () {
    test('every cycle carries the device state current at the time',
        () async {
      // Thermal is sampled every ten seconds and frames arrive twenty times
      // a second. A naive join would leave 199 rows in 200 with no thermal
      // column, and "12 FPS" and "12 FPS at SEVERE" are different findings.
      final String csv = await exporter.toCsv(
        await writeSession(
          seconds: 30,
          thermalOverTime: const <String>['none', 'severe'],
        ),
      );
      final List<String> lines = csv.trim().split('\n');
      expect(lines.first, startsWith('timestamp_us,frame_id,'));
      expect(lines.length, 601);

      for (final String line in lines.skip(1)) {
        final List<String> cells = line.split(',');
        expect(cells[8], isNotEmpty, reason: 'thermal column: $line');
        expect(cells[10], isNotEmpty, reason: 'battery column: $line');
      }
    });

    test('the stage timings survive as embedded JSON', () async {
      final String csv = await exporter.toCsv(await writeSession(seconds: 2));
      expect(csv, contains('Object Detection'));
      expect(csv, contains('""'), reason: 'quotes must be CSV-escaped');
    });

    test('an empty session produces a header and nothing else', () async {
      final File f = File('${dir.path}/empty.jsonl');
      await f.writeAsString('');
      expect((await exporter.toCsv(f)).trim().split('\n').length, 1);
      final TelemetrySummary s = await exporter.summarise(f);
      expect(s.frames, 0);
      expect(s.meanFps, 0);
    });
  });
}
