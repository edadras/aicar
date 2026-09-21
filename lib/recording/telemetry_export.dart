import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;

import 'recording_schema.dart';

/// Everything a session says about how it ran, reduced to numbers.
class TelemetrySummary {
  const TelemetrySummary({
    required this.frames,
    required this.durationSeconds,
    required this.meanFps,
    required this.meanLatencyMs,
    required this.p95LatencyMs,
    required this.stageMeanMs,
    required this.governorLevelSeconds,
    required this.insufficientFrameRateSeconds,
    required this.peakThermal,
    required this.minThermalHeadroom,
    required this.batteryStartPercent,
    required this.batteryEndPercent,
    required this.batteryDrainPerHour,
  });

  final int frames;
  final double durationSeconds;
  final double meanFps;
  final double meanLatencyMs;
  final double p95LatencyMs;

  /// Mean milliseconds per stage, over the cycles the stage actually ran.
  final Map<String, double> stageMeanMs;

  /// Seconds spent at each governor level.
  final Map<String, double> governorLevelSeconds;

  /// Seconds during which the frame rate was below what the speed needed.
  ///
  /// The single most useful number in this class. A session that averaged
  /// 14 FPS but spent four minutes under the requirement at motorway speed
  /// is not a session that ran at 14 FPS.
  final double insufficientFrameRateSeconds;

  final String peakThermal;
  final double? minThermalHeadroom;
  final int? batteryStartPercent;
  final int? batteryEndPercent;
  final double? batteryDrainPerHour;

  double get insufficientFraction =>
      durationSeconds <= 0 ? 0 : insufficientFrameRateSeconds / durationSeconds;

  Map<String, dynamic> toJson() => <String, dynamic>{
        'frames': frames,
        'durationS': double.parse(durationSeconds.toStringAsFixed(1)),
        'meanFps': double.parse(meanFps.toStringAsFixed(2)),
        'meanLatencyMs': double.parse(meanLatencyMs.toStringAsFixed(2)),
        'p95LatencyMs': double.parse(p95LatencyMs.toStringAsFixed(2)),
        'stages': <String, double>{
          for (final MapEntry<String, double> e in stageMeanMs.entries)
            e.key: double.parse(e.value.toStringAsFixed(2)),
        },
        'governorSeconds': <String, double>{
          for (final MapEntry<String, double> e
              in governorLevelSeconds.entries)
            e.key: double.parse(e.value.toStringAsFixed(1)),
        },
        'insufficientFpsS':
            double.parse(insufficientFrameRateSeconds.toStringAsFixed(1)),
        'peakThermal': peakThermal,
        if (minThermalHeadroom != null)
          'minHeadroom':
              double.parse(minThermalHeadroom!.toStringAsFixed(3)),
        if (batteryStartPercent != null) 'batteryStart': batteryStartPercent,
        if (batteryEndPercent != null) 'batteryEnd': batteryEndPercent,
        if (batteryDrainPerHour != null)
          'drainPerHour':
              double.parse(batteryDrainPerHour!.toStringAsFixed(1)),
      };

  /// Human-readable, for the screen and for pasting into a report.
  String get report {
    final StringBuffer b = StringBuffer();
    b.writeln('$frames frames over ${durationSeconds.toStringAsFixed(0)} s');
    b.writeln('Pipeline ${meanFps.toStringAsFixed(1)} FPS mean, '
        'latency ${meanLatencyMs.toStringAsFixed(0)} ms mean / '
        '${p95LatencyMs.toStringAsFixed(0)} ms p95');
    if (insufficientFrameRateSeconds > 0) {
      b.writeln('Below the rate the speed needed for '
          '${insufficientFrameRateSeconds.toStringAsFixed(0)} s '
          '(${(insufficientFraction * 100).toStringAsFixed(0)}% of the drive)');
    } else {
      b.writeln('Never below the rate the speed needed');
    }
    b.writeln('Peak thermal $peakThermal'
        '${minThermalHeadroom == null ? '' : ', headroom low '
            '${(minThermalHeadroom! * 100).round()}%'}');
    if (batteryDrainPerHour != null) {
      b.writeln('Battery $batteryStartPercent% → $batteryEndPercent%, '
          '${batteryDrainPerHour!.toStringAsFixed(0)} %/h');
    }
    final List<MapEntry<String, double>> stages = stageMeanMs.entries.toList()
      ..sort((MapEntry<String, double> a, MapEntry<String, double> b) =>
          b.value.compareTo(a.value));
    for (final MapEntry<String, double> e in stages.take(6)) {
      b.writeln('  ${e.key}: ${e.value.toStringAsFixed(1)} ms');
    }
    return b.toString();
  }
}

/// Reads a recorded session back and turns its performance records into CSV
/// and a summary.
///
/// Recordings already contain everything needed to answer "how did it run on
/// this phone?" — per-cycle latency and stage timings, the governor's
/// decision and its reason, and device health on its own clock. What was
/// missing was a way to get it off the phone and into a spreadsheet, which is
/// where a performance question actually gets settled.
class TelemetryExporter {
  const TelemetryExporter();

  /// Per-cycle CSV: one row per pipeline cycle, with the device state that
  /// was current at the time carried forward onto it.
  ///
  /// Carrying device state forward is the point. Thermal is sampled every ten
  /// seconds and frames arrive twenty times a second, so a naive join would
  /// leave 199 of every 200 rows with no thermal column — and "12 FPS" and
  /// "12 FPS at SEVERE" are completely different findings.
  Future<String> toCsv(File sessionFile) async {
    final StringBuffer out = StringBuffer(
      'timestamp_us,frame_id,latency_ms,governor_level,target_fps,'
      'achieved_fps,required_fps,insufficient,thermal,headroom,'
      'battery_percent,charging,stages_json\n',
    );

    String thermal = '';
    String headroom = '';
    String battery = '';
    String charging = '';

    await for (final SessionRecord r in _records(sessionFile)) {
      switch (r.type) {
        case RecordType.device:
          thermal = '${r.payload['thermal'] ?? ''}';
          headroom = '${r.payload['headroom'] ?? ''}';
          battery = '${r.payload['battery'] ?? ''}';
          charging = '${r.payload['charging'] ?? ''}';

        case RecordType.performance:
          final Map<String, dynamic>? perf =
              r.payload['perf'] as Map<String, dynamic>?;
          final Map<String, dynamic> stages =
              (r.payload['stages'] as Map<String, dynamic>?) ??
                  const <String, dynamic>{};
          out.writeln(<String>[
            '${r.timestampMicros}',
            '${r.payload['frameId'] ?? ''}',
            '${r.payload['latencyMs'] ?? ''}',
            '${perf?['level'] ?? ''}',
            '${perf?['targetFps'] ?? ''}',
            '${perf?['achievedFps'] ?? ''}',
            '${perf?['requiredFps'] ?? ''}',
            '${perf?['frameRateInsufficient'] == true ? 1 : 0}',
            thermal,
            headroom,
            battery,
            charging,
            '"${jsonEncode(stages).replaceAll('"', '""')}"',
          ].join(','));

        default:
          break;
      }
    }
    return out.toString();
  }

  Future<TelemetrySummary> summarise(File sessionFile) async {
    int frames = 0;
    int firstMicros = -1;
    int lastMicros = -1;
    double latencySum = 0;
    final List<double> latencies = <double>[];
    final Map<String, double> stageSum = <String, double>{};
    final Map<String, int> stageCount = <String, int>{};
    final Map<String, double> levelSeconds = <String, double>{};
    double insufficientSeconds = 0;

    int previousMicros = -1;
    String peakThermal = 'none';
    int peakThermalLevel = -1;
    double? minHeadroom;
    int? batteryStart;
    int? batteryEnd;
    int? batteryStartMicros;
    int? batteryEndMicros;
    bool sawCharging = false;

    await for (final SessionRecord r in _records(sessionFile)) {
      switch (r.type) {
        case RecordType.device:
          final String t = '${r.payload['thermal'] ?? 'unknown'}';
          final int level = _thermalRank(t);
          if (level > peakThermalLevel) {
            peakThermalLevel = level;
            peakThermal = t;
          }
          final double? h = (r.payload['headroom'] as num?)?.toDouble();
          if (h != null && (minHeadroom == null || h > minHeadroom)) {
            // "Minimum headroom" means the *worst* moment, and headroom
            // counts up towards 1.0 as the phone heats — so the worst moment
            // is the largest value, not the smallest.
            minHeadroom = h;
          }
          final int? b = (r.payload['battery'] as num?)?.toInt();
          if (b != null) {
            batteryStart ??= b;
            batteryStartMicros ??= r.timestampMicros;
            batteryEnd = b;
            batteryEndMicros = r.timestampMicros;
          }
          if (r.payload['charging'] == true) sawCharging = true;

        case RecordType.performance:
          frames++;
          if (firstMicros < 0) firstMicros = r.timestampMicros;
          lastMicros = r.timestampMicros;

          final double latency =
              (r.payload['latencyMs'] as num?)?.toDouble() ?? 0;
          latencySum += latency;
          latencies.add(latency);

          final Map<String, dynamic>? stages =
              r.payload['stages'] as Map<String, dynamic>?;
          if (stages != null) {
            stages.forEach((String k, dynamic v) {
              final double ms = (v as num).toDouble();
              if (ms <= 0) return;
              stageSum[k] = (stageSum[k] ?? 0) + ms;
              stageCount[k] = (stageCount[k] ?? 0) + 1;
            });
          }

          final double dt = previousMicros < 0
              ? 0
              : (r.timestampMicros - previousMicros) / 1e6;
          previousMicros = r.timestampMicros;
          // A gap longer than a couple of seconds is a pause, not a slow
          // frame, and must not be charged to any governor level.
          final double slice = dt > 0 && dt < 2 ? dt : 0;

          final Map<String, dynamic>? perf =
              r.payload['perf'] as Map<String, dynamic>?;
          final String level = '${perf?['level'] ?? 'unknown'}';
          levelSeconds[level] = (levelSeconds[level] ?? 0) + slice;
          if (perf?['frameRateInsufficient'] == true) {
            insufficientSeconds += slice;
          }

        default:
          break;
      }
    }

    final double duration =
        firstMicros < 0 ? 0 : (lastMicros - firstMicros) / 1e6;
    latencies.sort();

    double? drain;
    if (!sawCharging &&
        batteryStart != null &&
        batteryEnd != null &&
        batteryStartMicros != null &&
        batteryEndMicros != null) {
      final double hours =
          (batteryEndMicros - batteryStartMicros) / 1e6 / 3600;
      if (hours > 0.02) drain = (batteryStart - batteryEnd) / hours;
    }

    return TelemetrySummary(
      frames: frames,
      durationSeconds: duration,
      meanFps: duration <= 0 ? 0 : frames / duration,
      meanLatencyMs: frames == 0 ? 0 : latencySum / frames,
      p95LatencyMs: latencies.isEmpty
          ? 0
          : latencies[math.min(latencies.length - 1,
              ((latencies.length - 1) * 0.95).round())],
      stageMeanMs: <String, double>{
        for (final MapEntry<String, double> e in stageSum.entries)
          e.key: e.value / (stageCount[e.key] ?? 1),
      },
      governorLevelSeconds: levelSeconds,
      insufficientFrameRateSeconds: insufficientSeconds,
      peakThermal: peakThermal,
      minThermalHeadroom: minHeadroom,
      batteryStartPercent: batteryStart,
      batteryEndPercent: batteryEnd,
      batteryDrainPerHour: drain,
    );
  }

  Stream<SessionRecord> _records(File file) async* {
    final Stream<String> lines = file
        .openRead()
        .transform(utf8.decoder)
        .transform(const LineSplitter());
    await for (final String line in lines) {
      final SessionRecord? r = SessionRecord.decode(line);
      if (r != null) yield r;
    }
  }

  static int _thermalRank(String name) => switch (name) {
        'none' => 0,
        'light' => 1,
        'moderate' => 2,
        'severe' => 3,
        'critical' => 4,
        'emergency' => 5,
        'shutdown' => 6,
        _ => -1,
      };
}
