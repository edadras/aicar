import 'dart:async';


import 'package:battery_plus/battery_plus.dart';
import 'package:flutter/services.dart';

import '../core/logging.dart';
import '../core/ring_buffer.dart';

/// Android's thermal pressure levels, as `PowerManager.getCurrentThermalStatus`
/// reports them.
enum ThermalStatus {
  none('Normal', 0),
  light('Light', 1),
  moderate('Moderate', 2),
  severe('Severe', 3),
  critical('Critical', 4),
  emergency('Emergency', 5),
  shutdown('Shutdown imminent', 6),
  unknown('Unknown', -1);

  const ThermalStatus(this.label, this.level);
  final String label;
  final int level;

  /// From "moderate" upwards the SoC is already being clocked down, so a
  /// falling frame rate is thermal rather than a regression in the code.
  bool get isThrottling => level >= 2;

  static ThermalStatus fromLevel(int level) {
    for (final ThermalStatus s in ThermalStatus.values) {
      if (s.level == level) return s;
    }
    return ThermalStatus.unknown;
  }
}

/// A single sample of device health.
class SystemSample {
  const SystemSample({
    required this.timestamp,
    required this.batteryPercent,
    required this.isCharging,
    required this.thermal,
    this.thermalHeadroom,
  });

  final DateTime timestamp;
  final int batteryPercent;
  final bool isCharging;
  final ThermalStatus thermal;

  /// Android's 60-second thermal forecast, normalised so 1.0 is the
  /// throttling point. Null below API 30 or where the OEM does not implement
  /// it.
  ///
  /// More useful than [thermal] for deciding what to do, because it arrives
  /// *before* the clocks come down rather than after: at 0.9 there is still
  /// time to shed a stage and avoid throttling entirely.
  final double? thermalHeadroom;

  /// True when the forecast says throttling is close even though the current
  /// status is still clear.
  bool get isThermallyCommitted =>
      thermal.isThrottling || (thermalHeadroom ?? 0) >= 0.85;

  Map<String, dynamic> toJson() => <String, dynamic>{
        'ts': timestamp.millisecondsSinceEpoch,
        'battery': batteryPercent,
        'charging': isCharging,
        'thermal': thermal.name,
        if (thermalHeadroom != null)
          'headroom': double.parse(thermalHeadroom!.toStringAsFixed(3)),
      };
}

/// Watches battery and thermal state during a drive.
///
/// On a phone clamped to a dashboard in sunlight, thermal throttling — not
/// the algorithms — is usually what limits sustained frame rate. Recording it
/// alongside the profiler is what makes a performance measurement
/// interpretable: 12 FPS at "normal" and 12 FPS at "severe" mean completely
/// different things about the code.
class SystemMonitor {
  SystemMonitor({this.interval = const Duration(seconds: 10)});

  static const String _tag = 'SystemMonitor';
  static const MethodChannel _channel = MethodChannel('com.aicar/system');

  final Duration interval;
  final Battery _battery = Battery();
  final RingBuffer<SystemSample> _history = RingBuffer<SystemSample>(360);

  Timer? _timer;
  SystemSample? _latest;

  /// Called for every sample, so a recording can carry device health on the
  /// monitor's own clock.
  void Function(SystemSample)? onSample;

  SystemSample? get latest => _latest;
  List<SystemSample> get history => _history.toList();

  /// Battery drop per hour, extrapolated from the session so far. The
  /// headline number for whether a long drive is practical.
  double? get batteryDrainPercentPerHour {
    final List<SystemSample> samples = _history.toList();
    if (samples.length < 2) return null;
    final SystemSample first = samples.first;
    final SystemSample last = samples.last;
    if (last.isCharging || first.isCharging) return null;

    final double hours =
        last.timestamp.difference(first.timestamp).inSeconds / 3600;
    if (hours < 0.02) return null;
    return (first.batteryPercent - last.batteryPercent) / hours;
  }

  /// Highest thermal level seen this session.
  ThermalStatus get peakThermal {
    ThermalStatus peak = ThermalStatus.none;
    for (final SystemSample s in _history) {
      if (s.thermal.level > peak.level) peak = s.thermal;
    }
    return peak;
  }

  void start() {
    if (_timer != null) return;
    unawaited(_sample());
    _timer = Timer.periodic(interval, (_) => unawaited(_sample()));
    Log.info(_tag, 'monitoring started');
  }

  Future<void> _sample() async {
    try {
      final int level = await _battery.batteryLevel;
      final BatteryState state = await _battery.batteryState;
      final (ThermalStatus, double?) thermalReading = await _readThermal();
      final SystemSample sample = SystemSample(
        timestamp: DateTime.now(),
        batteryPercent: level,
        isCharging: state == BatteryState.charging ||
            state == BatteryState.full,
        thermal: thermalReading.$1,
        thermalHeadroom: thermalReading.$2,
      );
      _latest = sample;
      _history.add(sample);
      onSample?.call(sample);

      if (sample.thermal.isThrottling) {
        Log.warn(_tag,
            'thermal ${sample.thermal.label}: the SoC is being clocked down, '
            'so frame rate will fall regardless of settings');
      }
    } catch (e) {
      Log.warn(_tag, 'sample failed: $e');
    }
  }

  Future<(ThermalStatus, double?)> _readThermal() async {
    try {
      final Map<Object?, Object?>? result =
          await _channel.invokeMethod<Map<Object?, Object?>>('thermalStatus');
      if (result == null || result['available'] != true) {
        return (ThermalStatus.unknown, null);
      }
      return (
        ThermalStatus.fromLevel((result['status'] as num?)?.toInt() ?? -1),
        (result['headroom'] as num?)?.toDouble(),
      );
    } on MissingPluginException {
      return (ThermalStatus.unknown, null);
    } catch (_) {
      return (ThermalStatus.unknown, null);
    }
  }

  void stop() {
    _timer?.cancel();
    _timer = null;
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        if (_latest != null) ...<String, dynamic>{
          'battery': _latest!.batteryPercent,
          'charging': _latest!.isCharging,
          'thermal': _latest!.thermal.name,
        },
        if (_latest?.thermalHeadroom != null)
          'headroom':
              double.parse(_latest!.thermalHeadroom!.toStringAsFixed(3)),
        'peakThermal': peakThermal.name,
        if (batteryDrainPercentPerHour != null)
          'drainPerHour': double.parse(
              batteryDrainPercentPerHour!.toStringAsFixed(1)),
      };
}
