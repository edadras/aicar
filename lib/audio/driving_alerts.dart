import 'dart:async';

import '../core/logging.dart';
import '../decision/driving_decision.dart';
import '../world_model/hazard.dart';
import '../world_model/world_state.dart';
import 'alert_policy.dart';

/// Plays a tone or speaks a phrase. Implemented natively; stubbed in tests.
abstract class AlertSink {
  /// A short, sharp tone. Must return promptly — it is on the path of a
  /// collision warning.
  Future<void> tone(HazardSeverity severity);

  Future<void> speak(String text);

  /// Stop anything in progress, so an urgent alert is not queued behind a
  /// leisurely one.
  Future<void> stopSpeaking();

  Future<void> dispose();
}

/// Turns the world model into the few things worth saying out loud.
///
/// The hard part is not playing sounds, it is restraint. Announce everything
/// and the driver tunes the system out within a minute, and then misses the
/// one alert that mattered — so this layer exists mostly to *suppress*:
///
///  * one alert at a time, the most urgent;
///  * the same situation announced once, not once per frame;
///  * re-announced only after it has genuinely cleared, not because a TTC
///    flickered across a threshold;
///  * cautions no more than once every twenty seconds;
///  * and anything urgent interrupts anything leisurely, rather than queueing
///    behind it.
class DrivingAlerts {
  DrivingAlerts({
    required this.sink,
    this.policy = const AlertPolicy(),
    this.enabled = true,
  });

  static const String _tag = 'DrivingAlerts';

  final AlertSink sink;
  final AlertPolicy policy;

  /// Set false to mute without tearing anything down.
  bool enabled;

  final Map<String, double> _lastIssuedAt = <String, double>{};
  final Map<String, double> _lastSeenAt = <String, double>{};
  double _lastAnyAt = -1e9;
  double _lastCautionAt = -1e9;
  String? _activeCriticalKey;
  double _lastCriticalToneAt = -1e9;
  double _now = 0;

  /// The alert currently showing, for the minimal HUD.
  DrivingAlert? current;

  void reset() {
    _lastIssuedAt.clear();
    _lastSeenAt.clear();
    _lastAnyAt = -1e9;
    _lastCautionAt = -1e9;
    _activeCriticalKey = null;
    _lastCriticalToneAt = -1e9;
    current = null;
  }

  /// Call once per pipeline cycle.
  void update({
    required WorldState world,
    required DrivingDecision decision,
  }) {
    _now = world.timestampSeconds;
    final DrivingAlert? alert =
        policy.evaluate(world: world, decision: decision);
    current = alert;

    if (!enabled) return;

    if (alert == null) {
      _activeCriticalKey = null;
      return;
    }

    final double? previouslySeenAt = _lastSeenAt[alert.key];
    _lastSeenAt[alert.key] = _now;

    // A repeating critical alert keeps sounding for as long as it lasts,
    // rather than firing once and leaving the driver to wonder whether it
    // still applies.
    if (alert.repeatWhileActive &&
        alert.severity == HazardSeverity.critical) {
      if (_activeCriticalKey != alert.key ||
          _now - _lastCriticalToneAt >= policy.criticalRepeatSeconds) {
        _activeCriticalKey = alert.key;
        _lastCriticalToneAt = _now;
        _lastAnyAt = _now;
        unawaited(_issue(alert, interrupt: true));
      }
      return;
    }
    _activeCriticalKey = null;

    if (!_shouldIssue(alert, previouslySeenAt)) return;

    _lastIssuedAt[alert.key] = _now;
    _lastAnyAt = _now;
    if (alert.severity == HazardSeverity.caution) _lastCautionAt = _now;
    unawaited(_issue(alert, interrupt: false));
  }

  /// Whether this alert is news.
  ///
  /// Three questions, in order. Has it been said at all? Has the situation
  /// actually gone away and come back since, or has it simply been there the
  /// whole time? And if it is only a caution, has enough quiet passed that
  /// saying it is worth the interruption?
  bool _shouldIssue(DrivingAlert alert, double? previouslySeenAt) {
    final double? issuedAt = _lastIssuedAt[alert.key];

    if (issuedAt != null) {
      // Absent for long enough to count as a new event?
      final double gap =
          previouslySeenAt == null ? double.infinity : _now - previouslySeenAt;
      final bool cleared = gap >= policy.reArmSeconds;
      // Continuously present since we last said so is not new information.
      if (!cleared) return false;
    }

    if (alert.severity == HazardSeverity.caution) {
      if (_now - _lastCautionAt < policy.cautionMinimumGapSeconds) {
        return false;
      }
      // Never let a caution tread on the heels of something more urgent.
      if (_now - _lastAnyAt < 3) return false;
    }

    return true;
  }

  Future<void> _issue(DrivingAlert alert, {required bool interrupt}) async {
    try {
      if (interrupt) await sink.stopSpeaking();
      if (alert.channel.playsTone) await sink.tone(alert.severity);
      if (alert.channel.speaks) await sink.speak(alert.spokenText);
      Log.debug(_tag, 'alert: $alert');
    } catch (e) {
      // Audio failing must never affect driving output. A phone with no TTS
      // voice installed still shows everything on screen.
      Log.warn(_tag, 'alert failed: $e');
    }
  }

  Future<void> dispose() => sink.dispose();
}

/// An [AlertSink] that does nothing, for tests and for devices with no audio.
class SilentAlertSink implements AlertSink {
  const SilentAlertSink();

  @override
  Future<void> tone(HazardSeverity severity) async {}

  @override
  Future<void> speak(String text) async {}

  @override
  Future<void> stopSpeaking() async {}

  @override
  Future<void> dispose() async {}
}
