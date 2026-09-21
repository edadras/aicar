import '../decision/driving_decision.dart';
import '../world_model/hazard.dart';
import '../world_model/world_state.dart';

/// How an alert should reach the driver.
enum AlertChannel {
  /// A tone, immediately. For things where the 300 ms a speech engine takes
  /// to start is 300 ms too long.
  tone('tone'),

  /// A tone followed by speech.
  toneAndSpeech('tone+speech'),

  /// Speech alone.
  speech('speech'),

  /// Nothing audible. Shown on screen only.
  silent('silent');

  const AlertChannel(this.label);
  final String label;

  bool get playsTone =>
      this == AlertChannel.tone || this == AlertChannel.toneAndSpeech;
  bool get speaks =>
      this == AlertChannel.speech || this == AlertChannel.toneAndSpeech;
}

/// One thing worth telling the driver.
class DrivingAlert {
  const DrivingAlert({
    required this.key,
    required this.channel,
    required this.severity,
    required this.spokenText,
    required this.displayText,
    this.repeatWhileActive = false,
  });

  /// Stable identity, so the same situation is not announced twice.
  ///
  /// Deliberately coarse — every pedestrian at a crossing shares one key.
  /// An alert per track would be an alert every second in traffic, which is
  /// the fastest way to teach a driver to ignore the system.
  final String key;

  final AlertChannel channel;
  final HazardSeverity severity;

  /// What is said. Short: a driver hears the first three words.
  final String spokenText;

  /// What the minimal HUD shows.
  final String displayText;

  /// Whether the tone repeats for as long as the situation lasts.
  final bool repeatWhileActive;

  @override
  String toString() => 'DrivingAlert($key, ${channel.label}: $spokenText)';
}

/// Decides what to say, and — much more importantly — what not to.
///
/// An ADAS that announces everything is worse than one that is silent,
/// because a driver learns within a minute to tune it out, and then misses
/// the one alert that mattered. So the policy here is deliberately
/// parsimonious:
///
///  * **One thing at a time.** Only the most urgent alert is issued per
///    cycle. Two voices over each other is no voice at all.
///  * **Coarse identity.** Every pedestrian at a crossing is one alert, not
///    one per person.
///  * **Say it once.** An alert re-announces only after its situation has
///    genuinely cleared, not because it flickered across a threshold.
///  * **Silence for the visible.** A red light the driver is already stopped
///    at needs no announcement. The system speaks about things that are
///    *about to* require action, not about the present tense.
///  * **Tones for urgency, speech for information.** A speech engine takes a
///    few hundred milliseconds to start, which is fine for "speed bump
///    ahead" and far too slow for a collision warning.
class AlertPolicy {
  const AlertPolicy({
    this.criticalRepeatSeconds = 0.8,
    this.cautionMinimumGapSeconds = 20,
    this.reArmSeconds = 5,
    this.speakBelowSpeedMps = 0.8,
  });

  /// How often a critical tone repeats while the situation lasts.
  final double criticalRepeatSeconds;

  /// Minimum gap between two merely-cautionary announcements.
  final double cautionMinimumGapSeconds;

  /// How long a situation must be absent before the same alert may fire
  /// again. Without this, a TTC oscillating around a threshold produces an
  /// alert every frame.
  final double reArmSeconds;

  /// Below this speed the vehicle is stopped and most alerts are about
  /// something the driver is already looking at.
  final double speakBelowSpeedMps;

  /// The single alert this cycle deserves, or null for silence.
  DrivingAlert? evaluate({
    required WorldState world,
    required DrivingDecision decision,
  }) {
    final bool stopped = world.ego.speedMps < speakBelowSpeedMps;

    // --- Critical: a tone, now ------------------------------------------
    if (decision.state == DrivingState.emergencyBrakeSimulation) {
      return DrivingAlert(
        key: 'emergency',
        channel: AlertChannel.tone,
        severity: HazardSeverity.critical,
        spokenText: 'Brake',
        displayText: 'BRAKE',
        repeatWhileActive: true,
      );
    }

    final Hazard? worst = world.primaryHazard;
    if (worst != null &&
        worst.severity == HazardSeverity.critical &&
        !worst.type.isSystemHazard) {
      return DrivingAlert(
        key: 'critical-${worst.type.name}',
        channel: AlertChannel.tone,
        severity: HazardSeverity.critical,
        spokenText: worst.type.label,
        displayText: worst.type.label,
        repeatWhileActive: true,
      );
    }

    // --- Warnings: a tone and a few words -------------------------------
    if (decision.state == DrivingState.pedestrianYield && !stopped) {
      return const DrivingAlert(
        key: 'pedestrian',
        channel: AlertChannel.toneAndSpeech,
        severity: HazardSeverity.warning,
        spokenText: 'Pedestrian',
        displayText: 'PEDESTRIAN',
      );
    }

    if (worst != null &&
        worst.severity == HazardSeverity.warning &&
        !worst.type.isSystemHazard &&
        // A red light we are already stopped at is not news; the `stopped`
        // guard covers that and every other present-tense warning.
        !stopped) {
      return DrivingAlert(
        key: 'warning-${worst.type.name}',
        channel: AlertChannel.toneAndSpeech,
        severity: HazardSeverity.warning,
        spokenText: _shorten(worst.type.label),
        displayText: worst.type.label,
      );
    }

    // --- The system doubting itself -------------------------------------
    //
    // Worth saying exactly once, because it changes what the driver should
    // expect from everything else on the screen.
    if (world.autonomy.isLow && !stopped) {
      return const DrivingAlert(
        key: 'uncertain',
        channel: AlertChannel.speech,
        severity: HazardSeverity.caution,
        spokenText: 'System uncertain',
        displayText: 'UNCERTAIN',
      );
    }

    // --- Cautions: spoken, rarely ---------------------------------------
    if (stopped) return null;

    if (worst != null &&
        worst.severity == HazardSeverity.caution &&
        !worst.type.isSystemHazard) {
      return DrivingAlert(
        key: 'caution-${worst.type.name}',
        channel: AlertChannel.speech,
        severity: HazardSeverity.caution,
        spokenText: _shorten(worst.type.label),
        displayText: worst.type.label,
      );
    }

    return null;
  }

  /// Trim a hazard label to something a driver can take in at speed.
  static String _shorten(String label) {
    const Map<String, String> spoken = <String, String>{
      'SPEED BUMP AHEAD': 'Speed bump',
      'CROSSWALK AHEAD': 'Crossing',
      'INTERSECTION AHEAD': 'Junction',
      'CROSSING TRAFFIC': 'Crossing traffic',
      'OVER SPEED LIMIT': 'Over the limit',
      'RED LIGHT': 'Red light',
      'STOP SIGN': 'Stop sign',
      'GIVE WAY': 'Give way',
      'DRIVABLE AREA ENDS': 'Road ends',
      'FOLLOWING TOO CLOSE': 'Too close',
      'VEHICLE BRAKING AHEAD': 'Braking ahead',
      'STOPPED VEHICLE AHEAD': 'Stopped vehicle',
      'VEHICLE CUTTING IN': 'Cutting in',
      'LANE DEPARTURE': 'Lane departure',
    };
    return spoken[label] ?? label.toLowerCase();
  }
}
