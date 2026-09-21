import '../core/safety.dart';

/// Which indicator the stack would be operating.
enum TurnSignal {
  none('OFF'),
  left('LEFT'),
  right('RIGHT'),

  /// Both, for a stop that leaves the vehicle obstructing a live lane.
  hazard('HAZARD');

  const TurnSignal(this.label);
  final String label;

  bool get isActive => this != TurnSignal.none;
  bool get showsLeft => this == TurnSignal.left || this == TurnSignal.hazard;
  bool get showsRight => this == TurnSignal.right || this == TurnSignal.hazard;
}

/// The indicator state, with the reason it is on.
class TurnSignalState {
  const TurnSignalState({
    required this.signal,
    required this.reason,
    this.heldSeconds = 0,
  });

  static const TurnSignalState off =
      TurnSignalState(signal: TurnSignal.none, reason: '');

  final TurnSignal signal;

  /// Why it is on — "turning left in 22 m", "moving out around a parked van".
  /// Shown next to the arrow, because an indicator with no reason is just a
  /// blinking light.
  final String reason;

  final double heldSeconds;

  /// Blink phase at a given timestamp, at the 1.5 Hz a real relay runs at.
  ///
  /// Derived from the timestamp rather than a UI animation so that the HUD,
  /// a recording and a replay of that recording all show the same phase.
  bool blinkOn(int timestampMicros) =>
      signal.isActive && (timestampMicros ~/ 333333) % 2 == 0;

  /// Always true: this is a lamp on a screen. See [SafetyMode].
  bool get isSimulationOnly => SafetyMode.simulationOnly;

  Map<String, dynamic> toJson() => <String, dynamic>{
        'signal': signal.name,
        if (reason.isNotEmpty) 'reason': reason,
      };

  static TurnSignalState fromJson(Map<String, dynamic>? j) {
    if (j == null) return off;
    return TurnSignalState(
      signal: TurnSignal.values.firstWhere(
        (TurnSignal s) => s.name == j['signal'],
        orElse: () => TurnSignal.none,
      ),
      reason: j['reason'] as String? ?? '',
    );
  }

  @override
  String toString() =>
      'TurnSignal(${signal.label}${reason.isEmpty ? '' : ': $reason'})';
}

