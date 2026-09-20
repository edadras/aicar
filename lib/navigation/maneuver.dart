/// High-level route intent.
///
/// This is the *only* thing navigation is allowed to express to the driving
/// stack. Navigation knows the topology of the road network; it does not know
/// where the tarmac is, where the lane markings are, or what is in the way.
/// So it never produces a steering angle and never produces a path — it says
/// "you will be turning left soon", and the local planner decides what that
/// means given the road it can actually see.
///
/// Defined in its own leaf file so both the navigation layer and the road
/// layer can depend on it without a circular import.
enum ManeuverIntent {
  straight('STRAIGHT'),
  turnLeft('TURN_LEFT'),
  turnRight('TURN_RIGHT'),
  keepLeft('KEEP_LEFT'),
  keepRight('KEEP_RIGHT'),
  exit('EXIT'),
  unknown('UNKNOWN');

  const ManeuverIntent(this.label);
  final String label;

  /// Sign of the lateral bias this intent implies for corridor estimation.
  /// Deliberately small in magnitude — a hint, never a command.
  double get lateralBiasSign => switch (this) {
        ManeuverIntent.turnLeft || ManeuverIntent.keepLeft => -1,
        ManeuverIntent.turnRight ||
        ManeuverIntent.keepRight ||
        ManeuverIntent.exit =>
          1,
        _ => 0,
      };

  bool get isTurn =>
      this == ManeuverIntent.turnLeft || this == ManeuverIntent.turnRight;
}

/// The specific manoeuvre a route step calls for. Richer than [ManeuverIntent]
/// because it is what the turn-by-turn UI shows; the driving stack only ever
/// sees the reduced intent.
enum ManeuverType {
  depart('Depart', ManeuverIntent.straight),
  straight('Continue straight', ManeuverIntent.straight),
  turnLeft('Turn left', ManeuverIntent.turnLeft),
  turnRight('Turn right', ManeuverIntent.turnRight),
  slightLeft('Bear left', ManeuverIntent.keepLeft),
  slightRight('Bear right', ManeuverIntent.keepRight),
  sharpLeft('Sharp left', ManeuverIntent.turnLeft),
  sharpRight('Sharp right', ManeuverIntent.turnRight),
  uTurn('U-turn', ManeuverIntent.turnLeft),
  merge('Merge', ManeuverIntent.keepRight),
  onRamp('Take the on-ramp', ManeuverIntent.keepRight),
  offRamp('Take the exit', ManeuverIntent.exit),
  fork('Keep to the fork', ManeuverIntent.keepRight),
  roundaboutEnter('Enter the roundabout', ManeuverIntent.keepRight),
  roundaboutExit('Leave the roundabout', ManeuverIntent.exit),
  keepLeft('Keep left', ManeuverIntent.keepLeft),
  keepRight('Keep right', ManeuverIntent.keepRight),
  arrive('Arrive', ManeuverIntent.straight);

  const ManeuverType(this.label, this.intent);
  final String label;
  final ManeuverIntent intent;

  static ManeuverType fromOsrm(String type, String? modifier) {
    final String m = (modifier ?? '').toLowerCase();
    switch (type.toLowerCase()) {
      case 'depart':
        return ManeuverType.depart;
      case 'arrive':
        return ManeuverType.arrive;
      case 'merge':
        return ManeuverType.merge;
      case 'on ramp':
        return ManeuverType.onRamp;
      case 'off ramp':
        return ManeuverType.offRamp;
      case 'fork':
        return m.contains('left') ? ManeuverType.keepLeft : ManeuverType.fork;
      case 'roundabout':
      case 'rotary':
        return ManeuverType.roundaboutEnter;
      case 'exit roundabout':
      case 'exit rotary':
        return ManeuverType.roundaboutExit;
      case 'continue':
      case 'new name':
        return _fromModifier(m, ManeuverType.straight);
      case 'turn':
      case 'end of road':
      default:
        return _fromModifier(m, ManeuverType.straight);
    }
  }

  static ManeuverType _fromModifier(String m, ManeuverType fallback) =>
      switch (m) {
        'left' => ManeuverType.turnLeft,
        'right' => ManeuverType.turnRight,
        'slight left' => ManeuverType.slightLeft,
        'slight right' => ManeuverType.slightRight,
        'sharp left' => ManeuverType.sharpLeft,
        'sharp right' => ManeuverType.sharpRight,
        'uturn' => ManeuverType.uTurn,
        'straight' => ManeuverType.straight,
        _ => fallback,
      };
}
