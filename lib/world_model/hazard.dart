import '../core/geometry.dart';
import '../perception/object_class.dart';
import '../tracking/object_track.dart';

/// Kinds of hazard the world model surfaces.
///
/// A hazard is distinct from an object: it is a *situation*, and it is what
/// the decision engine and the HUD warning banner actually consume. "There is
/// a pedestrian" is an observation; "a pedestrian is entering the planned
/// path 2.1 seconds ahead" is a hazard.
enum HazardType {
  collisionImminent('COLLISION RISK'),
  pedestrianInPath('PEDESTRIAN IN PATH'),
  pedestrianCrossing('PEDESTRIAN CROSSING'),
  motorcycleCrossing('MOTORCYCLE CROSSING'),
  cyclistInPath('CYCLIST IN PATH'),
  animalInPath('ANIMAL IN PATH'),
  leadVehicleBraking('VEHICLE BRAKING AHEAD'),
  leadVehicleTooClose('FOLLOWING TOO CLOSE'),
  stoppedVehicleAhead('STOPPED VEHICLE AHEAD'),
  obstacleInPath('OBSTACLE IN PATH'),
  cutIn('VEHICLE CUTTING IN'),
  laneDeparture('LANE DEPARTURE'),
  roadEnds('DRIVABLE AREA ENDS'),
  redLight('RED LIGHT'),
  stopSign('STOP SIGN'),
  giveWay('GIVE WAY'),
  crosswalkAhead('CROSSWALK AHEAD'),
  speedBumpAhead('SPEED BUMP AHEAD'),
  intersectionAhead('INTERSECTION AHEAD'),
  crossingTraffic('CROSSING TRAFFIC'),
  speedLimitExceeded('OVER SPEED LIMIT'),
  lowVisibility('LOW VISIBILITY'),
  perceptionDegraded('PERCEPTION DEGRADED'),
  lowAutonomyConfidence('AUTONOMY CONFIDENCE LOW');

  const HazardType(this.label);
  final String label;

  /// Hazards that describe the *system* rather than the road. These never
  /// trigger braking on their own; they downgrade the decision to UNCERTAIN.
  bool get isSystemHazard =>
      this == HazardType.perceptionDegraded ||
      this == HazardType.lowAutonomyConfidence ||
      this == HazardType.lowVisibility;
}

enum HazardSeverity {
  info('INFO', 0),
  caution('CAUTION', 1),
  warning('WARNING', 2),
  critical('CRITICAL', 3);

  const HazardSeverity(this.label, this.level);
  final String label;
  final int level;

  bool operator >(HazardSeverity other) => level > other.level;
  bool operator >=(HazardSeverity other) => level >= other.level;
}

/// One identified hazard in the current scene.
class Hazard {
  const Hazard({
    required this.type,
    required this.severity,
    required this.description,
    required this.confidence,
    this.relatedTrackId,
    this.position,
    this.distanceMeters,
    this.timeToCollisionSeconds,
  });

  final HazardType type;
  final HazardSeverity severity;

  /// Human-readable explanation, shown in the HUD and written to the log.
  /// Always says *why*, because a warning without a reason is not actionable.
  final String description;

  final double confidence;
  final int? relatedTrackId;
  final Vec2? position;
  final double? distanceMeters;
  final double? timeToCollisionSeconds;

  /// Compact HUD block.
  String get displayText {
    final StringBuffer b = StringBuffer(type.label);
    if (distanceMeters != null) {
      b.write('\n${distanceMeters!.toStringAsFixed(1)}m');
    }
    if (timeToCollisionSeconds != null) {
      b.write('\nTTC ${timeToCollisionSeconds!.toStringAsFixed(1)}s');
    }
    return b.toString();
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'type': type.name,
        'severity': severity.name,
        'desc': description,
        'conf': double.parse(confidence.toStringAsFixed(3)),
        if (relatedTrackId != null) 'track': relatedTrackId,
        if (distanceMeters != null)
          'dist': double.parse(distanceMeters!.toStringAsFixed(2)),
        if (timeToCollisionSeconds != null)
          'ttc': double.parse(timeToCollisionSeconds!.toStringAsFixed(2)),
      };

  static Hazard fromJson(Map<String, dynamic> j) => Hazard(
        type: HazardType.values.firstWhere(
          (HazardType t) => t.name == j['type'],
          orElse: () => HazardType.perceptionDegraded,
        ),
        severity: HazardSeverity.values.firstWhere(
          (HazardSeverity s) => s.name == j['severity'],
          orElse: () => HazardSeverity.info,
        ),
        description: j['desc'] as String? ?? '',
        confidence: (j['conf'] as num?)?.toDouble() ?? 0,
        relatedTrackId: (j['track'] as num?)?.toInt(),
        distanceMeters: (j['dist'] as num?)?.toDouble(),
        timeToCollisionSeconds: (j['ttc'] as num?)?.toDouble(),
      );

  /// Build a hazard from a track whose risk has already been assessed.
  static Hazard? fromTrack(ObjectTrack track) {
    if (track.collisionRisk == CollisionRisk.low) return null;

    final HazardSeverity severity = switch (track.collisionRisk) {
      CollisionRisk.critical => HazardSeverity.critical,
      CollisionRisk.high => HazardSeverity.warning,
      CollisionRisk.medium => HazardSeverity.caution,
      CollisionRisk.low => HazardSeverity.info,
    };

    final HazardType type = switch (track.objectClass) {
      _ when track.collisionRisk == CollisionRisk.critical =>
        HazardType.collisionImminent,
      _ when track.isMotorcycleCrossing => HazardType.motorcycleCrossing,
      _ when track.objectClass == ObjectClass.person &&
              track.direction.isLateral =>
        HazardType.pedestrianCrossing,
      _ when track.objectClass == ObjectClass.person =>
        HazardType.pedestrianInPath,
      _ when track.objectClass == ObjectClass.bicycle =>
        HazardType.cyclistInPath,
      _ when track.objectClass == ObjectClass.animal =>
        HazardType.animalInPath,
      _ when track.direction == MotionDirection.enteringPath =>
        HazardType.cutIn,
      _ when track.objectClass.isVehicle &&
              track.direction == MotionDirection.stationary =>
        HazardType.stoppedVehicleAhead,
      _ when track.objectClass.isVehicle => HazardType.leadVehicleTooClose,
      _ => HazardType.obstacleInPath,
    };

    return Hazard(
      type: type,
      severity: severity,
      description: '${track.displayLabel} at '
          '${track.estimatedDistanceMeters.toStringAsFixed(1)} m, '
          '${track.direction.label.toLowerCase()}'
          '${track.timeToCollisionSeconds == null ? '' : ', TTC '
              '${track.timeToCollisionSeconds!.toStringAsFixed(1)} s'}',
      // A hazard is only as trustworthy as the perception behind it.
      confidence: track.confidence.value *
          clampDouble(track.distanceConfidence.value, 0.2, 1.0),
      relatedTrackId: track.id,
      position: track.position,
      distanceMeters: track.estimatedDistanceMeters,
      timeToCollisionSeconds: track.timeToCollisionSeconds,
    );
  }
}
