/// The road actors and obstacles this stack reasons about.
///
/// The set is deliberately driving-specific rather than a generic COCO list:
/// `laneRelation`, collision risk and the decision engine all branch on the
/// *kind* of thing ahead, and "potted plant" is not a category that needs a
/// branch. [ObjectClassMapping] translates from whatever vocabulary an
/// installed detector was trained on.
enum ObjectClass {
  person('person', isVulnerable: true),
  car('car', isVehicle: true),
  truck('truck', isVehicle: true),
  bus('bus', isVehicle: true),
  van('van', isVehicle: true),
  motorcycle('motorcycle', isVehicle: true, isVulnerable: true),
  bicycle('bicycle', isVehicle: true, isVulnerable: true),
  animal('animal', isVulnerable: true),
  trafficCone('traffic cone', isStaticObstacle: true),
  barrier('barrier', isStaticObstacle: true),
  roadDebris('road debris', isStaticObstacle: true),
  constructionEquipment('construction equipment', isStaticObstacle: true),
  trafficSign('traffic sign', isInfrastructure: true),
  trafficLight('traffic light', isInfrastructure: true),
  unknown('unknown');

  const ObjectClass(
    this.label, {
    this.isVehicle = false,
    this.isVulnerable = false,
    this.isStaticObstacle = false,
    this.isInfrastructure = false,
  });

  final String label;

  /// Moves on the road under its own power and obeys (roughly) lane rules.
  final bool isVehicle;

  /// A vulnerable road user. These get a larger safety margin, a lower
  /// time-to-collision threshold and their own decision states
  /// (`PEDESTRIAN_YIELD`, motorcycle crossing warnings).
  final bool isVulnerable;

  /// Does not move; blocks the drivable corridor.
  final bool isStaticObstacle;

  /// Conveys information rather than occupying space. Never a collision
  /// target, never an obstacle to plan around.
  final bool isInfrastructure;

  bool get isDynamic => isVehicle || this == person || this == animal;

  /// Anything the planner must not drive through.
  bool get isObstacle => !isInfrastructure && this != unknown;

  /// Physical size prior, metres. Used by the monocular depth fusion to turn
  /// apparent box size into a distance estimate, and as a sanity bound on the
  /// depth network's output.
  ///
  /// Values are rough population medians for European/Middle-Eastern traffic;
  /// the spread is captured in [sizePriorSpread], which is what actually
  /// determines how much the size cue is trusted.
  PhysicalSizePrior get sizePrior => switch (this) {
        ObjectClass.person =>
          const PhysicalSizePrior(height: 1.70, width: 0.50, length: 0.35),
        ObjectClass.car =>
          const PhysicalSizePrior(height: 1.50, width: 1.80, length: 4.50),
        ObjectClass.truck =>
          const PhysicalSizePrior(height: 3.30, width: 2.50, length: 9.00),
        ObjectClass.bus =>
          const PhysicalSizePrior(height: 3.20, width: 2.55, length: 11.00),
        ObjectClass.van =>
          const PhysicalSizePrior(height: 2.10, width: 1.95, length: 5.40),
        ObjectClass.motorcycle =>
          const PhysicalSizePrior(height: 1.55, width: 0.80, length: 2.10),
        ObjectClass.bicycle =>
          const PhysicalSizePrior(height: 1.70, width: 0.65, length: 1.75),
        ObjectClass.animal =>
          const PhysicalSizePrior(height: 0.90, width: 0.45, length: 1.20),
        ObjectClass.trafficCone =>
          const PhysicalSizePrior(height: 0.70, width: 0.35, length: 0.35),
        ObjectClass.barrier =>
          const PhysicalSizePrior(height: 1.00, width: 2.00, length: 0.40),
        ObjectClass.roadDebris =>
          const PhysicalSizePrior(height: 0.35, width: 0.60, length: 0.60),
        ObjectClass.constructionEquipment =>
          const PhysicalSizePrior(height: 3.00, width: 2.60, length: 6.00),
        ObjectClass.trafficSign =>
          const PhysicalSizePrior(height: 0.70, width: 0.70, length: 0.05),
        ObjectClass.trafficLight =>
          const PhysicalSizePrior(height: 0.90, width: 0.30, length: 0.30),
        ObjectClass.unknown =>
          const PhysicalSizePrior(height: 1.60, width: 1.20, length: 2.00),
      };

  /// Relative standard deviation of the size prior. A bus is a bus; "road
  /// debris" could be a plank or a mattress, so its size cue is nearly
  /// worthless and the fusion weights it accordingly.
  double get sizePriorSpread => switch (this) {
        ObjectClass.person => 0.08,
        ObjectClass.car => 0.12,
        ObjectClass.truck => 0.30,
        ObjectClass.bus => 0.12,
        ObjectClass.van => 0.18,
        ObjectClass.motorcycle => 0.12,
        ObjectClass.bicycle => 0.12,
        ObjectClass.animal => 0.45,
        ObjectClass.trafficCone => 0.15,
        ObjectClass.barrier => 0.40,
        ObjectClass.roadDebris => 0.80,
        ObjectClass.constructionEquipment => 0.45,
        ObjectClass.trafficSign => 0.35,
        ObjectClass.trafficLight => 0.25,
        ObjectClass.unknown => 0.60,
      };

  /// Extra longitudinal clearance the planner keeps, metres. Vulnerable road
  /// users get more because their next move is far less predictable.
  double get safetyMarginMeters => switch (this) {
        ObjectClass.person => 2.5,
        ObjectClass.animal => 2.5,
        ObjectClass.bicycle => 2.0,
        ObjectClass.motorcycle => 2.0,
        ObjectClass.truck => 1.5,
        ObjectClass.bus => 1.5,
        _ => 1.0,
      };

  /// Plausible maximum speed, m/s. Used to reject absurd velocity estimates
  /// produced by a tracker that briefly swapped two identities.
  double get maxPlausibleSpeedMps => switch (this) {
        ObjectClass.person => 6.0,
        ObjectClass.animal => 18.0,
        ObjectClass.bicycle => 14.0,
        ObjectClass.motorcycle => 60.0,
        ObjectClass.car => 70.0,
        ObjectClass.van => 55.0,
        ObjectClass.truck => 40.0,
        ObjectClass.bus => 35.0,
        _ => 0.0,
      };

  static ObjectClass fromName(String name) {
    for (final ObjectClass c in ObjectClass.values) {
      if (c.name == name || c.label == name) return c;
    }
    return ObjectClass.unknown;
  }
}

class PhysicalSizePrior {
  const PhysicalSizePrior({
    required this.height,
    required this.width,
    required this.length,
  });

  final double height;
  final double width;
  final double length;
}

/// Maps a detector's own label vocabulary onto [ObjectClass].
///
/// This is the seam that lets a COCO-trained YOLO, a BDD100K model and a
/// custom road-hazard model all drop into the same pipeline: only this table
/// changes, never the perception or planning code.
class ObjectClassMapping {
  const ObjectClassMapping(this.table);

  final Map<String, ObjectClass> table;

  /// COCO-80, which is what most off-the-shelf detectors ship with.
  /// Classes irrelevant to driving simply have no entry and are discarded.
  static const ObjectClassMapping coco = ObjectClassMapping(<String, ObjectClass>{
    'person': ObjectClass.person,
    'bicycle': ObjectClass.bicycle,
    'car': ObjectClass.car,
    'motorcycle': ObjectClass.motorcycle,
    'motorbike': ObjectClass.motorcycle,
    'bus': ObjectClass.bus,
    'truck': ObjectClass.truck,
    'train': ObjectClass.truck,
    'traffic light': ObjectClass.trafficLight,
    'stop sign': ObjectClass.trafficSign,
    'parking meter': ObjectClass.trafficSign,
    'bird': ObjectClass.animal,
    'cat': ObjectClass.animal,
    'dog': ObjectClass.animal,
    'horse': ObjectClass.animal,
    'sheep': ObjectClass.animal,
    'cow': ObjectClass.animal,
    'elephant': ObjectClass.animal,
    'bear': ObjectClass.animal,
    'zebra': ObjectClass.animal,
    'giraffe': ObjectClass.animal,
    'bench': ObjectClass.barrier,
    'fire hydrant': ObjectClass.barrier,
    'suitcase': ObjectClass.roadDebris,
    'chair': ObjectClass.roadDebris,
  });

  /// BDD100K / driving-dataset vocabulary, which already carries the
  /// road-specific classes this project wants.
  static const ObjectClassMapping driving = ObjectClassMapping(<String, ObjectClass>{
    'pedestrian': ObjectClass.person,
    'person': ObjectClass.person,
    'rider': ObjectClass.motorcycle,
    'car': ObjectClass.car,
    'truck': ObjectClass.truck,
    'bus': ObjectClass.bus,
    'van': ObjectClass.van,
    'caravan': ObjectClass.van,
    'trailer': ObjectClass.truck,
    'motor': ObjectClass.motorcycle,
    'motorcycle': ObjectClass.motorcycle,
    'bike': ObjectClass.bicycle,
    'bicycle': ObjectClass.bicycle,
    'traffic sign': ObjectClass.trafficSign,
    'traffic light': ObjectClass.trafficLight,
    'animal': ObjectClass.animal,
    'traffic cone': ObjectClass.trafficCone,
    'cone': ObjectClass.trafficCone,
    'barrier': ObjectClass.barrier,
    'guard rail': ObjectClass.barrier,
    'debris': ObjectClass.roadDebris,
    'road debris': ObjectClass.roadDebris,
    'construction': ObjectClass.constructionEquipment,
    'construction equipment': ObjectClass.constructionEquipment,
  });

  /// `null` means "this detector class is not relevant to driving" — the
  /// detection is dropped rather than forced into [ObjectClass.unknown],
  /// which would pollute the world model with couches and TVs.
  ObjectClass? map(String detectorLabel) =>
      table[detectorLabel.toLowerCase().trim()];

  ObjectClassMapping merge(Map<String, ObjectClass> extra) =>
      ObjectClassMapping(<String, ObjectClass>{...table, ...extra});
}
