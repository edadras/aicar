import 'dart:convert';

import '../core/safety.dart';
import '../decision/driving_decision.dart';
import '../perception/detection.dart';
import '../planning/planned_path.dart';
import '../pipeline/pipeline_result.dart';
import '../sensors/ego_motion.dart';
import '../simulation/simulated_control.dart';
import '../simulation/vehicle_state.dart';
import '../world_model/world_state.dart';

/// Kinds of record in a session's JSONL stream.
///
/// One file with a `type` discriminator, rather than a file per stream,
/// because the whole point of the recording is that the streams are
/// *interleaved in time*: a replay has to see the IMU samples that arrived
/// between two frames in the order they arrived.
enum RecordType {
  sessionHeader('header'),
  frame('frame'),
  imu('imu'),
  gps('gps'),
  perception('perception'),
  decision('decision'),
  control('control'),
  vehicle('vehicle'),
  performance('perf'),
  marker('marker'),
  sessionFooter('footer');

  const RecordType(this.tag);
  final String tag;

  static RecordType? fromTag(String tag) {
    for (final RecordType t in RecordType.values) {
      if (t.tag == tag) return t;
    }
    return null;
  }
}

/// Schema version, bumped whenever the record format changes in a way a
/// previous reader could not handle. Written into every session header so a
/// newer build can still read (or explicitly refuse) an older recording.
///
/// * **1** — the original format.
/// * **2** — adds road markings, the junction inference and the simulated
///   indicator to the world and control records. All three are optional
///   keys, so a version-1 reader still parses a version-2 line; the bump is
///   what lets a *reader* tell "this drive had no markings" apart from "this
///   build never looked for any", which are very different claims.
const int recordingSchemaVersion = 2;

/// One line of the JSONL stream.
class SessionRecord {
  const SessionRecord({
    required this.type,
    required this.timestampMicros,
    required this.payload,
  });

  final RecordType type;
  final int timestampMicros;
  final Map<String, dynamic> payload;

  String encode() => jsonEncode(<String, dynamic>{
        't': type.tag,
        'ts': timestampMicros,
        ...payload,
      });

  static SessionRecord? decode(String line) {
    if (line.trim().isEmpty) return null;
    try {
      final Map<String, dynamic> json =
          jsonDecode(line) as Map<String, dynamic>;
      final RecordType? type = RecordType.fromTag(json['t'] as String? ?? '');
      if (type == null) return null;
      return SessionRecord(
        type: type,
        timestampMicros: (json['ts'] as num?)?.toInt() ?? 0,
        payload: json,
      );
    } catch (_) {
      // A truncated final line is expected when a session ends with the app
      // being killed; skipping it is better than failing the whole replay.
      return null;
    }
  }
}

/// Metadata written at the start of every session.
class SessionHeader {
  const SessionHeader({
    required this.sessionId,
    required this.startedAt,
    required this.schemaVersion,
    required this.appVersion,
    required this.deviceModel,
    required this.androidVersion,
    required this.calibration,
    required this.models,
    required this.pipelineConfig,
    this.notes,
  });

  final String sessionId;
  final DateTime startedAt;
  final int schemaVersion;
  final String appVersion;
  final String deviceModel;
  final String androidVersion;

  /// Camera calibration as JSON. Without it a recording cannot be replayed
  /// correctly — every metric claim depends on it.
  final Map<String, dynamic> calibration;

  /// Which model filled each role, so a dataset can be traced to the models
  /// that produced it.
  final Map<String, String> models;

  final Map<String, dynamic> pipelineConfig;
  final String? notes;

  Map<String, dynamic> toJson() => <String, dynamic>{
        'sessionId': sessionId,
        'startedAt': startedAt.toIso8601String(),
        'schema': schemaVersion,
        'app': appVersion,
        'device': deviceModel,
        'android': androidVersion,
        'calibration': calibration,
        'models': models,
        'config': pipelineConfig,
        if (notes != null) 'notes': notes,
        // Provenance: anyone reading this dataset can see that no vehicle was
        // ever commanded, without having to take the code's word for it.
        'safetyMode': SafetyMode.recordingTag,
        'safetyBanner': SafetyMode.banner,
      };

  static SessionHeader fromJson(Map<String, dynamic> j) => SessionHeader(
        sessionId: j['sessionId'] as String,
        startedAt: DateTime.parse(j['startedAt'] as String),
        schemaVersion: (j['schema'] as num?)?.toInt() ?? 0,
        appVersion: j['app'] as String? ?? 'unknown',
        deviceModel: j['device'] as String? ?? 'unknown',
        androidVersion: j['android'] as String? ?? 'unknown',
        calibration:
            (j['calibration'] as Map<String, dynamic>?) ?? <String, dynamic>{},
        models: <String, String>{
          for (final MapEntry<String, dynamic> e
              in ((j['models'] as Map<String, dynamic>?) ??
                      <String, dynamic>{})
                  .entries)
            e.key: '${e.value}',
        },
        pipelineConfig:
            (j['config'] as Map<String, dynamic>?) ?? <String, dynamic>{},
        notes: j['notes'] as String?,
      );
}

/// Summary written when a session ends.
class SessionFooter {
  const SessionFooter({
    required this.endedAt,
    required this.durationSeconds,
    required this.frameCount,
    required this.distanceMeters,
    required this.meanProcessingFps,
    required this.droppedFrames,
    required this.decisionCounts,
  });

  final DateTime endedAt;
  final double durationSeconds;
  final int frameCount;
  final double distanceMeters;
  final double meanProcessingFps;
  final int droppedFrames;

  /// How many cycles were spent in each driving state — the quickest way to
  /// see what a drive actually contained.
  final Map<String, int> decisionCounts;

  Map<String, dynamic> toJson() => <String, dynamic>{
        'endedAt': endedAt.toIso8601String(),
        'duration': double.parse(durationSeconds.toStringAsFixed(1)),
        'frames': frameCount,
        'distance': double.parse(distanceMeters.toStringAsFixed(1)),
        'meanFps': double.parse(meanProcessingFps.toStringAsFixed(2)),
        'dropped': droppedFrames,
        'decisions': decisionCounts,
      };

  static SessionFooter fromJson(Map<String, dynamic> j) => SessionFooter(
        endedAt: DateTime.tryParse(j['endedAt'] as String? ?? '') ??
            DateTime.fromMillisecondsSinceEpoch(0),
        durationSeconds: (j['duration'] as num?)?.toDouble() ?? 0,
        frameCount: (j['frames'] as num?)?.toInt() ?? 0,
        distanceMeters: (j['distance'] as num?)?.toDouble() ?? 0,
        meanProcessingFps: (j['meanFps'] as num?)?.toDouble() ?? 0,
        droppedFrames: (j['dropped'] as num?)?.toInt() ?? 0,
        decisionCounts: <String, int>{
          for (final MapEntry<String, dynamic> e
              in ((j['decisions'] as Map<String, dynamic>?) ??
                      <String, dynamic>{})
                  .entries)
            e.key: (e.value as num).toInt(),
        },
      );
}

/// One replayable cycle, reassembled from the JSONL stream.
class ReplayFrame {
  const ReplayFrame({
    required this.frameId,
    required this.timestampMicros,
    required this.imagePath,
    this.world,
    this.path,
    this.decision,
    this.command,
    this.vehicle,
    this.ego,
    this.detections,
    this.imuSamples = const <ImuSample>[],
    this.gpsFix,
    this.stageTimings = const <String, double>{},
  });

  final int frameId;
  final int timestampMicros;

  /// Path to the recorded image for this frame, relative to the session
  /// directory. Null when only structured data was recorded.
  final String? imagePath;

  /// The world model as it was recorded. Present in "recorded results" replay,
  /// and used as the reference when re-running the AI.
  final WorldState? world;

  /// The path that was planned at the time.
  final PlannedPath? path;

  final DrivingDecision? decision;
  final SimulatedControlCommand? command;
  final VehicleState? vehicle;
  final EgoMotionState? ego;
  final DetectionResult? detections;

  /// IMU samples that arrived between the previous frame and this one.
  final List<ImuSample> imuSamples;

  final GeoPosition? gpsFix;
  final Map<String, double> stageTimings;

  double get timestampSeconds => timestampMicros / 1e6;
}

/// Build the records for one pipeline cycle.
List<SessionRecord> recordsForResult(
  PipelineResult result, {
  String? imagePath,
}) {
  final WorldState world = result.world;
  return <SessionRecord>[
    SessionRecord(
      type: RecordType.frame,
      timestampMicros: world.timestampMicros,
      payload: <String, dynamic>{
        'frameId': world.frameId,
        if (imagePath != null) 'image': imagePath,
        'ego': world.ego.toJson(),
      },
    ),
    SessionRecord(
      type: RecordType.perception,
      timestampMicros: world.timestampMicros,
      payload: <String, dynamic>{
        'frameId': world.frameId,
        'world': world.toJson(),
      },
    ),
    SessionRecord(
      type: RecordType.decision,
      timestampMicros: world.timestampMicros,
      payload: <String, dynamic>{
        'frameId': world.frameId,
        'decision': result.decision.toJson(),
        'path': result.path.toJson(),
      },
    ),
    SessionRecord(
      type: RecordType.control,
      timestampMicros: world.timestampMicros,
      payload: <String, dynamic>{
        'frameId': world.frameId,
        'command': result.command.toJson(),
        'vehicle': result.vehicle.toJson(),
      },
    ),
    SessionRecord(
      type: RecordType.performance,
      timestampMicros: world.timestampMicros,
      payload: <String, dynamic>{
        'frameId': world.frameId,
        'latencyMs':
            double.parse(result.totalLatencyMs.toStringAsFixed(2)),
        'stages': result.stageTimings.map(
          (String k, double v) =>
              MapEntry<String, double>(k, double.parse(v.toStringAsFixed(2))),
        ),
      },
    ),
  ];
}
