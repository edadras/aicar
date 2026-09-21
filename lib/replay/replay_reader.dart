import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

import 'package:image/image.dart' as img;
import 'package:path/path.dart' as p;

import '../camera/camera_calibration.dart';
import '../camera/camera_frame.dart';
import '../core/logging.dart';
import '../decision/driving_decision.dart';
import '../recording/recording_schema.dart';
import '../recording/session_store.dart';
import '../sensors/ego_motion.dart';
import '../simulation/simulated_control.dart';
import '../planning/planned_path.dart';
import '../simulation/vehicle_state.dart';
import '../world_model/world_state.dart';

/// Reads a recorded session back into [ReplayFrame]s.
///
/// The reader streams the JSONL rather than loading it: an hour-long drive is
/// hundreds of megabytes of records, and the Replay screen only ever needs a
/// window of it at a time.
class ReplayReader {
  ReplayReader(this.session);

  static const String _tag = 'ReplayReader';

  final RecordedSession session;

  SessionHeader get header => session.header;

  /// Calibration the drive was recorded with.
  ///
  /// Replay must use the *recorded* calibration, not the device's current
  /// one: every distance in the recording was computed with it, and replaying
  /// with a different mounting would silently change every result.
  CameraCalibration get calibration {
    try {
      return CameraCalibration.fromJson(header.calibration);
    } catch (e) {
      Log.warn(_tag, 'recorded calibration unreadable ($e); using defaults');
      return CameraCalibration.galaxyS23Default();
    }
  }

  /// Stream every replayable cycle in recorded order.
  ///
  /// Records between frames (IMU, GPS) are attached to the frame that follows
  /// them, which is what lets a re-run see the sensor history in the same
  /// order the live run did.
  Stream<ReplayFrame> frames() async* {
    final File stream = session.streamFile;
    if (!await stream.exists()) {
      Log.warn(_tag, 'no session.jsonl in ${session.directory.path}');
      return;
    }

    final CameraCalibration recordedCalibration = calibration;
    _PartialFrame? pending;
    final List<ImuSample> imuBuffer = <ImuSample>[];
    GeoPosition? gpsBuffer;

    final Stream<String> lines = stream
        .openRead()
        .transform(utf8.decoder)
        .transform(const LineSplitter());

    await for (final String line in lines) {
      final SessionRecord? record = SessionRecord.decode(line);
      if (record == null) continue;

      switch (record.type) {
        case RecordType.imu:
          try {
            imuBuffer.add(ImuSample.fromJson(record.payload));
          } catch (_) {
            // Skip a malformed sample rather than abandoning the replay.
          }

        case RecordType.gps:
          try {
            gpsBuffer = GeoPosition.fromJson(record.payload);
          } catch (_) {
            // As above.
          }

        case RecordType.frame:
          // A new frame completes the previous one.
          if (pending != null) {
            yield pending.build(calibration: recordedCalibration);
          }
          pending = _PartialFrame(
            frameId: (record.payload['frameId'] as num?)?.toInt() ?? 0,
            timestampMicros: record.timestampMicros,
            imagePath: record.payload['image'] as String?,
            imuSamples: List<ImuSample>.from(imuBuffer),
            gpsFix: gpsBuffer,
          );
          imuBuffer.clear();
          gpsBuffer = null;

          final Map<String, dynamic>? ego =
              record.payload['ego'] as Map<String, dynamic>?;
          if (ego != null) {
            try {
              pending.ego = EgoMotionState.fromJson(ego);
            } catch (_) {
              // Leave null; the re-run will recompute from IMU/GPS.
            }
          }

        case RecordType.perception:
          pending?.worldJson =
              record.payload['world'] as Map<String, dynamic>?;

        case RecordType.decision:
          final Map<String, dynamic>? d =
              record.payload['decision'] as Map<String, dynamic>?;
          if (d != null) {
            try {
              pending?.decision = DrivingDecision.fromJson(d);
            } catch (_) {}
          }
          pending?.pathJson =
              record.payload['path'] as Map<String, dynamic>?;

        case RecordType.control:
          final Map<String, dynamic>? c =
              record.payload['command'] as Map<String, dynamic>?;
          if (c != null) {
            try {
              pending?.command = SimulatedControlCommand.fromJson(c);
            } catch (_) {}
          }
          final Map<String, dynamic>? v =
              record.payload['vehicle'] as Map<String, dynamic>?;
          if (v != null) {
            try {
              pending?.vehicle = VehicleState.fromJson(v);
            } catch (_) {}
          }

        case RecordType.performance:
          final Map<String, dynamic>? stages =
              record.payload['stages'] as Map<String, dynamic>?;
          if (stages != null) {
            pending?.stageTimings = <String, double>{
              for (final MapEntry<String, dynamic> e in stages.entries)
                e.key: (e.value as num).toDouble(),
            };
          }

        case RecordType.sessionHeader:
        case RecordType.sessionFooter:
        case RecordType.marker:
        case RecordType.vehicle:
          break;
      }
    }

    if (pending != null) yield pending.build(calibration: recordedCalibration);
  }

  /// Count of replayable frames, for the scrubber.
  Future<int> countFrames() async {
    final File stream = session.streamFile;
    if (!await stream.exists()) return 0;
    int count = 0;
    await for (final String line in stream
        .openRead()
        .transform(utf8.decoder)
        .transform(const LineSplitter())) {
      if (line.contains('"t":"frame"')) count++;
    }
    return count;
  }

  /// Load a recorded image as a [CameraFrame] ready for the pipeline.
  ///
  /// Returns `null` when the frame has no image — which is exactly the case
  /// where "re-run the AI" is impossible, and the caller must say so rather
  /// than silently replaying recorded results.
  Future<CameraFrame?> loadImage(ReplayFrame frame) async {
    final String? relative = frame.imagePath;
    if (relative == null) return null;

    final File file = File(p.join(session.directory.path, relative));
    if (!await file.exists()) return null;

    try {
      final Uint8List bytes = await file.readAsBytes();
      final img.Image? decoded = img.decodeJpg(bytes);
      if (decoded == null) return null;

      final Uint8List rgb = Uint8List(decoded.width * decoded.height * 3);
      int i = 0;
      for (final img.Pixel pixel in decoded) {
        rgb[i++] = pixel.r.toInt();
        rgb[i++] = pixel.g.toInt();
        rgb[i++] = pixel.b.toInt();
      }

      return CameraFrame(
        id: frame.frameId,
        timestampMicros: frame.timestampMicros,
        width: decoded.width,
        height: decoded.height,
        bytes: rgb,
        format: PixelFormat.rgb888,
        calibration: calibration.scaledTo(decoded.width, decoded.height),
      );
    } catch (e) {
      Log.warn(_tag, 'failed to decode $relative: $e');
      return null;
    }
  }

  /// Markers dropped during the drive, for the replay timeline.
  Future<List<({int timestampMicros, String label})>> markers() async {
    final File stream = session.streamFile;
    if (!await stream.exists()) {
      return const <({int timestampMicros, String label})>[];
    }
    final List<({int timestampMicros, String label})> out =
        <({int timestampMicros, String label})>[];
    await for (final String line in stream
        .openRead()
        .transform(utf8.decoder)
        .transform(const LineSplitter())) {
      final SessionRecord? r = SessionRecord.decode(line);
      if (r?.type != RecordType.marker) continue;
      out.add((
        timestampMicros: r!.timestampMicros,
        label: r.payload['label'] as String? ?? '',
      ));
    }
    return out;
  }
}

class _PartialFrame {
  _PartialFrame({
    required this.frameId,
    required this.timestampMicros,
    required this.imagePath,
    required this.imuSamples,
    required this.gpsFix,
  });

  final int frameId;
  final int timestampMicros;
  final String? imagePath;
  final List<ImuSample> imuSamples;
  final GeoPosition? gpsFix;

  EgoMotionState? ego;
  Map<String, dynamic>? worldJson;
  Map<String, dynamic>? pathJson;
  DrivingDecision? decision;
  SimulatedControlCommand? command;
  VehicleState? vehicle;
  Map<String, double> stageTimings = const <String, double>{};

  ReplayFrame build({CameraCalibration? calibration}) => ReplayFrame(
        frameId: frameId,
        timestampMicros: timestampMicros,
        imagePath: imagePath,
        world: worldJson == null || calibration == null
            ? null
            : WorldState.fromRecordedJson(
                worldJson!,
                calibration: calibration,
              ),
        path: pathJson == null
            ? null
            : PlannedPath.fromJson(
                pathJson!,
                frameId: frameId,
                timestampMicros: timestampMicros,
              ),
        decision: decision,
        command: command,
        vehicle: vehicle,
        ego: ego,
        imuSamples: imuSamples,
        gpsFix: gpsFix,
        stageTimings: stageTimings,
      );
}
