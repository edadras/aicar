import 'dart:convert';
import 'dart:io';

import 'package:aicar/core/geometry.dart';
import 'package:aicar/decision/driving_decision.dart';
import 'package:aicar/perception/object_class.dart';
import 'package:aicar/planning/local_path_planner.dart';
import 'package:aicar/planning/planned_path.dart';
import 'package:aicar/recording/recording_schema.dart';
import 'package:aicar/recording/session_recorder.dart';
import 'package:aicar/recording/session_store.dart';
import 'package:aicar/replay/replay_reader.dart';
import 'package:aicar/sensors/ego_motion.dart';
import 'package:aicar/simulation/simulated_control.dart';
import 'package:aicar/simulation/vehicle_state.dart';
import 'package:aicar/pipeline/pipeline_result.dart';
import 'package:aicar/tracking/object_track.dart';
import 'package:aicar/world_model/world_state.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:path/path.dart' as p;

import '../support/world_builder.dart';

PipelineResult buildResult(int frameId, int ts, {DrivingState? state}) {
  final WorldState world = testWorld(
    frameId: frameId,
    timestampMicros: ts,
    tracks: <ObjectTrack>[
      testTrack(
        id: 7,
        objectClass: ObjectClass.car,
        position: Vec2(0.2, 30 - frameId.toDouble()),
        worldVelocity: const Vec2(0, 12),
      ),
    ],
  );
  final PlannedPath path = LocalPathPlanner().plan(world);
  return PipelineResult(
    world: world,
    path: path,
    decision: DrivingDecision(
      state: state ?? DrivingState.cruise,
      reason: 'test reason $frameId',
      confidence: 0.82,
      timestampMicros: ts,
      frameId: frameId,
      targetSpeedMps: 12,
    ),
    command: SimulatedControlCommand(
      steeringAngleDegrees: -3.5,
      throttlePercent: 21,
      brakePercent: 0,
      timestampMicros: ts,
      frameId: frameId,
    ),
    vehicle: VehicleState.stationary(timestampMicros: ts)
        .copyWith(speedMps: 13.4, distanceTravelledMeters: frameId * 0.7),
    collisions: const <dynamic>[].cast(),
    totalLatencyMicros: 48000,
    stageTimings: const <String, double>{'Total Pipeline': 48.0},
  );
}

void main() {
  late Directory root;

  setUp(() async {
    root = await Directory.systemTemp.createTemp('aicar_rec_test');
  });

  tearDown(() async {
    if (await root.exists()) await root.delete(recursive: true);
  });

  SessionHeader header(String id) => SessionHeader(
        sessionId: id,
        startedAt: DateTime(2026, 3, 4, 9, 30),
        schemaVersion: recordingSchemaVersion,
        appVersion: '0.14.0',
        deviceModel: 'SM-S911B',
        androidVersion: '14',
        calibration: testCalibration.toJson(),
        models: const <String, String>{
          'objectDetection': 'yolov8n-640-fp16',
          'depthEstimation': 'none',
        },
        pipelineConfig: const <String, dynamic>{'inference': 'medium'},
      );

  group('SessionRecorder', () {
    test('writes a header, records and a footer', () async {
      final SessionRecorder recorder = SessionRecorder(
        mode: RecordingMode.structuredOnly,
        rootDirectory: root,
      );
      await recorder.start(header);
      expect(recorder.isRecording, isTrue);

      for (int i = 0; i < 5; i++) {
        recorder.recordResult(buildResult(i, i * 50000));
        recorder.recordImu(ImuSample(
          accelerationMps2: const Vec3(0.1, 0.2, 9.8),
          angularRateRadPerS: const Vec3(0, 0, 0.01),
          timestampMicros: i * 50000 + 10000,
        ));
      }
      recorder.recordMarker('interesting bit');

      final SessionFooter? footer = await recorder.stop();
      expect(footer, isNotNull);
      expect(footer!.frameCount, 5);
      expect(footer.decisionCounts['cruise'], 5);

      final Directory dir = recorder.sessionDirectory!;
      expect(await File(p.join(dir.path, 'header.json')).exists(), isTrue);
      expect(await File(p.join(dir.path, 'footer.json')).exists(), isTrue);

      final List<String> lines =
          await File(p.join(dir.path, 'session.jsonl')).readAsLines();
      expect(lines.first, contains('"t":"header"'));
      expect(lines.last, contains('"t":"footer"'));
      expect(lines.where((String l) => l.contains('"t":"frame"')).length, 5);
      expect(lines.where((String l) => l.contains('"t":"imu"')).length, 5);
      expect(lines.where((String l) => l.contains('"t":"marker"')).length, 1);
    });

    test('every record line is valid JSON', () async {
      final SessionRecorder recorder = SessionRecorder(
        mode: RecordingMode.structuredOnly,
        rootDirectory: root,
      );
      await recorder.start(header);
      for (int i = 0; i < 3; i++) {
        recorder.recordResult(buildResult(i, i * 50000));
      }
      await recorder.stop();

      final List<String> lines = await File(
        p.join(recorder.sessionDirectory!.path, 'session.jsonl'),
      ).readAsLines();
      for (final String line in lines) {
        if (line.trim().isEmpty) continue;
        expect(() => jsonDecode(line), returnsNormally);
        expect(SessionRecord.decode(line), isNotNull);
      }
    });

    test('the header records that no vehicle was controlled', () async {
      final SessionRecorder recorder = SessionRecorder(
        mode: RecordingMode.structuredOnly,
        rootDirectory: root,
      );
      await recorder.start(header);
      await recorder.stop();

      final Map<String, dynamic> json = jsonDecode(
        await File(p.join(recorder.sessionDirectory!.path, 'header.json'))
            .readAsString(),
      ) as Map<String, dynamic>;
      expect(json['safetyMode'], 'SIMULATION_ONLY=true');
      expect(json['safetyBanner'], contains('no vehicle control'));
    });

    test('every control record is tagged as simulation output', () async {
      final SessionRecorder recorder = SessionRecorder(
        mode: RecordingMode.structuredOnly,
        rootDirectory: root,
      );
      await recorder.start(header);
      recorder.recordResult(buildResult(0, 0));
      await recorder.stop();

      final List<String> lines = await File(
        p.join(recorder.sessionDirectory!.path, 'session.jsonl'),
      ).readAsLines();
      final String control =
          lines.firstWhere((String l) => l.contains('"t":"control"'));
      expect(control, contains('SIMULATION_ONLY=true'));
    });
  });

  group('SessionStore', () {
    test('lists recorded drives newest first', () async {
      for (int s = 0; s < 3; s++) {
        final SessionRecorder recorder = SessionRecorder(
          mode: RecordingMode.structuredOnly,
          rootDirectory: root,
        );
        await recorder.start((String id) => SessionHeader(
              sessionId: id,
              startedAt: DateTime(2026, 3, 4 + s),
              schemaVersion: recordingSchemaVersion,
              appVersion: '0.14.0',
              deviceModel: 'SM-S911B',
              androidVersion: '14',
              calibration: testCalibration.toJson(),
              models: const <String, String>{},
              pipelineConfig: const <String, dynamic>{},
            ));
        recorder.recordResult(buildResult(0, 0));
        await recorder.stop();
      }

      final List<RecordedSession> sessions =
          await SessionStore(rootDirectory: root).list();
      expect(sessions, hasLength(3));
      expect(sessions.first.startedAt.isAfter(sessions.last.startedAt), isTrue);
      expect(sessions.first.isIncomplete, isFalse);
    });

    test('recovers a session that was cut short', () async {
      final SessionRecorder recorder = SessionRecorder(
        mode: RecordingMode.structuredOnly,
        rootDirectory: root,
      );
      await recorder.start(header);
      for (int i = 0; i < 4; i++) {
        recorder.recordResult(buildResult(i, i * 50000));
      }
      final Directory dir = recorder.sessionDirectory!;
      // Simulate the app being killed: flush the stream, then remove the
      // footer as if stop() never ran.
      await recorder.stop();
      await File(p.join(dir.path, 'footer.json')).delete();

      final List<RecordedSession> sessions =
          await SessionStore(rootDirectory: root).list();
      expect(sessions, hasLength(1));
      // The footer was recovered from the stream, so the drive is still
      // listed with its real statistics.
      expect(sessions.first.footer, isNotNull);
      expect(sessions.first.footer!.frameCount, 4);
      expect(sessions.first.footer!.decisionCounts['cruise'], 4);
    });

    test('deleting a drive removes it from disk', () async {
      final SessionRecorder recorder = SessionRecorder(
        mode: RecordingMode.structuredOnly,
        rootDirectory: root,
      );
      await recorder.start(header);
      recorder.recordResult(buildResult(0, 0));
      await recorder.stop();

      final SessionStore store = SessionStore(rootDirectory: root);
      final List<RecordedSession> before = await store.list();
      expect(before, hasLength(1));
      await store.delete(before.first);
      expect(await store.list(), isEmpty);
    });

    test('pruning removes the oldest drives first', () async {
      for (int s = 0; s < 3; s++) {
        final SessionRecorder recorder = SessionRecorder(
          mode: RecordingMode.structuredOnly,
          rootDirectory: root,
        );
        await recorder.start((String id) => SessionHeader(
              sessionId: id,
              startedAt: DateTime(2026, 3, 1 + s),
              schemaVersion: recordingSchemaVersion,
              appVersion: '0.14.0',
              deviceModel: 'SM-S911B',
              androidVersion: '14',
              calibration: testCalibration.toJson(),
              models: const <String, String>{},
              pipelineConfig: const <String, dynamic>{},
            ));
        for (int i = 0; i < 20; i++) {
          recorder.recordResult(buildResult(i, i * 50000));
        }
        await recorder.stop();
      }

      final SessionStore store = SessionStore(rootDirectory: root);
      final int total = await store.totalSizeBytes();
      expect(total, greaterThan(0));

      await store.pruneTo(total ~/ 2);
      final List<RecordedSession> remaining = await store.list();
      expect(remaining.length, lessThan(3));
      // The newest must survive.
      expect(remaining.first.startedAt, DateTime(2026, 3, 3));
    });
  });

  group('ReplayReader', () {
    test('reads back every recorded cycle in order', () async {
      final SessionRecorder recorder = SessionRecorder(
        mode: RecordingMode.structuredOnly,
        rootDirectory: root,
      );
      await recorder.start(header);
      for (int i = 0; i < 6; i++) {
        recorder.recordImu(ImuSample(
          accelerationMps2: const Vec3(0, 0, 9.81),
          angularRateRadPerS: const Vec3(0, 0, 0.02),
          timestampMicros: i * 50000 - 10000,
        ));
        recorder.recordResult(buildResult(i, i * 50000));
      }
      await recorder.stop();

      final RecordedSession session =
          (await SessionStore(rootDirectory: root).list()).first;
      final ReplayReader reader = ReplayReader(session);

      expect(await reader.countFrames(), 6);

      final List<ReplayFrame> frames = await reader.frames().toList();
      expect(frames, hasLength(6));
      for (int i = 0; i < frames.length; i++) {
        expect(frames[i].frameId, i);
        expect(frames[i].timestampMicros, i * 50000);
        // The IMU sample written just before each frame is attached to it.
        expect(frames[i].imuSamples, hasLength(1));
        expect(frames[i].decision, isNotNull);
        expect(frames[i].decision!.reason, 'test reason $i');
        expect(frames[i].command, isNotNull);
        expect(frames[i].command!.throttlePercent, closeTo(21, 0.1));
        expect(frames[i].vehicle, isNotNull);
      }
    });

    test('restores the world model and the planned path', () async {
      final SessionRecorder recorder = SessionRecorder(
        mode: RecordingMode.structuredOnly,
        rootDirectory: root,
      );
      await recorder.start(header);
      recorder.recordResult(buildResult(3, 150000));
      await recorder.stop();

      final RecordedSession session =
          (await SessionStore(rootDirectory: root).list()).first;
      final ReplayFrame frame =
          await ReplayReader(session).frames().first;

      expect(frame.world, isNotNull);
      expect(frame.world!.tracks, hasLength(1));
      expect(frame.world!.tracks.first.id, 7);
      expect(frame.world!.tracks.first.objectClass, ObjectClass.car);
      expect(frame.world!.lanes.laneWidthMeters, closeTo(3.5, 0.01));
      expect(frame.world!.autonomy.overall, greaterThan(0));

      expect(frame.path, isNotNull);
      expect(frame.path!.points, isNotEmpty);
      expect(frame.path!.source, PathSource.laneCenterline);
    });

    test('uses the calibration the drive was recorded with', () async {
      final SessionRecorder recorder = SessionRecorder(
        mode: RecordingMode.structuredOnly,
        rootDirectory: root,
      );
      await recorder.start(header);
      recorder.recordResult(buildResult(0, 0));
      await recorder.stop();

      final RecordedSession session =
          (await SessionStore(rootDirectory: root).list()).first;
      final ReplayReader reader = ReplayReader(session);
      expect(reader.calibration.cameraHeightMeters,
          testCalibration.cameraHeightMeters);
      expect(reader.calibration.pitchDegrees, testCalibration.pitchDegrees);
      expect(reader.calibration.isCalibrated, isTrue);
    });

    test('reads markers for the replay timeline', () async {
      final SessionRecorder recorder = SessionRecorder(
        mode: RecordingMode.structuredOnly,
        rootDirectory: root,
      );
      await recorder.start(header);
      recorder.recordResult(buildResult(0, 0));
      recorder.recordMarker('near miss', timestampMicros: 40000);
      recorder.recordResult(buildResult(1, 50000));
      await recorder.stop();

      final RecordedSession session =
          (await SessionStore(rootDirectory: root).list()).first;
      final List<({int timestampMicros, String label})> markers =
          await ReplayReader(session).markers();
      expect(markers, hasLength(1));
      expect(markers.first.label, 'near miss');
    });

    test('a truncated final line does not break the replay', () async {
      final SessionRecorder recorder = SessionRecorder(
        mode: RecordingMode.structuredOnly,
        rootDirectory: root,
      );
      await recorder.start(header);
      for (int i = 0; i < 3; i++) {
        recorder.recordResult(buildResult(i, i * 50000));
      }
      await recorder.stop();

      final File stream = File(
        p.join(recorder.sessionDirectory!.path, 'session.jsonl'),
      );
      await stream.writeAsString(
        '${await stream.readAsString()}{"t":"frame","ts":999,"frame',
      );

      final RecordedSession session =
          (await SessionStore(rootDirectory: root).list()).first;
      final List<ReplayFrame> frames =
          await ReplayReader(session).frames().toList();
      expect(frames, hasLength(3));
    });
  });
}
