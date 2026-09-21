import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

import 'package:image/image.dart' as img;
import 'package:path/path.dart' as p;
import 'package:path_provider/path_provider.dart';

import '../camera/camera_frame.dart';
import '../core/logging.dart';
import '../core/profiling.dart';
import '../decision/driving_decision.dart';
import '../pipeline/pipeline_result.dart';
import '../sensors/ego_motion.dart';
import 'recording_schema.dart';

/// How much of a drive to record.
enum RecordingMode {
  /// Structured data only. Tiny (a few MB per hour) and enough to review
  /// decisions, but the AI cannot be re-run because there are no images.
  structuredOnly('Data only'),

  /// Structured data plus JPEG frames at a reduced rate. This is the mode
  /// that makes "re-run the AI over this drive" possible.
  withFrames('Data + frames'),

  /// Structured data plus every frame. Large; for short diagnostic captures.
  withAllFrames('Data + every frame');

  const RecordingMode(this.label);
  final String label;

  bool get savesFrames => this != RecordingMode.structuredOnly;
}

/// Writes a drive to disk as a JSONL stream plus a directory of JPEG frames.
///
/// JSONL rather than a database for the stream: it is append-only (so a crash
/// or a battery pull costs at most the last line), it is readable by anything,
/// and it streams without loading a session into memory. The SQLite index in
/// [SessionStore] exists alongside it purely so the Recorded Drives screen can
/// list sessions without parsing them.
///
/// Image encoding happens off the pipeline's critical path: frames are queued
/// and written by a separate pump, so a slow flash write cannot stall
/// perception.
class SessionRecorder {
  SessionRecorder({
    this.mode = RecordingMode.withFrames,
    this.frameStrideForImages = 2,
    this.jpegQuality = 70,
    this.maxQueuedFrames = 6,
    this.flushEveryRecords = 40,
    Directory? rootDirectory,
  }) : _explicitRoot = rootDirectory;

  static const String _tag = 'SessionRecorder';

  final RecordingMode mode;

  /// Save an image every N processed frames. At 15 FPS a stride of 2 gives
  /// ~7 images per second, which is plenty to re-run perception against.
  final int frameStrideForImages;

  final int jpegQuality;

  /// Dropping an *image* under load is acceptable; dropping a structured
  /// record is not, so only the image queue is bounded.
  final int maxQueuedFrames;

  final int flushEveryRecords;
  final Directory? _explicitRoot;

  Directory? _sessionDirectory;

  // Closed in stop(), after the buffer is drained. The analyzer cannot follow
  // the ownership across the async drain, hence the ignore.
  // ignore: close_sinks
  IOSink? _sink;
  String? _sessionId;
  DateTime? _startedAt;

  /// Lines waiting to be written.
  ///
  /// Records are buffered rather than written straight to the sink because an
  /// `IOSink` cannot be written to while a `flush()` is in flight — doing so
  /// throws "StreamSink is bound to a stream" and kills the recording. All
  /// writing happens inside [_drain], which is serialised against itself, so
  /// a write and a flush can never overlap. It also batches syscalls, which
  /// keeps the flash out of the frame budget.
  final List<String> _buffer = <String>[];
  bool _draining = false;
  Timer? _flushTimer;
  int _frameCount = 0;
  int _imageCount = 0;
  int _droppedImages = 0;
  double _distanceMeters = 0;
  final Map<String, int> _decisionCounts = <String, int>{};

  final List<_PendingImage> _imageQueue = <_PendingImage>[];
  bool _imagePumpRunning = false;
  bool _recording = false;

  bool get isRecording => _recording;
  String? get sessionId => _sessionId;
  Directory? get sessionDirectory => _sessionDirectory;
  int get frameCount => _frameCount;
  int get imageCount => _imageCount;
  int get droppedImages => _droppedImages;
  double get distanceMeters => _distanceMeters;

  /// Bytes written so far, approximately.
  Future<int> approximateSizeBytes() async {
    final Directory? dir = _sessionDirectory;
    if (dir == null || !await dir.exists()) return 0;
    int total = 0;
    await for (final FileSystemEntity e in dir.list(recursive: true)) {
      if (e is File) total += await e.length();
    }
    return total;
  }

  Future<Directory> _rootDirectory() async {
    if (_explicitRoot != null) return _explicitRoot;
    final Directory documents = await getApplicationDocumentsDirectory();
    return Directory(p.join(documents.path, 'recordings'));
  }

  Future<void> start(SessionHeader Function(String sessionId) buildHeader) async {
    if (_recording) return;

    final DateTime now = DateTime.now();
    final String id = 'drive_${now.toIso8601String()
            .replaceAll(RegExp(r'[:.]'), '-')}';
    final Directory root = await _rootDirectory();
    final Directory dir = Directory(p.join(root.path, id));
    await dir.create(recursive: true);
    if (mode.savesFrames) {
      await Directory(p.join(dir.path, 'frames')).create(recursive: true);
    }

    _sessionId = id;
    _sessionDirectory = dir;
    _startedAt = now;
    _frameCount = 0;
    _imageCount = 0;
    _droppedImages = 0;
    _distanceMeters = 0;
    _decisionCounts.clear();

    _sink = File(p.join(dir.path, 'session.jsonl'))
        .openWrite(mode: FileMode.writeOnly);
    _recording = true;
    // Also drain on a timer: a quiet session (stopped at traffic lights, few
    // records) would otherwise keep its last minute only in memory, which is
    // exactly the data lost if the app is killed.
    _flushTimer = Timer.periodic(
      const Duration(seconds: 2),
      (_) => unawaited(_drain()),
    );

    final SessionHeader header = buildHeader(id);
    _write(SessionRecord(
      type: RecordType.sessionHeader,
      timestampMicros: 0,
      payload: header.toJson(),
    ));
    // The header also goes in its own file so the drive list can read it
    // without touching the (potentially large) stream.
    await File(p.join(dir.path, 'header.json')).writeAsString(
      const JsonEncoder.withIndent('  ').convert(header.toJson()),
    );

    Log.info(_tag, 'recording started: ${dir.path} (${mode.label})');
  }

  /// Record one pipeline cycle.
  void recordResult(PipelineResult result, {CameraFrame? frame}) {
    if (!_recording) return;

    _frameCount++;
    _decisionCounts.update(
      result.decision.state.name,
      (int v) => v + 1,
      ifAbsent: () => 1,
    );
    _distanceMeters = result.vehicle.distanceTravelledMeters;

    String? imagePath;
    if (mode.savesFrames &&
        frame != null &&
        (mode == RecordingMode.withAllFrames ||
            _frameCount % frameStrideForImages == 0)) {
      imagePath = p.join('frames', 'frame_${result.frameId}.jpg');
      _enqueueImage(frame, imagePath);
    }

    for (final SessionRecord r
        in recordsForResult(result, imagePath: imagePath)) {
      _write(r);
    }
  }

  /// Record a raw IMU sample. These arrive far faster than frames and are
  /// what make an accurate replay of ego motion possible.
  void recordImu(ImuSample sample) {
    if (!_recording) return;
    _write(SessionRecord(
      type: RecordType.imu,
      timestampMicros: sample.timestampMicros,
      payload: sample.toJson(),
    ));
  }

  void recordGps(GeoPosition fix) {
    if (!_recording) return;
    _write(SessionRecord(
      type: RecordType.gps,
      timestampMicros: fix.timestampMicros,
      payload: fix.toJson(),
    ));
  }

  /// Drop a user-visible marker into the stream — "something happened here".
  void recordMarker(String label, {int? timestampMicros}) {
    if (!_recording) return;
    _write(SessionRecord(
      type: RecordType.marker,
      timestampMicros:
          timestampMicros ?? DateTime.now().microsecondsSinceEpoch,
      payload: <String, dynamic>{'label': label},
    ));
    Log.info(_tag, 'marker: $label');
  }

  void _write(SessionRecord record) {
    if (_sink == null) return;
    _buffer.add(record.encode());
    if (_buffer.length >= flushEveryRecords) unawaited(_drain());
  }

  /// Write everything buffered, one batch at a time. Re-entrant calls return
  /// immediately; the in-flight drain picks up whatever they added.
  Future<void> _drain() async {
    if (_draining) return;
    final IOSink? sink = _sink;
    if (sink == null) return;

    _draining = true;
    try {
      while (_buffer.isNotEmpty) {
        final String chunk = _buffer.join('\n');
        _buffer.clear();
        sink.writeln(chunk);
        await sink.flush();
      }
    } catch (e) {
      Log.warn(_tag, 'write failed: $e');
    } finally {
      _draining = false;
    }
  }

  // --- image pump ---------------------------------------------------------

  void _enqueueImage(CameraFrame frame, String relativePath) {
    if (_imageQueue.length >= maxQueuedFrames) {
      // Under sustained load, dropping an image is the right trade: the
      // structured record still describes the frame, and perception must not
      // wait for the flash.
      _droppedImages++;
      return;
    }
    _imageQueue.add(_PendingImage(
      bytes: Uint8List.fromList(frame.bytes),
      width: frame.width,
      height: frame.height,
      channels: frame.bytesPerPixel,
      relativePath: relativePath,
    ));
    if (!_imagePumpRunning) unawaited(_pumpImages());
  }

  Future<void> _pumpImages() async {
    _imagePumpRunning = true;
    try {
      while (_imageQueue.isNotEmpty) {
        final _PendingImage pending = _imageQueue.removeAt(0);
        final Directory? dir = _sessionDirectory;
        if (dir == null) continue;
        try {
          final Uint8List jpeg = _encodeJpeg(pending);
          await File(p.join(dir.path, pending.relativePath))
              .writeAsBytes(jpeg, flush: false);
          _imageCount++;
        } catch (e) {
          Log.warn(_tag, 'failed to write ${pending.relativePath}: $e');
        }
        // Yield between frames so the encoder cannot monopolise the isolate.
        await Future<void>.delayed(Duration.zero);
      }
    } finally {
      _imagePumpRunning = false;
    }
  }

  Uint8List _encodeJpeg(_PendingImage pending) {
    final img.Image image = img.Image(
      width: pending.width,
      height: pending.height,
      numChannels: 3,
    );
    if (pending.channels == 3) {
      for (int y = 0; y < pending.height; y++) {
        for (int x = 0; x < pending.width; x++) {
          final int i = (y * pending.width + x) * 3;
          image.setPixelRgb(
            x,
            y,
            pending.bytes[i],
            pending.bytes[i + 1],
            pending.bytes[i + 2],
          );
        }
      }
    } else {
      for (int y = 0; y < pending.height; y++) {
        for (int x = 0; x < pending.width; x++) {
          final int l = pending.bytes[y * pending.width + x];
          image.setPixelRgb(x, y, l, l, l);
        }
      }
    }
    return img.encodeJpg(image, quality: jpegQuality);
  }

  // --- lifecycle ----------------------------------------------------------

  Future<SessionFooter?> stop({PipelineProfiler? profiler}) async {
    if (!_recording) return null;
    _recording = false;
    _flushTimer?.cancel();
    _flushTimer = null;

    // Let queued images finish before closing the session.
    int guard = 0;
    while (_imageQueue.isNotEmpty && guard++ < 200) {
      await Future<void>.delayed(const Duration(milliseconds: 25));
    }

    final DateTime now = DateTime.now();
    final SessionFooter footer = SessionFooter(
      endedAt: now,
      durationSeconds: _startedAt == null
          ? 0
          : now.difference(_startedAt!).inMilliseconds / 1000,
      frameCount: _frameCount,
      distanceMeters: _distanceMeters,
      meanProcessingFps: profiler?.processingFps ?? 0,
      droppedFrames: profiler?.droppedFrames ?? 0,
      decisionCounts: Map<String, int>.from(_decisionCounts),
    );

    _write(SessionRecord(
      type: RecordType.sessionFooter,
      timestampMicros: now.microsecondsSinceEpoch,
      payload: footer.toJson(),
    ));

    await _drain();
    // A drain already in flight when stop() was called may still be running;
    // wait for it before closing, or the close will race the last write.
    int drainGuard = 0;
    while (_draining && drainGuard++ < 200) {
      await Future<void>.delayed(const Duration(milliseconds: 10));
    }

    final IOSink? sink = _sink;
    _sink = null;
    if (sink != null) {
      await sink.flush();
      await sink.close();
    }

    final Directory? dir = _sessionDirectory;
    if (dir != null) {
      await File(p.join(dir.path, 'footer.json')).writeAsString(
        const JsonEncoder.withIndent('  ').convert(footer.toJson()),
      );
    }

    Log.info(
      _tag,
      'recording stopped: $_frameCount frames, $_imageCount images'
      '${_droppedImages > 0 ? ', $_droppedImages images dropped' : ''}',
    );
    return footer;
  }

  /// Most frequent decision state, for the drive summary.
  String get dominantDecision {
    String best = '-';
    int bestCount = 0;
    for (final MapEntry<String, int> e in _decisionCounts.entries) {
      if (e.value > bestCount) {
        bestCount = e.value;
        best = e.key;
      }
    }
    return best;
  }

  Map<String, int> get decisionCounts =>
      Map<String, int>.unmodifiable(_decisionCounts);
}

class _PendingImage {
  const _PendingImage({
    required this.bytes,
    required this.width,
    required this.height,
    required this.channels,
    required this.relativePath,
  });

  final Uint8List bytes;
  final int width;
  final int height;
  final int channels;
  final String relativePath;
}

/// Counts of each decision state, for the drive list.
Map<String, int> summariseDecisions(Iterable<DrivingDecision> decisions) {
  final Map<String, int> counts = <String, int>{};
  for (final DrivingDecision d in decisions) {
    counts.update(d.state.name, (int v) => v + 1, ifAbsent: () => 1);
  }
  return counts;
}
