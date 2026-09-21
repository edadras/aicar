import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as p;
import 'package:path_provider/path_provider.dart';

import '../core/logging.dart';
import 'recording_schema.dart';

/// A recorded drive as the Recorded Drives screen sees it.
class RecordedSession {
  const RecordedSession({
    required this.id,
    required this.directory,
    required this.header,
    this.footer,
    required this.sizeBytes,
    required this.hasFrames,
    required this.frameImageCount,
  });

  final String id;
  final Directory directory;
  final SessionHeader header;
  final SessionFooter? footer;
  final int sizeBytes;

  /// Whether images were recorded. Without them the AI cannot be re-run, and
  /// the Replay screen says so rather than offering a mode that cannot work.
  final bool hasFrames;

  final int frameImageCount;

  DateTime get startedAt => header.startedAt;
  double get durationSeconds => footer?.durationSeconds ?? 0;
  double get distanceMeters => footer?.distanceMeters ?? 0;

  /// True when the session ended without a footer — the app was killed, the
  /// battery died, or the phone overheated. The data before that point is
  /// still valid, which is the main reason the stream format is append-only.
  bool get isIncomplete => footer == null;

  String get sizeLabel {
    if (sizeBytes < 1024 * 1024) return '${(sizeBytes / 1024).round()} KB';
    if (sizeBytes < 1024 * 1024 * 1024) {
      return '${(sizeBytes / 1024 / 1024).toStringAsFixed(1)} MB';
    }
    return '${(sizeBytes / 1024 / 1024 / 1024).toStringAsFixed(2)} GB';
  }

  String get durationLabel {
    final int seconds = durationSeconds.round();
    final int minutes = seconds ~/ 60;
    return minutes > 0
        ? '$minutes m ${seconds % 60} s'
        : '$seconds s';
  }

  String get distanceLabel => distanceMeters >= 1000
      ? '${(distanceMeters / 1000).toStringAsFixed(2)} km'
      : '${distanceMeters.round()} m';

  File get streamFile => File(p.join(directory.path, 'session.jsonl'));
  Directory get framesDirectory => Directory(p.join(directory.path, 'frames'));
}

/// Finds and manages recorded drives on disk.
///
/// The filesystem is the source of truth — a session is a directory — so a
/// recording copied off the device with `adb pull` remains complete and
/// self-describing. Nothing here depends on an index that could get out of
/// step with the files.
class SessionStore {
  SessionStore({Directory? rootDirectory}) : _explicitRoot = rootDirectory;

  static const String _tag = 'SessionStore';

  final Directory? _explicitRoot;
  Directory? _root;

  Future<Directory> rootDirectory() async {
    if (_root != null) return _root!;
    if (_explicitRoot != null) {
      _root = _explicitRoot;
    } else {
      final Directory documents = await getApplicationDocumentsDirectory();
      _root = Directory(p.join(documents.path, 'recordings'));
    }
    if (!await _root!.exists()) await _root!.create(recursive: true);
    return _root!;
  }

  /// List every recorded drive, newest first.
  Future<List<RecordedSession>> list() async {
    final Directory root = await rootDirectory();
    final List<RecordedSession> sessions = <RecordedSession>[];

    await for (final FileSystemEntity entity in root.list()) {
      if (entity is! Directory) continue;
      final RecordedSession? session = await _load(entity);
      if (session != null) sessions.add(session);
    }

    sessions.sort((RecordedSession a, RecordedSession b) =>
        b.startedAt.compareTo(a.startedAt));
    return sessions;
  }

  Future<RecordedSession?> _load(Directory dir) async {
    final File headerFile = File(p.join(dir.path, 'header.json'));
    if (!await headerFile.exists()) return null;

    SessionHeader header;
    try {
      header = SessionHeader.fromJson(
        jsonDecode(await headerFile.readAsString()) as Map<String, dynamic>,
      );
    } catch (e) {
      Log.warn(_tag, 'unreadable header in ${dir.path}: $e');
      return null;
    }

    SessionFooter? footer;
    final File footerFile = File(p.join(dir.path, 'footer.json'));
    if (await footerFile.exists()) {
      try {
        footer = SessionFooter.fromJson(
          jsonDecode(await footerFile.readAsString()) as Map<String, dynamic>,
        );
      } catch (e) {
        Log.warn(_tag, 'unreadable footer in ${dir.path}: $e');
      }
    } else {
      // No footer: recover what we can by scanning the stream's tail.
      footer = await _recoverFooter(dir);
    }

    final Directory frames = Directory(p.join(dir.path, 'frames'));
    int frameImages = 0;
    bool hasFrames = false;
    if (await frames.exists()) {
      hasFrames = true;
      await for (final FileSystemEntity e in frames.list()) {
        if (e is File && e.path.endsWith('.jpg')) frameImages++;
      }
    }

    int size = 0;
    await for (final FileSystemEntity e in dir.list(recursive: true)) {
      if (e is File) {
        try {
          size += await e.length();
        } catch (_) {
          // A file can vanish mid-scan if a recording is being cleaned up.
        }
      }
    }

    return RecordedSession(
      id: p.basename(dir.path),
      directory: dir,
      header: header,
      footer: footer,
      sizeBytes: size,
      hasFrames: hasFrames && frameImages > 0,
      frameImageCount: frameImages,
    );
  }

  /// Rebuild an approximate footer for a session that was cut short.
  ///
  /// An interrupted drive is common (the app is killed in the background, the
  /// phone thermally throttles and is put away) and its data is perfectly
  /// usable, so the store recovers rather than hiding it.
  Future<SessionFooter?> _recoverFooter(Directory dir) async {
    final File stream = File(p.join(dir.path, 'session.jsonl'));
    if (!await stream.exists()) return null;

    int frames = 0;
    int lastTimestamp = 0;
    int firstTimestamp = -1;
    double distance = 0;
    final Map<String, int> decisions = <String, int>{};

    try {
      final Stream<String> lines = stream
          .openRead()
          .transform(utf8.decoder)
          .transform(const LineSplitter());
      await for (final String line in lines) {
        final SessionRecord? record = SessionRecord.decode(line);
        if (record == null) continue;
        if (firstTimestamp < 0 && record.timestampMicros > 0) {
          firstTimestamp = record.timestampMicros;
        }
        if (record.timestampMicros > lastTimestamp) {
          lastTimestamp = record.timestampMicros;
        }
        switch (record.type) {
          case RecordType.frame:
            frames++;
          case RecordType.decision:
            final Map<String, dynamic>? d =
                record.payload['decision'] as Map<String, dynamic>?;
            final String? state = d?['state'] as String?;
            if (state != null) {
              decisions.update(state, (int v) => v + 1, ifAbsent: () => 1);
            }
          case RecordType.control:
            final Map<String, dynamic>? v =
                record.payload['vehicle'] as Map<String, dynamic>?;
            final num? odo = v?['odo'] as num?;
            if (odo != null) distance = odo.toDouble();
          default:
            break;
        }
      }
    } catch (e) {
      Log.warn(_tag, 'could not recover footer for ${dir.path}: $e');
      return null;
    }

    if (frames == 0) return null;
    final double duration =
        firstTimestamp < 0 ? 0 : (lastTimestamp - firstTimestamp) / 1e6;

    return SessionFooter(
      endedAt: (await stream.stat()).modified,
      durationSeconds: duration,
      frameCount: frames,
      distanceMeters: distance,
      meanProcessingFps: duration > 0 ? frames / duration : 0,
      droppedFrames: 0,
      decisionCounts: decisions,
    );
  }

  Future<void> delete(RecordedSession session) async {
    if (await session.directory.exists()) {
      await session.directory.delete(recursive: true);
      Log.info(_tag, 'deleted ${session.id}');
    }
  }

  /// Total space used by all recordings.
  Future<int> totalSizeBytes() async {
    final List<RecordedSession> sessions = await list();
    int total = 0;
    for (final RecordedSession s in sessions) {
      total += s.sizeBytes;
    }
    return total;
  }

  /// Delete oldest sessions until the total is under [limitBytes].
  ///
  /// Long drives fill a phone quickly; without this the app eventually
  /// fails to write mid-drive, which is the worst possible time to find out.
  Future<int> pruneTo(int limitBytes) async {
    final List<RecordedSession> sessions = await list();
    int total = sessions.fold<int>(
        0, (int sum, RecordedSession s) => sum + s.sizeBytes);
    int deleted = 0;

    for (int i = sessions.length - 1; i >= 0 && total > limitBytes; i--) {
      await delete(sessions[i]);
      total -= sessions[i].sizeBytes;
      deleted++;
    }
    if (deleted > 0) {
      Log.info(_tag, 'pruned $deleted session(s) to stay under '
          '${(limitBytes / 1024 / 1024).round()} MB');
    }
    return deleted;
  }
}
