import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:crypto/crypto.dart';
import 'package:http/http.dart' as http;
import 'package:path/path.dart' as p;

import '../core/logging.dart';
import 'model_descriptor.dart';
import 'model_registry.dart';

/// Progress of one model download.
class ModelDownloadProgress {
  const ModelDownloadProgress({
    required this.modelId,
    required this.receivedBytes,
    required this.totalBytes,
    this.stage = 'downloading',
  });

  final String modelId;
  final int receivedBytes;
  final int totalBytes;

  /// `downloading`, `verifying` or `installing`.
  final String stage;

  double? get fraction =>
      totalBytes <= 0 ? null : (receivedBytes / totalBytes).clamp(0.0, 1.0);

  String get label {
    final double mb = receivedBytes / 1024 / 1024;
    if (totalBytes <= 0) return '${mb.toStringAsFixed(1)} MB';
    final double totalMb = totalBytes / 1024 / 1024;
    return '${mb.toStringAsFixed(1)} / ${totalMb.toStringAsFixed(1)} MB';
  }
}

/// Fetches models the app does not ship, into the runtime model directory.
///
/// Some useful models are too large to bundle — MiDaS is 63 MB, which would
/// more than triple the APK for a capability most drives do not need. They
/// are fetched on request instead, which keeps the decision where it belongs:
/// a bigger model costs data, storage, battery and thermal headroom, and the
/// person holding the phone is the one who should be spending those.
///
/// Nothing is ever fetched in the background or on first run.
class ModelDownloader {
  ModelDownloader({required this.registry, http.Client? client})
      : _client = client ?? http.Client();

  static const String _tag = 'ModelDownloader';

  final ModelRegistry registry;
  final http.Client _client;

  final Map<String, ModelDownloadProgress> _active =
      <String, ModelDownloadProgress>{};

  Map<String, ModelDownloadProgress> get active =>
      Map<String, ModelDownloadProgress>.unmodifiable(_active);

  bool isDownloading(String modelId) => _active.containsKey(modelId);

  /// Download and install [descriptor].
  ///
  /// The file lands in a `.part` alongside its destination and is only moved
  /// into place once its SHA-256 matches. A model whose hash does not match
  /// is deleted, not run: the wrong weights do not fail loudly, they produce
  /// confident nonsense, which is the worst failure this stack can have.
  Future<InstalledModel> download(
    ModelDescriptor descriptor, {
    void Function(ModelDownloadProgress)? onProgress,
  }) async {
    final String? url = descriptor.downloadUrl;
    if (url == null) {
      throw ArgumentError('${descriptor.id} has no download URL');
    }
    if (_active.containsKey(descriptor.id)) {
      throw StateError('${descriptor.id} is already downloading');
    }

    final Directory dir = await registry.modelDirectory();
    final File target =
        File(p.join(dir.path, p.basename(descriptor.assetOrFilePath)));
    final File part = File('${target.path}.part');

    void report(ModelDownloadProgress progress) {
      _active[descriptor.id] = progress;
      onProgress?.call(progress);
    }

    report(ModelDownloadProgress(
      modelId: descriptor.id,
      receivedBytes: 0,
      totalBytes: descriptor.sizeBytes ?? 0,
    ));

    try {
      final http.StreamedResponse response =
          await _client.send(http.Request('GET', Uri.parse(url)));
      if (response.statusCode != 200) {
        throw HttpException(
          'server returned ${response.statusCode}',
          uri: Uri.parse(url),
        );
      }

      final int total =
          response.contentLength ?? descriptor.sizeBytes ?? 0;
      final IOSink sink = part.openWrite();
      int received = 0;
      try {
        await for (final List<int> chunk in response.stream) {
          sink.add(chunk);
          received += chunk.length;
          report(ModelDownloadProgress(
            modelId: descriptor.id,
            receivedBytes: received,
            totalBytes: total,
          ));
        }
      } finally {
        await sink.close();
      }

      final String? expected = descriptor.downloadSha256;
      if (expected != null) {
        report(ModelDownloadProgress(
          modelId: descriptor.id,
          receivedBytes: received,
          totalBytes: total,
          stage: 'verifying',
        ));
        final String actual = await _sha256Of(part);
        if (actual.toLowerCase() != expected.toLowerCase()) {
          await part.delete();
          throw StateError(
            'checksum mismatch for ${descriptor.id}: expected $expected, '
            'got $actual. The file was discarded rather than installed.',
          );
        }
      }

      report(ModelDownloadProgress(
        modelId: descriptor.id,
        receivedBytes: received,
        totalBytes: total,
        stage: 'installing',
      ));
      if (await target.exists()) await target.delete();
      await part.rename(target.path);

      await registry.refresh();
      final InstalledModel? installed = registry.installed[descriptor.id];
      if (installed == null) {
        throw StateError(
          'downloaded ${descriptor.id} but the registry did not pick it up',
        );
      }
      Log.info(_tag, 'installed ${descriptor.id} '
          '(${(received / 1024 / 1024).toStringAsFixed(1)} MB)');
      return installed;
    } catch (e) {
      if (await part.exists()) {
        await part.delete();
      }
      Log.error(_tag, 'download of ${descriptor.id} failed', e);
      rethrow;
    } finally {
      _active.remove(descriptor.id);
    }
  }

  /// Hash the file in chunks.
  ///
  /// Streaming rather than `sha256.convert(await file.readAsBytes())`: a
  /// 63 MB model read into memory whole, on a phone that is also holding
  /// camera buffers and an interpreter, is how an out-of-memory kill happens.
  static Future<String> _sha256Of(File file) async {
    final _DigestCatcher catcher = _DigestCatcher();
    final ByteConversionSink sink = sha256.startChunkedConversion(catcher);
    await for (final List<int> chunk in file.openRead()) {
      sink.add(chunk);
    }
    sink.close();
    return catcher.digest!.toString();
  }

  void dispose() => _client.close();
}

class _DigestCatcher implements Sink<Digest> {
  Digest? digest;

  @override
  void add(Digest data) => digest = data;

  @override
  void close() {}
}
