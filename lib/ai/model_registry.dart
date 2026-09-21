import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as p;
import 'package:path_provider/path_provider.dart';
import 'package:shared_preferences/shared_preferences.dart';

import '../core/logging.dart';
import 'model_catalog.dart';
import 'model_descriptor.dart';

/// A model available on the device, paired with the descriptor that says how
/// to run it.
///
/// Two things can back one: a file the user installed into the model
/// directory, or an asset compiled into the APK. The rest of the app does not
/// care which — only [uninstall] does, because an asset cannot be deleted.
class InstalledModel {
  const InstalledModel({
    required this.descriptor,
    required this.file,
    required this.sizeBytes,
    required this.installedAt,
  });

  /// A model that ships inside the APK. It has no [file]: the native runtime
  /// resolves [ModelDescriptor.assetOrFilePath] against the asset bundle.
  InstalledModel.bundled(this.descriptor)
      : file = null,
        sizeBytes = descriptor.sizeBytes ?? 0,
        installedAt = null;

  final ModelDescriptor descriptor;

  /// The backing file, or `null` for a bundled asset.
  final File? file;
  final int sizeBytes;

  /// When the file landed on disk, or `null` for a bundled asset — it has
  /// been there since the app was installed.
  final DateTime? installedAt;

  bool get isBundled => file == null;

  String get sizeLabel {
    if (sizeBytes < 1024 * 1024) return '${(sizeBytes / 1024).round()} KB';
    return '${(sizeBytes / 1024 / 1024).toStringAsFixed(1)} MB';
  }
}

/// Discovers, validates and remembers the models available on the device.
///
/// Two sources feed it:
///
///  * the models bundled in the APK, so the app has working perception on
///    first launch with nothing to download;
///  * `.tflite` files the user drops into the app's model directory (via the
///    AI Models screen or `adb push`), matched against [ModelCatalog] by
///    filename or described by a `<name>.json` sidecar.
///
/// An installed file **shadows** a bundled model with the same id: a user who
/// pushes a newer export expects it to be the one that runs. This is the
/// mechanism that makes the "swap any model without touching the app"
/// requirement real rather than aspirational.
class ModelRegistry {
  ModelRegistry({Directory? modelDirectory}) : _explicitDir = modelDirectory;

  static const String _tag = 'ModelRegistry';
  static const String _prefsKeyPrefix = 'model.selected.';

  final Directory? _explicitDir;
  Directory? _dir;

  final Map<String, InstalledModel> _installed = <String, InstalledModel>{};
  final Map<ModelRole, String> _selection = <ModelRole, String>{};

  Map<String, InstalledModel> get installed =>
      Map<String, InstalledModel>.unmodifiable(_installed);

  /// Directory the user should push model files into.
  Future<Directory> modelDirectory() async {
    if (_dir != null) return _dir!;
    if (_explicitDir != null) {
      _dir = _explicitDir;
    } else {
      final Directory support = await getApplicationSupportDirectory();
      _dir = Directory(p.join(support.path, 'models'));
    }
    if (!await _dir!.exists()) {
      await _dir!.create(recursive: true);
    }
    return _dir!;
  }

  /// Scan the model directory and rebuild the installed-model map.
  Future<void> refresh() async {
    _installed.clear();
    final Directory dir = await modelDirectory();

    final List<FileSystemEntity> entries = await dir.list().toList();
    final Map<String, File> sidecars = <String, File>{};
    final List<File> weightFiles = <File>[];

    for (final FileSystemEntity e in entries) {
      if (e is! File) continue;
      final String ext = p.extension(e.path).toLowerCase();
      if (ext == '.json') {
        sidecars[p.basenameWithoutExtension(e.path)] = e;
      } else if (ext == '.tflite' || ext == '.bin') {
        weightFiles.add(e);
      }
    }

    for (final File f in weightFiles) {
      final String base = p.basenameWithoutExtension(f.path);
      final String fileName = p.basename(f.path);
      ModelDescriptor? descriptor;

      // 1. A sidecar always wins: it is the user's explicit statement of how
      //    to run this file.
      final File? sidecar = sidecars[base];
      if (sidecar != null) {
        try {
          descriptor = ModelDescriptor.fromJson(
            jsonDecode(await sidecar.readAsString()) as Map<String, dynamic>,
          );
        } catch (e) {
          Log.warn(_tag, 'invalid sidecar ${sidecar.path}: $e');
        }
      }

      // 2. Otherwise match the catalog by the filename it expects. Compare
      //    basenames: a bundled entry carries a full asset key, and a user
      //    pushing a newer export of it drops a bare filename in here.
      descriptor ??= ModelCatalog.all.values.cast<ModelDescriptor?>().firstWhere(
            (ModelDescriptor? d) => p.basename(d!.assetOrFilePath) == fileName,
            orElse: () => null,
          );

      if (descriptor == null) {
        Log.warn(
          _tag,
          'ignoring $fileName: unknown model and no $base.json sidecar. '
          'See docs/MODELS.md for the sidecar format.',
        );
        continue;
      }

      final FileStat stat = await f.stat();
      _installed[descriptor.id] = InstalledModel(
        descriptor: descriptor.copyWith(
          assetOrFilePath: f.path,
          isBundledAsset: false,
          sizeBytes: stat.size,
        ),
        file: f,
        sizeBytes: stat.size,
        installedAt: stat.modified,
      );
      Log.info(_tag, 'found ${descriptor.id} (${stat.size ~/ 1024} KB)');
    }

    // Bundled assets fill in behind whatever the user installed: an installed
    // file with the same id wins, because it is the more deliberate choice.
    for (final ModelDescriptor d in ModelCatalog.all.values) {
      if (!d.isBundledAsset) continue;
      if (_installed.containsKey(d.id)) {
        Log.info(_tag, 'bundled ${d.id} shadowed by an installed file');
        continue;
      }
      _installed[d.id] = InstalledModel.bundled(d);
      Log.info(_tag, 'bundled ${d.id} available');
    }

    await _loadSelection();
  }

  Future<void> _loadSelection() async {
    final SharedPreferences prefs = await SharedPreferences.getInstance();
    for (final ModelRole role in ModelRole.values) {
      final String? id = prefs.getString('$_prefsKeyPrefix${role.name}');
      if (id != null && _installed.containsKey(id)) {
        _selection[role] = id;
      } else {
        _selection.remove(role);
      }
    }
    // Auto-select a role the user has not chosen for. A user who installs one
    // detector expects it to just work, and a role with a bundled model should
    // never sit idle — but an ambiguous set of installed files is the user's
    // call, so we fall back to the bundled model rather than guessing between
    // them.
    for (final ModelRole role in ModelRole.values) {
      if (_selection.containsKey(role)) continue;
      final List<InstalledModel> candidates = forRole(role);
      final List<InstalledModel> pushed = candidates
          .where((InstalledModel m) => !m.isBundled)
          .toList();
      final InstalledModel? pick = pushed.length == 1
          ? pushed.first
          : candidates
              .cast<InstalledModel?>()
              .firstWhere((InstalledModel? m) => m!.isBundled,
                  orElse: () => null);
      if (pick != null) _selection[role] = pick.descriptor.id;
    }
  }

  List<InstalledModel> forRole(ModelRole role) => _installed.values
      .where((InstalledModel m) => m.descriptor.role == role)
      .toList();

  ModelDescriptor? selectedFor(ModelRole role) {
    final String? id = _selection[role];
    if (id == null) return null;
    return _installed[id]?.descriptor;
  }

  Future<void> select(ModelRole role, String? modelId) async {
    final SharedPreferences prefs = await SharedPreferences.getInstance();
    if (modelId == null) {
      _selection.remove(role);
      await prefs.remove('$_prefsKeyPrefix${role.name}');
    } else {
      if (!_installed.containsKey(modelId)) {
        throw ArgumentError('model $modelId is not installed');
      }
      _selection[role] = modelId;
      await prefs.setString('$_prefsKeyPrefix${role.name}', modelId);
    }
    Log.info(_tag, 'role ${role.name} -> ${modelId ?? 'none'}');
  }

  /// Install a model file that the user picked from storage.
  Future<InstalledModel> installFromFile(
    File source, {
    ModelDescriptor? descriptor,
  }) async {
    final Directory dir = await modelDirectory();
    final String targetName =
        descriptor?.assetOrFilePath ?? p.basename(source.path);
    final File target = File(p.join(dir.path, p.basename(targetName)));
    await source.copy(target.path);

    if (descriptor != null) {
      final File sidecar = File(
        p.join(dir.path, '${p.basenameWithoutExtension(target.path)}.json'),
      );
      await sidecar.writeAsString(
        const JsonEncoder.withIndent('  ').convert(descriptor.toJson()),
      );
    }

    await refresh();
    final InstalledModel? installed = descriptor == null
        ? _installed.values
            .cast<InstalledModel?>()
            .firstWhere((InstalledModel? m) => m!.file?.path == target.path,
                orElse: () => null)
        : _installed[descriptor.id];
    if (installed == null) {
      throw StateError(
        'Installed ${p.basename(target.path)} but could not determine how to '
        'run it. Provide a descriptor sidecar — see docs/MODELS.md.',
      );
    }
    return installed;
  }

  Future<void> uninstall(String modelId) async {
    final InstalledModel? m = _installed[modelId];
    if (m == null) return;
    final File? file = m.file;
    if (file == null) {
      throw ArgumentError(
        '$modelId ships inside the app and cannot be uninstalled. '
        'Select a different model for its role instead.',
      );
    }
    if (await file.exists()) await file.delete();
    final File sidecar = File(
      p.join(file.parent.path,
          '${p.basenameWithoutExtension(file.path)}.json'),
    );
    if (await sidecar.exists()) await sidecar.delete();
    for (final ModelRole role in ModelRole.values) {
      if (_selection[role] == modelId) await select(role, null);
    }
    await refresh();
  }

  /// Roles that have no model installed. The UI uses this to explain exactly
  /// which capabilities are missing rather than showing a generic warning.
  List<ModelRole> get missingRoles => <ModelRole>[
        for (final ModelRole role in ModelRole.values)
          if (selectedFor(role) == null) role,
      ];

  /// Human-readable summary for the dashboard.
  String get statusSummary {
    final int have = ModelRole.values.length - missingRoles.length;
    return '$have of ${ModelRole.values.length} model roles filled';
  }
}
