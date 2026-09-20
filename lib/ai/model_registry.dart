import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as p;
import 'package:path_provider/path_provider.dart';
import 'package:shared_preferences/shared_preferences.dart';

import '../core/logging.dart';
import 'model_catalog.dart';
import 'model_descriptor.dart';

/// A model file present on the device, paired with the descriptor that says
/// how to run it.
class InstalledModel {
  const InstalledModel({
    required this.descriptor,
    required this.file,
    required this.sizeBytes,
    required this.installedAt,
  });

  final ModelDescriptor descriptor;
  final File file;
  final int sizeBytes;
  final DateTime installedAt;

  String get sizeLabel {
    if (sizeBytes < 1024 * 1024) return '${(sizeBytes / 1024).round()} KB';
    return '${(sizeBytes / 1024 / 1024).toStringAsFixed(1)} MB';
  }
}

/// Discovers, validates and remembers the models installed on the device.
///
/// The app ships with no weights. Users drop `.tflite` files into the app's
/// model directory (via the AI Models screen or `adb push`); the registry
/// matches each file against [ModelCatalog] by filename, or reads a
/// `<name>.json` sidecar for models the catalog does not know.
///
/// This is the mechanism that makes the "swap any model without touching the
/// app" requirement real rather than aspirational.
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

      // 2. Otherwise match the catalog by the filename it expects.
      descriptor ??= ModelCatalog.all.values.cast<ModelDescriptor?>().firstWhere(
            (ModelDescriptor? d) => d!.assetOrFilePath == fileName,
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
    // Auto-select when exactly one model can fill a role: the common case is
    // a user who installed one detector and expects it to just work.
    for (final ModelRole role in ModelRole.values) {
      if (_selection.containsKey(role)) continue;
      final List<InstalledModel> candidates = forRole(role);
      if (candidates.length == 1) {
        _selection[role] = candidates.first.descriptor.id;
      }
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
            .firstWhere((InstalledModel? m) => m!.file.path == target.path,
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
    if (await m.file.exists()) await m.file.delete();
    final File sidecar = File(
      p.join(m.file.parent.path,
          '${p.basenameWithoutExtension(m.file.path)}.json'),
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
