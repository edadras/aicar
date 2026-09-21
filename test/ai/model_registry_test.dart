import 'dart:io';

import 'package:aicar/ai/model_catalog.dart';
import 'package:aicar/ai/model_descriptor.dart';
import 'package:aicar/ai/model_registry.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:shared_preferences/shared_preferences.dart';

/// The registry is what decides whether the app has perception at all, so
/// these tests cover the two claims the rest of the stack relies on: the
/// bundled detector is always there, and anything the user installs wins.
void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  late Directory dir;

  setUp(() async {
    SharedPreferences.setMockInitialValues(<String, Object>{});
    dir = await Directory.systemTemp.createTemp('aicar-models-');
  });

  tearDown(() async {
    if (await dir.exists()) await dir.delete(recursive: true);
  });

  /// A file that the catalogue recognises by name, so no sidecar is needed.
  Future<File> installYolo() async {
    final File f = File('${dir.path}/${ModelCatalog.yolov8n.assetOrFilePath}');
    await f.writeAsBytes(List<int>.filled(2048, 0));
    return f;
  }

  test('an empty model directory still has the bundled detector', () async {
    final ModelRegistry registry = ModelRegistry(modelDirectory: dir);
    await registry.refresh();

    final ModelDescriptor? selected =
        registry.selectedFor(ModelRole.objectDetection);
    expect(selected, isNotNull);
    expect(selected!.id, ModelCatalog.efficientDetLite0Id);
    expect(selected.isBundledAsset, isTrue,
        reason: 'the native runtime must load it from the asset bundle');
    expect(registry.missingRoles, isNot(contains(ModelRole.objectDetection)));
  });

  test('every other role is still honestly reported as empty', () async {
    final ModelRegistry registry = ModelRegistry(modelDirectory: dir);
    await registry.refresh();

    expect(
      registry.missingRoles,
      containsAll(<ModelRole>[
        ModelRole.depthEstimation,
        ModelRole.laneDetection,
        ModelRole.roadSegmentation,
      ]),
      reason: 'bundling a detector must not make the other roles look filled',
    );
  });

  test('a bundled model is not backed by a file', () async {
    final ModelRegistry registry = ModelRegistry(modelDirectory: dir);
    await registry.refresh();

    final InstalledModel m =
        registry.installed[ModelCatalog.efficientDetLite0Id]!;
    expect(m.isBundled, isTrue);
    expect(m.file, isNull);
    expect(m.sizeBytes, greaterThan(0),
        reason: 'the size comes from the descriptor, for the models screen');
  });

  test('a bundled model cannot be uninstalled', () async {
    final ModelRegistry registry = ModelRegistry(modelDirectory: dir);
    await registry.refresh();

    await expectLater(
      registry.uninstall(ModelCatalog.efficientDetLite0Id),
      throwsArgumentError,
    );
    expect(registry.installed, contains(ModelCatalog.efficientDetLite0Id));
  });

  test('an installed detector is preferred over the bundled one', () async {
    await installYolo();
    final ModelRegistry registry = ModelRegistry(modelDirectory: dir);
    await registry.refresh();

    // Both are offered...
    expect(
      registry.forRole(ModelRole.objectDetection).map<String>(
          (InstalledModel m) => m.descriptor.id),
      containsAll(<String>[
        ModelCatalog.yolov8nId,
        ModelCatalog.efficientDetLite0Id,
      ]),
    );
    // ...but the file the user deliberately pushed is the one that runs.
    expect(registry.selectedFor(ModelRole.objectDetection)!.id,
        ModelCatalog.yolov8nId);
  });

  test('an explicit choice survives a refresh', () async {
    await installYolo();
    final ModelRegistry registry = ModelRegistry(modelDirectory: dir);
    await registry.refresh();
    await registry.select(
        ModelRole.objectDetection, ModelCatalog.efficientDetLite0Id);
    await registry.refresh();

    expect(registry.selectedFor(ModelRole.objectDetection)!.id,
        ModelCatalog.efficientDetLite0Id);
  });

  test('an installed file shadows the bundled model of the same id', () async {
    final File f = File(
        '${dir.path}/${ModelCatalog.bundledDetectorAsset.split('/').last}');
    await f.writeAsBytes(List<int>.filled(4096, 0));

    final ModelRegistry registry = ModelRegistry(modelDirectory: dir);
    await registry.refresh();

    final InstalledModel m =
        registry.installed[ModelCatalog.efficientDetLite0Id]!;
    expect(m.isBundled, isFalse);
    expect(m.file!.path, f.path);
    expect(m.descriptor.isBundledAsset, isFalse,
        reason: 'the runtime must read it from disk, not from the APK');
    expect(m.descriptor.assetOrFilePath, f.path);
  });

  test('the bundled asset exists and is declared to Flutter', () async {
    // If either of these drifts, the app builds and then fails to load its
    // only detector at runtime on the device — where it is hardest to debug.
    final File asset = File(ModelCatalog.bundledDetectorAsset);
    expect(await asset.exists(), isTrue,
        reason: '${ModelCatalog.bundledDetectorAsset} is missing');
    expect(await asset.length(), ModelCatalog.efficientDetLite0.sizeBytes,
        reason: 'descriptor size is stale');

    final String pubspec = await File('pubspec.yaml').readAsString();
    expect(pubspec, contains('assets/models/'));
  });
}
