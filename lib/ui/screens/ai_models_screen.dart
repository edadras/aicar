import 'dart:async';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';

import '../../ai/inference_backend.dart';
import '../../ai/model_catalog.dart';
import '../../ai/model_descriptor.dart';
import '../../ai/model_downloader.dart';
import '../../ai/model_registry.dart';
import '../driving_session.dart';
import '../theme.dart';

/// Install, select and inspect the AI models.
///
/// One detector ships inside the app so perception works on first launch.
/// Everything else is installed at runtime: model files are large, separately
/// licensed and upgradable on their own schedule. This screen is what makes
/// that a workable arrangement rather than an obstacle — it says exactly what
/// is missing, what each role does without it, and where to put the file.
class AiModelsScreen extends StatefulWidget {
  const AiModelsScreen({super.key});

  @override
  State<AiModelsScreen> createState() => _AiModelsScreenState();
}

class _AiModelsScreenState extends State<AiModelsScreen> {
  String? _modelDirectory;
  List<String> _delegates = const <String>[];
  bool _backendAvailable = false;
  bool _busy = false;

  ModelDownloader? _downloader;
  ModelDownloadProgress? _progress;
  String? _downloadError;

  @override
  void dispose() {
    _downloader?.dispose();
    super.dispose();
  }

  /// Fetch a model the app does not ship.
  ///
  /// Confirmed first, every time. A download here is tens of megabytes of
  /// the user's data and, once it is running, a permanent share of their
  /// battery and thermal budget — none of which should start because a card
  /// was tapped.
  Future<void> _download(ModelDescriptor d) async {
    final DrivingSession session = context.read<DrivingSession>();
    final double mb = (d.sizeBytes ?? 0) / 1024 / 1024;
    final bool? go = await showDialog<bool>(
      context: context,
      builder: (BuildContext context) => AlertDialog(
        title: Text('Download ${d.name}?'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: <Widget>[
            Text('${mb.toStringAsFixed(0)} MB over your connection.'),
            const SizedBox(height: 8),
            if (d.licence != null) Text('Licence: ${d.licence}'),
            const SizedBox(height: 8),
            if (d.notes != null)
              Text(d.notes!, style: HudTheme.caption),
            const SizedBox(height: 8),
            const Text(
              'Running a larger model costs battery and thermal headroom, '
              'and on a hot dashboard that shows up as a lower frame rate.',
              style: HudTheme.caption,
            ),
          ],
        ),
        actions: <Widget>[
          TextButton(
            onPressed: () => Navigator.of(context).pop(false),
            child: const Text('Cancel'),
          ),
          FilledButton(
            onPressed: () => Navigator.of(context).pop(true),
            child: const Text('Download'),
          ),
        ],
      ),
    );
    if (go != true || !mounted) return;

    final ModelDownloader downloader =
        _downloader ??= ModelDownloader(registry: session.registry);
    setState(() {
      _downloadError = null;
      _progress = ModelDownloadProgress(
        modelId: d.id,
        receivedBytes: 0,
        totalBytes: d.sizeBytes ?? 0,
      );
    });

    try {
      await downloader.download(
        d,
        onProgress: (ModelDownloadProgress p) {
          if (mounted) setState(() => _progress = p);
        },
      );
      await session.rebuildPipeline();
      if (mounted) await _refresh();
    } catch (e) {
      if (mounted) setState(() => _downloadError = '$e');
    } finally {
      if (mounted) setState(() => _progress = null);
    }
  }

  @override
  void initState() {
    super.initState();
    unawaited(_refresh());
  }

  Future<void> _refresh() async {
    final DrivingSession session = context.read<DrivingSession>();
    final Directory dir = await session.registry.modelDirectory();
    await session.registry.refresh();
    final bool available = await session.backend.isAvailable();
    List<String> delegates = const <String>[];
    if (session.backend is TfLiteBackend) {
      delegates = await (session.backend as TfLiteBackend).supportedDelegates();
    }
    if (!mounted) return;
    setState(() {
      _modelDirectory = dir.path;
      _backendAvailable = available;
      _delegates = delegates;
    });
  }

  @override
  Widget build(BuildContext context) {
    final DrivingSession session = context.watch<DrivingSession>();

    return Scaffold(
      appBar: AppBar(
        title: const Text('AI models'),
        actions: <Widget>[
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: _busy ? null : () => _refresh(),
          ),
        ],
      ),
      body: ListView(
        padding: const EdgeInsets.fromLTRB(14, 0, 14, 28),
        children: <Widget>[
          const SectionHeader('Runtime'),
          Card(
            child: Padding(
              padding: const EdgeInsets.all(14),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  Row(
                    children: <Widget>[
                      Icon(
                        _backendAvailable
                            ? Icons.check_circle_outline
                            : Icons.error_outline,
                        size: 18,
                        color: _backendAvailable
                            ? HudTheme.accent
                            : HudTheme.critical,
                      ),
                      const SizedBox(width: 8),
                      Expanded(
                        child: Text(
                          session.backend.name,
                          style: HudTheme.body,
                        ),
                      ),
                    ],
                  ),
                  if (_delegates.isNotEmpty) ...<Widget>[
                    const SizedBox(height: 8),
                    Wrap(
                      spacing: 6,
                      children: <Widget>[
                        for (final String d in _delegates)
                          HudBadge(text: d.toUpperCase()),
                      ],
                    ),
                  ],
                  if (!_backendAvailable) ...<Widget>[
                    const SizedBox(height: 8),
                    const Text(
                      'No native inference runtime. Neural models cannot '
                      'run; the classical algorithms still will.',
                      style: HudTheme.caption,
                    ),
                  ],
                ],
              ),
            ),
          ),

          const SectionHeader(
            'Roles',
            subtitle: 'What fills each stage, and what happens without it',
          ),
          for (final ModelRole role in ModelRole.values)
            _RoleCard(
              role: role,
              registry: session.registry,
              onChanged: (String? id) async {
                setState(() => _busy = true);
                await session.registry.select(role, id);
                await session.rebuildPipeline();
                if (mounted) setState(() => _busy = false);
              },
            ),

          const SectionHeader('Installing models'),
          Card(
            child: Padding(
              padding: const EdgeInsets.all(14),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  const Text(
                    'Copy .tflite files into the model directory below, then '
                    'press refresh. A file whose name matches one in the '
                    'catalogue is configured automatically; anything else '
                    'needs a JSON sidecar of the same name describing its '
                    'input size, normalisation and output layout.\n\n'
                    'A model you install replaces the bundled one with the '
                    'same id. Bundled models cannot be deleted — select a '
                    'different one for the role instead.',
                    style: HudTheme.caption,
                  ),
                  const SizedBox(height: 10),
                  if (_modelDirectory != null)
                    InkWell(
                      onTap: () {
                        Clipboard.setData(
                            ClipboardData(text: _modelDirectory!));
                        ScaffoldMessenger.of(context).showSnackBar(
                          const SnackBar(content: Text('Path copied')),
                        );
                      },
                      child: Container(
                        width: double.infinity,
                        padding: const EdgeInsets.all(10),
                        decoration: BoxDecoration(
                          color: HudTheme.background,
                          borderRadius: BorderRadius.circular(8),
                          border: Border.all(color: HudTheme.outline),
                        ),
                        child: Text(
                          _modelDirectory!,
                          style: HudTheme.caption.copyWith(
                            fontFamily: HudTheme.monoFamily,
                            color: HudTheme.textPrimary,
                          ),
                        ),
                      ),
                    ),
                  const SizedBox(height: 10),
                  const Text(
                    'See docs/MODELS.md for the sidecar format and for the '
                    'export commands that produce compatible files.',
                    style: HudTheme.caption,
                  ),
                ],
              ),
            ),
          ),

          const SectionHeader('Known models'),
          if (_downloadError != null)
            Card(
              color: HudTheme.critical.withValues(alpha: 0.12),
              child: Padding(
                padding: const EdgeInsets.all(12),
                child: Text(_downloadError!, style: HudTheme.caption),
              ),
            ),
          for (final ModelDescriptor d in ModelCatalog.all.values)
            Padding(
              padding: const EdgeInsets.only(bottom: 8),
              child: Card(
                child: ListTile(
                  dense: true,
                  leading: Icon(
                    session.registry.installed.containsKey(d.id)
                        ? Icons.download_done
                        : Icons.download_outlined,
                    size: 18,
                    color: session.registry.installed.containsKey(d.id)
                        ? HudTheme.accent
                        : HudTheme.textDim,
                  ),
                  title: Text(d.name, style: HudTheme.body),
                  subtitle: Text(
                    '${d.role.label} · '
                    '${d.isBundledAsset ? 'ships with the app' : 'expects '
                        '${d.assetOrFilePath.split('/').last}'}'
                    '${d.licence == null ? '' : ' · ${d.licence}'}'
                    '${d.notes == null ? '' : '\n${d.notes}'}',
                    style: HudTheme.caption,
                  ),
                  isThreeLine: d.notes != null,
                  trailing: _downloadTrailingFor(
                    context: context,
                    d: d,
                    installed: session.registry.installed.containsKey(d.id),
                    progress: _progress,
                    onDownload: () => _download(d),
                  ),
                ),
              ),
            ),
        ],
      ),
    );
  }
}

/// The download control for one catalogue row, or nothing when the model is
/// already present or cannot be fetched.
Widget? _downloadTrailingFor({
  required BuildContext context,
  required ModelDescriptor d,
  required bool installed,
  required ModelDownloadProgress? progress,
  required VoidCallback onDownload,
}) {
  if (installed || !d.isDownloadable) return null;
  if (progress != null && progress.modelId == d.id) {
    return SizedBox(
      width: 92,
      child: Column(
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.end,
        children: <Widget>[
          LinearProgressIndicator(value: progress.fraction),
          const SizedBox(height: 4),
          Text(
            progress.stage == 'downloading'
                ? progress.label
                : progress.stage,
            style: HudTheme.caption.copyWith(fontSize: 10),
          ),
        ],
      ),
    );
  }
  return TextButton.icon(
    onPressed: progress == null ? onDownload : null,
    icon: const Icon(Icons.download, size: 16),
    label: Text(
      d.sizeBytes == null
          ? 'Get'
          : '${(d.sizeBytes! / 1024 / 1024).round()} MB',
    ),
  );
}

class _RoleCard extends StatelessWidget {
  const _RoleCard({
    required this.role,
    required this.registry,
    required this.onChanged,
  });

  final ModelRole role;
  final ModelRegistry registry;
  final ValueChanged<String?> onChanged;

  /// What the stack does for this role when no model is installed. Being
  /// specific here is the difference between a user thinking the app is
  /// broken and understanding exactly which capability is reduced.
  String get _fallbackDescription => switch (role) {
        ModelRole.objectDetection =>
          'Nothing. Vehicles, pedestrians and obstacles are NOT detected, '
              'and the stack reports degraded confidence rather than an '
              'empty road. The app ships with a detector, so this only '
              'happens if you turn it off.',
        ModelRole.depthEstimation =>
          'Distances still work, from ground-plane geometry, class size '
              'priors and motion parallax — with lower confidence.',
        ModelRole.laneDetection =>
          'A classical IPM + matched-filter detector. Good on clear '
              'markings, weaker at night and on worn paint.',
        ModelRole.roadSegmentation =>
          'A heuristic region-growing segmenter. Cannot distinguish asphalt '
              'from similarly-coloured pavement.',
        ModelRole.trafficSignDetection ||
        ModelRole.trafficSignClassification =>
          'Shape and colour analysis gives coarse categories; speed-limit '
              'digits are read by template matching.',
        ModelRole.trafficLightClassification =>
          'Hue and aspect-position analysis, which is the intended '
              'implementation rather than a fallback.',
      };

  @override
  Widget build(BuildContext context) {
    final List<InstalledModel> candidates = registry.forRole(role);
    final ModelDescriptor? selected = registry.selectedFor(role);

    return Padding(
      padding: const EdgeInsets.only(bottom: 10),
      child: Card(
        child: Padding(
          padding: const EdgeInsets.all(14),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: <Widget>[
              Row(
                children: <Widget>[
                  Expanded(
                    child: Text(role.label,
                        style: HudTheme.body
                            .copyWith(fontWeight: FontWeight.w600)),
                  ),
                  HudBadge(
                    text: selected == null ? 'NO MODEL' : 'ACTIVE',
                    color: selected == null
                        ? (role == ModelRole.objectDetection
                            ? HudTheme.critical
                            : HudTheme.caution)
                        : HudTheme.accent,
                  ),
                ],
              ),
              const SizedBox(height: 8),
              if (candidates.isEmpty)
                Text(_fallbackDescription, style: HudTheme.caption)
              else
                RadioGroup<String?>(
                  groupValue: selected?.id,
                  onChanged: onChanged,
                  child: Column(
                    children: <Widget>[
                      RadioListTile<String?>(
                        dense: true,
                        contentPadding: EdgeInsets.zero,
                        value: null,
                        title: const Text('None', style: HudTheme.body),
                        subtitle: Text(_fallbackDescription,
                            style: HudTheme.caption),
                      ),
                      for (final InstalledModel m in candidates)
                        RadioListTile<String?>(
                          dense: true,
                          contentPadding: EdgeInsets.zero,
                          value: m.descriptor.id,
                          title:
                              Text(m.descriptor.name, style: HudTheme.body),
                          subtitle: Text(
                            '${m.sizeLabel} · '
                            '${m.descriptor.inputWidth}x'
                            '${m.descriptor.inputHeight} · '
                            '${m.descriptor.delegate.name.toUpperCase()}'
                            '${m.isBundled ? ' · bundled' : ''}',
                            style: HudTheme.caption,
                          ),
                        ),
                    ],
                  ),
                ),
            ],
          ),
        ),
      ),
    );
  }
}
