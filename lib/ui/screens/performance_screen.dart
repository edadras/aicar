import 'package:flutter/material.dart';

import '../../camera/camera_service.dart';
import '../../core/profiling.dart';
import '../../debug/system_monitor.dart';
import '../driving_session.dart';
import '../theme.dart';

/// Where the frame budget goes, and the controls that change it.
class PerformanceScreen extends StatelessWidget {
  const PerformanceScreen({super.key, required this.session});

  final DrivingSession session;

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: session,
      builder: (BuildContext context, _) {
        final PipelineProfiler? profiler = session.profiler;
        final double budgetMs = session.config.camera.targetInferenceFps > 0
            ? 1000 / session.config.camera.targetInferenceFps
            : 50;

        return Scaffold(
          appBar: AppBar(
            title: const Text('Performance'),
            actions: <Widget>[
              IconButton(
                icon: const Icon(Icons.restart_alt),
                tooltip: 'Reset counters',
                onPressed: () {
                  profiler?.reset();
                  session.scheduler.reset();
                },
              ),
            ],
          ),
          body: ListView(
            padding: const EdgeInsets.fromLTRB(14, 0, 14, 28),
            children: <Widget>[
              const SectionHeader('Frame rate'),
              Card(
                child: Padding(
                  padding: const EdgeInsets.all(14),
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.spaceAround,
                    children: <Widget>[
                      HudReadout(
                        label: 'Camera',
                        value: session.cameraFps.toStringAsFixed(1),
                        unit: 'fps',
                      ),
                      HudReadout(
                        label: 'Pipeline',
                        value: session.processingFps.toStringAsFixed(1),
                        unit: 'fps',
                        valueColor: session.processingFps >= 10
                            ? HudTheme.accent
                            : HudTheme.caution,
                      ),
                      HudReadout(
                        label: 'Latency',
                        value: (profiler?.totalLatencyMs ?? 0)
                            .toStringAsFixed(0),
                        unit: 'ms',
                        valueColor:
                            (profiler?.totalLatencyMs ?? 0) > budgetMs
                                ? HudTheme.warning
                                : HudTheme.accent,
                      ),
                      HudReadout(
                        label: 'Dropped',
                        value: '${profiler?.droppedFrames ?? 0}',
                      ),
                    ],
                  ),
                ),
              ),

              const SectionHeader(
                'Device',
                subtitle: 'On a dashboard in sunlight, heat is usually the '
                    'real limit',
              ),
              Card(
                child: Padding(
                  padding: const EdgeInsets.all(14),
                  child: Builder(
                    builder: (BuildContext context) {
                      final SystemSample? sample =
                          session.systemMonitor.latest;
                      if (sample == null) {
                        return const Text(
                          'Start a drive to sample battery and thermal '
                          'state.',
                          style: HudTheme.caption,
                        );
                      }
                      final double? drain =
                          session.systemMonitor.batteryDrainPercentPerHour;
                      return Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: <Widget>[
                          Row(
                            mainAxisAlignment:
                                MainAxisAlignment.spaceAround,
                            children: <Widget>[
                              HudReadout(
                                label: 'Battery',
                                value: '${sample.batteryPercent}',
                                unit: '%',
                                valueColor: sample.batteryPercent < 20
                                    ? HudTheme.critical
                                    : HudTheme.textPrimary,
                              ),
                              HudReadout(
                                label: 'Drain',
                                value: drain == null
                                    ? '—'
                                    : drain.toStringAsFixed(0),
                                unit: '%/h',
                              ),
                              HudReadout(
                                label: 'Thermal',
                                value: sample.thermal.label,
                                valueColor: sample.thermal.isThrottling
                                    ? HudTheme.warning
                                    : HudTheme.accent,
                              ),
                            ],
                          ),
                          if (sample.thermal.isThrottling) ...<Widget>[
                            const SizedBox(height: 10),
                            const Text(
                              'The SoC is being clocked down. A falling frame '
                              'rate right now is thermal, not a change in the '
                              'code — lower the inference resolution or the '
                              'target FPS to recover.',
                              style: TextStyle(
                                fontSize: 12,
                                color: HudTheme.warning,
                                height: 1.35,
                              ),
                            ),
                          ],
                        ],
                      );
                    },
                  ),
                ),
              ),

              const SectionHeader(
                'Stage budget',
                subtitle: 'Average and 95th percentile, milliseconds',
              ),
              Card(
                child: Padding(
                  padding: const EdgeInsets.all(14),
                  child: Column(
                    children: <Widget>[
                      for (final StageProfile s
                          in profiler?.stages ?? const <StageProfile>[])
                        _StageBar(stage: s, budgetMs: budgetMs),
                      if ((profiler?.stages ?? const []).isEmpty)
                        const Text(
                          'Start a drive to collect timings.',
                          style: HudTheme.caption,
                        ),
                    ],
                  ),
                ),
              ),

              const SectionHeader(
                'Tuning',
                subtitle: 'The two knobs that actually move the frame rate',
              ),
              Card(
                child: Padding(
                  padding: const EdgeInsets.all(14),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[
                      Text('Inference resolution', style: HudTheme.hudLabel),
                      const SizedBox(height: 8),
                      SegmentedButton<InferenceResolution>(
                        segments: <ButtonSegment<InferenceResolution>>[
                          for (final InferenceResolution r
                              in InferenceResolution.values)
                            ButtonSegment<InferenceResolution>(
                              value: r,
                              label: Text('${r.width}'),
                            ),
                        ],
                        selected: <InferenceResolution>{
                          session.config.camera.inferenceResolution
                        },
                        onSelectionChanged:
                            (Set<InferenceResolution> selection) {
                          session.applyConfig(
                            session.config.copyWith(
                              camera: session.config.camera.copyWith(
                                inferenceResolution: selection.first,
                              ),
                            ),
                          );
                        },
                      ),
                      const SizedBox(height: 6),
                      const Text(
                        'Lower resolution trades detection range on small, '
                        'distant objects for frame rate.',
                        style: HudTheme.caption,
                      ),
                      const SizedBox(height: 16),
                      Text('Target inference FPS', style: HudTheme.hudLabel),
                      Slider(
                        value: session.config.camera.targetInferenceFps
                            .clamp(5, 30),
                        min: 5,
                        max: 30,
                        divisions: 25,
                        label: session.config.camera.targetInferenceFps
                            .toStringAsFixed(0),
                        onChanged: (double v) => session.applyConfig(
                          session.config.copyWith(
                            camera: session.config.camera
                                .copyWith(targetInferenceFps: v),
                          ),
                        ),
                      ),
                      const Text(
                        'Capping the rate below what the device can manage '
                        'reduces heat, which on a phone on a dashboard in '
                        'the sun is usually the real limit.',
                        style: HudTheme.caption,
                      ),
                    ],
                  ),
                ),
              ),

              const SectionHeader(
                'Stage cadence',
                subtitle: 'Expensive stages that change slowly need not run '
                    'every frame',
              ),
              Card(
                child: Column(
                  children: <Widget>[
                    _ToggleRow(
                      title: 'Object detection',
                      value: session.config.toggles.objectDetection,
                      onChanged: (bool v) => session.applyConfig(
                        session.config.copyWith(
                          toggles: session.config.toggles
                              .copyWith(objectDetection: v),
                        ),
                      ),
                    ),
                    _ToggleRow(
                      title: 'Segmentation',
                      subtitle: 'Runs every '
                          '${session.config.cadence.segmentationEveryNFrames}'
                          ' frames',
                      value: session.config.toggles.segmentation,
                      onChanged: (bool v) => session.applyConfig(
                        session.config.copyWith(
                          toggles: session.config.toggles
                              .copyWith(segmentation: v),
                        ),
                      ),
                    ),
                    _ToggleRow(
                      title: 'Depth estimation',
                      subtitle: 'Runs every '
                          '${session.config.cadence.depthEveryNFrames} frames',
                      value: session.config.toggles.depth,
                      onChanged: (bool v) => session.applyConfig(
                        session.config.copyWith(
                          toggles:
                              session.config.toggles.copyWith(depth: v),
                        ),
                      ),
                    ),
                    _ToggleRow(
                      title: 'Traffic signs',
                      value: session.config.toggles.trafficSigns,
                      onChanged: (bool v) => session.applyConfig(
                        session.config.copyWith(
                          toggles: session.config.toggles
                              .copyWith(trafficSigns: v),
                        ),
                      ),
                    ),
                    _ToggleRow(
                      title: 'Road edges',
                      value: session.config.toggles.roadEdges,
                      onChanged: (bool v) => session.applyConfig(
                        session.config.copyWith(
                          toggles: session.config.toggles
                              .copyWith(roadEdges: v),
                        ),
                      ),
                    ),
                  ],
                ),
              ),

              const SectionHeader('Frame scheduling'),
              Card(
                child: Padding(
                  padding: const EdgeInsets.all(14),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[
                      _kv('Accepted', '${session.scheduler.accepted}'),
                      _kv('Dropped while busy',
                          '${session.scheduler.droppedBusy}'),
                      _kv('Dropped by throttle',
                          '${session.scheduler.droppedThrottle}'),
                      _kv('Superseded by a newer frame',
                          '${session.scheduler.replaced}'),
                      const SizedBox(height: 8),
                      const Text(
                        'A superseded frame is not a failure: when the '
                        'pipeline is busy the newest frame replaces the one '
                        'waiting, because a stale view of the road is worse '
                        'than none when computing time-to-collision.',
                        style: HudTheme.caption,
                      ),
                    ],
                  ),
                ),
              ),
            ],
          ),
        );
      },
    );
  }

  Widget _kv(String k, String v) => Padding(
        padding: const EdgeInsets.symmetric(vertical: 2),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: <Widget>[
            Text(k, style: HudTheme.caption),
            Text(v,
                style: HudTheme.body.copyWith(
                    fontFamily: HudTheme.monoFamily, fontSize: 13)),
          ],
        ),
      );
}

class _StageBar extends StatelessWidget {
  const _StageBar({required this.stage, required this.budgetMs});

  final StageProfile stage;
  final double budgetMs;

  @override
  Widget build(BuildContext context) {
    final double fraction = (stage.averageMs / budgetMs).clamp(0.0, 1.0);
    final bool isTotal = stage.name == PipelineStageNames.total;
    final Color color = stage.averageMs > budgetMs
        ? HudTheme.critical
        : (stage.averageMs > budgetMs * 0.5
            ? HudTheme.caution
            : HudTheme.accent);

    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          Row(
            children: <Widget>[
              Expanded(
                child: Text(
                  stage.name,
                  style: HudTheme.body.copyWith(
                    fontSize: 13,
                    fontWeight:
                        isTotal ? FontWeight.w700 : FontWeight.w400,
                  ),
                ),
              ),
              Text(
                '${stage.averageMs.toStringAsFixed(1)} ms  '
                'p95 ${stage.p95Ms.toStringAsFixed(1)}',
                style: HudTheme.caption.copyWith(
                  fontFamily: HudTheme.monoFamily,
                  color: color,
                ),
              ),
            ],
          ),
          const SizedBox(height: 4),
          ClipRRect(
            borderRadius: BorderRadius.circular(2),
            child: LinearProgressIndicator(
              value: fraction,
              minHeight: isTotal ? 7 : 5,
              backgroundColor: HudTheme.outline,
              valueColor: AlwaysStoppedAnimation<Color>(color),
            ),
          ),
        ],
      ),
    );
  }
}

class _ToggleRow extends StatelessWidget {
  const _ToggleRow({
    required this.title,
    required this.value,
    required this.onChanged,
    this.subtitle,
  });

  final String title;
  final String? subtitle;
  final bool value;
  final ValueChanged<bool> onChanged;

  @override
  Widget build(BuildContext context) => SwitchListTile(
        dense: true,
        title: Text(title, style: HudTheme.body),
        subtitle:
            subtitle == null ? null : Text(subtitle!, style: HudTheme.caption),
        value: value,
        onChanged: onChanged,
      );
}
