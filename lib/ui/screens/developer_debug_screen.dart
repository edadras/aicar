import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import '../../core/logging.dart';
import '../../core/profiling.dart';
import '../../depth/depth_map.dart';
import '../../pipeline/pipeline_result.dart';
import '../../road/road_segmentation.dart';
import '../../pipeline/pipeline_factory.dart';
import '../driving_session.dart';
import '../hud/hud_panels.dart';
import '../theme.dart';

/// Compact debug card shown over the HUD.
class DebugOverlayCard extends StatelessWidget {
  const DebugOverlayCard({
    super.key,
    required this.session,
    required this.result,
  });

  final DrivingSession session;
  final PipelineResult result;

  @override
  Widget build(BuildContext context) {
    final DepthMap? depth = result.world.depth;
    final RoadSegmentation? seg = result.world.segmentation;

    return Container(
      width: 250,
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(
        color: HudTheme.background.withValues(alpha: 0.9),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: HudTheme.outline),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Text('DEBUG', style: HudTheme.hudLabel),
          const SizedBox(height: 6),
          _row('Frame', '#${result.frameId}'),
          _row('Latency', '${result.totalLatencyMs.toStringAsFixed(1)} ms'),
          _row('Camera FPS', session.cameraFps.toStringAsFixed(1)),
          _row('AI FPS', session.processingFps.toStringAsFixed(1)),
          _row('Dropped', '${session.scheduler.totalDropped}'),
          const Divider(height: 14, color: HudTheme.outline),
          _row('Tracks', '${result.world.tracks.length}'),
          _row('Lane mode', result.world.lanes.mode.badge),
          _row('Lane conf',
              '${(result.world.lanes.overallConfidence * 100).round()}%'),
          _row('Corridor',
              result.world.corridor == null
                  ? '—'
                  : '${(result.world.corridor!.confidence * 100).round()}%'),
          _row('Depth',
              depth == null || !depth.isUsable
                  ? 'unavailable'
                  : '${(depth.globalConfidence * 100).round()}%'
                      '${depth.isFitted ? ' fitted' : ' UNFITTED'}'),
          _row('Drivable',
              seg == null || !seg.isUsable
                  ? '—'
                  : '${(seg.drivableFraction * 100).round()}%'),
          _row('GPS',
              result.world.ego.position == null
                  ? 'no fix'
                  : '±${result.world.ego.position!.accuracyMeters
                      .toStringAsFixed(0)} m'),
          _row('Path',
              result.path.isUsable
                  ? '${result.path.maxRangeMeters.toStringAsFixed(0)} m'
                  : 'none'),
          if (result.world.degradedSubsystems.isNotEmpty) ...<Widget>[
            const Divider(height: 14, color: HudTheme.outline),
            Text(
              result.world.degradedSubsystems.join('\n'),
              style: HudTheme.caption.copyWith(
                color: HudTheme.warning,
                fontSize: 10.5,
              ),
            ),
          ],
        ],
      ),
    );
  }

  Widget _row(String label, String value) => Padding(
        padding: const EdgeInsets.symmetric(vertical: 1.5),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: <Widget>[
            Text(label,
                style: HudTheme.caption.copyWith(fontSize: 11)),
            Text(
              value,
              style: HudTheme.caption.copyWith(
                fontFamily: HudTheme.monoFamily,
                fontSize: 11,
                color: HudTheme.textPrimary,
              ),
            ),
          ],
        ),
      );
}

/// Full developer screen: every internal number, plus the in-app log.
class DeveloperDebugScreen extends StatefulWidget {
  const DeveloperDebugScreen({super.key, required this.session});

  final DrivingSession session;

  @override
  State<DeveloperDebugScreen> createState() => _DeveloperDebugScreenState();
}

class _DeveloperDebugScreenState extends State<DeveloperDebugScreen> {
  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: widget.session,
      builder: (BuildContext context, _) {
        final PipelineResult? result = widget.session.latest;
        return Scaffold(
          appBar: AppBar(
            title: const Text('Developer debug'),
            actions: <Widget>[
              IconButton(
                icon: const Icon(Icons.copy_all_outlined),
                tooltip: 'Copy the current world model as JSON',
                onPressed: result == null
                    ? null
                    : () {
                        Clipboard.setData(ClipboardData(
                          text: result.toJson().toString(),
                        ));
                        ScaffoldMessenger.of(context).showSnackBar(
                          const SnackBar(
                            content: Text('World model copied'),
                          ),
                        );
                      },
              ),
            ],
          ),
          body: ListView(
            padding: const EdgeInsets.fromLTRB(14, 0, 14, 24),
            children: <Widget>[
              const SectionHeader('Pipeline composition',
                  subtitle: 'Which implementation is filling each role'),
              if (widget.session.composition == null)
                const Card(
                  child: ListTile(
                    title: Text('Pipeline not built yet'),
                  ),
                )
              else
                Card(
                  child: Column(
                    children: <Widget>[
                      for (final RoleBinding b
                          in widget.session.composition!.roles.values)
                        ListTile(
                          dense: true,
                          leading: Icon(
                            b.isFallback
                                ? Icons.warning_amber_rounded
                                : Icons.check_circle_outline,
                            color: b.isFallback
                                ? HudTheme.caution
                                : HudTheme.accent,
                            size: 18,
                          ),
                          title: Text(b.role, style: HudTheme.body),
                          subtitle: Text(
                            '${b.implementation}\n${b.explanation}',
                            style: HudTheme.caption,
                          ),
                          isThreeLine: true,
                        ),
                    ],
                  ),
                ),

              const SectionHeader('Confidence'),
              if (result != null)
                AutonomyConfidencePanel(autonomy: result.world.autonomy),

              const SectionHeader('Stage timings'),
              Card(
                child: Padding(
                  padding: const EdgeInsets.all(12),
                  child: Column(
                    children: <Widget>[
                      for (final StageProfile s
                          in widget.session.profiler?.stages ??
                              const <StageProfile>[])
                        _StageRow(stage: s),
                      if ((widget.session.profiler?.stages ?? const []).isEmpty)
                        const Text('No timings yet',
                            style: HudTheme.caption),
                    ],
                  ),
                ),
              ),

              const SectionHeader('Frame scheduling'),
              Card(
                child: Padding(
                  padding: const EdgeInsets.all(12),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[
                      _kv('Target inference FPS',
                          widget.session.scheduler.targetFps
                              .toStringAsFixed(1)),
                      _kv('Accepted',
                          '${widget.session.scheduler.accepted}'),
                      _kv('Dropped (busy)',
                          '${widget.session.scheduler.droppedBusy}'),
                      _kv('Dropped (throttle)',
                          '${widget.session.scheduler.droppedThrottle}'),
                      _kv('Replaced by newer',
                          '${widget.session.scheduler.replaced}'),
                      const SizedBox(height: 6),
                      const Text(
                        'Frames are dropped by design: the camera must never '
                        'wait for inference, and a stale frame is worse than '
                        'no frame when computing time-to-collision.',
                        style: HudTheme.caption,
                      ),
                    ],
                  ),
                ),
              ),

              if (result != null) ...<Widget>[
                const SectionHeader('Tracks'),
                Card(
                  child: Padding(
                    padding: const EdgeInsets.all(12),
                    child: result.world.tracks.isEmpty
                        ? const Text('No confirmed tracks',
                            style: HudTheme.caption)
                        : Column(
                            children: <Widget>[
                              for (final track in result.world.tracks)
                                Padding(
                                  padding:
                                      const EdgeInsets.only(bottom: 6),
                                  child: Text(
                                    track.toString(),
                                    style: HudTheme.caption.copyWith(
                                      fontFamily: HudTheme.monoFamily,
                                      fontSize: 11,
                                    ),
                                  ),
                                ),
                            ],
                          ),
                  ),
                ),
              ],

              const SectionHeader('Log'),
              Card(
                child: Container(
                  height: 260,
                  padding: const EdgeInsets.all(10),
                  child: ListView(
                    reverse: true,
                    children: <Widget>[
                      for (final LogRecord r in Log.history.reversed)
                        Padding(
                          padding: const EdgeInsets.only(bottom: 3),
                          child: Text(
                            '${r.tag}: ${r.message}',
                            style: HudTheme.caption.copyWith(
                              fontFamily: HudTheme.monoFamily,
                              fontSize: 10.5,
                              color: switch (r.level) {
                                LogLevel.error => HudTheme.critical,
                                LogLevel.warn => HudTheme.caution,
                                LogLevel.info => HudTheme.textPrimary,
                                _ => HudTheme.textDim,
                              },
                            ),
                          ),
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

class _StageRow extends StatelessWidget {
  const _StageRow({required this.stage});

  final StageProfile stage;

  @override
  Widget build(BuildContext context) {
    // Scale bars against 40 ms: at a 20 FPS target that is a whole frame
    // budget, so a bar that fills the row is a stage that owns the frame.
    final double fraction = (stage.averageMs / 40).clamp(0.0, 1.0);
    final Color color = stage.averageMs > 30
        ? HudTheme.critical
        : (stage.averageMs > 15 ? HudTheme.caution : HudTheme.accent);

    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 3),
      child: Row(
        children: <Widget>[
          SizedBox(
            width: 124,
            child: Text(stage.name, style: HudTheme.caption),
          ),
          Expanded(
            child: ClipRRect(
              borderRadius: BorderRadius.circular(2),
              child: LinearProgressIndicator(
                value: fraction,
                minHeight: 5,
                backgroundColor: HudTheme.outline,
                valueColor: AlwaysStoppedAnimation<Color>(color),
              ),
            ),
          ),
          const SizedBox(width: 8),
          SizedBox(
            width: 92,
            child: Text(
              '${stage.averageMs.toStringAsFixed(1)} / '
              '${stage.p95Ms.toStringAsFixed(1)}',
              style: HudTheme.caption.copyWith(
                fontFamily: HudTheme.monoFamily,
                fontSize: 11,
                color: HudTheme.textPrimary,
              ),
              textAlign: TextAlign.right,
            ),
          ),
        ],
      ),
    );
  }
}
