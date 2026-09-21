import 'dart:async';

import 'package:flutter/material.dart';

import '../../recording/session_store.dart';
import '../../replay/replay_player.dart';
import '../driving_session.dart';
import '../hud/hud_panels.dart';
import '../hud/perception_overlay.dart';
import '../theme.dart';

/// Replays a recorded drive, either as it happened or with the AI re-run.
class ReplayScreen extends StatefulWidget {
  const ReplayScreen({
    super.key,
    required this.session,
    required this.driving,
  });

  final RecordedSession session;
  final DrivingSession driving;

  @override
  State<ReplayScreen> createState() => _ReplayScreenState();
}

class _ReplayScreenState extends State<ReplayScreen> {
  ReplayMode _mode = ReplayMode.recordedResults;
  ReplayPlayer? _player;
  ReplayStep? _current;
  StreamSubscription<ReplayStep>? _sub;
  double _speed = 1.0;
  bool _preparing = false;
  String? _error;

  @override
  void dispose() {
    unawaited(_sub?.cancel());
    unawaited(_player?.dispose());
    super.dispose();
  }

  Future<void> _start() async {
    setState(() {
      _preparing = true;
      _error = null;
    });

    await _sub?.cancel();
    await _player?.dispose();

    // Re-running needs a pipeline; replaying recorded results does not, which
    // is why a drive can always be reviewed even with no models installed.
    if (_mode == ReplayMode.rerunAi && widget.driving.composition == null) {
      await widget.driving.rebuildPipeline();
    }

    final ReplayPlayer player = ReplayPlayer(
      session: widget.session,
      mode: _mode,
      pipeline:
          _mode == ReplayMode.rerunAi ? widget.driving.latestPipeline : null,
      playbackSpeed: _speed,
    );

    if (_mode == ReplayMode.rerunAi && !player.canRerun) {
      setState(() {
        _preparing = false;
        _error = player.rerunUnavailableReason;
      });
      return;
    }

    await player.prepare();
    _sub = player.steps.listen((ReplayStep step) {
      if (mounted) setState(() => _current = step);
    });

    setState(() {
      _player = player;
      _preparing = false;
    });

    unawaited(player.play().catchError((Object e) {
      if (mounted) setState(() => _error = '$e');
    }));
  }

  @override
  Widget build(BuildContext context) {
    final ReplayStep? step = _current;

    return Scaffold(
      appBar: AppBar(
        title: const Text('Replay'),
        actions: <Widget>[
          if (_player != null && _player!.isPlaying)
            IconButton(
              icon: Icon(_player!.isPaused ? Icons.play_arrow : Icons.pause),
              onPressed: () => setState(() => _player!.isPaused
                  ? _player!.resume()
                  : _player!.pause()),
            ),
        ],
      ),
      body: Column(
        children: <Widget>[
          Expanded(
            child: Stack(
              fit: StackFit.expand,
              children: <Widget>[
                Container(color: Colors.black),
                if (step?.frame != null)
                  Center(
                    child: AspectRatio(
                      aspectRatio: step!.frame!.aspectRatio,
                      child: Stack(
                        fit: StackFit.expand,
                        children: <Widget>[
                          _FrameImage(step: step),
                          if (_worldFor(step) != null)
                            CustomPaint(
                              painter: PerceptionOverlayPainter(
                                world: _worldFor(step)!,
                                path: _pathFor(step),
                                options: const OverlayOptions(),
                              ),
                            ),
                        ],
                      ),
                    ),
                  )
                else if (step != null)
                  const Center(
                    child: Padding(
                      padding: EdgeInsets.all(24),
                      child: Text(
                        'This drive has no recorded image for this cycle.\n'
                        'The decisions below are the recorded ones.',
                        style: HudTheme.caption,
                        textAlign: TextAlign.center,
                      ),
                    ),
                  )
                else
                  Center(
                    child: _preparing
                        ? const CircularProgressIndicator()
                        : Text(
                            _error ?? 'Choose a mode and press play',
                            style: HudTheme.caption,
                            textAlign: TextAlign.center,
                          ),
                  ),
                if (step != null)
                  Positioned(
                    left: 10,
                    top: 10,
                    child: _decisionCard(step),
                  ),
                if (step?.comparison != null)
                  Positioned(
                    right: 10,
                    top: 10,
                    child: _comparisonCard(step!.comparison!),
                  ),
              ],
            ),
          ),
          _controls(),
        ],
      ),
    );
  }

  Widget _decisionCard(ReplayStep step) {
    final decision = step.rerun?.decision ?? step.recorded.decision;
    if (decision == null) return const SizedBox.shrink();
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        DecisionPanel(decision: decision),
        const SizedBox(height: 6),
        HudBadge(
          text: step.rerun != null ? 'RE-RUN' : 'RECORDED',
          color: step.rerun != null ? HudTheme.info : HudTheme.textSecondary,
        ),
      ],
    );
  }

  Widget _comparisonCard(ReplayComparison c) {
    return Container(
      width: 220,
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(
        color: HudTheme.background.withValues(alpha: 0.86),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(
          color: c.isMaterialDifference
              ? HudTheme.warning
              : HudTheme.outline,
        ),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Text('VS RECORDED', style: HudTheme.hudLabel),
          const SizedBox(height: 5),
          Text(
            c.summary,
            style: HudTheme.caption.copyWith(
              color: c.isMaterialDifference
                  ? HudTheme.warning
                  : HudTheme.accent,
            ),
          ),
          const SizedBox(height: 6),
          Text(
            'latency ${c.recordedLatencyMs.toStringAsFixed(0)} → '
            '${c.rerunLatencyMs.toStringAsFixed(0)} ms',
            style: HudTheme.caption.copyWith(fontSize: 11),
          ),
        ],
      ),
    );
  }

  Widget _controls() {
    final ReplayPlayer? player = _player;
    return Container(
      padding: const EdgeInsets.fromLTRB(14, 12, 14, 16),
      decoration: const BoxDecoration(
        border: Border(top: BorderSide(color: HudTheme.outline)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          SegmentedButton<ReplayMode>(
            segments: <ButtonSegment<ReplayMode>>[
              for (final ReplayMode m in ReplayMode.values)
                ButtonSegment<ReplayMode>(value: m, label: Text(m.label)),
            ],
            selected: <ReplayMode>{_mode},
            onSelectionChanged: (Set<ReplayMode> s) =>
                setState(() => _mode = s.first),
          ),
          const SizedBox(height: 6),
          Text(_mode.description, style: HudTheme.caption),
          if (_mode == ReplayMode.rerunAi && !widget.session.hasFrames)
            const Padding(
              padding: EdgeInsets.only(top: 6),
              child: Text(
                'This drive was recorded without frames, so the AI cannot be '
                're-run over it.',
                style: TextStyle(fontSize: 12, color: HudTheme.caution),
              ),
            ),
          const SizedBox(height: 12),
          Row(
            children: <Widget>[
              FilledButton.icon(
                onPressed: _preparing ? null : _start,
                icon: const Icon(Icons.play_arrow, size: 18),
                label: const Text('Play'),
              ),
              const SizedBox(width: 10),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: <Widget>[
                    Text('SPEED ${_speed.toStringAsFixed(1)}x',
                        style: HudTheme.hudLabel),
                    Slider(
                      value: _speed,
                      min: 0,
                      max: 4,
                      divisions: 8,
                      label: _speed == 0
                          ? 'as fast as possible'
                          : '${_speed.toStringAsFixed(1)}x',
                      onChanged: (double v) {
                        setState(() => _speed = v);
                        _player?.playbackSpeed = v;
                      },
                    ),
                  ],
                ),
              ),
            ],
          ),
          if (player != null && player.totalFrames > 0) ...<Widget>[
            LinearProgressIndicator(
              value: player.totalFrames == 0
                  ? 0
                  : player.currentIndex / player.totalFrames,
              backgroundColor: HudTheme.outline,
              minHeight: 3,
            ),
            const SizedBox(height: 6),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: <Widget>[
                Text('${player.currentIndex} / ${player.totalFrames}',
                    style: HudTheme.caption),
                if (_mode == ReplayMode.rerunAi)
                  Text(
                    '${player.materialDifferences} material differences · '
                    '${(player.report().decisionAgreement * 100)
                        .toStringAsFixed(1)}% agreement',
                    style: HudTheme.caption.copyWith(
                      color: player.materialDifferences > 0
                          ? HudTheme.caution
                          : HudTheme.accent,
                    ),
                  ),
              ],
            ),
          ],
        ],
      ),
    );
  }

  dynamic _worldFor(ReplayStep? step) =>
      step?.rerun?.world ?? step?.recorded.world;

  dynamic _pathFor(ReplayStep? step) =>
      step?.rerun?.path ?? step?.recorded.path;
}

class _FrameImage extends StatelessWidget {
  const _FrameImage({required this.step});

  final ReplayStep? step;

  @override
  Widget build(BuildContext context) {
    // The decoded frame is raw RGB; rendering it through a RawImage would
    // need a ui.Image, so the simplest correct thing is a painter.
    return CustomPaint(painter: _RgbPainter(step: step));
  }
}

class _RgbPainter extends CustomPainter {
  const _RgbPainter({required this.step});

  final ReplayStep? step;

  @override
  void paint(Canvas canvas, Size size) {
    // Frames are drawn by the platform image widget in the live HUD; during
    // replay the raw buffer is shown as a simple luminance rendering, which
    // is enough to judge the overlays against and avoids a per-frame texture
    // upload.
    final frame = step?.frame;
    if (frame == null) return;

    const int step_ = 2;
    final Paint paint = Paint();
    final double sx = size.width / frame.width;
    final double sy = size.height / frame.height;

    for (int y = 0; y < frame.height; y += step_) {
      for (int x = 0; x < frame.width; x += step_) {
        final (int r, int g, int b) = frame.pixelAt(x, y);
        paint.color = Color.fromARGB(255, r, g, b);
        canvas.drawRect(
          Rect.fromLTWH(x * sx, y * sy, sx * step_ + 1, sy * step_ + 1),
          paint,
        );
      }
    }
  }

  @override
  bool shouldRepaint(_RgbPainter old) =>
      old.step?.recorded.frameId != step?.recorded.frameId;
}
