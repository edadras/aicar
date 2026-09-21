import 'dart:async';
import 'dart:typed_data';
import 'dart:ui' as ui;

import '../../camera/camera_frame.dart';
import '../../decision/driving_decision.dart';

import 'package:flutter/material.dart';

import '../../planning/planned_path.dart';
import '../../recording/session_store.dart';
import '../../replay/replay_player.dart';
import '../driving_session.dart';
import '../hud/hud_panels.dart';
import '../hud/perception_overlay.dart';
import '../../world_model/world_state.dart';
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
                          _FrameImage(frame: step.frame!),
                          if (_worldFor(step) case final WorldState world)
                            CustomPaint(
                              painter: PerceptionOverlayPainter(
                                world: world,
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
    final DrivingDecision? decision =
        step.rerun?.decision ?? step.recorded.decision;
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

  /// Prefer the freshly computed world when re-running, otherwise the one
  /// that was recorded.
  WorldState? _worldFor(ReplayStep? step) =>
      step?.rerun?.world ?? step?.recorded.world;

  PlannedPath? _pathFor(ReplayStep? step) =>
      step?.rerun?.path ?? step?.recorded.path;
}

/// Renders a decoded replay frame.
///
/// The frame arrives as a raw RGB buffer. Drawing it as one rectangle per
/// pixel (or even per 2x2 block) costs tens of thousands of draw calls per
/// frame and makes replay unwatchable, so the buffer is converted once into a
/// `ui.Image` and blitted. Decoding is asynchronous, so the widget keeps the
/// previous image on screen while the next one is prepared rather than
/// flickering to black.
class _FrameImage extends StatefulWidget {
  const _FrameImage({required this.frame});

  final CameraFrame frame;

  @override
  State<_FrameImage> createState() => _FrameImageState();
}

class _FrameImageState extends State<_FrameImage> {
  ui.Image? _image;
  int? _decodedFrameId;
  bool _decoding = false;

  @override
  void initState() {
    super.initState();
    unawaited(_decode());
  }

  @override
  void didUpdateWidget(_FrameImage oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.frame.id != oldWidget.frame.id) unawaited(_decode());
  }

  @override
  void dispose() {
    _image?.dispose();
    super.dispose();
  }

  Future<void> _decode() async {
    if (_decoding || _decodedFrameId == widget.frame.id) return;
    _decoding = true;
    final CameraFrame frame = widget.frame;

    try {
      // decodeImageFromPixels wants RGBA; the pipeline's frames are RGB.
      final Uint8List rgba = Uint8List(frame.width * frame.height * 4);
      final Uint8List source = frame.bytes;
      if (frame.format == PixelFormat.rgb888) {
        for (int p = 0, q = 0; q < rgba.length; p += 3, q += 4) {
          rgba[q] = source[p];
          rgba[q + 1] = source[p + 1];
          rgba[q + 2] = source[p + 2];
          rgba[q + 3] = 255;
        }
      } else {
        for (int p = 0, q = 0; q < rgba.length; p++, q += 4) {
          final int l = source[p];
          rgba[q] = l;
          rgba[q + 1] = l;
          rgba[q + 2] = l;
          rgba[q + 3] = 255;
        }
      }

      final Completer<ui.Image> completer = Completer<ui.Image>();
      ui.decodeImageFromPixels(
        rgba,
        frame.width,
        frame.height,
        ui.PixelFormat.rgba8888,
        completer.complete,
      );
      final ui.Image image = await completer.future;

      if (!mounted) {
        image.dispose();
        return;
      }
      setState(() {
        _image?.dispose();
        _image = image;
        _decodedFrameId = frame.id;
      });
    } finally {
      _decoding = false;
    }
  }

  @override
  Widget build(BuildContext context) {
    final ui.Image? image = _image;
    if (image == null) return const ColoredBox(color: Colors.black);
    return RawImage(image: image, fit: BoxFit.fill);
  }
}
