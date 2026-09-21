import 'package:flutter/material.dart';
import 'package:intl/intl.dart';
import 'package:provider/provider.dart';

import '../../recording/session_store.dart';
import '../driving_session.dart';
import '../theme.dart';
import 'replay_screen.dart';

/// Lists recorded drives and opens them for replay.
class RecordedDrivesScreen extends StatefulWidget {
  const RecordedDrivesScreen({super.key});

  @override
  State<RecordedDrivesScreen> createState() => _RecordedDrivesScreenState();
}

class _RecordedDrivesScreenState extends State<RecordedDrivesScreen> {
  final SessionStore _store = SessionStore();
  late Future<List<RecordedSession>> _sessions;

  @override
  void initState() {
    super.initState();
    _sessions = _store.list();
  }

  void _reload() => setState(() => _sessions = _store.list());

  @override
  Widget build(BuildContext context) {
    final DateFormat formatter = DateFormat('d MMM yyyy, HH:mm');

    return Scaffold(
      appBar: AppBar(
        title: const Text('Recorded drives'),
        actions: <Widget>[
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: _reload,
          ),
        ],
      ),
      body: FutureBuilder<List<RecordedSession>>(
        future: _sessions,
        builder: (BuildContext context,
            AsyncSnapshot<List<RecordedSession>> snapshot) {
          if (snapshot.connectionState != ConnectionState.done) {
            return const Center(child: CircularProgressIndicator());
          }
          if (snapshot.hasError) {
            return Center(
              child: Text('Could not read recordings:\n${snapshot.error}',
                  style: HudTheme.caption, textAlign: TextAlign.center),
            );
          }

          final List<RecordedSession> sessions =
              snapshot.data ?? const <RecordedSession>[];
          if (sessions.isEmpty) {
            return const Center(
              child: Padding(
                padding: EdgeInsets.all(28),
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    Icon(Icons.videocam_off_outlined,
                        size: 42, color: HudTheme.textDim),
                    SizedBox(height: 14),
                    Text('No recorded drives yet', style: HudTheme.body),
                    SizedBox(height: 6),
                    Text(
                      'Start a live drive and press the record button. '
                      'Record with frames if you want to re-run the AI over '
                      'the drive afterwards.',
                      style: HudTheme.caption,
                      textAlign: TextAlign.center,
                    ),
                  ],
                ),
              ),
            );
          }

          return ListView.separated(
            padding: const EdgeInsets.fromLTRB(14, 12, 14, 28),
            itemCount: sessions.length,
            separatorBuilder: (_, __) => const SizedBox(height: 10),
            itemBuilder: (BuildContext context, int index) {
              final RecordedSession s = sessions[index];
              return Card(
                child: Padding(
                  padding: const EdgeInsets.all(14),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[
                      Row(
                        children: <Widget>[
                          Expanded(
                            child: Text(
                              formatter.format(s.startedAt),
                              style: HudTheme.body.copyWith(
                                  fontWeight: FontWeight.w600),
                            ),
                          ),
                          if (s.isIncomplete)
                            const HudBadge(
                              text: 'INTERRUPTED',
                              color: HudTheme.caution,
                            ),
                          if (!s.hasFrames) ...<Widget>[
                            const SizedBox(width: 6),
                            const HudBadge(
                              text: 'NO FRAMES',
                              color: HudTheme.textDim,
                            ),
                          ],
                        ],
                      ),
                      const SizedBox(height: 8),
                      Wrap(
                        spacing: 16,
                        runSpacing: 6,
                        children: <Widget>[
                          _stat('Duration', s.durationLabel),
                          _stat('Distance', s.distanceLabel),
                          _stat('Frames',
                              '${s.footer?.frameCount ?? 0}'),
                          _stat('Images', '${s.frameImageCount}'),
                          _stat('Size', s.sizeLabel),
                          _stat(
                            'Mean FPS',
                            (s.footer?.meanProcessingFps ?? 0)
                                .toStringAsFixed(1),
                          ),
                        ],
                      ),
                      if (s.footer != null &&
                          s.footer!.decisionCounts.isNotEmpty) ...<Widget>[
                        const SizedBox(height: 10),
                        Text('DECISIONS', style: HudTheme.hudLabel),
                        const SizedBox(height: 5),
                        Wrap(
                          spacing: 6,
                          runSpacing: 6,
                          children: <Widget>[
                            for (final MapEntry<String, int> e
                                in _sortedDecisions(s))
                              HudBadge(
                                text: '${e.key} ${e.value}',
                                color: _decisionColor(e.key),
                              ),
                          ],
                        ),
                      ],
                      const SizedBox(height: 10),
                      Text(
                        'Models: ${s.header.models.entries
                            .where((MapEntry<String, String> e) =>
                                e.value != 'none')
                            .map((MapEntry<String, String> e) => e.value)
                            .join(', ')}'
                        '${s.header.models.values.every(
                                (String v) => v == 'none')
                            ? 'classical algorithms only'
                            : ''}',
                        style: HudTheme.caption,
                      ),
                      const SizedBox(height: 10),
                      Row(
                        children: <Widget>[
                          FilledButton.icon(
                            onPressed: () => _open(s),
                            icon: const Icon(Icons.play_arrow, size: 18),
                            label: const Text('Replay'),
                          ),
                          const SizedBox(width: 8),
                          OutlinedButton.icon(
                            onPressed: () => _delete(s),
                            icon: const Icon(Icons.delete_outline, size: 18),
                            label: const Text('Delete'),
                          ),
                        ],
                      ),
                    ],
                  ),
                ),
              );
            },
          );
        },
      ),
    );
  }

  List<MapEntry<String, int>> _sortedDecisions(RecordedSession s) {
    final List<MapEntry<String, int>> entries =
        s.footer!.decisionCounts.entries.toList()
          ..sort((MapEntry<String, int> a, MapEntry<String, int> b) =>
              b.value.compareTo(a.value));
    return entries.take(5).toList();
  }

  Color _decisionColor(String state) => switch (state) {
        'emergencyBrakeSimulation' => HudTheme.critical,
        'pedestrianYield' || 'obstacleAvoidanceSimulation' =>
          HudTheme.warning,
        'uncertain' => HudTheme.warning,
        'stop' || 'wait' || 'slowDown' => HudTheme.caution,
        _ => HudTheme.accent,
      };

  Widget _stat(String label, String value) => Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Text(label.toUpperCase(),
              style: HudTheme.hudLabel.copyWith(fontSize: 10)),
          const SizedBox(height: 2),
          Text(value,
              style: HudTheme.body.copyWith(
                  fontFamily: HudTheme.monoFamily, fontSize: 14)),
        ],
      );

  Future<void> _open(RecordedSession s) async {
    final DrivingSession session = context.read<DrivingSession>();
    await Navigator.of(context).push(
      MaterialPageRoute<void>(
        builder: (_) => ReplayScreen(session: s, driving: session),
      ),
    );
    _reload();
  }

  Future<void> _delete(RecordedSession s) async {
    final bool? confirmed = await showDialog<bool>(
      context: context,
      builder: (BuildContext context) => AlertDialog(
        title: const Text('Delete this drive?'),
        content: Text(
          'This permanently removes ${s.sizeLabel} of recorded data, '
          'including the frames needed to re-run the AI over it.',
          style: HudTheme.caption,
        ),
        actions: <Widget>[
          TextButton(
            onPressed: () => Navigator.of(context).pop(false),
            child: const Text('Cancel'),
          ),
          FilledButton(
            onPressed: () => Navigator.of(context).pop(true),
            child: const Text('Delete'),
          ),
        ],
      ),
    );
    if (confirmed != true) return;
    await _store.delete(s);
    _reload();
  }
}
