import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../../ai/model_descriptor.dart';
import '../../core/safety.dart';
import '../driving_session.dart';
import '../hud/hud_panels.dart';
import '../theme.dart';
import 'ai_models_screen.dart';
import 'calibration_screen.dart';
import 'developer_debug_screen.dart';
import 'live_drive_screen.dart';
import 'navigation_screen.dart';
import 'performance_screen.dart';
import 'recorded_drives_screen.dart';
import 'settings_screen.dart';

/// Home screen: readiness at a glance, then the way into everything else.
class DashboardScreen extends StatelessWidget {
  const DashboardScreen({super.key});

  @override
  Widget build(BuildContext context) {
    final DrivingSession session = context.watch<DrivingSession>();
    final List<ModelRole> missing = session.registry.missingRoles;
    final bool calibrated = session.calibration.isCalibrated;

    return Scaffold(
      appBar: AppBar(
        title: const Text('Autonomous Driving Simulator'),
        actions: <Widget>[
          IconButton(
            icon: const Icon(Icons.settings_outlined),
            onPressed: () => Navigator.of(context).push(
              MaterialPageRoute<void>(
                builder: (_) => const SettingsScreen(),
              ),
            ),
          ),
        ],
      ),
      body: ListView(
        padding: const EdgeInsets.fromLTRB(14, 8, 14, 28),
        children: <Widget>[
          const Center(child: SimulationOnlyBadge()),
          const SizedBox(height: 4),
          const SectionHeader(
            'Readiness',
            subtitle: 'What works right now, and what does not',
          ),
          _ReadinessCard(
            calibrated: calibrated,
            missingRoles: missing,
            session: session,
          ),

          const SectionHeader('Drive'),
          _BigAction(
            icon: Icons.play_circle_outline,
            title: 'Live drive',
            subtitle: 'Camera, perception, planning and the simulated HUD',
            color: HudTheme.accent,
            onTap: () async {
              await session.start();
              if (!context.mounted) return;
              await Navigator.of(context).push(
                MaterialPageRoute<void>(
                  builder: (_) => const LiveDriveScreen(),
                ),
              );
              await session.stop();
            },
          ),
          const SizedBox(height: 10),
          _BigAction(
            icon: Icons.map_outlined,
            title: 'Navigation',
            subtitle: 'Pick a destination — route intent only, never steering',
            color: HudTheme.info,
            onTap: () => Navigator.of(context).push(
              MaterialPageRoute<void>(
                builder: (_) => const NavigationScreen(),
              ),
            ),
          ),

          const SectionHeader('Set up'),
          _Tile(
            icon: Icons.straighten,
            title: 'Calibration',
            subtitle: calibrated
                ? 'Camera height ${session.calibration.cameraHeightMeters
                    .toStringAsFixed(2)} m, pitch '
                    '${session.calibration.pitchDegrees.toStringAsFixed(1)}°'
                : 'Not calibrated — distances are estimates from defaults',
            warning: !calibrated,
            onTap: () => Navigator.of(context).push(
              MaterialPageRoute<void>(
                builder: (_) => const CalibrationScreen(),
              ),
            ),
          ),
          _Tile(
            icon: Icons.memory,
            title: 'AI models',
            subtitle: session.registry.statusSummary,
            warning: missing.isNotEmpty,
            onTap: () => Navigator.of(context).push(
              MaterialPageRoute<void>(
                builder: (_) => const AiModelsScreen(),
              ),
            ),
          ),

          const SectionHeader('Review'),
          _Tile(
            icon: Icons.video_library_outlined,
            title: 'Recorded drives',
            subtitle: 'Review a drive, or re-run the AI over it',
            onTap: () => Navigator.of(context).push(
              MaterialPageRoute<void>(
                builder: (_) => const RecordedDrivesScreen(),
              ),
            ),
          ),
          _Tile(
            icon: Icons.speed,
            title: 'Performance',
            subtitle: 'Per-stage timings and the end-to-end latency budget',
            onTap: () => Navigator.of(context).push(
              MaterialPageRoute<void>(
                builder: (_) => PerformanceScreen(session: session),
              ),
            ),
          ),
          _Tile(
            icon: Icons.terminal,
            title: 'Developer debug',
            subtitle: 'Every internal number, and the log',
            onTap: () => Navigator.of(context).push(
              MaterialPageRoute<void>(
                builder: (_) => DeveloperDebugScreen(session: session),
              ),
            ),
          ),

          const SectionHeader('Safety'),
          Card(
            child: Padding(
              padding: const EdgeInsets.all(14),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  Text(SafetyMode.banner,
                      style: HudTheme.body.copyWith(
                        fontWeight: FontWeight.w600,
                        color: HudTheme.info,
                      )),
                  const SizedBox(height: 8),
                  const Text(
                    'This build computes what an autonomous driving system '
                    'would do and shows it. It contains no code for any of '
                    'the following, and none is reachable from any screen:',
                    style: HudTheme.caption,
                  ),
                  const SizedBox(height: 8),
                  for (final String iface in SafetyMode.forbiddenInterfaces)
                    Padding(
                      padding: const EdgeInsets.only(bottom: 3),
                      child: Row(
                        children: <Widget>[
                          const Icon(Icons.block,
                              size: 13, color: HudTheme.textDim),
                          const SizedBox(width: 7),
                          Text(iface, style: HudTheme.caption),
                        ],
                      ),
                    ),
                  const SizedBox(height: 8),
                  const Text(
                    'The vehicle remains under the control of its human '
                    'driver at all times.',
                    style: HudTheme.caption,
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _ReadinessCard extends StatelessWidget {
  const _ReadinessCard({
    required this.calibrated,
    required this.missingRoles,
    required this.session,
  });

  final bool calibrated;
  final List<ModelRole> missingRoles;
  final DrivingSession session;

  @override
  Widget build(BuildContext context) {
    final bool detectorMissing =
        missingRoles.contains(ModelRole.objectDetection);
    final String? detectorName =
        session.registry.selectedFor(ModelRole.objectDetection)?.name;

    return Card(
      child: Padding(
        padding: const EdgeInsets.all(14),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: <Widget>[
            _ReadinessRow(
              ok: true,
              title: 'Camera, IMU and GPS',
              detail: 'Built in — no model files required',
            ),
            _ReadinessRow(
              ok: true,
              title: 'Lanes, drivable area, distance, planning',
              detail: 'Classical algorithms — no extra model files needed',
            ),
            _ReadinessRow(
              ok: !detectorMissing,
              title: 'Object detection',
              detail: detectorMissing
                  ? 'No detector selected — vehicles, pedestrians and '
                      'obstacles will NOT be detected'
                  : detectorName ?? 'Neural detector installed',
              severe: detectorMissing,
            ),
            _ReadinessRow(
              ok: calibrated,
              title: 'Camera calibration',
              detail: calibrated
                  ? 'Measured'
                  : 'Using defaults — every distance is less certain',
            ),
          ],
        ),
      ),
    );
  }
}

class _ReadinessRow extends StatelessWidget {
  const _ReadinessRow({
    required this.ok,
    required this.title,
    required this.detail,
    this.severe = false,
  });

  final bool ok;
  final String title;
  final String detail;
  final bool severe;

  @override
  Widget build(BuildContext context) {
    final Color color = ok
        ? HudTheme.accent
        : (severe ? HudTheme.critical : HudTheme.caution);
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 6),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          Icon(
            ok ? Icons.check_circle : Icons.error_outline,
            size: 18,
            color: color,
          ),
          const SizedBox(width: 10),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                Text(title, style: HudTheme.body),
                const SizedBox(height: 2),
                Text(detail, style: HudTheme.caption),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class _BigAction extends StatelessWidget {
  const _BigAction({
    required this.icon,
    required this.title,
    required this.subtitle,
    required this.color,
    required this.onTap,
  });

  final IconData icon;
  final String title;
  final String subtitle;
  final Color color;
  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) {
    return Material(
      color: HudTheme.surface,
      borderRadius: BorderRadius.circular(14),
      child: InkWell(
        borderRadius: BorderRadius.circular(14),
        onTap: onTap,
        child: Container(
          padding: const EdgeInsets.all(16),
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(14),
            border: Border.all(color: color.withValues(alpha: 0.4)),
          ),
          child: Row(
            children: <Widget>[
              Container(
                width: 44,
                height: 44,
                decoration: BoxDecoration(
                  color: color.withValues(alpha: 0.15),
                  borderRadius: BorderRadius.circular(11),
                ),
                child: Icon(icon, color: color),
              ),
              const SizedBox(width: 14),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: <Widget>[
                    Text(title,
                        style: HudTheme.body.copyWith(
                            fontSize: 16, fontWeight: FontWeight.w600)),
                    const SizedBox(height: 3),
                    Text(subtitle, style: HudTheme.caption),
                  ],
                ),
              ),
              const Icon(Icons.chevron_right, color: HudTheme.textDim),
            ],
          ),
        ),
      ),
    );
  }
}

class _Tile extends StatelessWidget {
  const _Tile({
    required this.icon,
    required this.title,
    required this.subtitle,
    required this.onTap,
    this.warning = false,
  });

  final IconData icon;
  final String title;
  final String subtitle;
  final VoidCallback onTap;
  final bool warning;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 8),
      child: Card(
        child: ListTile(
          leading: Icon(icon,
              color: warning ? HudTheme.caution : HudTheme.textSecondary),
          title: Text(title, style: HudTheme.body),
          subtitle: Text(subtitle, style: HudTheme.caption),
          trailing: const Icon(Icons.chevron_right,
              color: HudTheme.textDim, size: 20),
          onTap: onTap,
        ),
      ),
    );
  }
}
