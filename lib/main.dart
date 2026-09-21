import 'package:flutter/material.dart';
import 'package:wakelock_plus/wakelock_plus.dart';

import 'app.dart';
import 'core/logging.dart';
import 'core/safety.dart';

/// Mobile Autonomous Driving Simulator.
///
/// SIMULATION ONLY. The application analyses the road through the phone's
/// camera and sensors and computes what an autonomous driving system would
/// command. Those commands are displayed and logged; they are never sent
/// anywhere. See `lib/core/safety.dart` and `docs/SAFETY.md`.
Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();

  // The invariant is asserted at startup so a build that somehow violated it
  // would fail immediately and visibly rather than at an arbitrary moment.
  SafetyMode.assertSimulationOnly();
  Log.info('main', SafetyMode.banner);

  await configureSystemChrome();

  // A HUD that sleeps mid-drive is useless, and the phone is on a dashboard
  // where the screen is the only output.
  try {
    await WakelockPlus.enable();
  } catch (e) {
    Log.warn('main', 'could not keep the screen awake: $e');
  }

  FlutterError.onError = (FlutterErrorDetails details) {
    Log.error('flutter', details.summary.toString(), details.exception,
        details.stack);
    FlutterError.presentError(details);
  };

  runApp(const AiCarApp());
}
