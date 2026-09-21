import 'dart:convert';

import 'package:shared_preferences/shared_preferences.dart';

import '../camera/camera_calibration.dart';
import '../core/logging.dart';

/// Persists the camera calibration between runs.
///
/// Worth persisting because calibration is the slowest thing to redo and the
/// thing most likely to be forgotten: a phone left in the same cradle keeps
/// the same geometry across drives, and every metric output depends on it.
class CalibrationStore {
  CalibrationStore({this.key = 'camera.calibration'});

  static const String _tag = 'CalibrationStore';
  final String key;

  Future<CameraCalibration> load() async {
    try {
      final SharedPreferences prefs = await SharedPreferences.getInstance();
      final String? raw = prefs.getString(key);
      if (raw == null) return CameraCalibration.galaxyS23Default();
      return CameraCalibration.fromJson(
        jsonDecode(raw) as Map<String, dynamic>,
      );
    } catch (e) {
      // A corrupt stored calibration must not stop the app starting; the
      // defaults are clearly marked as uncalibrated, so the user is told.
      Log.warn(_tag, 'stored calibration unreadable ($e); using defaults');
      return CameraCalibration.galaxyS23Default();
    }
  }

  Future<void> save(CameraCalibration calibration) async {
    final SharedPreferences prefs = await SharedPreferences.getInstance();
    await prefs.setString(key, jsonEncode(calibration.toJson()));
    Log.info(_tag, 'saved $calibration');
  }

  Future<void> clear() async {
    final SharedPreferences prefs = await SharedPreferences.getInstance();
    await prefs.remove(key);
  }
}
