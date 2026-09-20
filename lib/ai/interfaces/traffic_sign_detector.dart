import '../../camera/camera_frame.dart';
import '../../perception/detection.dart';
import '../../perception/traffic_sign.dart';
import 'perception_model.dart';

/// Detects and classifies traffic signs, including reading the number on a
/// speed limit sign.
///
/// [candidates] lets the implementation reuse boxes a general object detector
/// already produced, which avoids running a second full-frame detector purely
/// to find signs.
abstract class TrafficSignDetector extends PerceptionModel {
  Future<List<TrafficSign>> detectSigns(
    CameraFrame frame, {
    List<Detection> candidates = const <Detection>[],
  });
}
