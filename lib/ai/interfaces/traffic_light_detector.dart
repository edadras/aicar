import '../../camera/camera_frame.dart';
import '../../perception/detection.dart';
import '../../perception/traffic_light.dart';
import 'perception_model.dart';

/// Detects traffic lights, classifies the illuminated aspect, and decides
/// whether the light governs the ego vehicle's movement.
abstract class TrafficLightDetector extends PerceptionModel {
  Future<List<TrafficLight>> detectLights(
    CameraFrame frame, {
    List<Detection> candidates = const <Detection>[],
  });
}
