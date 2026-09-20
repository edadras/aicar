import '../../camera/camera_frame.dart';
import '../../perception/detection.dart';
import 'perception_model.dart';

/// Finds road objects in a single frame.
///
/// Implementations: a native TFLite YOLO/SSD detector, and the explicit
/// "no model installed" detector that reports a degraded result rather than
/// an empty road.
abstract class ObjectDetector extends PerceptionModel {
  /// Run detection on [frame].
  ///
  /// Implementations must return a [DetectionResult] even on failure —
  /// throwing would stall the pipeline, and a degraded result carries strictly
  /// more information than an exception the caller has to interpret.
  Future<DetectionResult> detect(CameraFrame frame);

  /// Detection score below which candidates are discarded.
  double get scoreThreshold;

  /// Classes this detector can actually produce, for the UI and for telling
  /// the user what the installed model does and does not cover.
  List<String> get supportedLabels;
}
