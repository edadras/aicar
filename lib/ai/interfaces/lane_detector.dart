import '../../camera/camera_frame.dart';
import '../../road/lane.dart';
import '../../road/road_segmentation.dart';
import 'perception_model.dart';

/// Finds lane boundaries.
///
/// Two very different implementations satisfy this: a classical CV detector
/// that needs no weights at all, and a learned row-anchor model. Both must
/// return geometry in the metric vehicle frame so the planner never sees
/// pixels.
abstract class LaneDetector extends PerceptionModel {
  /// [segmentation] is optional context: when a road segmenter is available,
  /// restricting the lane search to drivable pixels removes most false
  /// positives from kerbs, shadows and adjacent carriageways.
  Future<LaneDetectionResult> detectLanes(
    CameraFrame frame, {
    RoadSegmentation? segmentation,
    LaneDetectionResult? previous,
  });
}
