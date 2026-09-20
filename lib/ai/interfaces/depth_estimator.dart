import '../../camera/camera_frame.dart';
import '../../depth/depth_map.dart';
import 'perception_model.dart';

/// Produces a dense depth map from a single image.
///
/// The Galaxy S23 has no depth sensor, so every implementation here is
/// monocular and therefore *relative* unless it was trained for metric depth.
/// Implementations must set [DepthMap.scale] honestly; the fusion layer
/// depends on knowing whether the numbers mean metres.
abstract class DepthEstimator extends PerceptionModel {
  Future<DepthMap> estimate(CameraFrame frame);

  /// What the raw output means.
  DepthScale get outputScale;

  /// Resolution of the produced map. Usually much smaller than the frame.
  (int, int) get outputSize;
}
