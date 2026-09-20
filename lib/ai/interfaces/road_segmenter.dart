import '../../camera/camera_frame.dart';
import '../../road/road_segmentation.dart';
import 'perception_model.dart';

/// Labels every pixel with a surface class and derives the drivable corridor.
///
/// This is what lets the stack drive where there are no lane markings at all:
/// the corridor comes from "where is there road", not from "where are the
/// white lines".
abstract class RoadSegmenter extends PerceptionModel {
  Future<RoadSegmentation> segment(CameraFrame frame);

  /// Surface classes this model distinguishes. A binary road/not-road model is
  /// perfectly usable; the UI just shows fewer categories.
  List<SurfaceClass> get supportedClasses;
}
