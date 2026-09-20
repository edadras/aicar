import '../../perception/detection.dart';
import '../../sensors/ego_motion.dart';
import '../../tracking/object_track.dart';

/// Turns per-frame detections into identified, persistent tracks.
///
/// Not a [PerceptionModel]: a tracker has no weights and no load step, but it
/// is still swappable — SORT, ByteTrack or a learned re-identification tracker
/// all fit behind this.
abstract class ObjectTracker {
  String get trackerName;

  /// Associate [detections] with existing tracks and advance the filter.
  ///
  /// [egoMotion] is required because the tracker works in the vehicle frame:
  /// without it, every parked car would appear to be accelerating towards us.
  List<ObjectTrack> update({
    required DetectionResult detections,
    required EgoMotionState egoMotion,
    required int timestampMicros,
  });

  List<ObjectTrack> get activeTracks;

  void reset();
}
