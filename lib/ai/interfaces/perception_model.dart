import '../model_descriptor.dart';

/// Common lifecycle for every swappable perception model.
///
/// The contract is deliberately small: load, report what you are, run, close.
/// Everything specific to a role lives in that role's own interface, so a new
/// model architecture only has to satisfy one narrow interface plus this.
abstract class PerceptionModel {
  /// Stable identifier shown in the AI Models screen and written into
  /// recordings so a dataset can be traced back to the model that produced it.
  String get modelId;

  String get displayName;

  ModelRole get role;

  /// `true` once the backing weights are resident and the model can run.
  ///
  /// A model that is not ready must never be silently skipped: its stage
  /// returns an explicitly degraded result so confidence collapses instead of
  /// the stack believing an empty scene.
  bool get isReady;

  /// Why the model is not ready, for the UI.
  String? get unavailableReason;

  /// Descriptor the model was configured from, if any.
  ModelDescriptor? get descriptor;

  Future<void> load();

  Future<void> close();
}
