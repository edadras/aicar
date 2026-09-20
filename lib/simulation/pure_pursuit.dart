import 'dart:math' as math;

import '../core/geometry.dart';
import '../planning/planned_path.dart';
import 'vehicle_state.dart';

/// Pure-pursuit lateral controller.
///
/// Picks a point on the planned path a look-ahead distance in front of the
/// vehicle and computes the constant-curvature arc that reaches it:
///
///   kappa = 2 * x_lookahead / L_d²
///
/// Chosen over a PID on cross-track error because pure pursuit is
/// geometrically exact for a kinematic vehicle, needs one parameter instead of
/// three, and — importantly for a system that draws its own reasoning on
/// screen — the arc it produces is the arc you can see on the HUD.
///
/// The look-ahead distance is speed-dependent: too short and the controller
/// oscillates, too long and it cuts corners. The usual `L_d = k*v + c` rule
/// with bounds is what makes it stable from a car park to a motorway.
class PurePursuitController {
  const PurePursuitController({
    this.lookaheadGain = 0.65,
    this.minLookaheadMeters = 4.0,
    this.maxLookaheadMeters = 22.0,
    this.crossTrackGain = 0.35,
    this.headingGain = 0.55,
  });

  /// Seconds of travel to look ahead. 0.65 is a common starting point and
  /// gives ~9 m at 50 km/h.
  final double lookaheadGain;

  final double minLookaheadMeters;
  final double maxLookaheadMeters;

  /// Extra correction proportional to the immediate lateral error. Pure
  /// pursuit alone converges slowly when the vehicle starts well off the path;
  /// a small Stanley-style term fixes that without destabilising it.
  final double crossTrackGain;

  final double headingGain;

  /// Road-wheel angle, radians, positive = right.
  double computeSteering({
    required PlannedPath path,
    required VehicleState vehicle,
    required VehicleParameters parameters,
    required double speedMps,
  }) {
    if (!path.isUsable) return 0;

    final double lookahead = clampDouble(
      lookaheadGain * math.max(speedMps, 1.0),
      minLookaheadMeters,
      math.min(maxLookaheadMeters, math.max(4.0, path.maxRangeMeters * 0.8)),
    );

    final double? targetLateral = path.lateralAt(lookahead);
    if (targetLateral == null) return 0;

    // Pure-pursuit curvature to the look-ahead point. The vehicle is at the
    // origin of the vehicle frame, so the geometry is just:
    //   kappa = 2*x / Ld²
    final double distanceSquared =
        lookahead * lookahead + targetLateral * targetLateral;
    if (distanceSquared < 1e-6) return 0;
    final double curvature = 2 * targetLateral / distanceSquared;

    // Immediate lateral error and heading error at the vehicle.
    final double crossTrack = path.lateralAt(0.5) ?? 0;
    final double headingError = path.headingAt(1.0);

    final double correction = crossTrackGain * crossTrack /
            math.max(speedMps, 3.0) +
        headingGain * headingError;

    final double steering = math.atan(
          curvature * parameters.wheelbaseMeters,
        ) +
        correction;

    return clampDouble(
      steering,
      -parameters.maxSteeringAngleRadians,
      parameters.maxSteeringAngleRadians,
    );
  }

  /// Look-ahead distance actually used, for the debug overlay.
  double lookaheadFor(double speedMps, double pathRange) => clampDouble(
        lookaheadGain * math.max(speedMps, 1.0),
        minLookaheadMeters,
        math.min(maxLookaheadMeters, math.max(4.0, pathRange * 0.8)),
      );
}
