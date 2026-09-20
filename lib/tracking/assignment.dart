import 'dart:math' as math;
import 'dart:typed_data';

/// Optimal one-to-one assignment (Hungarian / Kuhn–Munkres, O(n³)).
///
/// Greedy nearest-neighbour matching is the usual shortcut, and it is exactly
/// what causes identity swaps in dense traffic: two cars side by side each
/// grab the wrong detection because the first one matched first. Solving the
/// assignment globally removes that failure mode for the cost of a few
/// microseconds at realistic track counts (< 40).
class HungarianAlgorithm {
  const HungarianAlgorithm._();

  /// [cost] is row-major `rows x cols`. Returns `assignment[row] = col`, or
  /// `-1` when a row is left unassigned (possible when `cols < rows`).
  ///
  /// Implementation is the O(n³) potentials/shortest-augmenting-path variant,
  /// which is numerically stable with real-valued costs.
  static List<int> solve(Float64List cost, int rows, int cols) {
    if (rows == 0 || cols == 0) return List<int>.filled(rows, -1);

    final int n = rows;
    final int m = cols;
    final int size = math.max(n, m);

    // Pad to a square matrix with a large-but-finite cost so that padded
    // assignments are always worse than any real one.
    double maxCost = 0;
    for (int i = 0; i < cost.length; i++) {
      if (cost[i].isFinite && cost[i] > maxCost) maxCost = cost[i];
    }
    final double padCost = maxCost * 10 + 1000;

    final Float64List a = Float64List(size * size);
    for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
        if (i < n && j < m) {
          final double c = cost[i * m + j];
          a[i * size + j] = c.isFinite ? c : padCost;
        } else {
          a[i * size + j] = padCost;
        }
      }
    }

    // u/v are the dual potentials; way[] reconstructs the augmenting path.
    final Float64List u = Float64List(size + 1);
    final Float64List v = Float64List(size + 1);
    final Int32List p = Int32List(size + 1); // p[col] = row
    final Int32List way = Int32List(size + 1);

    for (int i = 1; i <= size; i++) {
      p[0] = i;
      int j0 = 0;
      final Float64List minv = Float64List(size + 1)
        ..fillRange(0, size + 1, double.infinity);
      final List<bool> used = List<bool>.filled(size + 1, false);

      do {
        used[j0] = true;
        final int i0 = p[j0];
        double delta = double.infinity;
        int j1 = 0;
        for (int j = 1; j <= size; j++) {
          if (used[j]) continue;
          final double cur = a[(i0 - 1) * size + (j - 1)] - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
        for (int j = 0; j <= size; j++) {
          if (used[j]) {
            u[p[j]] += delta;
            v[j] -= delta;
          } else {
            minv[j] -= delta;
          }
        }
        j0 = j1;
      } while (p[j0] != 0);

      do {
        final int j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0 != 0);
    }

    final List<int> assignment = List<int>.filled(n, -1);
    for (int j = 1; j <= size; j++) {
      final int row = p[j] - 1;
      final int col = j - 1;
      if (row >= 0 && row < n && col < m) {
        assignment[row] = col;
      }
    }
    return assignment;
  }
}

/// One (track, detection) association candidate with its component costs,
/// retained so the debug overlay can explain *why* a match was made.
class AssociationCandidate {
  const AssociationCandidate({
    required this.trackIndex,
    required this.detectionIndex,
    required this.iouCost,
    required this.metricCost,
    required this.classCost,
    required this.totalCost,
  });

  final int trackIndex;
  final int detectionIndex;
  final double iouCost;
  final double metricCost;
  final double classCost;
  final double totalCost;
}
