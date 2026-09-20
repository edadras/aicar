import 'dart:math' as math;
import 'dart:typed_data';

import 'package:aicar/tracking/assignment.dart';
import 'package:flutter_test/flutter_test.dart';

double totalCost(Float64List cost, int cols, List<int> assignment) {
  double sum = 0;
  for (int r = 0; r < assignment.length; r++) {
    if (assignment[r] >= 0) sum += cost[r * cols + assignment[r]];
  }
  return sum;
}

/// Brute-force optimum for small matrices, used as the reference.
double bruteForceOptimum(Float64List cost, int rows, int cols) {
  final List<int> cols0 = List<int>.generate(cols, (int i) => i);
  double best = double.infinity;

  void recurse(int row, List<int> remaining, double acc) {
    if (row == rows) {
      if (acc < best) best = acc;
      return;
    }
    if (remaining.isEmpty) return;
    for (int i = 0; i < remaining.length; i++) {
      final int c = remaining[i];
      final List<int> next = List<int>.from(remaining)..removeAt(i);
      recurse(row + 1, next, acc + cost[row * cols + c]);
    }
  }

  recurse(0, cols0, 0);
  return best;
}

void main() {
  group('HungarianAlgorithm', () {
    test('solves a known 3x3 assignment optimally', () {
      final Float64List cost = Float64List.fromList(<double>[
        4, 1, 3, //
        2, 0, 5, //
        3, 2, 2, //
      ]);
      final List<int> a = HungarianAlgorithm.solve(cost, 3, 3);
      expect(a.toSet().length, 3, reason: 'assignment must be a permutation');
      expect(totalCost(cost, 3, a), closeTo(5, 1e-9));
    });

    test('identity matrix picks the diagonal', () {
      const int n = 5;
      final Float64List cost = Float64List(n * n)..fillRange(0, n * n, 1);
      for (int i = 0; i < n; i++) {
        cost[i * n + i] = 0;
      }
      final List<int> a = HungarianAlgorithm.solve(cost, n, n);
      for (int i = 0; i < n; i++) {
        expect(a[i], i);
      }
    });

    test('handles more rows than columns by leaving rows unassigned', () {
      final Float64List cost = Float64List.fromList(<double>[
        1, 9, //
        9, 1, //
        5, 5, //
      ]);
      final List<int> a = HungarianAlgorithm.solve(cost, 3, 2);
      expect(a.where((int c) => c >= 0).length, 2);
      expect(a[0], 0);
      expect(a[1], 1);
      expect(a[2], -1);
    });

    test('handles more columns than rows', () {
      final Float64List cost = Float64List.fromList(<double>[
        7, 2, 9, 4, //
        3, 8, 1, 6, //
      ]);
      final List<int> a = HungarianAlgorithm.solve(cost, 2, 4);
      expect(a[0], 1);
      expect(a[1], 2);
      expect(totalCost(cost, 4, a), closeTo(3, 1e-9));
    });

    test('empty inputs are safe', () {
      expect(HungarianAlgorithm.solve(Float64List(0), 0, 0), isEmpty);
      expect(HungarianAlgorithm.solve(Float64List(0), 3, 0), <int>[-1, -1, -1]);
    });

    test('matches brute force on random square matrices', () {
      final math.Random rng = math.Random(7);
      for (int trial = 0; trial < 40; trial++) {
        final int n = 2 + rng.nextInt(4);
        final Float64List cost = Float64List(n * n);
        for (int i = 0; i < n * n; i++) {
          cost[i] = (rng.nextDouble() * 20).roundToDouble();
        }
        final List<int> a = HungarianAlgorithm.solve(cost, n, n);
        expect(a.toSet().length, n, reason: 'must be a permutation (n=$n)');
        expect(
          totalCost(cost, n, a),
          closeTo(bruteForceOptimum(cost, n, n), 1e-9),
          reason: 'trial $trial (n=$n)',
        );
      }
    });

    test('matches brute force on random rectangular matrices', () {
      final math.Random rng = math.Random(11);
      for (int trial = 0; trial < 30; trial++) {
        final int rows = 2 + rng.nextInt(3);
        final int cols = rows + rng.nextInt(3);
        final Float64List cost = Float64List(rows * cols);
        for (int i = 0; i < rows * cols; i++) {
          cost[i] = (rng.nextDouble() * 15).roundToDouble();
        }
        final List<int> a = HungarianAlgorithm.solve(cost, rows, cols);
        expect(a.where((int c) => c >= 0).length, rows);
        expect(
          totalCost(cost, cols, a),
          closeTo(bruteForceOptimum(cost, rows, cols), 1e-9),
          reason: 'trial $trial (${rows}x$cols)',
        );
      }
    });

    test('non-finite costs are treated as forbidden, not crashes', () {
      final Float64List cost = Float64List.fromList(<double>[
        double.infinity, 1, //
        2, double.infinity, //
      ]);
      final List<int> a = HungarianAlgorithm.solve(cost, 2, 2);
      expect(a[0], 1);
      expect(a[1], 0);
    });
  });
}
