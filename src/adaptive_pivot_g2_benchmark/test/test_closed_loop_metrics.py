# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

import math
import unittest

from adaptive_pivot_g2_benchmark.closed_loop_metrics import (
    calculate_curve_exit_metrics,
)


class TestClosedLoopMetrics(unittest.TestCase):

    def test_detects_curve_exit_and_physical_offset(self):
        path = [
            (math.sin(index * math.pi / 40.0),
             1.0 - math.cos(index * math.pi / 40.0))
            for index in range(21)
        ]
        path.extend([
            (1.0 + 0.05 * index, 1.0)
            for index in range(1, 21)
        ])
        states = []
        for index, point in enumerate(path):
            offset = 0.05 if point[0] > 1.05 else 0.0
            states.append((
                0.1 * index,
                point[0],
                point[1] + offset,
                0.0,
                0.2,
                0.0,
            ))

        metrics = calculate_curve_exit_metrics(path, states)

        self.assertGreaterEqual(metrics['planned_curve_exit_count'], 1)
        self.assertGreater(metrics['curve_exit_sample_count'], 0)
        self.assertGreater(
            metrics['curve_exit_tracking_max_error_m'], 0.045
        )
        self.assertAlmostEqual(
            metrics['curve_exit_max_abs_linear_mps'], 0.2
        )

    def test_empty_and_invalid_parameters_are_explicit(self):
        metrics = calculate_curve_exit_metrics([], [])
        self.assertEqual(metrics['planned_curve_exit_count'], 0)
        with self.assertRaises(ValueError):
            calculate_curve_exit_metrics([], [], post_curve_distance=0.0)


if __name__ == '__main__':
    unittest.main()
