# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

import math
import unittest

from adaptive_pivot_g2_benchmark.localization_metrics import (
    align_odometry_trace,
    calculate_localization_metrics,
    calculate_pose_trace_error_metrics,
)


class TestLocalizationMetrics(unittest.TestCase):

    def test_tracks_growth_and_final_covariance(self):
        metrics = calculate_localization_metrics([
            (0.0, 0.01, 0.02, 0.001, 0.002, 0.003),
            (0.1, 0.03, 0.04, 0.004, 0.005, 0.006),
            (0.2, 0.05, 0.06, 0.007, 0.008, 0.009),
        ])
        self.assertEqual(metrics['localization_sample_count'], 3)
        self.assertAlmostEqual(
            metrics['localization_position_error_mean_m'], 0.03
        )
        self.assertAlmostEqual(
            metrics['localization_position_error_final_m'], 0.05
        )
        self.assertAlmostEqual(
            metrics['localization_yaw_error_final_rad'], 0.06
        )
        self.assertAlmostEqual(
            metrics['localization_covariance_xy_final_m2'], 0.008
        )

    def test_invalid_samples_are_rejected(self):
        empty = calculate_localization_metrics([])
        invalid = calculate_localization_metrics([
            (0.0, math.nan, 0.0, 0.0, 0.0, 0.0),
            (0.1, -1.0, 0.0, 0.0, 0.0, 0.0),
        ])
        self.assertEqual(invalid, empty)

    def test_aligns_relative_odometry_and_measures_drift(self):
        ground_truth = [
            (0.0, 5.0, -2.0, math.pi / 2.0),
            (1.0, 5.0, -1.0, math.pi / 2.0),
            (2.0, 5.0, 0.0, math.pi / 2.0),
        ]
        raw_odometry = [
            (0.0, 0.0, 0.0, 0.0),
            (1.0, 1.0, 0.0, 0.0),
            (2.0, 2.1, 0.0, 0.0),
        ]
        aligned = align_odometry_trace(raw_odometry, ground_truth)
        self.assertAlmostEqual(aligned[1][1], 5.0)
        self.assertAlmostEqual(aligned[1][2], -1.0)
        metrics = calculate_pose_trace_error_metrics(
            ground_truth, aligned, 'odometry'
        )
        self.assertEqual(metrics['odometry_sample_count'], 3)
        self.assertAlmostEqual(
            metrics['odometry_position_error_final_m'], 0.1
        )
        self.assertAlmostEqual(metrics['odometry_yaw_error_max_rad'], 0.0)


if __name__ == '__main__':
    unittest.main()
