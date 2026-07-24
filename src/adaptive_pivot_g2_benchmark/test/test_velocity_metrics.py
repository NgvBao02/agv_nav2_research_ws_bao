# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

import math
import unittest

from adaptive_pivot_g2_benchmark.velocity_metrics import (
    calculate_shaper_metrics,
    calculate_velocity_metrics,
)


class TestVelocityMetrics(unittest.TestCase):

    def test_constant_command_has_zero_acceleration_and_jerk(self):
        metrics = calculate_velocity_metrics([
            (0.0, 0.3, 0.0),
            (0.1, 0.3, 0.0),
            (0.2, 0.3, 0.0),
        ])
        self.assertAlmostEqual(metrics['mean_abs_command_linear_mps'], 0.3)
        self.assertAlmostEqual(metrics['rms_command_linear_mps'], 0.3)
        self.assertAlmostEqual(metrics['max_abs_command_acceleration_mps2'], 0.0)
        self.assertAlmostEqual(metrics['max_abs_command_jerk_mps3'], 0.0)
        self.assertAlmostEqual(metrics['cruise_command_fraction'], 1.0)

    def test_ramp_recovers_acceleration_and_wheel_speed(self):
        metrics = calculate_velocity_metrics([
            (0.0, 0.0, 0.0),
            (0.1, 0.02, 0.4),
            (0.2, 0.04, 0.4),
            (0.3, 0.06, 0.4),
        ], wheel_separation_m=0.20)
        self.assertAlmostEqual(
            metrics['max_abs_command_acceleration_mps2'], 0.2
        )
        self.assertAlmostEqual(metrics['max_abs_command_jerk_mps3'], 0.0)
        self.assertAlmostEqual(
            metrics['p95_abs_command_angular_radps'], 0.4
        )
        self.assertAlmostEqual(metrics['max_command_wheel_linear_mps'], 0.10)
        self.assertAlmostEqual(
            metrics['max_abs_command_angular_acceleration_radps2'], 4.0
        )
        self.assertAlmostEqual(
            metrics['max_abs_command_lateral_acceleration_mps2'], 0.024
        )
        self.assertAlmostEqual(
            metrics['p95_abs_command_wheel_linear_mps'], 0.097
        )

    def test_safety_step_is_visible_in_jerk_metric(self):
        metrics = calculate_velocity_metrics([
            (0.0, 0.0, 0.0),
            (0.1, 0.01, 0.0),
            (0.2, 0.03, 0.0),
            (0.3, 0.0, 0.0),
        ])
        self.assertGreater(metrics['max_abs_command_jerk_mps3'], 0.0)
        self.assertTrue(math.isfinite(metrics['p95_abs_command_jerk_mps3']))

    def test_nominal_shaper_jerk_is_separated_from_safety_override(self):
        metrics = calculate_shaper_metrics([
            (0.2, False),
            (-0.9, False),
            (12.0, True),
            (math.nan, False),
        ])

        self.assertEqual(metrics['adaptive_speed_telemetry_sample_count'], 3)
        self.assertEqual(metrics['adaptive_speed_nominal_jerk_sample_count'], 2)
        self.assertAlmostEqual(
            metrics['adaptive_speed_nominal_max_abs_jerk_mps3'], 0.9
        )
        self.assertAlmostEqual(
            metrics['adaptive_speed_safety_override_fraction'], 1.0 / 3.0
        )
        self.assertAlmostEqual(
            metrics['adaptive_speed_override_p95_abs_jerk_mps3'], 12.0
        )

    def test_empty_and_non_finite_samples_are_safe(self):
        empty = calculate_velocity_metrics([])
        self.assertEqual(empty['max_command_linear_mps'], 0.0)
        invalid = calculate_velocity_metrics([
            (math.nan, 1.0, 0.0),
            (0.0, math.inf, 0.0),
        ])
        self.assertEqual(invalid, empty)

    def test_rejects_invalid_robot_geometry(self):
        with self.assertRaises(ValueError):
            calculate_velocity_metrics([], wheel_separation_m=0.0)


if __name__ == '__main__':
    unittest.main()
