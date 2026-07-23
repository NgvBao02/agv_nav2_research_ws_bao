import math
import subprocess
import sys
import unittest

from adaptive_pivot_g2_benchmark.batch_benchmark import calculate_path_deviation
from adaptive_pivot_g2_benchmark.clearance_metrics import (
    calculate_footprint_clearance,
)
from adaptive_pivot_g2_benchmark.compare_paths import (
    calculate_maneuver_metrics,
    calculate_path_metrics,
    calculate_tracking_metrics,
    condition_trajectory_for_metrics,
    resample_polyline,
)
from adaptive_pivot_g2_benchmark.execution_matrix import (
    _aggregate,
    _process_group_exists,
    _terminate_trial_process_group,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


class TestPathMetrics(unittest.TestCase):

    @staticmethod
    def _pose(x, y, yaw):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose

    def test_straight_path_has_length_and_zero_curvature(self):
        metrics = calculate_path_metrics([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
        self.assertEqual(metrics['point_count'], 3)
        self.assertEqual(metrics['path_length_m'], 2.0)
        self.assertEqual(metrics['max_abs_curvature_1pm'], 0.0)

    def test_right_angle_has_nonzero_curvature(self):
        metrics = calculate_path_metrics([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
        self.assertGreater(metrics['max_abs_curvature_1pm'], 0.0)

    def test_resampling_uses_uniform_spacing_and_preserves_endpoints(self):
        sampled = resample_polyline([(0.0, 0.0), (0.12, 0.0)], spacing=0.05)
        self.assertEqual(sampled[0], (0.0, 0.0))
        self.assertEqual(sampled[-1], (0.12, 0.0))
        self.assertEqual(len(sampled), 4)

    def test_trajectory_conditioning_rejects_millimetre_amcl_jitter(self):
        noisy_line = [
            (index * 0.005, 0.002 if index % 2 else -0.002)
            for index in range(201)
        ]
        conditioned = condition_trajectory_for_metrics(noisy_line)
        metrics = calculate_path_metrics(conditioned, curvature_stride=3)
        self.assertLess(metrics['max_abs_curvature_1pm'], 0.25)
        self.assertAlmostEqual(metrics['path_length_m'], 1.0, places=2)

    def test_tracking_metrics_use_distance_to_segments(self):
        reference = [(0.0, 0.0), (2.0, 0.0)]
        executed = [(0.5, 0.1), (1.0, 0.1), (2.0, 0.1)]
        metrics = calculate_tracking_metrics(executed, reference)
        self.assertAlmostEqual(metrics['tracking_rmse_m'], 0.1)
        self.assertAlmostEqual(metrics['tracking_max_error_m'], 0.1)
        self.assertAlmostEqual(metrics['final_position_error_m'], 0.1)

    def test_path_deviation_is_zero_for_resampled_same_line(self):
        reference = [(0.0, 0.0), (2.0, 0.0)]
        output = [(0.25, 0.0), (0.75, 0.0), (1.75, 0.0)]
        metrics = calculate_path_deviation(output, reference)
        self.assertAlmostEqual(metrics['deviation_rmse_m'], 0.0)
        self.assertAlmostEqual(metrics['deviation_max_m'], 0.0)

    def test_maneuver_metrics_do_not_treat_pivot_as_translation_curvature(self):
        path = Path()
        path.poses = [
            self._pose(0.0, 0.0, 0.0),
            self._pose(1.0, 0.0, 0.0),
            self._pose(1.0, 0.0, 0.5 * math.pi),
            self._pose(1.0, 1.0, 0.5 * math.pi),
        ]

        metrics = calculate_maneuver_metrics(path)

        self.assertEqual(metrics['translation_segment_count'], 2)
        self.assertEqual(metrics['pivot_marker_count'], 1)
        self.assertAlmostEqual(metrics['pivot_total_angle_rad'], 0.5 * math.pi)
        self.assertAlmostEqual(metrics['translation_path_length_m'], 2.0)
        self.assertAlmostEqual(metrics['translation_curvature_energy_1pm'], 0.0)

    def test_footprint_clearance_uses_robot_boundary_not_only_center(self):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.info.resolution = 0.1
        occupancy_grid.info.width = 100
        occupancy_grid.info.height = 100
        occupancy_grid.data = [0] * 10000
        for row in range(100):
            occupancy_grid.data[row * 100 + 50] = 100
        path = Path()
        path.poses = [self._pose(4.0, 5.0, 0.0)]

        metrics = calculate_footprint_clearance(path, occupancy_grid)

        self.assertGreater(metrics['footprint_clearance_min_m'], 0.6)
        self.assertLess(metrics['footprint_clearance_min_m'], 0.9)
        self.assertEqual(metrics['footprint_collision_sample_count'], 0)

    def test_execution_aggregate_reports_success_and_conditional_metrics(self):
        records = [
            {
                'method': 'simple',
                'success': True,
                'execution_time_s': 10.0,
            },
            {
                'method': 'simple',
                'success': True,
                'execution_time_s': 14.0,
            },
            {
                'method': 'simple',
                'success': False,
                'execution_time_s': 100.0,
            },
            {
                'method': 'simple',
                'success': True,
                'execution_time_s': math.nan,
            },
        ]

        aggregate = _aggregate(records, ['simple'])['simple']

        self.assertEqual(aggregate['trial_count'], 4)
        self.assertEqual(aggregate['success_count'], 3)
        self.assertEqual(aggregate['success_rate'], 0.75)
        self.assertEqual(aggregate['execution_time_s_sample_count'], 2)
        self.assertEqual(aggregate['execution_time_s_mean'], 12.0)
        self.assertAlmostEqual(
            aggregate['execution_time_s_stdev'], math.sqrt(8.0)
        )

    def test_trial_cleanup_terminates_only_its_dedicated_process_group(self):
        process = subprocess.Popen(
            [sys.executable, '-c', 'import time; time.sleep(60)'],
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            self.assertTrue(_process_group_exists(process.pid))
            _terminate_trial_process_group(process)
            self.assertFalse(_process_group_exists(process.pid))
            self.assertIsNotNone(process.poll())
        finally:
            _terminate_trial_process_group(process)
