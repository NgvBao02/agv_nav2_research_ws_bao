import json
import math
import os
from pathlib import Path as FilePath
import subprocess
import sys
import tempfile
import time
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
    normalize_planner_id,
    normalize_smoother_visibility,
    PathComparisonNode,
    PLANNER_IDS,
    resample_polyline,
    SMOOTHER_IDS,
)
from adaptive_pivot_g2_benchmark.execution_matrix import (
    _aggregate,
    _arguments,
    _compact_summary_record,
    _is_infrastructure_failure,
    _matching_successful_record,
    _process_group_exists,
    _run_launch,
    _terminate_trial_process_group,
)
from adaptive_pivot_g2_benchmark.path_contract import (
    anchor_path_goal,
    anchor_path_start,
    canonicalize_planner_path,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String


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

    def test_path_contract_removes_only_redundant_consecutive_poses(self):
        path = Path()
        path.poses = [
            self._pose(0.0, 0.0, 0.0),
            self._pose(0.0, 0.0, 0.0),
            self._pose(1.0, 0.0, 0.0),
        ]

        canonical, removed = canonicalize_planner_path(path)

        self.assertEqual(removed, 1)
        self.assertEqual(len(canonical.poses), 2)
        self.assertEqual(len(path.poses), 3)

    def test_path_contract_preserves_in_place_heading_change(self):
        path = Path()
        path.poses = [
            self._pose(0.0, 0.0, 0.0),
            self._pose(0.0, 0.0, 0.5 * math.pi),
            self._pose(1.0, 0.0, 0.5 * math.pi),
        ]

        canonical, removed = canonicalize_planner_path(path)

        self.assertEqual(removed, 0)
        self.assertEqual(len(canonical.poses), 3)

    def test_path_contract_anchors_grid_cell_endpoint_to_requested_goal(self):
        path = Path()
        path.header.frame_id = 'map'
        path.poses = [
            self._pose(0.0, 0.0, 0.0),
            self._pose(0.975, 0.975, 0.0),
        ]
        goal = self._pose(1.0, 1.0, 0.4)
        goal.header.frame_id = 'map'

        anchored, adjustment = anchor_path_goal(path, goal)

        self.assertAlmostEqual(adjustment, math.hypot(0.025, 0.025))
        self.assertEqual(anchored.poses[-1].pose.position.x, 1.0)
        self.assertEqual(anchored.poses[-1].pose.position.y, 1.0)
        self.assertAlmostEqual(
            anchored.poses[-1].pose.orientation.z, math.sin(0.2)
        )
        self.assertEqual(path.poses[-1].pose.position.x, 0.975)
        with self.assertRaises(ValueError):
            anchor_path_goal(path, self._pose(2.0, 2.0, 0.0))

    def test_path_contract_anchors_grid_cell_start_to_requested_pose(self):
        path = Path()
        path.header.frame_id = 'map'
        path.poses = [
            self._pose(0.025, -0.025, 0.3),
            self._pose(1.0, 1.0, 0.8),
        ]
        start = self._pose(0.0, 0.0, 0.1)
        start.header.frame_id = 'map'

        anchored, adjustment = anchor_path_start(path, start)

        self.assertAlmostEqual(adjustment, math.hypot(0.025, 0.025))
        self.assertEqual(anchored.poses[0].pose.position.x, 0.0)
        self.assertEqual(anchored.poses[0].pose.position.y, 0.0)
        self.assertAlmostEqual(
            anchored.poses[0].pose.orientation.z, math.sin(0.05)
        )
        self.assertEqual(path.poses[0].pose.position.x, 0.025)
        with self.assertRaises(ValueError):
            anchor_path_start(path, self._pose(-2.0, -2.0, 0.0))

    def test_planner_selector_accepts_only_configured_exact_ids(self):
        for planner_id in PLANNER_IDS:
            self.assertEqual(normalize_planner_id(planner_id), planner_id)
        with self.assertRaises(ValueError):
            normalize_planner_id('GridBased')
        with self.assertRaises(ValueError):
            normalize_planner_id('thetastar')
        self.assertEqual(normalize_planner_id('  Smac2D  '), 'Smac2D')

    def test_smoother_visibility_accepts_only_exact_ordered_ids(self):
        payload = (
            '{"methods":["adaptive_hybrid","simple","simple",'
            '"pivot_g2_fixed"]}'
        )
        self.assertEqual(
            normalize_smoother_visibility(payload),
            ('simple', 'pivot_g2_fixed', 'adaptive_hybrid'),
        )
        self.assertEqual(
            normalize_smoother_visibility('{"methods":[]}'), ()
        )
        with self.assertRaises(ValueError):
            normalize_smoother_visibility('{"methods":["pivot-g2"]}')
        with self.assertRaises(ValueError):
            normalize_smoother_visibility('["simple"]')
        with self.assertRaises(ValueError):
            normalize_smoother_visibility('not-json')

    def test_smoother_visibility_filters_and_restores_cached_paths(self):
        previous_domain = os.environ.get('ROS_DOMAIN_ID')
        os.environ['ROS_DOMAIN_ID'] = str(120 + os.getpid() % 100)
        rclpy.init()
        comparison = PathComparisonNode()
        probe = rclpy.create_node('smoother_visibility_contract_test')
        executor = SingleThreadedExecutor()
        executor.add_node(comparison)
        executor.add_node(probe)
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        path_states = {}
        visibility_states = []
        for method in SMOOTHER_IDS:
            probe.create_subscription(
                Path,
                f'/research/path/{method}',
                lambda message, selected=method: path_states.__setitem__(
                    selected, bool(message.poses)
                ),
                latched_qos,
            )
        probe.create_subscription(
            String,
            '/research/smoother_visibility_active',
            lambda message: visibility_states.append(
                normalize_smoother_visibility(message.data)
            ),
            latched_qos,
        )
        selector = probe.create_publisher(
            String, '/research/smoother_visibility', latched_qos
        )
        cached_path = Path()
        cached_path.header.frame_id = 'map'
        cached_path.poses = [self._pose(0.0, 0.0, 0.0)]
        comparison._method_path_cache = {
            'simple': cached_path,
            'pivot_g2': cached_path,
        }

        def spin_until(predicate, timeout=3.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                if predicate():
                    return True
            return False

        try:
            self.assertTrue(
                spin_until(
                    lambda: visibility_states
                    and visibility_states[-1] == SMOOTHER_IDS
                )
            )
            selection = String()
            selection.data = '{"methods":["simple"]}'
            selector.publish(selection)
            self.assertTrue(
                spin_until(
                    lambda: visibility_states
                    and visibility_states[-1] == ('simple',)
                    and len(path_states) == len(SMOOTHER_IDS)
                )
            )
            self.assertTrue(path_states['simple'])
            self.assertTrue(
                all(
                    not visible for method, visible in path_states.items()
                    if method != 'simple'
                )
            )

            selection.data = '{"methods":["pivot_g2"]}'
            selector.publish(selection)
            self.assertTrue(
                spin_until(
                    lambda: visibility_states
                    and visibility_states[-1] == ('pivot_g2',)
                    and path_states.get('pivot_g2') is True
                    and path_states.get('simple') is False
                )
            )
        finally:
            executor.remove_node(probe)
            executor.remove_node(comparison)
            probe.destroy_node()
            comparison.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
            if previous_domain is None:
                os.environ.pop('ROS_DOMAIN_ID', None)
            else:
                os.environ['ROS_DOMAIN_ID'] = previous_domain

    def test_smoother_toggle_clears_paths_and_reports_state(self):
        previous_domain = os.environ.get('ROS_DOMAIN_ID')
        os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
        rclpy.init()
        comparison = PathComparisonNode()
        probe = rclpy.create_node('smoother_toggle_contract_test')
        executor = SingleThreadedExecutor()
        executor.add_node(comparison)
        executor.add_node(probe)
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        states = []
        cleared_methods = set()
        probe.create_subscription(
            Bool,
            '/research/smoothers_active',
            lambda message: states.append(message.data),
            latched_qos,
        )
        for method in comparison.SMOOTHERS:
            probe.create_subscription(
                Path,
                f'/research/path/{method}',
                lambda message, selected=method: (
                    cleared_methods.add(selected)
                    if not message.poses else None
                ),
                latched_qos,
            )
        toggle = probe.create_publisher(
            Bool, '/research/smoothers_enabled', latched_qos
        )

        def spin_until(predicate, timeout=3.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                if predicate():
                    return True
            return False

        try:
            self.assertTrue(spin_until(lambda: states and states[-1]))
            message = Bool()
            message.data = False
            toggle.publish(message)
            self.assertTrue(
                spin_until(
                    lambda: states
                    and states[-1] is False
                    and cleared_methods == set(comparison.SMOOTHERS)
                )
            )
            self.assertIsNone(comparison._raw_path)
            self.assertEqual(comparison._pending_smoothers, [])

            message.data = True
            toggle.publish(message)
            self.assertTrue(spin_until(lambda: states and states[-1]))
            self.assertEqual(cleared_methods, set(comparison.SMOOTHERS))
        finally:
            executor.remove_node(probe)
            executor.remove_node(comparison)
            probe.destroy_node()
            comparison.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
            if previous_domain is None:
                os.environ.pop('ROS_DOMAIN_ID', None)
            else:
                os.environ['ROS_DOMAIN_ID'] = previous_domain

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

    def test_footprint_clearance_detects_obstacle_enclosed_by_robot(self):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.info.resolution = 0.05
        occupancy_grid.info.width = 40
        occupancy_grid.info.height = 40
        occupancy_grid.data = [0] * 1600
        occupancy_grid.data[20 * 40 + 20] = 100
        path = Path()
        path.poses = [self._pose(1.025, 1.025, 0.0)]

        metrics = calculate_footprint_clearance(path, occupancy_grid)

        self.assertEqual(metrics['footprint_clearance_min_m'], 0.0)
        self.assertEqual(metrics['footprint_collision_sample_count'], 1)

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

    def test_execution_matrix_accepts_multiple_planners_and_scenario_file(self):
        options = _arguments([
            '--scenario-file', '/tmp/open_arena_scenarios.yaml',
            '--scenario', 'west_east_center',
            '--planners', 'NavFnAStar', 'ThetaStar', 'Smac2D',
            '--methods', 'raw', 'adaptive_hybrid',
        ])

        self.assertEqual(
            options.planners, ['NavFnAStar', 'ThetaStar', 'Smac2D']
        )
        self.assertEqual(options.methods, ['raw', 'adaptive_hybrid'])
        self.assertEqual(
            options.scenario_file, '/tmp/open_arena_scenarios.yaml'
        )

    def test_execution_matrix_resumes_only_matching_successful_trial(self):
        with tempfile.TemporaryDirectory() as directory:
            result_path = FilePath(directory) / 'trial.json'
            result_path.write_text(json.dumps({
                'scenario': 'west_east_center',
                'planner': 'ThetaStar',
                'method': 'pivot_g2',
                'fixed_speed_limit_mps': 0.22,
                'success': True,
                'repetition': 2,
                'configuration_sha256': 'config-a',
            }), encoding='utf-8')

            record = _matching_successful_record(
                result_path,
                'west_east_center',
                'ThetaStar',
                'pivot_g2',
                0.22,
                2,
                'config-a',
            )
            self.assertIsNotNone(record)
            self.assertTrue(record['resumed'])
            self.assertIsNone(_matching_successful_record(
                result_path,
                'west_east_center',
                'ThetaStar',
                'raw',
                0.22,
                2,
                'config-a',
            ))
            self.assertIsNone(_matching_successful_record(
                result_path,
                'west_east_center',
                'ThetaStar',
                'pivot_g2',
                0.22,
                2,
                'config-b',
            ))
            result_path.write_text(json.dumps({
                'scenario': 'west_east_center',
                'planner': 'ThetaStar',
                'method': 'pivot_g2',
                'fixed_speed_limit_mps': 0.22,
                'success': True,
            }), encoding='utf-8')
            self.assertIsNone(_matching_successful_record(
                result_path,
                'west_east_center',
                'ThetaStar',
                'pivot_g2',
                0.22,
                2,
            ))

    def test_execution_matrix_retries_setup_but_not_controller_failures(self):
        self.assertTrue(_is_infrastructure_failure({
            'success': False,
            'error': 'RuntimeError: Nav2 did not reach the fully active state',
        }))
        self.assertTrue(_is_infrastructure_failure({
            'success': False,
            'error': 'trial did not produce a result file',
        }))
        self.assertFalse(_is_infrastructure_failure({
            'success': False,
            'error': 'RuntimeError: FollowPath action timed out',
            'controller_status': 2,
        }))
        self.assertFalse(_is_infrastructure_failure({'success': True}))

    def test_execution_matrix_summary_does_not_duplicate_high_rate_traces(self):
        compact = _compact_summary_record({
            'method': 'pivot_g2',
            'success': True,
            'tracking_rmse_m': 0.01,
            'ground_truth_state_trace': [[0.0, 1.0, 2.0]],
            'adaptive_speed_trace': [[0.0, 'tracking']],
        })

        self.assertEqual(compact['tracking_rmse_m'], 0.01)
        self.assertNotIn('ground_truth_state_trace', compact)
        self.assertNotIn('adaptive_speed_trace', compact)
        self.assertEqual(
            compact['omitted_trace_fields'],
            ['adaptive_speed_trace', 'ground_truth_state_trace'],
        )

    def test_execution_matrix_retains_trial_console_log(self):
        with tempfile.TemporaryDirectory() as directory:
            log_path = FilePath(directory) / 'trial.log'
            process, return_code = _run_launch(
                [sys.executable, '-c', 'print("diagnostic evidence")'],
                os.environ.copy(),
                5.0,
                log_path,
            )

            self.assertEqual(return_code, 0)
            self.assertEqual(process.returncode, 0)
            self.assertIn(
                'diagnostic evidence',
                log_path.read_text(encoding='utf-8'),
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
