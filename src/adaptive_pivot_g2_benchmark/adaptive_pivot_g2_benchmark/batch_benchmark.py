# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Run reproducible, same-input Nav2 smoother experiments and write CSV/JSON."""

import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import statistics
import time
from typing import Dict, Iterable, List, Optional, Sequence

from action_msgs.msg import GoalStatus
from adaptive_pivot_g2_benchmark.clearance_metrics import (
    calculate_footprint_clearance,
)
from adaptive_pivot_g2_benchmark.compare_paths import (
    _point_to_segment_distance,
    calculate_maneuver_metrics,
    calculate_path_metrics,
    duration_seconds,
    Point,
    resample_polyline,
)
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, SmoothPath
from nav_msgs.msg import OccupancyGrid, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml


SMOOTHERS = {
    'simple': 'simple_smoother',
    'savitzky_golay': 'savitzky_golay',
    'constrained': 'constrained',
    'pivot_g2': 'pivot_g2',
    'adaptive_hybrid': 'adaptive_hybrid',
}


def _path_points(path: NavPath) -> List[Point]:
    return [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]


def _path_hash(points: Iterable[Point]) -> str:
    canonical = [[round(point[0], 9), round(point[1], 9)] for point in points]
    encoded = json.dumps(canonical, separators=(',', ':')).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def calculate_path_deviation(
    output_points: Sequence[Point], reference_points: Sequence[Point]
) -> Dict[str, float]:
    """Measure directed output-to-reference geometric deviation."""
    if not output_points or not reference_points:
        return {}
    if len(reference_points) == 1:
        distances = [
            math.hypot(
                point[0] - reference_points[0][0],
                point[1] - reference_points[0][1],
            )
            for point in output_points
        ]
    else:
        segments = list(zip(reference_points, reference_points[1:]))
        distances = [
            min(
                _point_to_segment_distance(point, first, last)
                for first, last in segments
            )
            for point in output_points
        ]
    return {
        'deviation_rmse_m': math.sqrt(
            sum(distance * distance for distance in distances) / len(distances)
        ),
        'deviation_max_m': max(distances),
    }


def _pose(frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(0.5 * yaw)
    pose.pose.orientation.w = math.cos(0.5 * yaw)
    return pose


class BatchBenchmark(Node):
    """Call planner/smoother actions serially for deterministic experiments."""

    def __init__(self) -> None:
        super().__init__('adaptive_pivot_g2_batch_benchmark')
        default_scenarios = str(
            Path(get_package_share_directory('adaptive_pivot_g2_benchmark'))
            / 'config'
            / 'research_scenarios.yaml'
        )
        self.declare_parameter('scenario_file', default_scenarios)
        self.declare_parameter('output_csv', '/tmp/pivot_g2_benchmark.csv')
        self.declare_parameter('output_json', '/tmp/pivot_g2_benchmark_summary.json')
        self.declare_parameter('planners', ['ThetaStar', 'GridBased'])
        self.declare_parameter('repetitions', 1)
        self.declare_parameter('resample_spacing', 0.05)
        self.declare_parameter('max_smoothing_duration', 3.0)
        self.declare_parameter('check_for_collisions', True)
        self.declare_parameter('server_timeout', 30.0)

        self.scenario_file = Path(str(self.get_parameter('scenario_file').value))
        self.output_csv = Path(str(self.get_parameter('output_csv').value))
        self.output_json = Path(str(self.get_parameter('output_json').value))
        self.planners = list(self.get_parameter('planners').value)
        self.repetitions = int(self.get_parameter('repetitions').value)
        self.resample_spacing = float(self.get_parameter('resample_spacing').value)
        self.max_smoothing_duration = float(
            self.get_parameter('max_smoothing_duration').value
        )
        self.check_for_collisions = bool(
            self.get_parameter('check_for_collisions').value
        )
        self.server_timeout = float(self.get_parameter('server_timeout').value)

        if self.repetitions < 1:
            raise ValueError('repetitions must be at least one')
        if self.resample_spacing <= 0.0:
            raise ValueError('resample_spacing must be positive')
        if not self.planners:
            raise ValueError('at least one planner ID is required')

        self.planner_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose'
        )
        self.smoother_client = ActionClient(self, SmoothPath, 'smooth_path')
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.occupancy_grid: Optional[OccupancyGrid] = None
        self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos
        )
        self.latest_pivot_diagnostics: Optional[Dict] = None
        self.latest_hybrid_diagnostics: Optional[Dict] = None
        self.create_subscription(
            String,
            '/research/pivot_g2/diagnostics',
            self._diagnostics_callback,
            10,
        )
        self.create_subscription(
            String,
            '/research/adaptive_hybrid/diagnostics',
            self._hybrid_diagnostics_callback,
            10,
        )

    def _diagnostics_callback(self, message: String) -> None:
        try:
            self.latest_pivot_diagnostics = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Ignoring malformed Pivot-G2 diagnostics JSON')

    def _hybrid_diagnostics_callback(self, message: String) -> None:
        try:
            self.latest_hybrid_diagnostics = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Ignoring malformed hybrid diagnostics JSON')

    def _map_callback(self, message: OccupancyGrid) -> None:
        self.occupancy_grid = message

    def _wait_for_servers(self) -> None:
        for name, client in (
            ('compute_path_to_pose', self.planner_client),
            ('smooth_path', self.smoother_client),
        ):
            if not client.wait_for_server(timeout_sec=self.server_timeout):
                raise RuntimeError(f'action server {name!r} did not become ready')
        deadline = time.monotonic() + self.server_timeout
        while self.occupancy_grid is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.occupancy_grid is None:
            raise RuntimeError('static occupancy grid did not become ready')

    def _load_scenarios(self) -> List[Dict]:
        with self.scenario_file.open(encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        scenarios = document.get('scenarios', []) if isinstance(document, dict) else []
        if not scenarios:
            raise ValueError(f'no scenarios found in {self.scenario_file}')
        required = {'name', 'start', 'goal'}
        for scenario in scenarios:
            if not required.issubset(scenario):
                raise ValueError(f'scenario is missing one of {sorted(required)}: {scenario}')
            if len(scenario['start']) < 2 or len(scenario['goal']) < 2:
                raise ValueError(f'scenario start/goal must contain x and y: {scenario}')
        return scenarios

    def _spin_future(self, future):
        rclpy.spin_until_future_complete(self, future)
        if future.cancelled() or future.exception() is not None:
            raise RuntimeError(f'ROS action future failed: {future.exception()}')
        return future.result()

    def _plan(self, planner_id: str, scenario: Dict):
        start = scenario['start']
        goal = scenario['goal']
        default_yaw = math.atan2(goal[1] - start[1], goal[0] - start[0])
        request = ComputePathToPose.Goal()
        request.start = _pose(
            'map', float(start[0]), float(start[1]),
            float(start[2]) if len(start) > 2 else default_yaw,
        )
        request.goal = _pose(
            'map', float(goal[0]), float(goal[1]),
            float(goal[2]) if len(goal) > 2 else default_yaw,
        )
        request.use_start = True
        request.planner_id = planner_id
        wall_started = time.perf_counter()
        goal_handle = self._spin_future(self.planner_client.send_goal_async(request))
        if not goal_handle.accepted:
            return None, 'planner rejected the goal', time.perf_counter() - wall_started
        response = self._spin_future(goal_handle.get_result_async())
        wall_elapsed = time.perf_counter() - wall_started
        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or response.result.error_code != 0
        ):
            detail = (
                f'status={response.status}, code={response.result.error_code}, '
                f'message={response.result.error_msg!r}'
            )
            return None, detail, wall_elapsed
        return response.result, '', wall_elapsed

    def _smooth(self, raw_path: NavPath, method: str):
        request = SmoothPath.Goal()
        request.path = raw_path
        request.smoother_id = SMOOTHERS[method]
        seconds = max(0.0, self.max_smoothing_duration)
        request.max_smoothing_duration.sec = int(seconds)
        request.max_smoothing_duration.nanosec = int((seconds % 1.0) * 1.0e9)
        request.check_for_collisions = self.check_for_collisions
        self.latest_pivot_diagnostics = None
        self.latest_hybrid_diagnostics = None
        wall_started = time.perf_counter()
        goal_handle = self._spin_future(self.smoother_client.send_goal_async(request))
        if not goal_handle.accepted:
            return None, 'smoother rejected the goal', time.perf_counter() - wall_started
        response = self._spin_future(goal_handle.get_result_async())
        wall_elapsed = time.perf_counter() - wall_started
        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or response.result.error_code != 0
        ):
            detail = (
                f'status={response.status}, code={response.result.error_code}, '
                f'message={response.result.error_msg!r}'
            )
            return None, detail, wall_elapsed
        # Diagnostics are published immediately before the action result, but
        # DDS delivery can arrive one executor cycle later for very short
        # straight paths. Give the selected research plugin a bounded wall-time
        # window so its decision is attached to the correct CSV row.
        research_method = method in {'pivot_g2', 'adaptive_hybrid'}
        deadline = time.monotonic() + 0.20
        while research_method and time.monotonic() < deadline:
            diagnostics_ready = (
                method == 'pivot_g2'
                and self.latest_pivot_diagnostics is not None
            ) or (
                method == 'adaptive_hybrid'
                and self.latest_hybrid_diagnostics is not None
            )
            if diagnostics_ready:
                break
            rclpy.spin_once(self, timeout_sec=0.01)
        return response.result, '', wall_elapsed

    def _success_row(
        self,
        scenario: Dict,
        planner_id: str,
        repetition: int,
        method: str,
        raw_points: Sequence[Point],
        output_path: NavPath,
        algorithm_time: float,
        wall_time: float,
    ) -> Dict:
        output_points = _path_points(output_path)
        common_points = resample_polyline(output_points, self.resample_spacing)
        row = {
            'scenario': scenario['name'],
            'planner': planner_id,
            'repetition': repetition,
            'method': method,
            'success': True,
            'error': '',
            'raw_path_sha256': _path_hash(raw_points),
            'output_path_sha256': _path_hash(output_points),
            'native_point_count': len(output_points),
            'metric_spacing_m': self.resample_spacing,
            'algorithm_time_s': algorithm_time,
            'wall_time_s': wall_time,
            **calculate_path_metrics(common_points),
            **calculate_maneuver_metrics(output_path, self.resample_spacing),
            **calculate_footprint_clearance(output_path, self.occupancy_grid),
            **calculate_path_deviation(common_points, raw_points),
        }
        if method == 'pivot_g2' and self.latest_pivot_diagnostics:
            for key, value in self.latest_pivot_diagnostics.items():
                if key not in {'method'}:
                    row[f'pivot_{key}'] = value
        if method == 'adaptive_hybrid' and self.latest_hybrid_diagnostics:
            for key, value in self.latest_hybrid_diagnostics.items():
                if key != 'method':
                    row[f'hybrid_{key}'] = value
        return row

    def run(self) -> List[Dict]:
        """Execute all configured experiments and persist machine-readable results."""
        self._wait_for_servers()
        scenarios = self._load_scenarios()
        rows: List[Dict] = []
        total = len(scenarios) * len(self.planners) * self.repetitions
        completed = 0
        for planner_id in self.planners:
            for scenario in scenarios:
                for repetition in range(1, self.repetitions + 1):
                    completed += 1
                    self.get_logger().info(
                        f'[{completed}/{total}] {planner_id} / {scenario["name"]} '
                        f'/ repetition {repetition}'
                    )
                    plan_result, error, plan_wall_time = self._plan(
                        planner_id, scenario
                    )
                    if plan_result is None:
                        for method in ('raw', *SMOOTHERS):
                            rows.append(
                                {
                                    'scenario': scenario['name'],
                                    'planner': planner_id,
                                    'repetition': repetition,
                                    'method': method,
                                    'success': False,
                                    'error': f'planning failed: {error}',
                                }
                            )
                        continue

                    raw_path = plan_result.path
                    raw_points = _path_points(raw_path)
                    rows.append(
                        self._success_row(
                            scenario,
                            planner_id,
                            repetition,
                            'raw',
                            raw_points,
                            raw_path,
                            duration_seconds(plan_result.planning_time),
                            plan_wall_time,
                        )
                    )
                    for method in SMOOTHERS:
                        result, error, wall_time = self._smooth(raw_path, method)
                        if result is None:
                            rows.append(
                                {
                                    'scenario': scenario['name'],
                                    'planner': planner_id,
                                    'repetition': repetition,
                                    'method': method,
                                    'success': False,
                                    'error': f'smoothing failed: {error}',
                                    'raw_path_sha256': _path_hash(raw_points),
                                }
                            )
                            continue
                        rows.append(
                            self._success_row(
                                scenario,
                                planner_id,
                                repetition,
                                method,
                                raw_points,
                                result.path,
                                duration_seconds(result.smoothing_duration),
                                wall_time,
                            )
                        )
        self._write_results(rows)
        return rows

    def _write_results(self, rows: Sequence[Dict]) -> None:
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = sorted({key for row in rows for key in row})
        with self.output_csv.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        aggregate = {}
        for planner in self.planners:
            for method in ('raw', *SMOOTHERS):
                selected = [
                    row
                    for row in rows
                    if row.get('planner') == planner
                    and row.get('method') == method
                    and row.get('success')
                ]
                key = f'{planner}/{method}'
                aggregate[key] = {
                    'success_count': len(selected),
                    'failure_count': sum(
                        1
                        for row in rows
                        if row.get('planner') == planner
                        and row.get('method') == method
                        and not row.get('success')
                    ),
                }
                for metric in (
                    'path_length_m',
                    'max_abs_curvature_1pm',
                    'curvature_energy_1pm',
                    'translation_max_abs_curvature_1pm',
                    'translation_curvature_energy_1pm',
                    'pivot_marker_count',
                    'pivot_total_angle_rad',
                    'footprint_clearance_min_m',
                    'footprint_clearance_p05_m',
                    'footprint_clearance_mean_m',
                    'footprint_collision_sample_count',
                    'deviation_rmse_m',
                    'deviation_max_m',
                    'algorithm_time_s',
                    'wall_time_s',
                ):
                    values = [float(row[metric]) for row in selected if metric in row]
                    if values:
                        aggregate[key][f'mean_{metric}'] = statistics.fmean(values)
                        aggregate[key][f'median_{metric}'] = statistics.median(values)

        summary = {
            'generated_at_utc': datetime.now(timezone.utc).isoformat(),
            'scenario_file': str(self.scenario_file),
            'planners': self.planners,
            'repetitions': self.repetitions,
            'resample_spacing_m': self.resample_spacing,
            'row_count': len(rows),
            'aggregate': aggregate,
        }
        self.output_json.parent.mkdir(parents=True, exist_ok=True)
        with self.output_json.open('w', encoding='utf-8') as stream:
            json.dump(summary, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write('\n')
        self.get_logger().info(f'Wrote {len(rows)} rows to {self.output_csv}')
        self.get_logger().info(f'Wrote aggregate summary to {self.output_json}')


def main(args: Optional[List[str]] = None) -> None:
    """Run the batch benchmark once, then exit with persisted results."""
    rclpy.init(args=args)
    node = BatchBenchmark()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
