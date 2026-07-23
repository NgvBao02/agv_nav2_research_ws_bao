# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Plan once, compare every configured smoother, and optionally follow one path."""

from copy import deepcopy
import json
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from action_msgs.msg import GoalStatus
from adaptive_pivot_g2_benchmark.path_contract import (
    canonicalize_planner_path,
)
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath, SmoothPath
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


Point = Tuple[float, float]
PLANNER_IDS = (
    'NavFnAStar',
    'NavFnDijkstra',
    'ThetaStar',
    'Smac2D',
    'SmacHybrid',
)


def normalize_planner_id(planner_id: str) -> str:
    """Return one exact configured planner ID or raise a clear error."""
    selected = planner_id.strip()
    if selected not in PLANNER_IDS:
        raise ValueError(
            f'planner_id={selected!r} must be one of {PLANNER_IDS}'
        )
    return selected


def calculate_path_metrics(
    points: Iterable[Point], curvature_stride: int = 1
) -> Dict[str, float]:
    """Return geometry-only metrics that are directly comparable across methods."""
    if curvature_stride < 1:
        raise ValueError('curvature_stride must be at least one')
    values = list(points)
    length = sum(
        math.hypot(current[0] - previous[0], current[1] - previous[1])
        for previous, current in zip(values, values[1:])
    )
    maximum_curvature = 0.0
    curvature_energy = 0.0
    curvature_samples = (
        (
            values[index - curvature_stride],
            values[index],
            values[index + curvature_stride],
        )
        for index in range(
            curvature_stride,
            len(values) - curvature_stride,
            curvature_stride,
        )
    )
    for first, middle, last in curvature_samples:
        side_a = math.hypot(middle[0] - first[0], middle[1] - first[1])
        side_b = math.hypot(last[0] - middle[0], last[1] - middle[1])
        chord = math.hypot(last[0] - first[0], last[1] - first[1])
        denominator = side_a * side_b * chord
        if denominator <= 1.0e-12:
            continue
        cross = (
            (middle[0] - first[0]) * (last[1] - first[1])
            - (middle[1] - first[1]) * (last[0] - first[0])
        )
        curvature = 2.0 * cross / denominator
        maximum_curvature = max(maximum_curvature, abs(curvature))
        curvature_energy += curvature * curvature * 0.5 * (side_a + side_b)
    return {
        'point_count': len(values),
        'path_length_m': length,
        'max_abs_curvature_1pm': maximum_curvature,
        'curvature_energy_1pm': curvature_energy,
    }


def resample_polyline(
    points: Iterable[Point], spacing: float = 0.05
) -> List[Point]:
    """Resample a polyline at nearly uniform arc-length intervals."""
    if spacing <= 0.0:
        raise ValueError('spacing must be positive')
    cleaned: List[Point] = []
    for point in points:
        if not cleaned or math.hypot(
            point[0] - cleaned[-1][0], point[1] - cleaned[-1][1]
        ) > 1.0e-9:
            cleaned.append(point)
    if len(cleaned) < 2:
        return cleaned

    sampled = [cleaned[0]]
    traversed = 0.0
    next_distance = spacing
    for first, last in zip(cleaned, cleaned[1:]):
        segment_length = math.hypot(last[0] - first[0], last[1] - first[1])
        segment_end = traversed + segment_length
        while next_distance <= segment_end + 1.0e-12:
            ratio = (next_distance - traversed) / segment_length
            sampled.append(
                (
                    first[0] + ratio * (last[0] - first[0]),
                    first[1] + ratio * (last[1] - first[1]),
                )
            )
            next_distance += spacing
        traversed = segment_end
    if math.hypot(
        cleaned[-1][0] - sampled[-1][0], cleaned[-1][1] - sampled[-1][1]
    ) > 1.0e-9:
        sampled.append(cleaned[-1])
    return sampled


def _planar_yaw(pose: PoseStamped) -> float:
    quaternion = pose.pose.orientation
    sine = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosine = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sine, cosine)


def calculate_maneuver_metrics(
    path: Path,
    spacing: float = 0.05,
    duplicate_position_tolerance: float = 1.0e-4,
    minimum_pivot_angle: float = math.radians(5.0),
) -> Dict[str, float]:
    """Measure translational geometry separately from explicit pivot markers."""
    if spacing <= 0.0:
        raise ValueError('spacing must be positive')
    if duplicate_position_tolerance < 0.0 or minimum_pivot_angle < 0.0:
        raise ValueError('maneuver marker tolerances must be non-negative')
    if not path.poses:
        return {
            'translation_segment_count': 0,
            'pivot_marker_count': 0,
            'pivot_total_angle_rad': 0.0,
            'pivot_max_angle_rad': 0.0,
            'translation_path_length_m': 0.0,
            'translation_max_abs_curvature_1pm': 0.0,
            'translation_curvature_energy_1pm': 0.0,
        }

    first_point = (
        path.poses[0].pose.position.x,
        path.poses[0].pose.position.y,
    )
    segments: List[List[Point]] = [[first_point]]
    pivot_angles: List[float] = []
    for previous, current in zip(path.poses, path.poses[1:]):
        previous_point = (
            previous.pose.position.x,
            previous.pose.position.y,
        )
        current_point = (
            current.pose.position.x,
            current.pose.position.y,
        )
        distance = math.hypot(
            current_point[0] - previous_point[0],
            current_point[1] - previous_point[1],
        )
        heading_delta = math.atan2(
            math.sin(_planar_yaw(current) - _planar_yaw(previous)),
            math.cos(_planar_yaw(current) - _planar_yaw(previous)),
        )
        if (
            distance <= duplicate_position_tolerance
            and abs(heading_delta) >= minimum_pivot_angle
        ):
            pivot_angles.append(abs(heading_delta))
            segments.append([current_point])
        elif distance > duplicate_position_tolerance:
            segments[-1].append(current_point)

    translation_segments = [segment for segment in segments if len(segment) >= 2]
    length = 0.0
    maximum_curvature = 0.0
    curvature_energy = 0.0
    for segment in translation_segments:
        metrics = calculate_path_metrics(resample_polyline(segment, spacing))
        length += metrics['path_length_m']
        maximum_curvature = max(
            maximum_curvature, metrics['max_abs_curvature_1pm']
        )
        curvature_energy += metrics['curvature_energy_1pm']
    return {
        'translation_segment_count': len(translation_segments),
        'pivot_marker_count': len(pivot_angles),
        'pivot_total_angle_rad': sum(pivot_angles),
        'pivot_max_angle_rad': max(pivot_angles, default=0.0),
        'translation_path_length_m': length,
        'translation_max_abs_curvature_1pm': maximum_curvature,
        'translation_curvature_energy_1pm': curvature_energy,
    }


def condition_trajectory_for_metrics(
    points: Iterable[Point], spacing: float = 0.05, smoothing_radius: int = 2
) -> List[Point]:
    """
    Suppress localization jitter before estimating executed curvature.

    Planned paths are evaluated exactly as returned by each smoother. Executed
    trajectories are different: they arrive at nonuniform time intervals and
    contain millimetre-scale AMCL jitter. Uniform resampling followed by a
    short symmetric moving average prevents that jitter from being reported as
    physically impossible curvature.
    """
    sampled = resample_polyline(points, spacing)
    if smoothing_radius <= 0 or len(sampled) < 3:
        return sampled
    smoothed: List[Point] = []
    last_index = len(sampled) - 1
    for index, point in enumerate(sampled):
        if index in (0, last_index):
            smoothed.append(point)
            continue
        first = max(0, index - smoothing_radius)
        last = min(len(sampled), index + smoothing_radius + 1)
        window = sampled[first:last]
        smoothed.append(
            (
                sum(value[0] for value in window) / len(window),
                sum(value[1] for value in window) / len(window),
            )
        )
    return smoothed


def _point_to_segment_distance(
    point: Point, first: Point, last: Point
) -> float:
    delta_x = last[0] - first[0]
    delta_y = last[1] - first[1]
    denominator = delta_x * delta_x + delta_y * delta_y
    if denominator <= 1.0e-18:
        return math.hypot(point[0] - first[0], point[1] - first[1])
    projection = (
        (point[0] - first[0]) * delta_x + (point[1] - first[1]) * delta_y
    ) / denominator
    projection = max(0.0, min(1.0, projection))
    nearest = (
        first[0] + projection * delta_x,
        first[1] + projection * delta_y,
    )
    return math.hypot(point[0] - nearest[0], point[1] - nearest[1])


def calculate_tracking_metrics(
    executed_points: Sequence[Point], reference_points: Sequence[Point]
) -> Dict[str, float]:
    """Return controller tracking and final-position errors."""
    if not executed_points or not reference_points:
        return {}
    if len(reference_points) == 1:
        distances = [
            math.hypot(
                point[0] - reference_points[0][0],
                point[1] - reference_points[0][1],
            )
            for point in executed_points
        ]
    else:
        segments = list(zip(reference_points, reference_points[1:]))
        distances = [
            min(
                _point_to_segment_distance(point, first, last)
                for first, last in segments
            )
            for point in executed_points
        ]
    final_error = math.hypot(
        executed_points[-1][0] - reference_points[-1][0],
        executed_points[-1][1] - reference_points[-1][1],
    )
    return {
        'tracking_rmse_m': math.sqrt(
            sum(distance * distance for distance in distances) / len(distances)
        ),
        'tracking_max_error_m': max(distances),
        'final_position_error_m': final_error,
    }


def duration_seconds(duration) -> float:
    """Convert a builtin_interfaces/Duration message to seconds."""
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


class PathComparisonNode(Node):
    """Use the same planner result as input to all Nav2 smoother plugins."""

    SMOOTHERS = {
        'simple': 'simple_smoother',
        'savitzky_golay': 'savitzky_golay',
        'constrained': 'constrained',
        'pivot_g2': 'pivot_g2',
        'adaptive_hybrid': 'adaptive_hybrid',
    }

    def __init__(self) -> None:
        super().__init__('adaptive_pivot_g2_path_comparison')
        self.declare_parameter('goal_topic', '/research/goal_pose')
        self.declare_parameter('planner_selector_topic', '/planner_selector')
        self.declare_parameter(
            'smoother_selector_topic', '/research/smoothers_enabled'
        )
        self.declare_parameter('planner_id', 'ThetaStar')
        self.declare_parameter('replan_on_planner_change', True)
        self.declare_parameter('smoothers_enabled', True)
        self.declare_parameter('execute_method', 'simple')
        self.declare_parameter('execute', True)
        self.declare_parameter('check_for_collisions', True)
        self.declare_parameter('max_smoothing_duration', 3.0)

        self._planner_id = normalize_planner_id(
            str(self.get_parameter('planner_id').value)
        )
        self._replan_on_planner_change = bool(
            self.get_parameter('replan_on_planner_change').value
        )
        self._smoothers_enabled = bool(
            self.get_parameter('smoothers_enabled').value
        )
        self._execute_method = str(self.get_parameter('execute_method').value)
        self._execute = bool(self.get_parameter('execute').value)
        self._check_collisions = bool(self.get_parameter('check_for_collisions').value)
        self._max_smoothing_duration = float(
            self.get_parameter('max_smoothing_duration').value
        )
        valid_methods = {'none', 'raw', *self.SMOOTHERS.keys()}
        if self._execute_method not in valid_methods:
            raise ValueError(
                f'execute_method={self._execute_method!r} must be one of {valid_methods}'
            )

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_publishers = {
            method: self.create_publisher(Path, f'/research/path/{method}', latched_qos)
            for method in ('raw', *self.SMOOTHERS.keys())
        }
        self._executed_path_publisher = self.create_publisher(
            Path, '/research/path/executed', latched_qos
        )
        self._metrics_publisher = self.create_publisher(
            String, '/research/metrics', latched_qos
        )
        self._planner_status_publisher = self.create_publisher(
            String, '/research/planner_active', latched_qos
        )
        self._smoother_status_publisher = self.create_publisher(
            Bool, '/research/smoothers_active', latched_qos
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._goal_callback,
            10,
        )
        self.create_subscription(
            String, '/research/execute_method', self._method_callback, latched_qos
        )
        self.create_subscription(
            String,
            str(self.get_parameter('planner_selector_topic').value),
            self._planner_callback,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('smoother_selector_topic').value),
            self._smoother_toggle_callback,
            latched_qos,
        )

        self._planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._smoother_client = ActionClient(self, SmoothPath, 'smooth_path')
        self._controller_client = ActionClient(self, FollowPath, 'follow_path')

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._executed_path = Path()
        self._executed_path.header.frame_id = 'map'
        self._last_trajectory_point: Optional[Point] = None
        self._trajectory_timer = self.create_timer(0.10, self._record_trajectory)

        self._generation = 0
        self._smoother_generation = 0
        self._planning_planner_id = self._planner_id
        self._last_goal_pose: Optional[PoseStamped] = None
        self._active_planner_goal = None
        self._active_smoother_goal = None
        self._active_controller_goal = None
        self._execution_started_at: Optional[Time] = None
        self._execution_reference_points: List[Point] = []
        self._published_methods = set()
        self._pending_smoothers = []
        self._raw_path: Optional[Path] = None
        self._publish_active_planner()
        self._publish_smoothers_active()
        self.get_logger().info(
            'Ready: publish a PoseStamped on /research/goal_pose. '
            f'Planner is {self._planner_id!r}; execution method is '
            f'{self._execute_method!r}; smoothers enabled is '
            f'{self._smoothers_enabled}.'
        )

    def _servers_ready(self) -> bool:
        clients = (self._planner_client, self._smoother_client, self._controller_client)
        return all(client.server_is_ready() for client in clients)

    def _method_callback(self, message: String) -> None:
        method = message.data.strip()
        if method not in {'none', 'raw', *self.SMOOTHERS.keys()}:
            self.get_logger().error(f'Ignoring unknown execution method: {method!r}')
            return
        self._execute_method = method
        self.get_logger().info(f'Next goal will execute method: {method}')

    def _publish_active_planner(self) -> None:
        message = String()
        message.data = self._planner_id
        self._planner_status_publisher.publish(message)

    def _publish_smoothers_active(self) -> None:
        message = Bool()
        message.data = self._smoothers_enabled
        self._smoother_status_publisher.publish(message)

    def _clear_smoother_paths(self) -> None:
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        empty_path.header.stamp = self.get_clock().now().to_msg()
        for method in self.SMOOTHERS:
            self._path_publishers[method].publish(empty_path)

    def _smoother_toggle_callback(self, message: Bool) -> None:
        enabled = bool(message.data)
        changed = enabled != self._smoothers_enabled
        self._smoothers_enabled = enabled
        self._publish_smoothers_active()

        if not enabled:
            self._smoother_generation += 1
            self._pending_smoothers = []
            if self._active_smoother_goal is not None:
                self._active_smoother_goal.cancel_goal_async()
                self._active_smoother_goal = None
            self._clear_smoother_paths()
        elif self._raw_path is not None:
            self._start_smoothing(self._raw_path, self._generation)
        elif self._last_goal_pose is not None:
            self._start_planning(
                deepcopy(self._last_goal_pose), source='smoother_toggle'
            )

        event = String()
        event.data = json.dumps(
            {
                'event': 'smoothers_toggled',
                'enabled': enabled,
                'changed': changed,
                'has_raw_path': self._raw_path is not None,
                'generation': self._generation,
            },
            sort_keys=True,
        )
        self._metrics_publisher.publish(event)
        self.get_logger().info(event.data)

    def _planner_callback(self, message: String) -> None:
        try:
            selected = normalize_planner_id(message.data)
        except ValueError as error:
            self.get_logger().error(f'Ignoring planner selection: {error}')
            self._publish_active_planner()
            return

        changed = selected != self._planner_id
        self._planner_id = selected
        self._publish_active_planner()
        event = String()
        event.data = json.dumps(
            {
                'event': 'planner_selected',
                'planner': selected,
                'changed': changed,
                'has_last_goal': self._last_goal_pose is not None,
            },
            sort_keys=True,
        )
        self._metrics_publisher.publish(event)
        self.get_logger().info(event.data)
        if (
            self._replan_on_planner_change
            and self._last_goal_pose is not None
        ):
            self._start_planning(
                deepcopy(self._last_goal_pose), source='planner_selector'
            )

    def _goal_callback(self, goal_pose: PoseStamped) -> None:
        self._last_goal_pose = deepcopy(goal_pose)
        self._start_planning(goal_pose, source='rviz_goal')

    def _start_planning(
        self, goal_pose: PoseStamped, source: str
    ) -> None:
        if not self._servers_ready():
            self.get_logger().error(
                'Nav2 action servers are not active yet; retry the goal shortly.'
            )
            return
        self._generation += 1
        generation = self._generation
        self._smoother_generation += 1
        self._planning_planner_id = self._planner_id
        self._published_methods.clear()
        self._pending_smoothers = []
        self._raw_path = None
        if self._active_planner_goal is not None:
            self._active_planner_goal.cancel_goal_async()
            self._active_planner_goal = None
        if self._active_smoother_goal is not None:
            self._active_smoother_goal.cancel_goal_async()
            self._active_smoother_goal = None
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        empty_path.header.stamp = self.get_clock().now().to_msg()
        for publisher in self._path_publishers.values():
            publisher.publish(empty_path)

        goal = ComputePathToPose.Goal()
        goal.goal = goal_pose
        goal.planner_id = self._planning_planner_id
        goal.use_start = False
        self.get_logger().info(
            f'Planning generation {generation} with '
            f'{self._planning_planner_id} from {source} to '
            f'({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})'
        )
        future = self._planner_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed, token=generation: self._planner_goal_response(completed, token)
        )

    def _planner_goal_response(self, future, generation: int) -> None:
        if generation != self._generation:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Planner rejected the goal.')
            return
        self._active_planner_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, token=generation: self._planner_result(completed, token)
        )

    def _planner_result(self, future, generation: int) -> None:
        if generation != self._generation:
            return
        self._active_planner_goal = None
        response = future.result()
        result = response.result
        if response.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != 0:
            self.get_logger().error(
                f'Planning failed: status={response.status}, code={result.error_code}, '
                f'message={result.error_msg!r}'
            )
            return

        try:
            raw_path, removed_duplicates = canonicalize_planner_path(
                result.path
            )
        except ValueError as error:
            self.get_logger().error(f'Planner returned an invalid path: {error}')
            return
        if removed_duplicates:
            self.get_logger().warning(
                'Removed '
                f'{removed_duplicates} redundant consecutive planner pose(s) '
                'before comparing smoothers.'
            )
        self._publish_path_and_metrics(
            'raw', raw_path, duration_seconds(result.planning_time), 'planning'
        )
        if self._execute and self._execute_method == 'raw':
            self._follow_path(raw_path, 'raw', generation)

        # Nav2's smoother server owns a single action server.  Sending all
        # methods concurrently makes acceptance order timing-dependent and can
        # sporadically abort one baseline.  Serialize the requests so every
        # algorithm receives exactly the same raw path deterministically.
        self._raw_path = raw_path
        if self._smoothers_enabled:
            self._start_smoothing(raw_path, generation)

    def _start_smoothing(self, raw_path: Path, generation: int) -> None:
        if generation != self._generation or not self._smoothers_enabled:
            return
        self._smoother_generation += 1
        smoother_generation = self._smoother_generation
        if self._active_smoother_goal is not None:
            self._active_smoother_goal.cancel_goal_async()
            self._active_smoother_goal = None
        self._clear_smoother_paths()
        self._raw_path = raw_path
        self._pending_smoothers = list(self.SMOOTHERS.items())
        self._send_next_smoother(generation, smoother_generation)

    def _send_next_smoother(
        self, generation: int, smoother_generation: int
    ) -> None:
        if (
            generation != self._generation
            or smoother_generation != self._smoother_generation
            or not self._smoothers_enabled
            or not self._pending_smoothers
        ):
            return
        method, smoother_id = self._pending_smoothers.pop(0)
        goal = SmoothPath.Goal()
        goal.path = self._raw_path
        goal.smoother_id = smoother_id
        seconds = max(0.0, self._max_smoothing_duration)
        goal.max_smoothing_duration.sec = int(seconds)
        goal.max_smoothing_duration.nanosec = int((seconds % 1.0) * 1.0e9)
        goal.check_for_collisions = self._check_collisions
        send_future = self._smoother_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda completed, selected=method, plan_token=generation,
            smoother_token=smoother_generation:
            self._smoother_goal_response(
                completed, selected, plan_token, smoother_token
            )
        )

    def _smoother_goal_response(
        self,
        future,
        method: str,
        generation: int,
        smoother_generation: int,
    ) -> None:
        goal_handle = future.result()
        if (
            generation != self._generation
            or smoother_generation != self._smoother_generation
            or not self._smoothers_enabled
        ):
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().error(f'{method}: smoother rejected the goal.')
            self._send_next_smoother(generation, smoother_generation)
            return
        self._active_smoother_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, selected=method, plan_token=generation,
            smoother_token=smoother_generation:
            self._smoother_result(
                completed, selected, plan_token, smoother_token
            )
        )

    def _smoother_result(
        self,
        future,
        method: str,
        generation: int,
        smoother_generation: int,
    ) -> None:
        if (
            generation != self._generation
            or smoother_generation != self._smoother_generation
            or not self._smoothers_enabled
        ):
            return
        self._active_smoother_goal = None
        response = future.result()
        result = response.result
        if response.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != 0:
            self.get_logger().error(
                f'{method}: smoothing failed: status={response.status}, '
                f'code={result.error_code}, message={result.error_msg!r}'
            )
            self._send_next_smoother(generation, smoother_generation)
            return
        elapsed = duration_seconds(result.smoothing_duration)
        self._publish_path_and_metrics(method, result.path, elapsed, 'smoothing')
        if self._execute and self._execute_method == method:
            self._follow_path(result.path, method, generation)
        self._send_next_smoother(generation, smoother_generation)

    def _publish_path_and_metrics(
        self, method: str, path: Path, elapsed_seconds: float, stage: str
    ) -> None:
        self._path_publishers[method].publish(path)
        self._published_methods.add(method)
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
        payload = {
            'event': 'path_ready',
            'planner': self._planning_planner_id,
            'generation': self._generation,
            'method': method,
            f'{stage}_time_s': elapsed_seconds,
            **calculate_path_metrics(points),
            **calculate_maneuver_metrics(path),
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._metrics_publisher.publish(message)
        self.get_logger().info(message.data)

    def _follow_path(self, path: Path, method: str, generation: int) -> None:
        if generation != self._generation or self._execute_method == 'none':
            return
        if self._active_controller_goal is not None:
            self._active_controller_goal.cancel_goal_async()
        self._executed_path = Path()
        self._executed_path.header.frame_id = 'map'
        self._last_trajectory_point = None
        self._execution_started_at = self.get_clock().now()
        self._execution_reference_points = [
            (pose.pose.position.x, pose.pose.position.y) for pose in path.poses
        ]

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'
        goal.progress_checker_id = 'progress_checker'
        future = self._controller_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed, selected=method, token=generation:
            self._controller_goal_response(completed, selected, token)
        )

    def _controller_goal_response(self, future, method: str, generation: int) -> None:
        if generation != self._generation:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{method}: controller rejected the path.')
            return
        self._active_controller_goal = goal_handle
        self.get_logger().info(f'Executing {method} with the shared FollowPath controller.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, selected=method, token=generation:
            self._controller_result(completed, selected, token)
        )

    def _controller_result(self, future, method: str, generation: int) -> None:
        if generation != self._generation:
            return
        response = future.result()
        result = response.result
        succeeded = response.status == GoalStatus.STATUS_SUCCEEDED and result.error_code == 0
        recorded_points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in self._executed_path.poses
        ]
        # The trajectory timer may be up to 100 ms behind the action result.
        # Include the newest transform so final-position error is evaluated at
        # the same instant at which the controller declares success.
        try:
            final_transform = self._tf_buffer.lookup_transform(
                'map', 'base_link', Time()
            )
            recorded_points.append(
                (
                    final_transform.transform.translation.x,
                    final_transform.transform.translation.y,
                )
            )
        except TransformException:
            pass
        conditioned_points = condition_trajectory_for_metrics(recorded_points)
        execution_time = 0.0
        if self._execution_started_at is not None:
            execution_time = (
                self.get_clock().now() - self._execution_started_at
            ).nanoseconds * 1.0e-9
        payload = {
            'event': 'execution_finished',
            'planner': self._planning_planner_id,
            'generation': self._generation,
            'method': method,
            'success': succeeded,
            'status': response.status,
            'error_code': result.error_code,
            'error_msg': result.error_msg,
            'execution_time_s': execution_time,
            'recorded_point_count': len(recorded_points),
            'trajectory_metric_spacing_m': 0.05,
            'trajectory_curvature_stride': 3,
            **calculate_path_metrics(conditioned_points, curvature_stride=3),
            **calculate_tracking_metrics(recorded_points, self._execution_reference_points),
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._metrics_publisher.publish(message)
        log = self.get_logger().info if succeeded else self.get_logger().error
        log(message.data)
        self._active_controller_goal = None
        self._execution_started_at = None

    def _record_trajectory(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return
        point = (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )
        if self._last_trajectory_point is not None:
            moved = math.hypot(
                point[0] - self._last_trajectory_point[0],
                point[1] - self._last_trajectory_point[1],
            )
            if moved < 0.005:
                return
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.orientation = transform.transform.rotation
        self._executed_path.header.stamp = pose.header.stamp
        self._executed_path.poses.append(pose)
        self._executed_path_publisher.publish(self._executed_path)
        self._last_trajectory_point = point


def main(args: Optional[List[str]] = None) -> None:
    """Run the path comparison node."""
    rclpy.init(args=args)
    node = PathComparisonNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
