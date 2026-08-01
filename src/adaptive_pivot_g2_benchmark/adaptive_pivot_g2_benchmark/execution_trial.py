# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Execute one reproducible Nav2 path trial and persist ground-truth metrics."""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from adaptive_pivot_g2_benchmark.batch_benchmark import (
    _path_hash,
    _path_points,
    _pose,
    SMOOTHERS,
)
from adaptive_pivot_g2_benchmark.clearance_metrics import (
    calculate_footprint_clearance,
)
from adaptive_pivot_g2_benchmark.closed_loop_metrics import (
    calculate_curve_exit_metrics,
)
from adaptive_pivot_g2_benchmark.compare_paths import (
    calculate_maneuver_metrics,
    calculate_path_metrics,
    calculate_tracking_metrics,
    condition_trajectory_for_metrics,
    duration_seconds,
)
from adaptive_pivot_g2_benchmark.path_contract import (
    anchor_path_goal,
    anchor_path_start,
    canonicalize_planner_path,
)
from adaptive_pivot_g2_benchmark.localization_metrics import (
    align_odometry_trace,
    calculate_localization_metrics,
    calculate_pose_trace_error_metrics,
)
from adaptive_pivot_g2_benchmark.initial_heading import (
    resolve_scenario_start_heading,
)
from adaptive_pivot_g2_benchmark.velocity_metrics import (
    calculate_velocity_metrics,
)
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath, SmoothPath
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


Point = Tuple[float, float]


def _angle_error(first: float, last: float) -> float:
    return math.atan2(math.sin(first - last), math.cos(first - last))


def _yaw_from_odometry(message: Odometry) -> float:
    return _yaw_from_quaternion(message.pose.pose.orientation)


def _yaw_from_quaternion(quaternion) -> float:
    sine = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosine = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sine, cosine)


class ExecutionTrial(Node):
    """Plan, smooth, execute, and measure one method from a fresh simulation."""

    def __init__(self) -> None:
        super().__init__('adaptive_pivot_g2_execution_trial')
        default_scenarios = str(
            Path(get_package_share_directory('adaptive_pivot_g2_benchmark'))
            / 'config'
            / 'research_scenarios.yaml'
        )
        self.declare_parameter('scenario_file', default_scenarios)
        self.declare_parameter('scenario', 'lower_left_diagonal')
        self.declare_parameter('planner', 'ThetaStar')
        self.declare_parameter('method', 'pivot_g2')
        self.declare_parameter('output_json', '/tmp/pivot_g2_execution_trial.json')
        self.declare_parameter('server_timeout_s', 60.0)
        self.declare_parameter('trial_timeout_s', 180.0)
        self.declare_parameter('post_action_settle_timeout_s', 3.0)
        self.declare_parameter('localization_settle_s', 2.0)
        self.declare_parameter('initial_localization_tolerance_m', 0.08)
        self.declare_parameter('initial_localization_yaw_tolerance_rad', 0.10)
        self.declare_parameter('check_for_collisions', True)
        self.declare_parameter('ground_truth_position_tolerance_m', 0.10)
        self.declare_parameter('ground_truth_yaw_tolerance_rad', 0.15)

        self.scenario_file = Path(str(self.get_parameter('scenario_file').value))
        self.scenario_name = str(self.get_parameter('scenario').value)
        self.planner_id = str(self.get_parameter('planner').value)
        self.method = str(self.get_parameter('method').value)
        self.output_json = Path(str(self.get_parameter('output_json').value))
        self.server_timeout = float(
            self.get_parameter('server_timeout_s').value
        )
        self.trial_timeout = float(self.get_parameter('trial_timeout_s').value)
        self.post_action_settle_timeout = float(
            self.get_parameter('post_action_settle_timeout_s').value
        )
        self.localization_settle = float(
            self.get_parameter('localization_settle_s').value
        )
        self.initial_localization_tolerance = float(
            self.get_parameter('initial_localization_tolerance_m').value
        )
        self.initial_localization_yaw_tolerance = float(
            self.get_parameter(
                'initial_localization_yaw_tolerance_rad'
            ).value
        )
        self.check_for_collisions = bool(
            self.get_parameter('check_for_collisions').value
        )
        self.ground_truth_position_tolerance = float(
            self.get_parameter('ground_truth_position_tolerance_m').value
        )
        self.ground_truth_yaw_tolerance = float(
            self.get_parameter('ground_truth_yaw_tolerance_rad').value
        )
        if (
            self.ground_truth_position_tolerance <= 0.0
            or self.ground_truth_yaw_tolerance <= 0.0
            or self.initial_localization_tolerance <= 0.0
            or self.initial_localization_yaw_tolerance <= 0.0
            or self.post_action_settle_timeout <= 0.0
        ):
            raise ValueError('goal and localization tolerances must be positive')
        if self.method not in {'raw', *SMOOTHERS}:
            raise ValueError(f'unknown execution method: {self.method!r}')

        self.planner_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose'
        )
        self.smoother_client = ActionClient(self, SmoothPath, 'smooth_path')
        self.controller_client = ActionClient(self, FollowPath, 'follow_path')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigation_active_client = self.create_client(
            Trigger, '/lifecycle_manager_navigation/is_active'
        )
        self.clear_costmap_clients = [
            self.create_client(
                ClearEntireCostmap,
                '/global_costmap/clear_entirely_global_costmap',
            ),
            self.create_client(
                ClearEntireCostmap,
                '/local_costmap/clear_entirely_local_costmap',
            ),
        ]
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self._ground_truth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self._odometry_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_callback,
            10,
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos
        )
        self.occupancy_grid: Optional[OccupancyGrid] = None
        self.create_subscription(Twist, '/cmd_vel', self._command_callback, 20)
        self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self._controller_command_callback,
            20,
        )
        self.create_subscription(
            CollisionMonitorState,
            '/collision_monitor_state',
            self._collision_callback,
            10,
        )
        self.latest_ground_truth: Optional[Odometry] = None
        self.latest_odometry: Optional[Odometry] = None
        self.latest_amcl_pose: Optional[PoseWithCovarianceStamped] = None
        self.executing = False
        self.trajectory: List[Point] = []
        self.trajectory_stamps: List[float] = []
        self.command_samples: List[Tuple[float, float, float]] = []
        self.controller_command_samples: List[
            Tuple[float, float, float]
        ] = []
        self.ground_truth_velocity_samples: List[
            Tuple[float, float, float]
        ] = []
        self.ground_truth_state_samples: List[
            Tuple[float, float, float, float, float, float]
        ] = []
        self.odometry_state_samples: List[
            Tuple[float, float, float, float, float, float]
        ] = []
        self.estimated_map_state_samples: List[
            Tuple[float, float, float, float]
        ] = []
        self.localization_samples: List[
            Tuple[float, float, float, float, float, float]
        ] = []
        self.action_completion_stamp: Optional[float] = None
        self.action_completion_ground_truth = None
        self.action_completion_command: Optional[Tuple[float, float, float]] = (
            None
        )
        self.collision_interventions = 0
        self.last_collision_action = CollisionMonitorState.DO_NOTHING
        self.environment = 'unknown'

    def _ground_truth_callback(self, message: Odometry) -> None:
        self.latest_ground_truth = message
        if not self.executing:
            return
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        self.ground_truth_velocity_samples.append((
            stamp,
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
        ))
        self.ground_truth_state_samples.append((
            stamp,
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            _yaw_from_quaternion(message.pose.pose.orientation),
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
        ))
        self._record_localization_sample(stamp)
        point = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
        )
        if not self.trajectory or math.hypot(
            point[0] - self.trajectory[-1][0],
            point[1] - self.trajectory[-1][1],
        ) >= 1.0e-4:
            self.trajectory.append(point)
            self.trajectory_stamps.append(stamp)

    def _amcl_callback(self, message: PoseWithCovarianceStamped) -> None:
        self.latest_amcl_pose = message

    def _odometry_callback(self, message: Odometry) -> None:
        self.latest_odometry = message
        if not self.executing:
            return
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        self.odometry_state_samples.append((
            stamp,
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            _yaw_from_quaternion(message.pose.pose.orientation),
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
        ))

    def _record_localization_sample(self, stamp: float) -> None:
        if self.latest_amcl_pose is None:
            return
        try:
            estimated_position, estimated_orientation = (
                self._estimated_map_pose()
            )
        except (RuntimeError, TransformException):
            return
        ground_truth_pose = self.latest_ground_truth.pose.pose
        estimated_yaw = _yaw_from_quaternion(estimated_orientation)
        position_error = math.hypot(
            estimated_position.x - ground_truth_pose.position.x,
            estimated_position.y - ground_truth_pose.position.y,
        )
        yaw_error = abs(_angle_error(
            estimated_yaw,
            _yaw_from_quaternion(ground_truth_pose.orientation),
        ))
        covariance = self.latest_amcl_pose.pose.covariance
        self.localization_samples.append((
            stamp,
            position_error,
            yaw_error,
            max(0.0, float(covariance[0])),
            max(0.0, float(covariance[7])),
            max(0.0, float(covariance[35])),
        ))
        self.estimated_map_state_samples.append((
            stamp,
            float(estimated_position.x),
            float(estimated_position.y),
            float(estimated_yaw),
        ))

    def _estimated_map_pose(self):
        transform = self.tf_buffer.lookup_transform(
            'map', 'base_link', Time()
        ).transform
        return transform.translation, transform.rotation

    def _current_localization_error(self) -> Tuple[float, float]:
        if self.latest_ground_truth is None:
            raise RuntimeError('localization comparison data is unavailable')
        ground_truth_pose = self.latest_ground_truth.pose.pose
        estimated_position, estimated_orientation = self._estimated_map_pose()
        return (
            math.hypot(
                estimated_position.x - ground_truth_pose.position.x,
                estimated_position.y - ground_truth_pose.position.y,
            ),
            abs(_angle_error(
                _yaw_from_quaternion(estimated_orientation),
                _yaw_from_quaternion(ground_truth_pose.orientation),
            )),
        )

    def _command_callback(self, message: Twist) -> None:
        if self.executing:
            stamp = self.get_clock().now().nanoseconds * 1.0e-9
            self.command_samples.append(
                (stamp, message.linear.x, message.angular.z)
            )

    def _controller_command_callback(self, message: Twist) -> None:
        if self.executing:
            stamp = self.get_clock().now().nanoseconds * 1.0e-9
            self.controller_command_samples.append(
                (stamp, message.linear.x, message.angular.z)
            )

    def _map_callback(self, message: OccupancyGrid) -> None:
        self.occupancy_grid = message

    def _collision_callback(self, message: CollisionMonitorState) -> None:
        if not self.executing:
            return
        if (
            message.action_type != CollisionMonitorState.DO_NOTHING
            and message.action_type != self.last_collision_action
        ):
            self.collision_interventions += 1
        self.last_collision_action = message.action_type

    def _load_scenario(self) -> Dict:
        with self.scenario_file.open(encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        self.environment = str(
            document.get('environment', 'research_warehouse')
        )
        for scenario in document.get('scenarios', []):
            if scenario.get('name') == self.scenario_name:
                return scenario
        raise ValueError(
            f'scenario {self.scenario_name!r} not found in {self.scenario_file}'
        )

    def _wait_for_system(self) -> None:
        if not self.navigation_active_client.wait_for_service(
            timeout_sec=self.server_timeout
        ):
            raise RuntimeError('navigation lifecycle status service is unavailable')
        deadline = time.monotonic() + self.server_timeout
        while time.monotonic() < deadline:
            future = self.navigation_active_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=min(2.0, self.server_timeout)
            )
            if (
                future.done()
                and not future.cancelled()
                and future.exception() is None
                and future.result().success
            ):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            raise RuntimeError('Nav2 did not reach the fully active state')
        for name, client in (
            ('compute_path_to_pose', self.planner_client),
            ('smooth_path', self.smoother_client),
            ('follow_path', self.controller_client),
        ):
            if not client.wait_for_server(timeout_sec=self.server_timeout):
                raise RuntimeError(f'action server {name!r} did not become ready')
        deadline = time.monotonic() + self.server_timeout
        while self.latest_ground_truth is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_ground_truth is None:
            raise RuntimeError('ground-truth odometry did not become ready')
        deadline = time.monotonic() + self.server_timeout
        while self.occupancy_grid is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.occupancy_grid is None:
            raise RuntimeError('static occupancy grid did not become ready')

    def _clear_costmaps(self) -> None:
        for client in self.clear_costmap_clients:
            if not client.wait_for_service(timeout_sec=self.server_timeout):
                raise RuntimeError(
                    f'costmap clear service {client.srv_name!r} did not become ready'
                )
            self._wait_result(
                client.call_async(ClearEntireCostmap.Request()),
                self.server_timeout,
            )

    def _publish_initial_pose(self, start: List[float], yaw: float) -> None:
        self.latest_amcl_pose = None
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = float(start[0])
        message.pose.pose.position.y = float(start[1])
        message.pose.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * yaw)
        message.pose.covariance[0] = 0.0025
        message.pose.covariance[7] = 0.0025
        message.pose.covariance[35] = 0.01
        for _ in range(3):
            self.initial_pose_publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.1)
        deadline = time.monotonic() + self.localization_settle
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_amcl_pose is None:
            raise RuntimeError('AMCL did not publish a pose after initialization')

    def _validate_initial_localization(self) -> Tuple[float, float]:
        position_error, yaw_error = self._current_localization_error()
        if (
            position_error > self.initial_localization_tolerance
            or yaw_error > self.initial_localization_yaw_tolerance
        ):
            raise RuntimeError(
                'AMCL did not converge to Gazebo ground truth: '
                f'position_error={position_error:.3f} m, '
                f'yaw_error={yaw_error:.3f} rad'
            )
        return position_error, yaw_error

    def _validate_physical_spawn(self, start: List[float], yaw: float) -> None:
        if self.latest_ground_truth is None:
            raise RuntimeError('ground truth disappeared before the trial')
        position = self.latest_ground_truth.pose.pose.position
        position_error = math.hypot(
            position.x - float(start[0]), position.y - float(start[1])
        )
        yaw_error = abs(_angle_error(
            _yaw_from_odometry(self.latest_ground_truth), yaw
        ))
        if (
            position_error > self.ground_truth_position_tolerance
            or yaw_error > self.ground_truth_yaw_tolerance
        ):
            raise RuntimeError(
                'Gazebo spawn does not match the scenario start: '
                f'position_error={position_error:.3f} m, '
                f'yaw_error={yaw_error:.3f} rad'
            )
        self._clear_costmaps()
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_result(self, future, timeout: Optional[float] = None):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('ROS action future timed out')
        if future.cancelled() or future.exception() is not None:
            raise RuntimeError(f'ROS action future failed: {future.exception()}')
        return future.result()

    def _plan(self, scenario: Dict, start_yaw: float, goal_yaw: float):
        request = ComputePathToPose.Goal()
        request.start = _pose(
            'map', float(scenario['start'][0]), float(scenario['start'][1]),
            start_yaw,
        )
        request.goal = _pose(
            'map', float(scenario['goal'][0]), float(scenario['goal'][1]),
            goal_yaw,
        )
        request.use_start = True
        request.planner_id = self.planner_id
        goal_handle = self._wait_result(
            self.planner_client.send_goal_async(request), self.server_timeout
        )
        if not goal_handle.accepted:
            raise RuntimeError('planner rejected the trial')
        response = self._wait_result(
            goal_handle.get_result_async(), self.server_timeout
        )
        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or response.result.error_code != 0
        ):
            raise RuntimeError(
                f'planning failed: code={response.result.error_code}, '
                f'message={response.result.error_msg!r}'
            )
        return response.result

    def _smooth(self, raw_path):
        if self.method == 'raw':
            return raw_path, 0.0
        request = SmoothPath.Goal()
        request.path = raw_path
        request.smoother_id = SMOOTHERS[self.method]
        request.max_smoothing_duration.sec = 3
        request.check_for_collisions = self.check_for_collisions
        goal_handle = self._wait_result(
            self.smoother_client.send_goal_async(request), self.server_timeout
        )
        if not goal_handle.accepted:
            raise RuntimeError(f'{self.method} smoother rejected the trial')
        response = self._wait_result(
            goal_handle.get_result_async(), self.server_timeout
        )
        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or response.result.error_code != 0
        ):
            raise RuntimeError(
                f'{self.method} smoothing failed: '
                f'code={response.result.error_code}, '
                f'message={response.result.error_msg!r}'
            )
        return response.result.path, duration_seconds(
            response.result.smoothing_duration
        )

    def _execute(self, path):
        self.trajectory.clear()
        self.trajectory_stamps.clear()
        self.command_samples.clear()
        self.controller_command_samples.clear()
        self.ground_truth_velocity_samples.clear()
        self.ground_truth_state_samples.clear()
        self.odometry_state_samples.clear()
        self.estimated_map_state_samples.clear()
        self.localization_samples.clear()
        self.action_completion_stamp = None
        self.action_completion_ground_truth = None
        self.action_completion_command = None
        self.collision_interventions = 0
        self.last_collision_action = CollisionMonitorState.DO_NOTHING
        self.executing = True
        started = self.get_clock().now()
        request = FollowPath.Goal()
        request.path = path
        request.controller_id = 'FollowPath'
        request.goal_checker_id = 'general_goal_checker'
        request.progress_checker_id = 'progress_checker'
        goal_handle = self._wait_result(
            self.controller_client.send_goal_async(request), self.server_timeout
        )
        if not goal_handle.accepted:
            self.executing = False
            raise RuntimeError('controller rejected the trial')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.trial_timeout
        )
        if not result_future.done():
            goal_handle.cancel_goal_async()
            self.executing = False
            raise TimeoutError(
                f'controller exceeded {self.trial_timeout:.1f} seconds'
            )
        response = result_future.result()
        self.action_completion_stamp = (
            self.get_clock().now().nanoseconds * 1.0e-9
        )
        self.action_completion_ground_truth = self.latest_ground_truth
        if self.command_samples:
            self.action_completion_command = self.command_samples[-1]
        action_elapsed = (
            self.get_clock().now() - started
        ).nanoseconds * 1.0e-9
        physically_settled = False
        stable_samples = 0
        settle_deadline = (
            time.monotonic() + self.post_action_settle_timeout
        )
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_ground_truth is None:
                stable_samples = 0
                continue
            twist = self.latest_ground_truth.twist.twist
            if (
                abs(twist.linear.x) <= 0.01
                and abs(twist.angular.z) <= 0.02
            ):
                stable_samples += 1
                if stable_samples >= 3:
                    physically_settled = True
                    break
            else:
                stable_samples = 0
        elapsed = (self.get_clock().now() - started).nanoseconds * 1.0e-9
        self.executing = False
        success = (
            response.status == GoalStatus.STATUS_SUCCEEDED
            and response.result.error_code == 0
            and physically_settled
        )
        return (
            response,
            success,
            elapsed,
            action_elapsed,
            elapsed - action_elapsed,
            physically_settled,
        )

    def run(self) -> Dict:
        """Run one trial and return its publication-oriented metric record."""
        scenario = self._load_scenario()
        start = scenario['start']
        goal = scenario['goal']
        default_yaw = math.atan2(goal[1] - start[1], goal[0] - start[0])
        start_heading = resolve_scenario_start_heading(
            scenario,
            self.environment,
            Path(get_package_share_directory('vacuum_robot_gazebo'))
            / 'maps',
        )
        start_yaw = start_heading.yaw
        goal_yaw = float(goal[2]) if len(goal) > 2 else default_yaw
        self._wait_for_system()
        self._publish_initial_pose(start, start_yaw)
        self._validate_physical_spawn(start, start_yaw)
        initial_localization_position_error, initial_localization_yaw_error = (
            self._validate_initial_localization()
        )
        plan_result = self._plan(scenario, start_yaw, goal_yaw)
        planner_output_path = plan_result.path
        requested_start = _pose(
            'map', float(start[0]), float(start[1]), start_yaw
        )
        requested_goal = _pose(
            'map', float(goal[0]), float(goal[1]), goal_yaw
        )
        anchored_planner_path, planner_start_adjustment = anchor_path_start(
            planner_output_path, requested_start
        )
        anchored_planner_path, planner_goal_adjustment = anchor_path_goal(
            anchored_planner_path, requested_goal
        )
        raw_path, removed_duplicates = canonicalize_planner_path(
            anchored_planner_path
        )
        selected_path, smoothing_time = self._smooth(raw_path)
        selected_path, selected_start_adjustment = anchor_path_start(
            selected_path, requested_start
        )
        selected_path, selected_goal_adjustment = anchor_path_goal(
            selected_path, requested_goal
        )
        (
            response,
            controller_success,
            execution_time,
            controller_action_time,
            physical_settle_time,
            physically_settled,
        ) = self._execute(selected_path)

        raw_points = _path_points(raw_path)
        selected_points = _path_points(selected_path)
        executed_points = condition_trajectory_for_metrics(self.trajectory)
        final_position_error = None
        final_yaw_error = None
        if self.latest_ground_truth is not None:
            position = self.latest_ground_truth.pose.pose.position
            final_position_error = math.hypot(
                position.x - float(goal[0]), position.y - float(goal[1])
            )
            final_yaw_error = abs(_angle_error(
                _yaw_from_odometry(self.latest_ground_truth), goal_yaw
            ))
        ground_truth_goal_reached = (
            final_position_error is not None
            and final_yaw_error is not None
            and final_position_error <= self.ground_truth_position_tolerance
            and final_yaw_error <= self.ground_truth_yaw_tolerance
        )
        success = controller_success and ground_truth_goal_reached
        traveled_distance = sum(
            math.hypot(last[0] - first[0], last[1] - first[1])
            for first, last in zip(self.trajectory, self.trajectory[1:])
        )
        goal_distances = [
            math.hypot(point[0] - float(goal[0]), point[1] - float(goal[1]))
            for point in self.trajectory
        ]
        nearest_goal_distance = min(goal_distances, default=math.inf)
        post_nearest_goal_travel = 0.0
        if goal_distances:
            nearest_goal_index = min(
                range(len(goal_distances)), key=goal_distances.__getitem__
            )
            post_nearest_goal_travel = sum(
                math.hypot(last[0] - first[0], last[1] - first[1])
                for first, last in zip(
                    self.trajectory[nearest_goal_index:],
                    self.trajectory[nearest_goal_index + 1:],
                )
            )
        post_action_travel = 0.0
        if self.action_completion_stamp is not None:
            post_action_points = [
                point
                for stamp, point in zip(
                    self.trajectory_stamps, self.trajectory
                )
                if stamp >= self.action_completion_stamp
            ]
            post_action_travel = sum(
                math.hypot(last[0] - first[0], last[1] - first[1])
                for first, last in zip(
                    post_action_points, post_action_points[1:]
                )
            )
        completion_position_error = None
        completion_yaw_error = None
        completion_actual_linear = None
        completion_actual_angular = None
        post_action_yaw_change = None
        if self.action_completion_ground_truth is not None:
            completion_pose = self.action_completion_ground_truth.pose.pose
            completion_twist = self.action_completion_ground_truth.twist.twist
            completion_position_error = math.hypot(
                completion_pose.position.x - float(goal[0]),
                completion_pose.position.y - float(goal[1]),
            )
            completion_yaw = _yaw_from_quaternion(
                completion_pose.orientation
            )
            completion_yaw_error = abs(_angle_error(
                completion_yaw, goal_yaw
            ))
            completion_actual_linear = completion_twist.linear.x
            completion_actual_angular = completion_twist.angular.z
            if self.latest_ground_truth is not None:
                post_action_yaw_change = abs(_angle_error(
                    _yaw_from_odometry(self.latest_ground_truth),
                    completion_yaw,
                ))
        completion_command_linear = None
        completion_command_angular = None
        if self.action_completion_command is not None:
            completion_command_linear = self.action_completion_command[1]
            completion_command_angular = self.action_completion_command[2]
        post_action_commands = [
            sample for sample in self.command_samples
            if (
                self.action_completion_stamp is not None
                and sample[0] >= self.action_completion_stamp
            )
        ]
        post_action_actual = [
            sample for sample in self.ground_truth_velocity_samples
            if (
                self.action_completion_stamp is not None
                and sample[0] >= self.action_completion_stamp
            )
        ]
        final_command_linear = (
            self.command_samples[-1][1] if self.command_samples else 0.0
        )
        final_command_angular = (
            self.command_samples[-1][2] if self.command_samples else 0.0
        )
        final_actual_linear = (
            self.ground_truth_velocity_samples[-1][1]
            if self.ground_truth_velocity_samples else 0.0
        )
        final_actual_angular = (
            self.ground_truth_velocity_samples[-1][2]
            if self.ground_truth_velocity_samples else 0.0
        )
        stopped_samples = sum(
            1
            for _, linear, angular in self.command_samples
            if abs(linear) < 0.01 and abs(angular) < 0.02
        )
        velocity_metrics = calculate_velocity_metrics(self.command_samples)
        actual_velocity_metrics = {
            f'actual_{key.replace("command_", "")}': value
            for key, value in calculate_velocity_metrics(
                self.ground_truth_velocity_samples
            ).items()
        }
        controller_velocity_metrics = {
            f'controller_{key.replace("command_", "")}': value
            for key, value in calculate_velocity_metrics(
                self.controller_command_samples
            ).items()
        }
        localization_metrics = calculate_localization_metrics(
            self.localization_samples
        )
        aligned_odometry_trace = align_odometry_trace(
            self.odometry_state_samples,
            self.ground_truth_state_samples,
        )
        odometry_metrics = calculate_pose_trace_error_metrics(
            self.ground_truth_state_samples,
            aligned_odometry_trace,
            'odometry',
        )
        estimated_pose_metrics = calculate_pose_trace_error_metrics(
            self.ground_truth_state_samples,
            self.estimated_map_state_samples,
            'estimated_pose',
        )
        estimated_executed_points = condition_trajectory_for_metrics([
            (sample[1], sample[2])
            for sample in self.estimated_map_state_samples
        ])
        odometry_executed_points = condition_trajectory_for_metrics([
            (sample[1], sample[2])
            for sample in aligned_odometry_trace
        ])
        trace_start_stamp = min(
            (
                samples[0][0]
                for samples in (
                    self.command_samples,
                    self.controller_command_samples,
                    self.ground_truth_state_samples,
                    self.odometry_state_samples,
                    self.estimated_map_state_samples,
                    self.localization_samples,
                )
                if samples
            ),
            default=0.0,
        )
        final_estimated_position_error = None
        final_estimated_yaw_error = None
        try:
            estimated_position, estimated_orientation = (
                self._estimated_map_pose()
            )
            final_estimated_position_error = math.hypot(
                estimated_position.x - float(goal[0]),
                estimated_position.y - float(goal[1]),
            )
            final_estimated_yaw_error = abs(_angle_error(
                _yaw_from_quaternion(estimated_orientation), goal_yaw
            ))
        except TransformException:
            pass
        result = {
            'generated_at_utc': datetime.now(timezone.utc).isoformat(),
            'environment': self.environment,
            'scenario_file': str(self.scenario_file),
            'scenario': self.scenario_name,
            'planner': self.planner_id,
            'method': self.method,
            'success': success,
            'controller_succeeded': controller_success,
            'controller_action_succeeded': (
                response.status == GoalStatus.STATUS_SUCCEEDED
                and response.result.error_code == 0
            ),
            'physically_settled': physically_settled,
            'ground_truth_goal_reached': ground_truth_goal_reached,
            'ground_truth_position_tolerance_m': (
                self.ground_truth_position_tolerance
            ),
            'ground_truth_yaw_tolerance_rad': self.ground_truth_yaw_tolerance,
            'controller_status': response.status,
            'controller_error_code': response.result.error_code,
            'controller_error_msg': response.result.error_msg,
            'start': [float(start[0]), float(start[1]), start_yaw],
            'initial_heading_source': start_heading.source,
            'initial_heading_direct_bearing_rad': start_heading.direct_yaw,
            'initial_heading_free_probe_m': start_heading.free_distance,
            'goal': [float(goal[0]), float(goal[1]), goal_yaw],
            'raw_path_sha256': _path_hash(raw_points),
            'planner_output_path_sha256': _path_hash(
                _path_points(planner_output_path)
            ),
            'removed_duplicate_pose_count': removed_duplicates,
            'planner_start_anchor_adjustment_m': (
                planner_start_adjustment
            ),
            'planner_goal_anchor_adjustment_m': planner_goal_adjustment,
            'selected_start_anchor_adjustment_m': (
                selected_start_adjustment
            ),
            'selected_goal_anchor_adjustment_m': selected_goal_adjustment,
            'selected_path_sha256': _path_hash(selected_points),
            'planning_time_s': duration_seconds(plan_result.planning_time),
            'smoothing_time_s': smoothing_time,
            'execution_time_s': execution_time,
            'controller_action_time_s': controller_action_time,
            'physical_settle_time_s': physical_settle_time,
            'trajectory_sample_count': len(self.trajectory),
            'command_sample_count': len(self.command_samples),
            'nearest_ground_truth_goal_distance_m': nearest_goal_distance,
            'post_nearest_goal_travel_m': post_nearest_goal_travel,
            'post_action_travel_m': post_action_travel,
            'post_action_yaw_change_rad': post_action_yaw_change,
            'action_completion_position_error_m': completion_position_error,
            'action_completion_yaw_error_rad': completion_yaw_error,
            'action_completion_command_linear_mps': (
                completion_command_linear
            ),
            'action_completion_command_angular_radps': (
                completion_command_angular
            ),
            'action_completion_actual_linear_mps': (
                completion_actual_linear
            ),
            'action_completion_actual_angular_radps': (
                completion_actual_angular
            ),
            'post_action_max_command_linear_mps': max(
                (abs(sample[1]) for sample in post_action_commands),
                default=0.0,
            ),
            'post_action_max_command_angular_radps': max(
                (abs(sample[2]) for sample in post_action_commands),
                default=0.0,
            ),
            'post_action_max_actual_linear_mps': max(
                (abs(sample[1]) for sample in post_action_actual),
                default=0.0,
            ),
            'post_action_max_actual_angular_radps': max(
                (abs(sample[2]) for sample in post_action_actual),
                default=0.0,
            ),
            'final_command_linear_mps': final_command_linear,
            'final_command_angular_radps': final_command_angular,
            'final_actual_linear_mps': final_actual_linear,
            'final_actual_angular_radps': final_actual_angular,
            'localization_pose_source': 'tf_map_to_base_link',
            'initial_localization_position_error_m': (
                initial_localization_position_error
            ),
            'initial_localization_yaw_error_rad': (
                initial_localization_yaw_error
            ),
            'final_estimated_position_error_m': (
                final_estimated_position_error
            ),
            'final_estimated_yaw_error_rad': final_estimated_yaw_error,
            'stopped_command_fraction': (
                stopped_samples / len(self.command_samples)
                if self.command_samples else 0.0
            ),
            **velocity_metrics,
            **actual_velocity_metrics,
            **controller_velocity_metrics,
            **localization_metrics,
            **odometry_metrics,
            **estimated_pose_metrics,
            **{
                f'estimated_{key}': value
                for key, value in calculate_tracking_metrics(
                    estimated_executed_points, selected_points
                ).items()
            },
            **{
                f'odometry_{key}': value
                for key, value in calculate_tracking_metrics(
                    odometry_executed_points, selected_points
                ).items()
            },
            'traveled_distance_m': traveled_distance,
            'final_position_error_m': final_position_error,
            'final_yaw_error_rad': final_yaw_error,
            'collision_monitor_interventions': self.collision_interventions,
            **{
                f'planned_{key}': value
                for key, value in calculate_path_metrics(selected_points).items()
            },
            **{
                f'planned_{key}': value
                for key, value in calculate_maneuver_metrics(selected_path).items()
            },
            **{
                f'planned_{key}': value
                for key, value in calculate_footprint_clearance(
                    selected_path, self.occupancy_grid
                ).items()
            },
            **{
                f'executed_{key}': value
                for key, value in calculate_path_metrics(executed_points).items()
            },
            **calculate_tracking_metrics(executed_points, selected_points),
            **calculate_curve_exit_metrics(
                selected_points, self.ground_truth_state_samples
            ),
            'selected_path_xy': [
                [float(point[0]), float(point[1])]
                for point in selected_points
            ],
            'selected_path_poses': [
                [
                    float(pose.pose.position.x),
                    float(pose.pose.position.y),
                    float(_yaw_from_quaternion(pose.pose.orientation)),
                ]
                for pose in selected_path.poses
            ],
            'executed_path_xy': [
                [float(point[0]), float(point[1])]
                for point in executed_points
            ],
            'localization_error_trace': [
                [
                    float(sample[0] - trace_start_stamp),
                    *[float(value) for value in sample[1:]],
                ]
                for sample in self.localization_samples
            ],
            'command_velocity_trace': [
                [
                    float(sample[0] - trace_start_stamp),
                    float(sample[1]),
                    float(sample[2]),
                ]
                for sample in self.command_samples
            ],
            'controller_command_velocity_trace': [
                [
                    float(sample[0] - trace_start_stamp),
                    float(sample[1]),
                    float(sample[2]),
                ]
                for sample in self.controller_command_samples
            ],
            'ground_truth_state_trace': [
                [
                    float(sample[0] - trace_start_stamp),
                    *[float(value) for value in sample[1:]],
                ]
                for sample in self.ground_truth_state_samples
            ],
            'odometry_state_trace': [
                [
                    float(sample[0] - trace_start_stamp),
                    *[float(value) for value in sample[1:]],
                ]
                for sample in self.odometry_state_samples
            ],
            'aligned_odometry_state_trace': [
                [
                    float(sample[0] - trace_start_stamp),
                    *[float(value) for value in sample[1:]],
                ]
                for sample in aligned_odometry_trace
            ],
            'estimated_map_state_trace': [
                [
                    float(sample[0] - trace_start_stamp),
                    *[float(value) for value in sample[1:]],
                ]
                for sample in self.estimated_map_state_samples
            ],
            'state_trace_fields': [
                'time_s',
                'x_m',
                'y_m',
                'yaw_rad',
                'linear_mps',
                'angular_radps',
            ],
            'pose_trace_fields': [
                'time_s',
                'x_m',
                'y_m',
                'yaw_rad',
            ],
        }
        self.output_json.parent.mkdir(parents=True, exist_ok=True)
        with self.output_json.open('w', encoding='utf-8') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write('\n')
        self.get_logger().info(
            f'Execution trial finished: method={self.method}, success={success}, '
            f'time={execution_time:.3f}s, output={self.output_json}'
        )
        return result

    def write_failure(self, error: Exception) -> None:
        """Persist a failed setup/trial so matrix runs cannot fail silently."""
        result = {
            'generated_at_utc': datetime.now(timezone.utc).isoformat(),
            'environment': self.environment,
            'scenario_file': str(self.scenario_file),
            'scenario': self.scenario_name,
            'planner': self.planner_id,
            'method': self.method,
            'success': False,
            'error': f'{type(error).__name__}: {error}',
        }
        self.output_json.parent.mkdir(parents=True, exist_ok=True)
        with self.output_json.open('w', encoding='utf-8') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write('\n')


def main(args: Optional[List[str]] = None) -> None:
    """Run one execution trial, write JSON, and exit."""
    rclpy.init(args=args)
    node = ExecutionTrial()
    try:
        node.run()
    except Exception as error:
        node.write_failure(error)
        node.get_logger().error(f'Execution trial failed: {error}')
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
