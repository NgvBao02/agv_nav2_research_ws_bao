#!/usr/bin/env python3
# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Keep RViz alive while replacing a complete Gazebo and Nav2 session."""

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
from typing import Dict, Optional

from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


SNAP_GUI_VARIABLES = frozenset(
    {
        'GDK_PIXBUF_MODULEDIR',
        'GDK_PIXBUF_MODULE_FILE',
        'GIO_LAUNCHED_DESKTOP_FILE',
        'GIO_LAUNCHED_DESKTOP_FILE_PID',
        'GIO_MODULE_DIR',
        'GTK_EXE_PREFIX',
        'GTK_IM_MODULE_FILE',
        'GTK_MODULES',
        'GTK_PATH',
        'XDG_DATA_DIRS_VSCODE_SNAP_ORIG',
    }
)


def sanitized_session_environment(
    source: Optional[Dict[str, str]] = None,
) -> Dict[str, str]:
    """Remove desktop-Snap library injection while preserving ROS settings."""
    environment = dict(os.environ if source is None else source)
    for name in tuple(environment):
        if name == 'SNAP' or name.startswith('SNAP_'):
            environment.pop(name, None)
    for name in SNAP_GUI_VARIABLES:
        environment.pop(name, None)

    data_directories = environment.get('XDG_DATA_DIRS', '').split(':')
    clean_directories = []
    for directory in data_directories:
        if not directory or '/snap/' in directory:
            continue
        if directory not in clean_directories:
            clean_directories.append(directory)
    if not clean_directories:
        clean_directories = ['/usr/local/share', '/usr/share']
    environment['XDG_DATA_DIRS'] = ':'.join(clean_directories)

    data_home = environment.get('XDG_DATA_HOME', '')
    if not data_home or '/snap/' in data_home:
        environment['XDG_DATA_HOME'] = str(Path.home() / '.local' / 'share')
    return environment


def sanitized_rviz_environment(
    source: Optional[Dict[str, str]] = None,
) -> Dict[str, str]:
    """Return a GUI-safe RViz environment with a clean DDS teardown."""
    environment = sanitized_session_environment(source)
    if environment.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp':
        # On ROS 2 Jazzy, RViz can unload rmw_cyclonedds_cpp while the DDS
        # event thread is still finishing, producing a SIGSEGV on Ctrl-C.
        # Fast DDS interoperates with the CycloneDDS simulation nodes and
        # shuts the RViz process down deterministically.
        environment['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    return environment


def _process_session_from_stat(stat_text: str) -> tuple[int, str]:
    """Return Linux process session ID and state from /proc/PID/stat."""
    command_end = stat_text.rfind(')')
    if command_end < 0:
        raise ValueError('Malformed process stat')
    fields = stat_text[command_end + 2:].split()
    if len(fields) < 4:
        raise ValueError('Incomplete process stat')
    return int(fields[3]), fields[0]


def session_process_ids(session_id: int) -> list[int]:
    """List live processes belonging to an exact Linux session."""
    process_ids = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat_text = (entry / 'stat').read_text(encoding='utf-8')
            candidate_session, state = _process_session_from_stat(
                stat_text
            )
        except (FileNotFoundError, PermissionError, ValueError):
            continue
        if candidate_session == session_id and state not in {'X', 'Z'}:
            process_ids.append(int(entry.name))
    return sorted(process_ids)


def _is_gz_sim_command(command_line: str) -> bool:
    """Return whether a process command invokes the Gazebo Sim CLI."""
    words = command_line.replace('\0', ' ').split()
    return any(
        Path(first).name == 'gz' and second == 'sim'
        for first, second in zip(words, words[1:])
    )


def session_gazebo_process_ids(session_id: int) -> list[int]:
    """List Gazebo wrapper, server, and GUI processes in one session."""
    process_ids = []
    for process_id in session_process_ids(session_id):
        try:
            command_line = (
                Path('/proc') / str(process_id) / 'cmdline'
            ).read_text(encoding='utf-8')
        except (FileNotFoundError, PermissionError, UnicodeDecodeError):
            continue
        if _is_gz_sim_command(command_line):
            process_ids.append(process_id)
    return process_ids


@dataclass(frozen=True)
class EnvironmentSpec:
    """A matched world/map basename and a collision-free robot spawn pose."""

    x: float
    y: float
    yaw: float


ENVIRONMENTS: Dict[str, EnvironmentSpec] = {
    'research_warehouse': EnvironmentSpec(-2.5, -3.0, 0.0),
    'warehouse_long_aisles': EnvironmentSpec(-4.5, -3.25, math.pi / 2.0),
    'warehouse_cross_aisles': EnvironmentSpec(-5.0, 0.0, 0.0),
    'warehouse_dispatch': EnvironmentSpec(-3.45, -1.0, math.pi / 2.0),
    'narrow_aisles': EnvironmentSpec(-5.0, 0.0, 0.0),
    'office_maze': EnvironmentSpec(-5.2, -3.0, 0.0),
    'open_arena': EnvironmentSpec(-5.0, 0.0, 0.0),
}


@dataclass(frozen=True)
class SessionOptions:
    """Arguments forwarded to the ordinary single-environment launch file."""

    gui: bool = True
    nav2: bool = True
    compare: bool = True
    execute: bool = True
    execute_method: str = 'simple'
    planner_id: str = 'ThetaStar'


def _launch_bool(value: bool) -> str:
    return 'true' if value else 'false'


def _launch_float(value: float) -> str:
    """Preserve the decimal point required by typed ROS double parameters."""
    return repr(float(value))


def build_session_command(
    environment: str,
    options: SessionOptions,
    ros2_executable: str = 'ros2',
    initial_sim_time: float = 0.0,
) -> list[str]:
    """Build a shell-free command for exactly one supported environment."""
    if environment not in ENVIRONMENTS:
        raise ValueError(f'Unsupported environment: {environment}')
    spec = ENVIRONMENTS[environment]
    return [
        ros2_executable,
        'launch',
        'vacuum_robot_gazebo',
        'simulation.launch.py',
        f'environment:={environment}',
        'rviz:=false',
        f'gui:={_launch_bool(options.gui)}',
        f'nav2:={_launch_bool(options.nav2)}',
        f'compare:={_launch_bool(options.compare)}',
        f'execute:={_launch_bool(options.execute)}',
        f'execute_method:={options.execute_method}',
        f'planner_id:={options.planner_id}',
        f'initial_sim_time:={_launch_float(initial_sim_time)}',
        f'x_pose:={_launch_float(spec.x)}',
        f'y_pose:={_launch_float(spec.y)}',
        f'yaw:={_launch_float(spec.yaw)}',
    ]


class EnvironmentManager(Node):
    """Own and atomically replace the child simulation process group."""

    def __init__(self) -> None:
        super().__init__('environment_manager')
        self.declare_parameter('environment', 'research_warehouse')
        self.declare_parameter('gui', True)
        self.declare_parameter('nav2', True)
        self.declare_parameter('compare', True)
        self.declare_parameter('execute', True)
        self.declare_parameter('execute_method', 'simple')
        self.declare_parameter('planner_id', 'ThetaStar')
        self.declare_parameter('startup_timeout', 75.0)
        self.declare_parameter('shutdown_timeout', 15.0)

        initial_environment = str(
            self.get_parameter('environment').value
        )
        if initial_environment not in ENVIRONMENTS:
            raise ValueError(
                f'Unsupported initial environment: {initial_environment}'
            )
        self._options = SessionOptions(
            gui=bool(self.get_parameter('gui').value),
            nav2=bool(self.get_parameter('nav2').value),
            compare=bool(self.get_parameter('compare').value),
            execute=bool(self.get_parameter('execute').value),
            execute_method=str(
                self.get_parameter('execute_method').value
            ),
            planner_id=str(self.get_parameter('planner_id').value),
        )
        self._startup_timeout = max(
            5.0, float(self.get_parameter('startup_timeout').value)
        )
        self._shutdown_timeout = max(
            2.0, float(self.get_parameter('shutdown_timeout').value)
        )

        package_share = Path(
            get_package_share_directory('vacuum_robot_gazebo')
        )
        for environment in ENVIRONMENTS:
            for directory, suffix in (
                ('worlds', '.sdf'),
                ('maps', '.yaml'),
            ):
                path = package_share / directory / f'{environment}{suffix}'
                if not path.is_file():
                    raise FileNotFoundError(
                        f'Missing environment asset: {path}'
                    )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._active_publisher = self.create_publisher(
            String, '/research/environment_active', qos
        )
        self._status_publisher = self.create_publisher(
            String, '/research/environment_status', qos
        )
        self._selection_subscription = self.create_subscription(
            String,
            '/research/environment_selector',
            self._selection_callback,
            qos,
        )
        self._latest_ground_truth_monotonic = 0.0
        self._latest_sim_time_seconds = 0.0
        self._ground_truth_subscription = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self._ground_truth_callback,
            qos_profile_sensor_data,
        )
        self._clock_subscription = self.create_subscription(
            Clock,
            '/clock',
            self._clock_callback,
            qos_profile_sensor_data,
        )

        self._lifecycle_clients = []
        if self._options.nav2:
            self._lifecycle_clients = [
                (
                    node_name,
                    self.create_client(
                        GetState, f'/{node_name}/get_state'
                    ),
                )
                for node_name in (
                    'map_server',
                    'planner_server',
                    'controller_server',
                    'smoother_server',
                )
            ]

        self._condition = threading.Condition()
        self._process_lock = threading.Lock()
        self._requested_environment: Optional[str] = initial_environment
        self._active_environment: Optional[str] = None
        self._process: Optional[subprocess.Popen] = None
        self._switch_in_progress = False
        self._closed = False
        self._ros2_executable = shutil.which('ros2') or 'ros2'
        self._worker = threading.Thread(
            target=self._worker_loop,
            name='gazebo-environment-switcher',
            daemon=True,
        )
        self._worker.start()
        self._monitor_timer = self.create_timer(
            1.0, self._monitor_child_process
        )

    def _publish_status(
        self, state: str, environment: str, message: str
    ) -> None:
        if self._closed or not rclpy.ok(context=self.context):
            return
        status = String()
        status.data = json.dumps(
            {
                'state': state,
                'environment': environment,
                'message': message,
            },
            ensure_ascii=False,
            separators=(',', ':'),
        )
        try:
            self._status_publisher.publish(status)
        except Exception:
            if not rclpy.ok(context=self.context):
                return
            raise

    def _publish_active(self, environment: str) -> bool:
        if self._closed or not rclpy.ok(context=self.context):
            return False
        active = String()
        active.data = environment
        try:
            self._active_publisher.publish(active)
        except Exception:
            if not rclpy.ok(context=self.context):
                return False
            raise
        return True

    def _selection_callback(self, message: String) -> None:
        target = message.data.strip()
        if target not in ENVIRONMENTS:
            self.get_logger().error(
                f'Rejected unsupported environment {target!r}'
            )
            self._publish_status(
                'error',
                self._active_environment or '',
                f'Môi trường không hợp lệ: {target}',
            )
            return
        with self._condition:
            self._requested_environment = target
            self._condition.notify()

    def _worker_loop(self) -> None:
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(
                        lambda: self._closed or
                        self._requested_environment is not None
                    )
                    if self._closed:
                        break
                    target = self._requested_environment
                    self._requested_environment = None
                if target is None:
                    continue
                if target == self._active_environment:
                    self._publish_status(
                        'active',
                        target,
                        f'Môi trường {target} đang hoạt động.',
                    )
                    continue
                self._switch_environment(target)
        finally:
            self._stop_session()

    def _switch_environment(self, target: str) -> None:
        self._switch_in_progress = True
        try:
            previous = self._active_environment
            if previous is not None or self._current_process() is not None:
                self._publish_status(
                    'stopping',
                    target,
                    f'Đang dừng môi trường {previous or "hiện tại"}...',
                )
                self._stop_session()
            if self._closed:
                return

            self._active_environment = None
            self._publish_status(
                'starting',
                target,
                f'Đang khởi động Gazebo và Nav2 với map {target}...',
            )
            initial_sim_time = (
                self._latest_sim_time_seconds + 1.0
                if self._latest_sim_time_seconds > 0.0 else 0.0
            )
            command = build_session_command(
                target,
                self._options,
                self._ros2_executable,
                initial_sim_time,
            )
            self.get_logger().info(
                f'Starting environment session: {" ".join(command)}'
            )
            process = subprocess.Popen(
                command,
                start_new_session=True,
                env=sanitized_session_environment(),
            )
            with self._process_lock:
                self._process = process

            session_started = time.monotonic()
            ready, detail = self._wait_until_ready(
                process, session_started
            )
            if not ready:
                if self._closed or not rclpy.ok(context=self.context):
                    self._stop_session()
                    return
                self.get_logger().error(
                    f'Environment {target} failed to become ready: '
                    f'{detail}'
                )
                self._publish_status(
                    'error',
                    target,
                    f'Không thể khởi động {target}: {detail}',
                )
                self._stop_session()
                return

            self._active_environment = target
            if not self._publish_active(target):
                self._stop_session()
                return
            self._publish_status(
                'active',
                target,
                f'Đang chạy {target}: Gazebo world và Nav2 map đã đồng bộ.',
            )
            self.get_logger().info(
                f'Environment {target} is active'
            )
        except (OSError, ValueError) as error:
            self.get_logger().error(
                f'Failed to switch to environment {target}: {error}'
            )
            self._publish_status(
                'error',
                target,
                f'Lỗi khi đổi sang {target}: {error}',
            )
            self._stop_session()
        finally:
            self._switch_in_progress = False

    def _wait_until_ready(
        self,
        process: subprocess.Popen,
        session_started: Optional[float] = None,
    ) -> tuple[bool, str]:
        if session_started is None:
            session_started = time.monotonic()
        deadline = time.monotonic() + self._startup_timeout
        if self._options.nav2:
            for node_name, client in self._lifecycle_clients:
                while time.monotonic() < deadline:
                    if self._closed:
                        return False, 'manager đang tắt'
                    return_code = process.poll()
                    if return_code is not None:
                        return False, f'launch đã thoát (mã {return_code})'
                    remaining = deadline - time.monotonic()
                    if client.wait_for_service(
                        timeout_sec=min(1.0, max(0.0, remaining))
                    ):
                        break
                else:
                    return False, (
                        f'không thấy lifecycle service {node_name}'
                    )

                while time.monotonic() < deadline:
                    future = client.call_async(GetState.Request())
                    response_deadline = min(
                        deadline, time.monotonic() + 2.0
                    )
                    while (
                        not future.done() and
                        time.monotonic() < response_deadline
                    ):
                        if self._closed:
                            return False, 'manager đang tắt'
                        if process.poll() is not None:
                            return False, 'launch đã thoát khi chờ Nav2'
                        time.sleep(0.05)
                    if future.done():
                        try:
                            response = future.result()
                        except Exception:
                            response = None
                        if (
                            response is not None and
                            response.current_state.id ==
                            State.PRIMARY_STATE_ACTIVE
                        ):
                            break
                    time.sleep(0.2)
                else:
                    return False, (
                        f'{node_name} không chuyển sang active'
                    )

        while time.monotonic() < deadline:
            if self._closed:
                return False, 'manager đang tắt'
            return_code = process.poll()
            if return_code is not None:
                return False, f'launch đã thoát (mã {return_code})'
            if self._latest_ground_truth_monotonic >= session_started:
                return True, 'Gazebo ground truth and Nav2 are active'
            time.sleep(0.05)
        return False, 'Gazebo không phát /ground_truth/odom'

    def _ground_truth_callback(self, _: Odometry) -> None:
        self._latest_ground_truth_monotonic = time.monotonic()

    def _clock_callback(self, message: Clock) -> None:
        seconds = (
            float(message.clock.sec) +
            float(message.clock.nanosec) * 1.0e-9
        )
        self._latest_sim_time_seconds = max(
            self._latest_sim_time_seconds, seconds
        )

    def _current_process(self) -> Optional[subprocess.Popen]:
        with self._process_lock:
            return self._process

    def _stop_session(self) -> None:
        with self._process_lock:
            process = self._process
            self._process = None
        self._active_environment = None
        if process is None or process.poll() is not None:
            return

        try:
            process_group = os.getpgid(process.pid)
        except ProcessLookupError:
            return
        try:
            process_session = os.getsid(process.pid)
        except ProcessLookupError:
            return
        try:
            # Signal the ros2 launch parent once. It forwards SIGINT to its
            # children; broadcasting SIGINT to the whole group as well would
            # deliver duplicate interrupts while Python nodes are cleaning up.
            process.send_signal(signal.SIGINT)
            # ros_gz_sim starts Gazebo's server / GUI in their own process
            # groups, so the launch parent cannot reliably forward SIGINT to
            # them.  Interrupt only Gazebo processes here; ROS nodes still
            # receive exactly one signal from the nested launch service.
            self._signal_processes(
                session_gazebo_process_ids(process_session),
                signal.SIGINT,
            )
            process.wait(timeout=self._shutdown_timeout)
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            if rclpy.ok(context=self.context):
                self.get_logger().warning(
                    'Simulation did not stop after SIGINT; sending SIGTERM'
                )
        else:
            if self._terminate_session_descendants(process_session):
                return
            if rclpy.ok(context=self.context):
                self.get_logger().error(
                    'Simulation descendants survived graceful shutdown; '
                    'sending SIGKILL'
                )
            self._signal_session(process_session, signal.SIGKILL)
            return

        try:
            os.killpg(process_group, signal.SIGTERM)
            process.wait(timeout=5.0)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            if rclpy.ok(context=self.context):
                self.get_logger().error(
                    'Simulation did not stop after SIGTERM; sending SIGKILL'
                )
        if self._terminate_session_descendants(process_session):
            return

        try:
            os.killpg(process_group, signal.SIGKILL)
            process.wait(timeout=2.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        self._signal_session(process_session, signal.SIGKILL)

    def _terminate_session_descendants(self, session_id: int) -> bool:
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            if not session_process_ids(session_id):
                return True
            time.sleep(0.05)

        remaining = session_process_ids(session_id)
        if remaining:
            if rclpy.ok(context=self.context):
                self.get_logger().warning(
                    'Interrupting stale simulation descendants: '
                    f'{remaining}'
                )
            # gz-sim's server and GUI create independent process groups in the
            # launch session.  SIGTERM leaves those groups alive on Jazzy,
            # whereas SIGINT runs their normal rendering / transport teardown.
            self._signal_session(session_id, signal.SIGINT)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if not session_process_ids(session_id):
                return True
            time.sleep(0.05)
        return not session_process_ids(session_id)

    @staticmethod
    def _signal_session(session_id: int, signal_number: int) -> None:
        EnvironmentManager._signal_processes(
            session_process_ids(session_id), signal_number
        )

    @staticmethod
    def _signal_processes(
        process_ids: list[int], signal_number: int
    ) -> None:
        for process_id in process_ids:
            try:
                os.kill(process_id, signal_number)
            except ProcessLookupError:
                continue

    def _monitor_child_process(self) -> None:
        if self._switch_in_progress:
            return
        process = self._current_process()
        if process is None:
            return
        return_code = process.poll()
        if return_code is None:
            ground_truth_age = (
                time.monotonic() -
                self._latest_ground_truth_monotonic
            )
            if (
                self._active_environment is None or
                ground_truth_age <= 3.0
            ):
                return
            failed_environment = self._active_environment
            self._publish_status(
                'error',
                failed_environment,
                'Gazebo đã ngừng phát dữ liệu ground-truth.',
            )
            self.get_logger().error(
                'Gazebo ground-truth stream stopped; '
                'terminating the stale session'
            )
            self._stop_session()
            return
        with self._process_lock:
            if self._process is process:
                self._process = None
        failed_environment = self._active_environment or ''
        self._active_environment = None
        self._publish_status(
            'error',
            failed_environment,
            f'Phiên mô phỏng đã thoát ngoài dự kiến (mã {return_code}).',
        )

    def close(self) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._condition.notify_all()
        self._worker.join(timeout=self._shutdown_timeout + 8.0)
        if self._worker.is_alive():
            if rclpy.ok(context=self.context):
                self.get_logger().error(
                    'Environment worker did not stop before timeout'
                )
            self._stop_session()


def main(args=None) -> None:
    rclpy.init(args=args)
    manager = EnvironmentManager()
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.close()
        manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
