# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Unit contracts for safe, synchronized environment switching."""

import importlib.util
import math
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_manager():
    path = PACKAGE_ROOT / 'scripts' / 'environment_manager.py'
    spec = importlib.util.spec_from_file_location(
        'environment_manager', path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _read_pgm(path):
    with path.open('rb') as stream:
        assert stream.readline().strip() == b'P5'
        dimensions = stream.readline()
        while dimensions.startswith(b'#'):
            dimensions = stream.readline()
        width, height = (int(value) for value in dimensions.split())
        assert int(stream.readline()) == 255
        pixels = stream.read()
    return width, height, pixels


def test_catalog_matches_assets_and_uses_free_spawn_cells():
    manager = _load_manager()
    expected = {
        path.stem for path in (PACKAGE_ROOT / 'worlds').glob('*.sdf')
    }
    assert set(manager.ENVIRONMENTS) == expected

    for name, spawn in manager.ENVIRONMENTS.items():
        map_path = PACKAGE_ROOT / 'maps' / f'{name}.yaml'
        with map_path.open(encoding='utf-8') as stream:
            metadata = yaml.safe_load(stream)
        width, height, pixels = _read_pgm(
            PACKAGE_ROOT / 'maps' / metadata['image']
        )
        resolution = float(metadata['resolution'])
        origin_x, origin_y = metadata['origin'][:2]
        column = int((spawn.x - origin_x) / resolution)
        map_row = int((spawn.y - origin_y) / resolution)
        image_row = height - 1 - map_row
        assert 0 <= column < width
        assert 0 <= image_row < height
        assert pixels[image_row * width + column] == 254
        assert math.isfinite(spawn.yaw)


def test_command_restarts_complete_session_with_matched_pose():
    manager = _load_manager()
    options = manager.SessionOptions(
        gui=False,
        nav2=True,
        compare=True,
        execute=False,
        execute_method='adaptive_hybrid',
        planner_id='SmacHybrid',
    )
    command = manager.build_session_command(
        'warehouse_long_aisles', options, '/usr/bin/ros2'
    )
    assert command[:4] == [
        '/usr/bin/ros2',
        'launch',
        'vacuum_robot_gazebo',
        'simulation.launch.py',
    ]
    assert 'environment:=warehouse_long_aisles' in command
    assert 'rviz:=false' in command
    assert 'gui:=false' in command
    assert 'nav2:=true' in command
    assert 'compare:=true' in command
    assert 'execute:=false' in command
    assert 'execute_method:=adaptive_hybrid' in command
    assert 'planner_id:=SmacHybrid' in command
    assert 'initial_sim_time:=0.0' in command
    assert 'x_pose:=-4.5' in command
    assert 'y_pose:=-3.25' in command


def test_command_preserves_ros_double_parameter_types():
    manager = _load_manager()
    command = manager.build_session_command(
        'research_warehouse', manager.SessionOptions()
    )
    assert 'x_pose:=-2.5' in command
    assert 'y_pose:=-3.0' in command
    assert 'yaw:=0.0' in command


def test_command_forwards_monotonic_gazebo_initial_time():
    manager = _load_manager()
    command = manager.build_session_command(
        'warehouse_cross_aisles',
        manager.SessionOptions(),
        initial_sim_time=83.125,
    )
    assert 'initial_sim_time:=83.125' in command


def test_command_rejects_unknown_or_injected_environment():
    manager = _load_manager()
    with pytest.raises(ValueError):
        manager.build_session_command(
            '../research_warehouse', manager.SessionOptions()
        )
    with pytest.raises(ValueError):
        manager.build_session_command(
            'research_warehouse; shutdown', manager.SessionOptions()
        )


def test_sanitized_environment_removes_snap_gui_injection_only():
    manager = _load_manager()
    source = {
        'SNAP': '/snap/code/237',
        'SNAP_ARCH': 'amd64',
        'GTK_PATH': '/snap/code/237/usr/lib/gtk-3.0',
        'GDK_PIXBUF_MODULE_FILE': (
            '/home/user/snap/code/common/loaders.cache'
        ),
        'GIO_MODULE_DIR': '/home/user/snap/code/common/gio-modules',
        'XDG_DATA_HOME': '/home/user/snap/code/237/.local/share',
        'XDG_DATA_DIRS': (
            '/snap/code/237/usr/share:/usr/share:/usr/share'
        ),
        'ROS_DOMAIN_ID': '50',
        'GZ_PARTITION': 'ui_validation_50',
        'LD_LIBRARY_PATH': '/opt/ros/jazzy/lib',
    }

    result = manager.sanitized_session_environment(source)

    assert 'SNAP' not in result
    assert 'SNAP_ARCH' not in result
    assert 'GTK_PATH' not in result
    assert 'GDK_PIXBUF_MODULE_FILE' not in result
    assert 'GIO_MODULE_DIR' not in result
    assert '/snap/' not in result['XDG_DATA_HOME']
    assert result['XDG_DATA_DIRS'] == '/usr/share'
    assert result['ROS_DOMAIN_ID'] == '50'
    assert result['GZ_PARTITION'] == 'ui_validation_50'
    assert result['LD_LIBRARY_PATH'] == '/opt/ros/jazzy/lib'


def test_sanitized_environment_supplies_portable_xdg_defaults():
    manager = _load_manager()
    result = manager.sanitized_session_environment(
        {'PATH': '/usr/bin'}
    )
    assert result['PATH'] == '/usr/bin'
    assert result['XDG_DATA_DIRS'] == '/usr/local/share:/usr/share'
    assert result['XDG_DATA_HOME'].endswith('/.local/share')


def test_rviz_uses_fastdds_to_avoid_cyclonedds_unload_crash():
    manager = _load_manager()
    result = manager.sanitized_rviz_environment(
        {
            'PATH': '/usr/bin',
            'ROS_DOMAIN_ID': '51',
            'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
        }
    )
    assert result['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
    assert result['ROS_DOMAIN_ID'] == '51'

    custom = manager.sanitized_rviz_environment(
        {'PATH': '/usr/bin', 'RMW_IMPLEMENTATION': 'rmw_zenoh_cpp'}
    )
    assert custom['RMW_IMPLEMENTATION'] == 'rmw_zenoh_cpp'


def test_linux_stat_parser_handles_spaces_and_parentheses_in_command():
    manager = _load_manager()
    stat_text = (
        '1234 (gz sim (server)) S 10 20 3456 40 0 0 0 0 0'
    )
    session_id, state = manager._process_session_from_stat(stat_text)
    assert session_id == 3456
    assert state == 'S'


@pytest.mark.parametrize(
    ('command_line', 'expected'),
    [
        ('gz\0sim\0-r\0world.sdf\0', True),
        ('/opt/ros/jazzy/bin/gz sim server', True),
        ('ruby /opt/ros/jazzy/bin/gz sim -s world.sdf', True),
        ('python3 environment_manager.py', False),
        ('ros2 topic echo /gz/sim/status', False),
    ],
)
def test_gazebo_process_command_detection(command_line, expected):
    manager = _load_manager()
    assert manager._is_gz_sim_command(command_line) is expected


def test_session_process_scan_contains_current_non_zombie_process():
    import os

    manager = _load_manager()
    assert os.getpid() in manager.session_process_ids(os.getsid(0))
