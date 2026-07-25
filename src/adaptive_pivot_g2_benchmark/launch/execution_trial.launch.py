"""Start a fresh Gazebo/Nav2 stack and run exactly one execution trial."""

import os
from pathlib import Path

from adaptive_pivot_g2_benchmark.initial_heading import (
    resolve_scenario_start_heading,
)
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _launch_setup(context):
    gazebo_share = get_package_share_directory('vacuum_robot_gazebo')
    scenario_file = LaunchConfiguration('scenario_file').perform(context)
    scenario_name = LaunchConfiguration('scenario').perform(context)
    method = LaunchConfiguration('method').perform(context)
    planner = LaunchConfiguration('planner').perform(context)
    output_json = LaunchConfiguration('output_json').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    fixed_speed_limit = LaunchConfiguration(
        'fixed_speed_limit_mps'
    ).perform(context)
    with Path(scenario_file).open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    environment = str(document.get('environment', 'research_warehouse'))
    selected = next(
        (
            scenario
            for scenario in document.get('scenarios', [])
            if scenario.get('name') == scenario_name
        ),
        None,
    )
    if selected is None:
        raise RuntimeError(f'unknown execution scenario: {scenario_name!r}')
    start = selected['start']
    heading = resolve_scenario_start_heading(
        selected, environment, Path(gazebo_share) / 'maps'
    )
    start_yaw = heading.yaw

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'gui': gui,
            'rviz': 'false',
            'compare': 'false',
            'execute': 'false',
            'environment': environment,
            'x_pose': str(float(start[0])),
            'y_pose': str(float(start[1])),
            'yaw': str(start_yaw),
        }.items(),
    )
    trial = Node(
        package='adaptive_pivot_g2_benchmark',
        executable='execution_trial',
        name='execution_trial',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scenario_file': scenario_file,
            'scenario': scenario_name,
            'method': method,
            'planner': planner,
            'output_json': output_json,
            'fixed_speed_limit_mps': float(fixed_speed_limit),
        }],
    )
    stop_when_finished = RegisterEventHandler(
        OnProcessExit(
            target_action=trial,
            on_exit=[EmitEvent(event=Shutdown(reason='execution trial completed'))],
        )
    )
    return [simulation, trial, stop_when_finished]


def generate_launch_description():
    benchmark_share = get_package_share_directory('adaptive_pivot_g2_benchmark')
    default_scenarios = os.path.join(
        benchmark_share, 'config', 'research_scenarios.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('scenario_file', default_value=default_scenarios),
        DeclareLaunchArgument('scenario', default_value='lower_left_diagonal'),
        DeclareLaunchArgument('method', default_value='pivot_g2'),
        DeclareLaunchArgument('planner', default_value='ThetaStar'),
        DeclareLaunchArgument(
            'output_json', default_value='/tmp/pivot_g2_execution_trial.json'
        ),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('fixed_speed_limit_mps', default_value='0.0'),
        OpaqueFunction(function=_launch_setup),
    ])
