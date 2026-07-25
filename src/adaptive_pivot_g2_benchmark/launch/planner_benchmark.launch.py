# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Run one complete multi-planner geometry benchmark in an isolated stack."""

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
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _launch_setup(context):
    gazebo_share = get_package_share_directory('vacuum_robot_gazebo')
    scenario_file = Path(
        LaunchConfiguration('scenario_file').perform(context)
    ).resolve()
    with scenario_file.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    environment = str(document.get('environment', 'research_warehouse'))
    scenarios = document.get('scenarios', [])
    if not scenarios:
        raise RuntimeError(f'no scenarios found in {scenario_file}')
    start = scenarios[0]['start']
    heading = resolve_scenario_start_heading(
        scenarios[0], environment, Path(gazebo_share) / 'maps'
    )
    start_yaw = heading.yaw

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_share, 'launch', 'simulation.launch.py'
            )
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': 'false',
            'compare': 'false',
            'execute': 'false',
            'environment': environment,
            'x_pose': str(float(start[0])),
            'y_pose': str(float(start[1])),
            'yaw': str(start_yaw),
        }.items(),
    )
    benchmark = Node(
        package='adaptive_pivot_g2_benchmark',
        executable='batch_benchmark',
        name='planner_geometry_benchmark',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scenario_file': str(scenario_file),
            'output_csv': LaunchConfiguration('output_csv'),
            'output_json': LaunchConfiguration('output_json'),
            'repetitions': ParameterValue(
                LaunchConfiguration('repetitions'), value_type=int
            ),
            'resample_spacing': ParameterValue(
                LaunchConfiguration('resample_spacing'), value_type=float
            ),
        }],
    )
    stop_when_finished = RegisterEventHandler(
        OnProcessExit(
            target_action=benchmark,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason='planner benchmark completed')
                )
            ],
        )
    )
    return [simulation, benchmark, stop_when_finished]


def generate_launch_description():
    benchmark_share = get_package_share_directory(
        'adaptive_pivot_g2_benchmark'
    )
    default_scenarios = os.path.join(
        benchmark_share, 'config', 'open_arena_scenarios.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_file', default_value=default_scenarios
        ),
        DeclareLaunchArgument(
            'output_csv', default_value='/tmp/planner_benchmark.csv'
        ),
        DeclareLaunchArgument(
            'output_json',
            default_value='/tmp/planner_benchmark_summary.json',
        ),
        DeclareLaunchArgument('repetitions', default_value='1'),
        DeclareLaunchArgument('resample_spacing', default_value='0.05'),
        DeclareLaunchArgument('gui', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
