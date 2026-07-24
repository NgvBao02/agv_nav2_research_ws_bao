"""Launch persistent RViz with a switchable Gazebo and Nav2 session."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('vacuum_robot_gazebo')
    rviz_config = os.path.join(
        package_share, 'rviz', 'research_comparison.rviz'
    )

    environment = LaunchConfiguration('environment')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('rviz')
    use_nav2 = LaunchConfiguration('nav2')
    use_comparison = LaunchConfiguration('compare')
    execute = LaunchConfiguration('execute')
    execute_method = LaunchConfiguration('execute_method')
    planner_id = LaunchConfiguration('planner_id')

    manager = Node(
        package='vacuum_robot_gazebo',
        executable='environment_manager.py',
        name='environment_manager',
        output='screen',
        sigterm_timeout='25.0',
        sigkill_timeout='5.0',
        parameters=[
            {
                'environment': environment,
                'gui': gui,
                'nav2': use_nav2,
                'compare': use_comparison,
                'execute': execute,
                'execute_method': execute_method,
                'planner_id': planner_id,
            }
        ],
    )
    rviz = Node(
        package='vacuum_robot_gazebo',
        executable='sanitized_rviz.py',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('gui', default_value='true'),
            DeclareLaunchArgument(
                'environment',
                default_value='research_warehouse',
                description='Initial matched Gazebo world and Nav2 map',
            ),
            DeclareLaunchArgument('rviz', default_value='true'),
            DeclareLaunchArgument('nav2', default_value='true'),
            DeclareLaunchArgument('compare', default_value='true'),
            DeclareLaunchArgument('execute', default_value='true'),
            DeclareLaunchArgument(
                'execute_method', default_value='simple'
            ),
            DeclareLaunchArgument(
                'planner_id', default_value='ThetaStar'
            ),
            manager,
            rviz,
        ]
    )
