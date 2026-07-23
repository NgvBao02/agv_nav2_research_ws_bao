"""Inspect the robot URDF in RViz without starting Gazebo or Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('vacuum_robot_gazebo')
    urdf_path = os.path.join(package_share, 'urdf', 'vacuum_robot.urdf')
    rviz_path = os.path.join(package_share, 'rviz', 'urdf_check.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as stream:
        robot_description = stream.read()

    return LaunchDescription(
        [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='urdf_robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            ),
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='urdf_joint_state_publisher_gui',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='urdf_check_rviz',
                output='screen',
                arguments=['-d', rviz_path],
            ),
        ]
    )
