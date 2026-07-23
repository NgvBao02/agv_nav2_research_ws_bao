"""Launch the real-profile robot, Gazebo map, Nav2, RViz, and comparison runner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_share = get_package_share_directory('vacuum_robot_gazebo')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    nav2_share = get_package_share_directory('nav2_bringup')

    model = os.path.join(package_share, 'models', 'vacuum_robot', 'model.sdf')
    urdf = os.path.join(package_share, 'urdf', 'vacuum_robot.urdf')
    bridge_config = os.path.join(package_share, 'config', 'bridge.yaml')
    nav2_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(package_share, 'rviz', 'research_comparison.rviz')

    environment = LaunchConfiguration('environment')
    environment_filename = PythonExpression(["'", environment, "' + '.sdf'"])
    map_filename = PythonExpression(["'", environment, "' + '.yaml'"])
    world = PathJoinSubstitution(
        [package_share, 'worlds', environment_filename]
    )
    map_yaml = PathJoinSubstitution(
        [package_share, 'maps', map_filename]
    )
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('rviz')
    use_nav2 = LaunchConfiguration('nav2')
    use_comparison = LaunchConfiguration('compare')
    execute = LaunchConfiguration('execute')
    execute_method = LaunchConfiguration('execute_method')
    planner_id = LaunchConfiguration('planner_id')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    with open(urdf, 'r', encoding='utf-8') as stream:
        robot_description = stream.read()

    # Keep AMCL's first particle cloud aligned with the Gazebo spawn pose.
    # Without this rewrite, a non-default spawn yaw briefly projects lidar
    # points through the old pose and can leave phantom obstacles at the start.
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={
            'amcl.ros__parameters.initial_pose.x': x_pose,
            'amcl.ros__parameters.initial_pose.y': y_pose,
            'amcl.ros__parameters.initial_pose.yaw': yaw,
        },
        convert_types=True,
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        condition=IfCondition(gui),
        launch_arguments={'gz_args': ['-r -v 3 ', world]}.items(),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        condition=UnlessCondition(gui),
        launch_arguments={'gz_args': ['-r -s -v 3 ', world]}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_vacuum_robot',
        output='screen',
        arguments=[
            '-world', environment,
            '-file', model,
            '-name', 'vacuum_robot',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.002',
            '-Y', yaw,
        ],
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        condition=IfCondition(use_nav2),
        launch_arguments={
            'slam': 'False',
            'map': map_yaml,
            'use_sim_time': 'True',
            'params_file': configured_nav2_params,
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )
    comparison = Node(
        package='adaptive_pivot_g2_benchmark',
        executable='compare_paths',
        name='path_comparison',
        output='screen',
        condition=IfCondition(use_comparison),
        parameters=[
            {
                'use_sim_time': True,
                'execute': execute,
                'execute_method': execute_method,
                'planner_id': planner_id,
            }
        ],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
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
                description=(
                    'Matched Gazebo SDF and Nav2 map basename: '
                    'research_warehouse, open_arena, narrow_aisles, or '
                    'office_maze; warehouse layouts: '
                    'warehouse_long_aisles, warehouse_cross_aisles, or '
                    'warehouse_dispatch'
                ),
            ),
            DeclareLaunchArgument('rviz', default_value='true'),
            DeclareLaunchArgument('nav2', default_value='true'),
            DeclareLaunchArgument('compare', default_value='true'),
            DeclareLaunchArgument('execute', default_value='true'),
            DeclareLaunchArgument(
                'execute_method',
                default_value='simple',
                description=(
                    'none, raw, simple, savitzky_golay, constrained, '
                    'pivot_g2, or adaptive_hybrid'
                ),
            ),
            DeclareLaunchArgument(
                'planner_id',
                default_value='ThetaStar',
                description=(
                    'Nav2 planner plugin ID: NavFnAStar, NavFnDijkstra, '
                    'ThetaStar, Smac2D, or SmacHybrid'
                ),
            ),
            DeclareLaunchArgument('x_pose', default_value='-5.0'),
            DeclareLaunchArgument('y_pose', default_value='-3.0'),
            DeclareLaunchArgument('yaw', default_value='0.0'),
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH', os.path.join(package_share, 'models')
            ),
            gazebo_gui,
            gazebo_headless,
            spawn_robot,
            bridge,
            robot_state_publisher,
            nav2,
            comparison,
            rviz,
        ]
    )
