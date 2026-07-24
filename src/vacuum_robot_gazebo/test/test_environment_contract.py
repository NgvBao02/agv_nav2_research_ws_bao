# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Regression tests for matched Gazebo, Nav2, and benchmark environments."""

import importlib.util
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BENCHMARK_ROOT = PACKAGE_ROOT.parent / 'adaptive_pivot_g2_benchmark'
GENERATED_ENVIRONMENTS = (
    'open_arena',
    'narrow_aisles',
    'office_maze',
    'warehouse_long_aisles',
    'warehouse_cross_aisles',
    'warehouse_dispatch',
)
ALL_ENVIRONMENTS = ('research_warehouse', *GENERATED_ENVIRONMENTS)
PLANNER_TYPES = {
    'NavFnAStar': 'nav2_navfn_planner::NavfnPlanner',
    'NavFnDijkstra': 'nav2_navfn_planner::NavfnPlanner',
    'ThetaStar': 'nav2_theta_star_planner::ThetaStarPlanner',
    'Smac2D': 'nav2_smac_planner::SmacPlanner2D',
    'SmacHybrid': 'nav2_smac_planner::SmacPlannerHybrid',
}


def _load_generator():
    path = PACKAGE_ROOT / 'scripts' / 'generate_benchmark_environments.py'
    spec = importlib.util.spec_from_file_location(
        'generate_benchmark_environments', path
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
    assert len(pixels) == width * height
    return width, height, pixels


def test_generated_environment_files_are_current():
    generator = _load_generator()
    with tempfile.TemporaryDirectory() as directory:
        temporary = Path(directory)
        temporary_gazebo = temporary / 'vacuum_robot_gazebo'
        temporary_benchmark = temporary / 'adaptive_pivot_g2_benchmark'
        generated = generator.generate(
            temporary_gazebo, temporary_benchmark
        )
        assert len(generated) == 4 * len(GENERATED_ENVIRONMENTS)
        for temporary_path in generated:
            if temporary_path.is_relative_to(temporary_gazebo):
                relative = temporary_path.relative_to(temporary_gazebo)
                committed = PACKAGE_ROOT / relative
            else:
                relative = temporary_path.relative_to(temporary_benchmark)
                committed = BENCHMARK_ROOT / relative
            assert committed.read_bytes() == temporary_path.read_bytes()


def test_every_environment_has_matching_world_and_map_metadata():
    for name in ALL_ENVIRONMENTS:
        world_path = PACKAGE_ROOT / 'worlds' / f'{name}.sdf'
        map_yaml_path = PACKAGE_ROOT / 'maps' / f'{name}.yaml'
        map_image_path = PACKAGE_ROOT / 'maps' / f'{name}.pgm'
        assert world_path.is_file()
        assert map_yaml_path.is_file()
        assert map_image_path.is_file()

        world = ET.parse(world_path).getroot().find('world')
        assert world is not None
        assert world.get('name') == name
        with map_yaml_path.open(encoding='utf-8') as stream:
            metadata = yaml.safe_load(stream)
        assert metadata['image'] == map_image_path.name
        assert metadata['resolution'] == 0.05
        assert metadata['origin'] == [-6.0, -4.0, 0.0]
        width, height, pixels = _read_pgm(map_image_path)
        assert (width, height) == (240, 160)
        assert 0 in pixels
        assert 254 in pixels
        assert set(pixels).issubset({0, 254})


def test_each_environment_has_named_reachable_scenarios():
    for name in ALL_ENVIRONMENTS:
        scenario_name = (
            'research_scenarios.yaml'
            if name == 'research_warehouse'
            else f'{name}_scenarios.yaml'
        )
        with (
            BENCHMARK_ROOT / 'config' / scenario_name
        ).open(encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        assert document['environment'] == name
        scenarios = document['scenarios']
        assert len(scenarios) >= 8
        assert len({scenario['name'] for scenario in scenarios}) == len(
            scenarios
        )

        _, _, pixels = _read_pgm(
            PACKAGE_ROOT / 'maps' / f'{name}.pgm'
        )
        for scenario in scenarios:
            for key in ('start', 'goal'):
                point = scenario[key]
                assert len(point) >= 2
                column = int((float(point[0]) + 6.0) / 0.05)
                row = int((float(point[1]) + 4.0) / 0.05)
                assert 0 <= column < 240
                assert 0 <= row < 160
                image_row = 159 - row
                assert pixels[image_row * 240 + column] == 254


def test_nav2_loads_the_declared_planner_families():
    with (
        PACKAGE_ROOT / 'config' / 'nav2_params.yaml'
    ).open(encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)
    planner = parameters['planner_server']['ros__parameters']
    assert planner['planner_plugins'] == list(PLANNER_TYPES)
    for planner_id, plugin_type in PLANNER_TYPES.items():
        assert planner[planner_id]['plugin'] == plugin_type
    assert planner['NavFnAStar']['use_astar'] is True
    assert planner['NavFnDijkstra']['use_astar'] is False
    assert planner['SmacHybrid']['motion_model_for_search'] == 'DUBIN'
    assert planner['SmacHybrid']['smooth_path'] is False
    smoother = parameters['smoother_server']['ros__parameters']
    assert smoother['constrained']['path_downsampling_factor'] == 1
    assert (
        parameters['local_costmap']['local_costmap']['ros__parameters'][
            'initial_transform_timeout'
        ]
        >= 10.0
    )
    assert (
        parameters['global_costmap']['global_costmap']['ros__parameters'][
            'initial_transform_timeout'
        ]
        >= 10.0
    )

    launch_source = (
        PACKAGE_ROOT / 'launch' / 'simulation.launch.py'
    ).read_text(encoding='utf-8')
    assert 'OnProcessExit' in launch_source
    assert 'target_action=spawn_robot' in launch_source
    assert 'nav2_start_delay' in launch_source


def test_research_rviz_loads_the_custom_planner_selector():
    with (
        PACKAGE_ROOT / 'rviz' / 'research_comparison.rviz'
    ).open(encoding='utf-8') as stream:
        configuration = yaml.safe_load(stream)
    selector_panels = [
        panel for panel in configuration['Panels']
        if panel.get('Name') == 'Selector'
    ]
    assert selector_panels == [{
        'Class': 'adaptive_pivot_g2_rviz/Planner Selector',
        'Name': 'Selector',
    }]

    package = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    runtime_dependencies = {
        dependency.text for dependency in package.findall('exec_depend')
    }
    assert 'adaptive_pivot_g2_rviz' in runtime_dependencies


def test_research_rviz_has_one_display_for_every_comparison_path():
    with (
        PACKAGE_ROOT / 'rviz' / 'research_comparison.rviz'
    ).open(encoding='utf-8') as stream:
        configuration = yaml.safe_load(stream)

    topics = set()

    def collect_topics(displays):
        for display in displays:
            topic = display.get('Topic')
            if isinstance(topic, dict):
                value = topic.get('Value')
                if isinstance(value, str):
                    topics.add(value)
            children = display.get('Displays')
            if isinstance(children, list):
                collect_topics(children)

    collect_topics(
        configuration['Visualization Manager']['Displays']
    )
    expected = {
        '/research/path/raw',
        '/research/path/simple',
        '/research/path/savitzky_golay',
        '/research/path/constrained',
        '/research/path/pivot_g2_fixed',
        '/research/path/pivot_g2',
        '/research/path/adaptive_hybrid_fixed',
        '/research/path/adaptive_hybrid',
        '/research/path/executed',
    }
    assert expected.issubset(topics)


def test_motion_limits_are_consistent_across_nav2_and_gazebo():
    with (
        PACKAGE_ROOT / 'config' / 'nav2_params.yaml'
    ).open(encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)
    with (
        PACKAGE_ROOT / 'config' / 'real_robot_profile.yaml'
    ).open(encoding='utf-8') as stream:
        real_profile = yaml.safe_load(stream)
    model = ET.parse(
        PACKAGE_ROOT / 'models' / 'vacuum_robot' / 'model.sdf'
    ).getroot()
    drive = model.find(".//plugin[@name='gz::sim::systems::DiffDrive']")
    assert drive is not None

    expected = {
        'wheel_separation': 0.2548,
        'max_linear_speed': 0.30,
        'max_angular_speed': 0.80,
        'max_wheel_speed': 0.36,
        'max_lateral_acceleration': 0.18,
        'max_linear_acceleration': 0.35,
        'max_linear_deceleration': 0.45,
        'max_angular_acceleration': 1.20,
    }
    controller = parameters['controller_server']['ros__parameters'][
        'FollowPath'
    ]
    assert controller['desired_linear_vel'] == expected['max_linear_speed']
    assert (
        controller['adaptive_wheel_separation']
        == expected['wheel_separation']
    )
    assert (
        controller['adaptive_max_linear_speed']
        == expected['max_linear_speed']
    )
    assert (
        controller['adaptive_max_angular_speed']
        == expected['max_angular_speed']
    )
    assert (
        controller['adaptive_max_wheel_linear_speed']
        == expected['max_wheel_speed']
    )
    assert (
        controller['adaptive_max_lateral_acceleration']
        == expected['max_lateral_acceleration']
    )
    assert (
        controller['adaptive_max_linear_acceleration']
        == expected['max_linear_acceleration']
    )
    assert (
        controller['adaptive_max_linear_deceleration']
        == expected['max_linear_deceleration']
    )
    assert (
        controller['adaptive_max_angular_acceleration']
        == expected['max_angular_acceleration']
    )
    assert controller['pivot_control_period'] == 1.0 / (
        parameters['controller_server']['ros__parameters'][
            'controller_frequency'
        ]
    )

    smoother = parameters['smoother_server']['ros__parameters']
    pivot_profiles = [
        smoother['pivot_g2_fixed'],
        smoother['pivot_g2'],
        smoother['adaptive_hybrid_fixed']['pivot'],
        smoother['adaptive_hybrid']['pivot'],
    ]
    for profile in pivot_profiles:
        for key, value in expected.items():
            assert profile[key] == value
        assert (
            profile['corner_angle_threshold']
            == controller['minimum_pivot_angle']
        )

    hybrid_profiles = [
        smoother['adaptive_hybrid_fixed'],
        smoother['adaptive_hybrid'],
    ]
    for profile in hybrid_profiles:
        assert (
            profile['pivot_duplicate_position_tolerance']
            == controller['pivot_duplicate_position_tolerance']
        )
        assert (
            profile['minimum_pivot_angle']
            == controller['minimum_pivot_angle']
        )
        assert (
            profile['minimum_pivot_angle']
            == profile['pivot']['corner_angle_threshold']
        )

    goal_checker = parameters['controller_server']['ros__parameters'][
        'general_goal_checker'
    ]
    assert (
        controller['terminal_hold_position_tolerance']
        < goal_checker['xy_goal_tolerance']
        <= controller['terminal_release_position_tolerance']
        < controller['terminal_staging_position_tolerance']
    )
    assert (
        controller['pivot_yaw_tolerance']
        < goal_checker['yaw_goal_tolerance']
        <= 0.05
    )
    assert (
        controller['pivot_stopped_linear_velocity']
        == goal_checker['trans_stopped_velocity']
    )
    assert (
        controller['pivot_stopped_angular_velocity']
        == goal_checker['rot_stopped_velocity']
    )
    assert (
        controller['terminal_precision_max_linear_speed']
        <= controller['terminal_max_linear_speed']
        <= controller['adaptive_max_linear_speed']
    )
    assert (
        controller['pivot_position_tolerance']
        <= controller['terminal_staging_position_tolerance']
    )

    assert float(drive.findtext('max_linear_velocity')) == expected[
        'max_linear_speed'
    ]
    assert float(drive.findtext('max_angular_velocity')) == expected[
        'max_angular_speed'
    ]
    assert float(drive.findtext('max_linear_acceleration')) == expected[
        'max_linear_acceleration'
    ]
    assert -float(drive.findtext('min_linear_acceleration')) == expected[
        'max_linear_deceleration'
    ]
    assert float(drive.findtext('max_angular_acceleration')) == expected[
        'max_angular_acceleration'
    ]
    # Gazebo's calibrated odometry separation is deliberately different from
    # the physical rolling-tread separation used by kinematics and metrics.
    assert float(drive.findtext('wheel_separation')) == 0.2809
    left_wheel_y = float(
        model.find(".//link[@name='left_wheel']/pose").text.split()[1]
    )
    right_wheel_y = float(
        model.find(".//link[@name='right_wheel']/pose").text.split()[1]
    )
    assert left_wheel_y - right_wheel_y == expected['wheel_separation']
    assert (
        real_profile['robot']['wheel_separation_m']
        == expected['wheel_separation']
    )
