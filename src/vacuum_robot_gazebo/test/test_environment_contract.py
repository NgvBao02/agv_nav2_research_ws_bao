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
    assert smoother['pstmo']['minimum_trim_distance'] == 0.02
    assert smoother['pstmo']['maximum_trim_distance'] == 0.8
    legacy_search_parameters = {
        'initial_search_samples',
        'maximum_evaluations_per_corner',
        'trim_tolerance',
        'objective_tolerance',
        'retained_candidates_per_corner',
        'minimum_bezier_control_fraction',
        'maximum_bezier_control_fraction',
        'bezier_control_fraction_samples',
        'bezier_control_fraction',
        'max_trim_fraction',
    }
    assert legacy_search_parameters.isdisjoint(smoother['pstmo'])
    hybrid_pivot = smoother['adaptive_hybrid']['pivot']
    assert hybrid_pivot['minimum_trim_distance'] == 0.02
    assert hybrid_pivot['maximum_trim_distance'] == 0.8
    assert hybrid_pivot['initial_search_samples'] == 6
    assert hybrid_pivot['maximum_evaluations_per_corner'] == 20
    assert hybrid_pivot['minimum_bezier_control_fraction'] == 0.08
    assert hybrid_pivot['maximum_bezier_control_fraction'] == 0.45
    assert hybrid_pivot['bezier_control_fraction_samples'] == 1
    assert hybrid_pivot['bezier_control_fraction'] == 0.35
    removed_los_parameters = {
        'line_of_sight_pruning',
        'line_of_sight_footprint_padding',
        'compare_los_against_no_los',
        'los_selection_minimum_improvement',
        'los_path_length_weight',
        'los_max_curvature_weight',
        'los_curvature_energy_weight',
        'los_pivot_rotation_weight',
        'los_proximity_cost_weight',
        'los_raw_fallback_penalty',
    }
    assert removed_los_parameters.isdisjoint(smoother['pstmo'])
    assert removed_los_parameters.isdisjoint(smoother['adaptive_hybrid']['pivot'])
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


def test_pstmo_preprocessing_modes_are_internal_and_diagnostics_are_single_pipeline():
    workspace_src = PACKAGE_ROOT.parent
    header_source = (
        workspace_src
        / 'adaptive_pivot_g2_nav2'
        / 'include'
        / 'adaptive_pivot_g2_nav2'
        / 'adaptive_pivot_g2_smoother.hpp'
    ).read_text(encoding='utf-8')
    smoother_source = (
        workspace_src
        / 'adaptive_pivot_g2_nav2'
        / 'src'
        / 'adaptive_pivot_g2_smoother.cpp'
    ).read_text(encoding='utf-8')
    hybrid_source = (
        workspace_src
        / 'adaptive_pivot_g2_nav2'
        / 'src'
        / 'safety_gated_hybrid_smoother.cpp'
    ).read_text(encoding='utf-8')

    assert 'kConditionThenLos' in header_source
    assert 'preprocessing_mode_{PreprocessingMode::kConditionOnly}' in header_source
    assert (
        'candidate_search_mode_{\n'
        '    CandidateSearchMode::kHierarchicalAlphaTwoTrim}'
    ) in header_source
    assert 'PreprocessingMode::kConditionOnly' in hybrid_source
    assert 'CandidateSearchMode::kLegacyJointDq' in hybrid_source
    assert 'return smooth_pipeline(path, max_time);' in smoother_source
    assert 'padFootprint' not in smoother_source

    required_diagnostics = {
        'preprocessing_mode',
        'los_executed',
        'pipeline_execution_count',
        'final_invariants_verified',
        'los_input_points',
        'los_output_points',
        'los_attempted_shortcuts',
        'los_accepted_shortcuts',
        'los_safety_rejections',
        'los_runtime_s',
        'los_rejection_reason',
    }
    for field in required_diagnostics:
        assert f'\\"{field}\\"' in smoother_source

    removed_diagnostics = {
        'los_selection_enabled',
        'los_selected',
        'los_selection_reason',
        'los_no_los_completed',
        'los_no_los_quality',
        'los_footprint_padding_m',
        'los_fallback_to_input',
        'los_fallback_reason',
    }
    for field in removed_diagnostics:
        assert f'\\"{field}\\"' not in smoother_source


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
        '/research/path/pstmo',
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
    assert controller['plugin'] == (
        'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    )
    assert controller['desired_linear_vel'] == expected['max_linear_speed']
    assert controller['rotate_to_heading_angular_vel'] <= expected[
        'max_angular_speed'
    ]
    assert controller['max_angular_accel'] == expected[
        'max_angular_acceleration'
    ]
    assert controller['lookahead_dist'] == 0.50
    assert controller['min_lookahead_dist'] == 0.50
    assert controller['max_lookahead_dist'] == 0.50
    assert controller['use_velocity_scaled_lookahead_dist'] is False
    assert controller['use_regulated_linear_velocity_scaling'] is False
    assert controller['use_cost_regulated_linear_velocity_scaling'] is False
    assert 0.0 < controller['rotate_to_heading_min_angle'] <= 0.10
    assert not any(key.startswith('adaptive_') for key in controller)
    assert not any(key.startswith('pivot_') for key in controller)
    assert not any(key.startswith('terminal_') for key in controller)

    progress_checker = parameters['controller_server']['ros__parameters'][
        'progress_checker'
    ]
    assert progress_checker['plugin'] == (
        'nav2_controller::PoseProgressChecker'
    )
    assert progress_checker['required_movement_radius'] > 0.0
    assert progress_checker['required_movement_angle'] > 0.0

    velocity_smoother = parameters['velocity_smoother']['ros__parameters']
    assert velocity_smoother['max_velocity'] == [
        expected['max_linear_speed'], 0.0, expected['max_angular_speed']
    ]
    assert velocity_smoother['min_velocity'] == [
        -expected['max_linear_speed'], 0.0, -expected['max_angular_speed']
    ]
    assert velocity_smoother['max_accel'] == [
        expected['max_linear_acceleration'],
        0.0,
        expected['max_angular_acceleration'],
    ]
    assert velocity_smoother['max_decel'] == [
        -expected['max_linear_deceleration'],
        0.0,
        -expected['max_angular_acceleration'],
    ]

    smoother = parameters['smoother_server']['ros__parameters']
    assert smoother['smoother_plugins'] == [
        'simple_smoother',
        'savitzky_golay',
        'constrained',
        'pstmo',
        'adaptive_hybrid',
    ]
    pivot_profiles = [
        smoother['pstmo'],
        smoother['adaptive_hybrid']['pivot'],
    ]
    for profile in pivot_profiles:
        for key, value in expected.items():
            assert profile[key] == value
        assert 'radius_search_mode' not in profile
        assert 'radius_candidates' not in profile
    hybrid = smoother['adaptive_hybrid']
    assert hybrid['minimum_pivot_angle'] == hybrid['pivot'][
        'corner_angle_threshold'
    ]

    goal_checker = parameters['controller_server']['ros__parameters'][
        'general_goal_checker'
    ]
    assert goal_checker['plugin'] == 'nav2_controller::SimpleGoalChecker'
    assert 0.0 < goal_checker['xy_goal_tolerance'] <= 0.10
    assert 0.0 < goal_checker['yaw_goal_tolerance'] <= 0.10
    assert 'trans_stopped_velocity' not in goal_checker
    assert 'rot_stopped_velocity' not in goal_checker
    assert goal_checker['stateful'] is True

    route_operations = parameters['route_server']['ros__parameters'][
        'operations'
    ]
    assert route_operations == ['ReroutingService', 'CollisionMonitor']
    assert 'AdjustSpeedLimit' not in parameters['route_server'][
        'ros__parameters'
    ]

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
    assert float(drive.findtext('wheel_separation')) == 0.2834
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
    assert (
        real_profile['simulation_calibration'][
            'effective_wheel_separation_m'
        ]
        == float(drive.findtext('wheel_separation'))
    )

    # Nav2 stays inside the rated-load speed region of the supplied GA25,
    # while URDF/SDF retain no-load speed and stall torque as hard absolutes.
    motor = real_profile['drive']
    assert (
        expected['max_wheel_speed']
        <= motor['theoretical_rated_load_linear_speed_mps']
        < motor['theoretical_no_load_linear_speed_mps']
    )
    assert (
        expected['max_linear_speed'] <= expected['max_wheel_speed']
    )
    for joint_name in ('left_wheel_joint', 'right_wheel_joint'):
        joint = model.find(f".//joint[@name='{joint_name}']")
        assert (
            float(joint.findtext('axis/limit/velocity'))
            == motor['maximum_joint_velocity_radps']
        )
        assert (
            float(joint.findtext('axis/limit/effort'))
            == motor['maximum_joint_effort_nm']
        )

    power = real_profile['power']
    assert power['motor_rail']['regulator_required'] is True
    assert (
        power['pack']['full_voltage_assumption_v']
        > motor['rated_voltage_v']
    )
    assert (
        power['pack']['theoretical_continuous_discharge_current_a']
        > motor['two_motor_stall_current_a']
    )
