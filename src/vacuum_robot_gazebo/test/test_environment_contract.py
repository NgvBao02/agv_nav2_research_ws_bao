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
