#!/usr/bin/env python3
# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""
Generate matched Gazebo worlds, Nav2 maps, and benchmark scenarios.

The geometry below is the single source of truth for each added environment.
Running this script updates both the SDF collision boxes and the PGM occupancy
map, avoiding a visually plausible Gazebo world that disagrees with Nav2.
"""

from collections import deque
from dataclasses import dataclass
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


RESOLUTION = 0.05
ORIGIN_X = -6.0
ORIGIN_Y = -4.0
WIDTH = 240
HEIGHT = 160
FLOOR_SIZE_X = WIDTH * RESOLUTION
FLOOR_SIZE_Y = HEIGHT * RESOLUTION
ROBOT_VALIDATION_MARGIN = 0.22


@dataclass(frozen=True)
class Box:
    """Axis-aligned static collision box."""

    name: str
    center_x: float
    center_y: float
    size_x: float
    size_y: float
    height: float = 1.2
    color: str = '0.16 0.36 0.72 1'


@dataclass(frozen=True)
class Environment:
    """One reproducible world/map/scenario bundle."""

    description: str
    obstacles: tuple
    scenarios: tuple


BOUNDARIES = (
    Box('boundary_north', 0.0, 3.95, 12.0, 0.10, 1.0, '0.85 0.85 0.88 1'),
    Box('boundary_south', 0.0, -3.95, 12.0, 0.10, 1.0, '0.85 0.85 0.88 1'),
    Box('boundary_west', -5.95, 0.0, 0.10, 8.0, 1.0, '0.85 0.85 0.88 1'),
    Box('boundary_east', 5.95, 0.0, 0.10, 8.0, 1.0, '0.85 0.85 0.88 1'),
)


ENVIRONMENTS = {
    'open_arena': Environment(
        description=(
            'Sparse open arena for long diagonals, isolated detours, and '
            'planner path-length/runtime comparisons.'
        ),
        obstacles=(
            Box('pillar_west', -3.0, 1.6, 0.8, 0.8),
            Box('block_center', -0.4, -0.6, 1.2, 1.2),
            Box('screen_east', 2.3, 1.4, 0.5, 2.4),
            Box('block_southeast', 4.2, -2.3, 1.0, 0.7),
        ),
        scenarios=(
            ('west_east_center', (-5.0, 0.0), (5.0, 0.0)),
            ('southwest_northeast', (-5.0, -3.0), (5.0, 3.0)),
            ('northwest_southeast', (-5.0, 3.0), (5.0, -3.0)),
            ('center_block_detour', (-2.2, -0.6), (1.2, -0.6)),
            ('screen_detour', (1.2, 1.4), (3.5, 1.4)),
            ('long_lower_lane', (-5.0, -2.8), (3.2, -2.8)),
            ('long_upper_lane', (-5.0, 2.8), (5.0, 2.8)),
            ('short_open_diagonal', (-5.0, -3.0), (-2.5, -2.0)),
        ),
    ),
    'narrow_aisles': Environment(
        description=(
            'Alternating shelves with 1.4 m longitudinal aisles and 0.9 m '
            'end passages for clearance, zig-zag, and failure analysis.'
        ),
        obstacles=(
            Box('shelf_a', -3.0, 0.6, 0.6, 4.8),
            Box('shelf_b', -1.0, -0.6, 0.6, 4.8),
            Box('shelf_c', 1.0, 0.6, 0.6, 4.8),
            Box('shelf_d', 3.0, -0.6, 0.6, 4.8),
        ),
        scenarios=(
            ('serpentine_west_east', (-5.0, 0.0), (5.0, 0.0)),
            ('serpentine_east_west', (5.0, 0.0), (-5.0, 0.0)),
            ('left_vertical_aisle', (-4.2, -2.8), (-4.2, 2.8)),
            ('inner_vertical_aisle', (-2.0, -1.4), (-2.0, 1.4)),
            ('bottom_alternating_cross', (-4.5, -2.6), (2.0, -2.6)),
            ('top_alternating_cross', (-2.0, 2.6), (4.5, 2.6)),
            ('southwest_northeast_weave', (-5.0, -3.0), (5.0, 3.0)),
            ('northwest_southeast_weave', (-5.0, 3.0), (5.0, -3.0)),
        ),
    ),
    'office_maze': Environment(
        description=(
            'Broken office partition walls with offset doorways, producing '
            'L-turns, U-turns, and room-to-room routes.'
        ),
        obstacles=(
            Box('west_partition_south', -2.5, -2.85, 0.15, 2.10, 1.1),
            Box('west_partition_middle', -2.5, 0.10, 0.15, 1.80, 1.1),
            Box('west_partition_north', -2.5, 2.95, 0.15, 1.90, 1.1),
            Box('east_partition_south', 2.0, -3.20, 0.15, 1.40, 1.1),
            Box('east_partition_middle', 2.0, 0.15, 0.15, 3.30, 1.1),
            Box('east_partition_north', 2.0, 3.35, 0.15, 1.10, 1.1),
            Box('north_office_wall_west', -4.85, 1.4, 2.10, 0.15, 1.1),
            Box('north_office_wall_center', -1.40, 1.4, 2.80, 0.15, 1.1),
            Box('north_office_wall_east', 3.45, 1.4, 4.90, 0.15, 1.1),
            Box(
                'desk_west', -4.4, -0.2, 1.0, 0.6, 0.8,
                '0.55 0.28 0.08 1',
            ),
            Box(
                'desk_center', -0.4, -1.6, 1.0, 0.6, 0.8,
                '0.55 0.28 0.08 1',
            ),
            Box(
                'desk_east', 4.1, 2.4, 1.0, 0.6, 0.8,
                '0.55 0.28 0.08 1',
            ),
        ),
        scenarios=(
            ('office_long_diagonal', (-5.2, -3.0), (5.2, 3.0)),
            ('office_reverse_diagonal', (5.2, 3.0), (-5.2, -3.0)),
            ('west_room_to_room', (-4.2, -2.5), (-4.2, 2.8)),
            ('east_room_to_room', (4.2, -2.5), (5.0, 2.8)),
            ('lower_cross_offices', (-5.0, -1.2), (5.0, -1.2)),
            ('upper_cross_offices', (-5.0, 2.3), (5.0, 2.3)),
            ('central_u_turn', (-1.5, -3.0), (-1.5, 2.8)),
            ('east_partition_detour', (0.0, -3.0), (5.0, 0.5)),
        ),
    ),
}


def _element(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if text is not None:
        element.text = str(text)
    return element


def _add_box(link, box):
    pose = f'{box.center_x} {box.center_y} {0.5 * box.height} 0 0 0'
    size = f'{box.size_x} {box.size_y} {box.height}'
    collision = _element(link, 'collision', name=box.name)
    _element(collision, 'pose', pose)
    geometry = _element(collision, 'geometry')
    box_geometry = _element(geometry, 'box')
    _element(box_geometry, 'size', size)

    visual = _element(link, 'visual', name=f'{box.name}_visual')
    _element(visual, 'pose', pose)
    geometry = _element(visual, 'geometry')
    box_geometry = _element(geometry, 'box')
    _element(box_geometry, 'size', size)
    material = _element(visual, 'material')
    _element(material, 'ambient', box.color)
    _element(material, 'diffuse', box.color)


def _world_tree(name, environment):
    root = ET.Element('sdf', {'version': '1.9'})
    world = _element(root, 'world', name=name)
    physics = _element(world, 'physics', name='research_physics', type='ode')
    _element(physics, 'max_step_size', '0.003')
    _element(physics, 'real_time_factor', '1.0')
    for filename, plugin_name in (
        ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
        ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
        ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
        ('gz-sim-imu-system', 'gz::sim::systems::Imu'),
    ):
        _element(world, 'plugin', filename=filename, name=plugin_name)
    sensors = _element(
        world,
        'plugin',
        filename='gz-sim-sensors-system',
        name='gz::sim::systems::Sensors',
    )
    _element(sensors, 'render_engine', 'ogre2')

    scene = _element(world, 'scene')
    _element(scene, 'ambient', '0.8 0.8 0.8 1')
    _element(scene, 'background', '0.18 0.22 0.28 1')
    _element(scene, 'shadows', 'false')
    _element(scene, 'grid', 'true')
    light = _element(world, 'light', name='sun', type='directional')
    _element(light, 'pose', '0 0 10 0 0 0')
    _element(light, 'diffuse', '0.9 0.9 0.9 1')
    _element(light, 'specular', '0.2 0.2 0.2 1')
    _element(light, 'direction', '-0.5 0.2 -1.0')

    model = _element(world, 'model', name=f'{name}_layout')
    _element(model, 'static', 'true')
    link = _element(model, 'link', name='layout_link')
    floor = Box(
        'floor', 0.0, 0.0, FLOOR_SIZE_X, FLOOR_SIZE_Y, 0.05,
        '0.72 0.72 0.72 1',
    )
    # Floor top is z=0, unlike regular boxes whose base is z=0.
    floor_pose = '0 0 -0.025 0 0 0'
    collision = _element(link, 'collision', name='floor_collision')
    _element(collision, 'pose', floor_pose)
    geometry = _element(collision, 'geometry')
    box_geometry = _element(geometry, 'box')
    _element(
        box_geometry, 'size',
        f'{floor.size_x} {floor.size_y} {floor.height}',
    )
    visual = _element(link, 'visual', name='floor_visual')
    _element(visual, 'pose', floor_pose)
    geometry = _element(visual, 'geometry')
    box_geometry = _element(geometry, 'box')
    _element(
        box_geometry, 'size',
        f'{floor.size_x} {floor.size_y} {floor.height}',
    )
    material = _element(visual, 'material')
    _element(material, 'ambient', floor.color)
    _element(material, 'diffuse', floor.color)

    for box in (*BOUNDARIES, *environment.obstacles):
        _add_box(link, box)
    ET.indent(root, space='  ')
    return ET.ElementTree(root)


def _occupied(x, y, obstacles, margin=0.0):
    return any(
        abs(x - box.center_x) <= 0.5 * box.size_x + margin
        and abs(y - box.center_y) <= 0.5 * box.size_y + margin
        for box in obstacles
    )


def _map_cell(point):
    column = int((point[0] - ORIGIN_X) / RESOLUTION)
    row = int((point[1] - ORIGIN_Y) / RESOLUTION)
    return column, row


def _validate_scenarios(name, environment):
    obstacles = (*BOUNDARIES, *environment.obstacles)
    blocked = {
        (column, row)
        for row in range(HEIGHT)
        for column in range(WIDTH)
        if _occupied(
            ORIGIN_X + (column + 0.5) * RESOLUTION,
            ORIGIN_Y + (row + 0.5) * RESOLUTION,
            obstacles,
            ROBOT_VALIDATION_MARGIN,
        )
    }
    for scenario_name, start, goal in environment.scenarios:
        first = _map_cell(start)
        last = _map_cell(goal)
        if first in blocked or last in blocked:
            raise ValueError(
                f'{name}/{scenario_name}: start or goal violates the '
                'robot validation margin'
            )
        queue = deque([first])
        visited = {first}
        while queue and last not in visited:
            column, row = queue.popleft()
            for delta_column, delta_row in (
                (-1, -1), (-1, 0), (-1, 1), (0, -1),
                (0, 1), (1, -1), (1, 0), (1, 1),
            ):
                candidate = (
                    column + delta_column, row + delta_row
                )
                if (
                    0 <= candidate[0] < WIDTH
                    and 0 <= candidate[1] < HEIGHT
                    and candidate not in blocked
                    and candidate not in visited
                ):
                    visited.add(candidate)
                    queue.append(candidate)
        if last not in visited:
            raise ValueError(
                f'{name}/{scenario_name}: no footprint-margin route exists'
            )


def _write_pgm(output, obstacles):
    pixels = bytearray()
    for image_row in range(HEIGHT):
        map_row = HEIGHT - 1 - image_row
        y = ORIGIN_Y + (map_row + 0.5) * RESOLUTION
        for column in range(WIDTH):
            x = ORIGIN_X + (column + 0.5) * RESOLUTION
            pixels.append(0 if _occupied(x, y, obstacles) else 254)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('wb') as stream:
        stream.write(f'P5\n{WIDTH} {HEIGHT}\n255\n'.encode('ascii'))
        stream.write(pixels)


def generate(gazebo_root, benchmark_root):
    """Write every declared environment and return generated paths."""
    generated = []
    for name, environment in ENVIRONMENTS.items():
        _validate_scenarios(name, environment)
        obstacles = (*BOUNDARIES, *environment.obstacles)

        world_path = gazebo_root / 'worlds' / f'{name}.sdf'
        world_path.parent.mkdir(parents=True, exist_ok=True)
        _world_tree(name, environment).write(
            world_path, encoding='utf-8', xml_declaration=True
        )
        generated.append(world_path)

        map_path = gazebo_root / 'maps' / f'{name}.pgm'
        _write_pgm(map_path, obstacles)
        generated.append(map_path)
        map_yaml_path = gazebo_root / 'maps' / f'{name}.yaml'
        with map_yaml_path.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(
                {
                    'image': map_path.name,
                    'mode': 'trinary',
                    'resolution': RESOLUTION,
                    'origin': [ORIGIN_X, ORIGIN_Y, 0.0],
                    'negate': 0,
                    'occupied_thresh': 0.65,
                    'free_thresh': 0.25,
                },
                stream,
                sort_keys=False,
            )
        generated.append(map_yaml_path)

        scenario_path = (
            benchmark_root / 'config' / f'{name}_scenarios.yaml'
        )
        scenario_path.parent.mkdir(parents=True, exist_ok=True)
        with scenario_path.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(
                {
                    'environment': name,
                    'description': environment.description,
                    'scenarios': [
                        {
                            'name': scenario_name,
                            'start': list(start),
                            'goal': list(goal),
                        }
                        for scenario_name, start, goal
                        in environment.scenarios
                    ],
                },
                stream,
                sort_keys=False,
                allow_unicode=True,
            )
        generated.append(scenario_path)
    return generated


def main():
    gazebo_root = Path(__file__).resolve().parents[1]
    workspace_src = gazebo_root.parent
    benchmark_root = workspace_src / 'adaptive_pivot_g2_benchmark'
    if len(sys.argv) != 1:
        raise SystemExit('This generator does not accept command-line arguments.')
    for path in generate(gazebo_root, benchmark_root):
        print(path)


if __name__ == '__main__':
    main()
