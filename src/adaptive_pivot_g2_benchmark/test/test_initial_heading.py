# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Tests for the map-aware default spawn orientation."""

import math
from pathlib import Path

from adaptive_pivot_g2_benchmark.initial_heading import (
    resolve_initial_heading,
    resolve_scenario_start_heading,
)


def _write_map(tmp_path: Path, occupied_cells):
    width = 120
    height = 80
    pixels = bytearray([255] * (width * height))
    for column, map_row in occupied_cells:
        image_row = height - 1 - map_row
        pixels[image_row * width + column] = 0
    pgm = tmp_path / 'synthetic.pgm'
    pgm.write_bytes(
        f'P5\n{width} {height}\n255\n'.encode('ascii') + bytes(pixels)
    )
    yaml_path = tmp_path / 'synthetic.yaml'
    yaml_path.write_text(
        'image: synthetic.pgm\n'
        'mode: trinary\n'
        'resolution: 0.05\n'
        'origin: [0.0, 0.0, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n',
        encoding='utf-8',
    )
    return yaml_path


def test_resolver_avoids_blocked_direct_bearing(tmp_path):
    wall = [
        (column, row)
        for column in range(29, 34)
        for row in range(31, 50)
    ]
    map_yaml = _write_map(tmp_path, wall)
    result = resolve_initial_heading(
        [1.0, 2.0], [5.0, 2.0], map_yaml, max_probe_distance=1.0
    )
    assert abs(result.yaw) >= math.radians(30.0)
    assert math.cos(result.yaw) >= 0.0
    assert result.free_distance >= 0.90
    assert result.source == 'map_aware_grid_route'


def test_resolver_keeps_clear_direct_bearing(tmp_path):
    map_yaml = _write_map(tmp_path, [])
    result = resolve_initial_heading([1.0, 2.0], [5.0, 2.0], map_yaml)
    assert math.isclose(result.yaw, 0.0, abs_tol=1.0e-12)
    assert math.isclose(result.free_distance, 1.0)
    assert result.source == 'map_aware_direct_line_of_sight'


def test_explicit_scenario_yaw_is_never_replaced(tmp_path):
    scenario = {'start': [1.0, 2.0, -0.7], 'goal': [5.0, 2.0]}
    result = resolve_scenario_start_heading(
        scenario, 'missing_environment', tmp_path
    )
    assert math.isclose(result.yaw, -0.7)
    assert result.source == 'scenario_explicit'


def test_missing_map_has_deterministic_direct_fallback(tmp_path):
    scenario = {'start': [1.0, 2.0], 'goal': [2.0, 3.0]}
    result = resolve_scenario_start_heading(
        scenario, 'missing_environment', tmp_path
    )
    assert math.isclose(result.yaw, math.pi / 4.0)
    assert result.source == 'direct_bearing_fallback'
