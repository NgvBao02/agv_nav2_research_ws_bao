# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Resolve a reproducible, footprint-safe initial yaw for map scenarios."""

import heapq
import math
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt
import yaml


@dataclass(frozen=True)
class OccupancyMap:
    """Minimal trinary Nav2 map needed by the heading resolver."""

    width: int
    height: int
    pixels: bytes
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    negate: bool
    free_threshold: float


@dataclass(frozen=True)
class InitialHeadingResolution:
    """Selected yaw together with diagnostics explaining the choice."""

    yaw: float
    direct_yaw: float
    free_distance: float
    source: str


def _normalized_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _pgm_tokens(data: bytes) -> Tuple[List[bytes], int]:
    tokens: List[bytes] = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and chr(data[index]).isspace():
            index += 1
        if index < len(data) and data[index] == ord('#'):
            newline = data.find(b'\n', index)
            if newline < 0:
                raise ValueError('PGM comment has no terminating newline')
            index = newline + 1
            continue
        start = index
        while (
            index < len(data)
            and not chr(data[index]).isspace()
            and data[index] != ord('#')
        ):
            index += 1
        if start == index:
            raise ValueError('PGM header ended before four tokens')
        tokens.append(data[start:index])
    while index < len(data) and chr(data[index]).isspace():
        index += 1
    return tokens, index


@lru_cache(maxsize=16)
def load_occupancy_map(map_yaml: str) -> OccupancyMap:
    """Load a P5 Nav2 map without adding an image-library dependency."""
    yaml_path = Path(map_yaml).resolve()
    with yaml_path.open(encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream)
    image_path = (yaml_path.parent / str(metadata['image'])).resolve()
    raw = image_path.read_bytes()
    tokens, payload_index = _pgm_tokens(raw)
    if tokens[0] != b'P5':
        raise ValueError(f'only binary P5 maps are supported: {image_path}')
    width = int(tokens[1])
    height = int(tokens[2])
    maximum = int(tokens[3])
    if width <= 0 or height <= 0 or maximum != 255:
        raise ValueError(f'unsupported PGM geometry or depth: {image_path}')
    pixels = raw[payload_index:]
    if len(pixels) != width * height:
        raise ValueError(
            f'PGM payload has {len(pixels)} bytes, expected {width * height}'
        )
    origin = metadata.get('origin', [0.0, 0.0, 0.0])
    resolution = float(metadata['resolution'])
    free_threshold = float(metadata.get('free_thresh', 0.25))
    if (
        len(origin) < 3
        or resolution <= 0.0
        or not 0.0 <= free_threshold <= 1.0
    ):
        raise ValueError(f'invalid Nav2 map metadata: {yaml_path}')
    return OccupancyMap(
        width=width,
        height=height,
        pixels=pixels,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]),
        negate=bool(int(metadata.get('negate', 0))),
        free_threshold=free_threshold,
    )


def _world_to_cell(
    occupancy: OccupancyMap, x: float, y: float
) -> Tuple[int, int]:
    dx = x - occupancy.origin_x
    dy = y - occupancy.origin_y
    cosine = math.cos(occupancy.origin_yaw)
    sine = math.sin(occupancy.origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    column = math.floor(local_x / occupancy.resolution)
    map_row = math.floor(local_y / occupancy.resolution)
    return occupancy.height - 1 - map_row, column


def _cell_to_world(
    occupancy: OccupancyMap, row: int, column: int
) -> Tuple[float, float]:
    local_x = (column + 0.5) * occupancy.resolution
    local_y = (occupancy.height - 1 - row + 0.5) * occupancy.resolution
    cosine = math.cos(occupancy.origin_yaw)
    sine = math.sin(occupancy.origin_yaw)
    return (
        occupancy.origin_x + cosine * local_x - sine * local_y,
        occupancy.origin_y + sine * local_x + cosine * local_y,
    )


def _is_free(occupancy: OccupancyMap, x: float, y: float) -> bool:
    row, column = _world_to_cell(occupancy, x, y)
    if (
        row < 0
        or row >= occupancy.height
        or column < 0
        or column >= occupancy.width
    ):
        return False
    value = occupancy.pixels[row * occupancy.width + column]
    probability = (
        value / 255.0 if occupancy.negate else (255 - value) / 255.0
    )
    # Unknown cells are deliberately treated as occupied. Only cells below
    # Nav2's free threshold may influence an automatic spawn orientation.
    return probability <= occupancy.free_threshold


def _axis_samples(lower: float, upper: float, spacing: float) -> Iterable[float]:
    count = max(1, int(math.ceil((upper - lower) / spacing)))
    for index in range(count + 1):
        yield lower + (upper - lower) * index / count


def _footprint_samples(
    body_length: float,
    body_width: float,
    spacing: float,
) -> List[Tuple[float, float]]:
    half_length = 0.5 * body_length
    half_width = 0.5 * body_width
    return [
        (x, y)
        for x in _axis_samples(-half_length, half_length, spacing)
        for y in _axis_samples(-half_width, half_width, spacing)
    ]


def _footprint_is_free(
    occupancy: OccupancyMap,
    center_x: float,
    center_y: float,
    yaw: float,
    footprint_samples: Sequence[Tuple[float, float]],
) -> bool:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return all(
        _is_free(
            occupancy,
            center_x + cosine * local_x - sine * local_y,
            center_y + sine * local_x + cosine * local_y,
        )
        for local_x, local_y in footprint_samples
    )


def _free_probe_distance(
    occupancy: OccupancyMap,
    start_x: float,
    start_y: float,
    yaw: float,
    max_probe_distance: float,
    distance_step: float,
    footprint_samples: Sequence[Tuple[float, float]],
) -> float:
    distance = 0.0
    last_free = 0.0
    while distance <= max_probe_distance + 1.0e-12:
        if not _footprint_is_free(
            occupancy,
            start_x + distance * math.cos(yaw),
            start_y + distance * math.sin(yaw),
            yaw,
            footprint_samples,
        ):
            return last_free
        last_free = distance
        distance += distance_step
    return max_probe_distance


def _configuration_clearance(occupancy: OccupancyMap) -> np.ndarray:
    pixels = np.frombuffer(occupancy.pixels, dtype=np.uint8).reshape(
        occupancy.height, occupancy.width
    )
    probabilities = (
        pixels.astype(float) / 255.0
        if occupancy.negate
        else (255.0 - pixels.astype(float)) / 255.0
    )
    free = probabilities <= occupancy.free_threshold
    return distance_transform_edt(free) * occupancy.resolution


def _grid_route(
    occupancy: OccupancyMap,
    start: Tuple[float, float],
    goal: Tuple[float, float],
    minimum_clearance: float,
) -> List[Tuple[int, int]]:
    start_cell = _world_to_cell(occupancy, *start)
    goal_cell = _world_to_cell(occupancy, *goal)
    for row, column in (start_cell, goal_cell):
        if (
            row < 0
            or row >= occupancy.height
            or column < 0
            or column >= occupancy.width
        ):
            return []
    clearance = _configuration_clearance(occupancy)
    traversable = clearance >= minimum_clearance
    traversable[start_cell] = True
    traversable[goal_cell] = True
    neighbours = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
    )
    frontier = [(0.0, start_cell)]
    costs = {start_cell: 0.0}
    parents = {}
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal_cell:
            route = [current]
            while current != start_cell:
                current = parents[current]
                route.append(current)
            route.reverse()
            return route
        row, column = current
        current_cost = costs[current]
        for row_step, column_step, step_cost in neighbours:
            next_row = row + row_step
            next_column = column + column_step
            if (
                next_row < 0
                or next_row >= occupancy.height
                or next_column < 0
                or next_column >= occupancy.width
                or not traversable[next_row, next_column]
            ):
                continue
            if row_step != 0 and column_step != 0:
                if (
                    not traversable[row + row_step, column]
                    or not traversable[row, column + column_step]
                ):
                    continue
            local_clearance = max(
                occupancy.resolution, clearance[next_row, next_column]
            )
            proximity_cost = 0.03 / local_clearance
            candidate_cost = current_cost + step_cost + proximity_cost
            neighbour = (next_row, next_column)
            if candidate_cost >= costs.get(neighbour, math.inf):
                continue
            costs[neighbour] = candidate_cost
            parents[neighbour] = current
            heuristic = math.hypot(
                goal_cell[0] - next_row, goal_cell[1] - next_column
            )
            heapq.heappush(
                frontier, (candidate_cost + heuristic, neighbour)
            )
    return []


def _route_initial_yaw(
    occupancy: OccupancyMap,
    route: Sequence[Tuple[int, int]],
    start_x: float,
    start_y: float,
    lookahead_distance: float,
) -> float:
    if len(route) < 2:
        raise ValueError('grid route must contain at least two cells')
    points = [(start_x, start_y)]
    points.extend(_cell_to_world(occupancy, *cell) for cell in route[1:])
    distance = 0.0
    previous = points[0]
    target = points[-1]
    for point in points[1:]:
        segment = math.hypot(point[0] - previous[0], point[1] - previous[1])
        if segment <= 1.0e-12:
            previous = point
            continue
        if distance + segment >= lookahead_distance:
            ratio = (lookahead_distance - distance) / segment
            target = (
                previous[0] + ratio * (point[0] - previous[0]),
                previous[1] + ratio * (point[1] - previous[1]),
            )
            break
        distance += segment
        previous = point
    return math.atan2(target[1] - start_y, target[0] - start_x)


def resolve_initial_heading(
    start: Sequence[float],
    goal: Sequence[float],
    map_yaml: Path,
    body_length: float = 0.44,
    body_width: float = 0.34,
    max_probe_distance: float = 1.00,
    angular_step: float = math.radians(5.0),
) -> InitialHeadingResolution:
    """Choose a forward, footprint-safe yaw instead of a blind goal bearing."""
    if len(start) < 2 or len(goal) < 2:
        raise ValueError('start and goal must contain x and y')
    values = (
        float(start[0]), float(start[1]), float(goal[0]), float(goal[1]),
        body_length, body_width, max_probe_distance, angular_step,
    )
    if (
        not all(math.isfinite(value) for value in values)
        or body_length <= 0.0
        or body_width <= 0.0
        or max_probe_distance <= 0.0
        or angular_step <= 0.0
        or angular_step > math.pi
    ):
        raise ValueError('initial-heading inputs must be finite and positive')
    start_x, start_y, goal_x, goal_y = values[:4]
    direct_yaw = math.atan2(goal_y - start_y, goal_x - start_x)
    occupancy = load_occupancy_map(str(Path(map_yaml).resolve()))
    sample_spacing = min(0.5 * occupancy.resolution, 0.025)
    distance_step = min(occupancy.resolution, 0.05)
    footprint_samples = _footprint_samples(
        body_length, body_width, sample_spacing
    )
    direct_distance = math.hypot(goal_x - start_x, goal_y - start_y)
    if direct_distance > 1.0e-12:
        direct_free_distance = _free_probe_distance(
            occupancy,
            start_x,
            start_y,
            direct_yaw,
            direct_distance,
            distance_step,
            footprint_samples,
        )
        if direct_free_distance >= direct_distance - distance_step:
            return InitialHeadingResolution(
                yaw=direct_yaw,
                direct_yaw=direct_yaw,
                free_distance=min(direct_distance, max_probe_distance),
                source='map_aware_direct_line_of_sight',
            )
    route = _grid_route(
        occupancy,
        (start_x, start_y),
        (goal_x, goal_y),
        minimum_clearance=0.5 * body_width + 0.5 * occupancy.resolution,
    )
    if len(route) >= 2:
        route_yaw = _route_initial_yaw(
            occupancy,
            route,
            start_x,
            start_y,
            lookahead_distance=min(0.50, max_probe_distance),
        )
        route_free_distance = _free_probe_distance(
            occupancy,
            start_x,
            start_y,
            route_yaw,
            max_probe_distance,
            distance_step,
            footprint_samples,
        )
        if route_free_distance >= min(0.35, max_probe_distance):
            return InitialHeadingResolution(
                yaw=route_yaw,
                direct_yaw=direct_yaw,
                free_distance=route_free_distance,
                source='map_aware_grid_route',
            )

    candidate_count = max(1, int(math.ceil(math.pi / angular_step)))
    offsets = [0.0]
    for index in range(1, candidate_count + 1):
        offset = min(math.pi, index * angular_step)
        offsets.extend((offset, -offset))
    candidates = []
    seen = set()
    for offset in offsets:
        yaw = _normalized_angle(direct_yaw + offset)
        key = round(yaw, 12)
        if key in seen:
            continue
        seen.add(key)
        free_distance = _free_probe_distance(
            occupancy,
            start_x,
            start_y,
            yaw,
            max_probe_distance,
            distance_step,
            footprint_samples,
        )
        progress = math.cos(_normalized_angle(yaw - direct_yaw))
        # Free swept distance is primary. Progress and the small turn penalty
        # make ties deterministic and avoid selecting a backward corridor when
        # a forward alternative has the same clearance.
        score = (
            free_distance
            + 0.25 * max_probe_distance * progress
            - 0.02 * abs(_normalized_angle(yaw - direct_yaw)) / math.pi
        )
        candidates.append((score, free_distance, progress, -abs(offset), yaw))

    forward = [candidate for candidate in candidates if candidate[2] >= 0.0]
    selected = max(forward or candidates)
    return InitialHeadingResolution(
        yaw=selected[4],
        direct_yaw=direct_yaw,
        free_distance=selected[1],
        source='map_aware_footprint_probe',
    )


def resolve_scenario_start_heading(
    scenario: Dict,
    environment: str,
    map_directory: Path,
) -> InitialHeadingResolution:
    """Preserve explicit yaw and resolve only underspecified scenarios."""
    start = scenario['start']
    goal = scenario['goal']
    direct_yaw = math.atan2(
        float(goal[1]) - float(start[1]),
        float(goal[0]) - float(start[0]),
    )
    if len(start) > 2:
        return InitialHeadingResolution(
            yaw=float(start[2]),
            direct_yaw=direct_yaw,
            free_distance=0.0,
            source='scenario_explicit',
        )
    map_yaml = Path(map_directory) / f'{environment}.yaml'
    try:
        return resolve_initial_heading(start, goal, map_yaml)
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError):
        return InitialHeadingResolution(
            yaw=direct_yaw,
            direct_yaw=direct_yaw,
            free_distance=0.0,
            source='direct_bearing_fallback',
        )
