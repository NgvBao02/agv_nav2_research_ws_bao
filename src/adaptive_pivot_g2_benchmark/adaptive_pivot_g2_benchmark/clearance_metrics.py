# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Orientation-aware full-footprint clearance metrics for static ROS maps."""

import math
from typing import Dict, List, Tuple

from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
from scipy.ndimage import distance_transform_edt


Point = Tuple[float, float]
Pose2D = Tuple[float, float, float]


def _yaw(pose) -> float:
    quaternion = pose.orientation
    sine = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosine = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sine, cosine)


def _interpolated_poses(
    path: Path,
    linear_spacing: float,
    angular_spacing: float,
) -> List[Pose2D]:
    if not path.poses:
        return []
    first = path.poses[0].pose
    result = [(first.position.x, first.position.y, _yaw(first))]
    for previous_stamped, current_stamped in zip(path.poses, path.poses[1:]):
        previous = previous_stamped.pose
        current = current_stamped.pose
        delta_x = current.position.x - previous.position.x
        delta_y = current.position.y - previous.position.y
        distance = math.hypot(delta_x, delta_y)
        previous_yaw = _yaw(previous)
        yaw_delta = math.atan2(
            math.sin(_yaw(current) - previous_yaw),
            math.cos(_yaw(current) - previous_yaw),
        )
        steps = max(
            1,
            int(math.ceil(distance / linear_spacing)),
            int(math.ceil(abs(yaw_delta) / angular_spacing)),
        )
        for index in range(1, steps + 1):
            ratio = index / steps
            result.append((
                previous.position.x + ratio * delta_x,
                previous.position.y + ratio * delta_y,
                previous_yaw + ratio * yaw_delta,
            ))
    return result


def _footprint_perimeter(
    length: float,
    width: float,
    spacing: float,
) -> List[Point]:
    half_length = 0.5 * length
    half_width = 0.5 * width
    samples = []
    count_x = max(1, int(math.ceil(length / spacing)))
    count_y = max(1, int(math.ceil(width / spacing)))
    for index in range(count_x + 1):
        x = -half_length + length * index / count_x
        samples.extend([(x, -half_width), (x, half_width)])
    for index in range(1, count_y):
        y = -half_width + width * index / count_y
        samples.extend([(-half_length, y), (half_length, y)])
    return samples


def calculate_footprint_clearance(
    path: Path,
    occupancy_grid: OccupancyGrid,
    footprint_length: float = 0.44,
    footprint_width: float = 0.34,
    path_spacing: float = 0.05,
    angular_spacing: float = math.radians(5.0),
    obstacle_threshold: int = 65,
) -> Dict[str, float]:
    """Return clearance of the swept rectangular footprint to occupied cells."""
    resolution = float(occupancy_grid.info.resolution)
    width = int(occupancy_grid.info.width)
    height = int(occupancy_grid.info.height)
    if resolution <= 0.0 or width <= 0 or height <= 0:
        raise ValueError('occupancy grid metadata is invalid')
    if footprint_length <= 0.0 or footprint_width <= 0.0:
        raise ValueError('footprint dimensions must be positive')
    if path_spacing <= 0.0 or angular_spacing <= 0.0:
        raise ValueError('clearance sampling spacing must be positive')
    data = np.asarray(occupancy_grid.data, dtype=np.int16)
    if data.size != width * height:
        raise ValueError('occupancy grid data size does not match metadata')
    occupancy = data.reshape((height, width))
    obstacle = (occupancy < 0) | (occupancy >= obstacle_threshold)
    distance_field = distance_transform_edt(~obstacle) * resolution
    origin = occupancy_grid.info.origin.position
    footprint = _footprint_perimeter(
        footprint_length, footprint_width, min(0.5 * resolution, 0.025)
    )
    pose_clearances = []
    for x, y, yaw in _interpolated_poses(
        path, path_spacing, angular_spacing
    ):
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        clearance = math.inf
        for local_x, local_y in footprint:
            world_x = x + cosine * local_x - sine * local_y
            world_y = y + sine * local_x + cosine * local_y
            column = int(math.floor((world_x - origin.x) / resolution))
            row = int(math.floor((world_y - origin.y) / resolution))
            if column < 0 or column >= width or row < 0 or row >= height:
                clearance = 0.0
                break
            cell_clearance = max(
                0.0,
                float(distance_field[row, column])
                - 0.5 * math.sqrt(2.0) * resolution,
            )
            clearance = min(clearance, cell_clearance)
        pose_clearances.append(clearance)
    if not pose_clearances:
        return {}
    values = np.asarray(pose_clearances, dtype=np.float64)
    return {
        'footprint_clearance_min_m': float(np.min(values)),
        'footprint_clearance_p05_m': float(np.percentile(values, 5.0)),
        'footprint_clearance_mean_m': float(np.mean(values)),
        'footprint_collision_sample_count': int(np.count_nonzero(values <= 0.0)),
        'footprint_pose_sample_count': int(values.size),
    }
