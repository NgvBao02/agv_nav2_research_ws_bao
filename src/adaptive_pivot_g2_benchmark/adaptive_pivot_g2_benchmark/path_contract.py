# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Common geometric input contract for planner and smoother comparisons."""

from copy import deepcopy
import math

from nav_msgs.msg import Path


def _yaw(pose):
    quaternion = pose.pose.orientation
    sine = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosine = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sine, cosine)


def canonicalize_planner_path(
    path: Path,
    position_tolerance: float = 1.0e-9,
    orientation_tolerance: float = 1.0e-9,
):
    """
    Remove only consecutive poses that are geometrically redundant.

    A same-position pose with a different heading is preserved because it is a
    meaningful in-place-rotation marker for a differential-drive robot.
    Returns a deep-copied path and the number of removed poses.
    """
    if position_tolerance < 0.0 or orientation_tolerance < 0.0:
        raise ValueError('path canonicalization tolerances must be non-negative')
    canonical = deepcopy(path)
    canonical.poses = []
    removed = 0
    for pose in path.poses:
        if canonical.poses:
            previous = canonical.poses[-1]
            distance = math.hypot(
                pose.pose.position.x - previous.pose.position.x,
                pose.pose.position.y - previous.pose.position.y,
            )
            heading_delta = math.atan2(
                math.sin(_yaw(pose) - _yaw(previous)),
                math.cos(_yaw(pose) - _yaw(previous)),
            )
            if (
                distance <= position_tolerance
                and abs(heading_delta) <= orientation_tolerance
            ):
                removed += 1
                continue
        canonical.poses.append(deepcopy(pose))
    if len(canonical.poses) < 2:
        raise ValueError('canonical planner path contains fewer than two poses')
    return canonical, removed
