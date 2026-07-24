# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Common geometric input contract for planner and smoother comparisons."""

from copy import deepcopy
import math

from nav_msgs.msg import Path


def anchor_path_goal(
    path: Path,
    goal_pose,
    maximum_adjustment: float = 0.08,
):
    """
    Restore the exact requested goal after grid-based planning/smoothing.

    Grid planners commonly return the centre of the accepted goal cell.  The
    FollowPath action has no separate goal pose, so leaving that quantisation
    in the path silently changes the physical destination.  Large corrections
    are rejected because they may indicate an approximate or unsafe plan.
    """
    if not math.isfinite(maximum_adjustment) or maximum_adjustment < 0.0:
        raise ValueError('maximum goal adjustment must be finite and non-negative')
    if len(path.poses) < 2:
        raise ValueError('cannot anchor a path with fewer than two poses')
    path_frame = path.header.frame_id or path.poses[-1].header.frame_id
    goal_frame = goal_pose.header.frame_id
    if path_frame and goal_frame and path_frame != goal_frame:
        raise ValueError(
            f'path/goal frame mismatch: {path_frame!r} != {goal_frame!r}'
        )
    endpoint = path.poses[-1].pose.position
    goal = goal_pose.pose.position
    adjustment = math.hypot(goal.x - endpoint.x, goal.y - endpoint.y)
    if not math.isfinite(adjustment):
        raise ValueError('path goal adjustment is non-finite')
    if adjustment > maximum_adjustment:
        raise ValueError(
            f'planner endpoint is {adjustment:.3f} m from requested goal '
            f'(limit {maximum_adjustment:.3f} m)'
        )
    anchored = deepcopy(path)
    anchored.poses[-1].pose = deepcopy(goal_pose.pose)
    if goal_frame:
        anchored.poses[-1].header.frame_id = goal_frame
    return anchored, adjustment


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
