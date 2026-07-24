# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Ground-truth curve and curve-exit tracking metrics."""

import math
from statistics import fmean
from typing import Dict, List, Sequence, Tuple

from adaptive_pivot_g2_benchmark.compare_paths import resample_polyline


Point = Tuple[float, float]


def _percentile(values: List[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = fraction * float(len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    ratio = position - float(lower)
    return ordered[lower] + ratio * (ordered[upper] - ordered[lower])


def _path_geometry(
    reference_points: Sequence[Point],
    spacing: float,
    curvature_window: float,
):
    points = resample_polyline(reference_points, spacing)
    if len(points) < 3:
        return points, [], []
    distances = [0.0]
    for first, last in zip(points, points[1:]):
        distances.append(
            distances[-1] + math.hypot(
                last[0] - first[0], last[1] - first[1]
            )
        )
    stride = max(1, int(round(curvature_window / spacing)))
    curvatures = [0.0] * len(points)
    for index in range(stride, len(points) - stride):
        first = points[index - stride]
        middle = points[index]
        last = points[index + stride]
        side_a = math.hypot(
            middle[0] - first[0], middle[1] - first[1]
        )
        side_b = math.hypot(last[0] - middle[0], last[1] - middle[1])
        chord = math.hypot(last[0] - first[0], last[1] - first[1])
        denominator = side_a * side_b * chord
        if denominator <= 1.0e-12:
            continue
        cross = (
            (middle[0] - first[0]) * (last[1] - first[1])
            - (middle[1] - first[1]) * (last[0] - first[0])
        )
        curvatures[index] = 2.0 * cross / denominator
    if len(points) > 2 * stride:
        for index in range(stride):
            curvatures[index] = curvatures[stride]
            curvatures[-index - 1] = curvatures[-stride - 1]
    return points, distances, curvatures


def _project_to_path(
    point: Point,
    path: Sequence[Point],
    distances: Sequence[float],
    minimum_progress: float,
) -> Tuple[float, float]:
    best_error = math.inf
    best_progress = minimum_progress
    for index, (first, last) in enumerate(zip(path, path[1:])):
        segment_length = distances[index + 1] - distances[index]
        if distances[index + 1] < minimum_progress - 0.03:
            continue
        delta_x = last[0] - first[0]
        delta_y = last[1] - first[1]
        squared_length = delta_x * delta_x + delta_y * delta_y
        if squared_length <= 1.0e-18:
            continue
        ratio = (
            (point[0] - first[0]) * delta_x
            + (point[1] - first[1]) * delta_y
        ) / squared_length
        ratio = max(0.0, min(1.0, ratio))
        progress = distances[index] + ratio * segment_length
        if progress < minimum_progress - 0.03:
            continue
        projected_x = first[0] + ratio * delta_x
        projected_y = first[1] + ratio * delta_y
        error = math.hypot(
            point[0] - projected_x, point[1] - projected_y
        )
        if (
            error < best_error - 1.0e-12
            or (
                abs(error - best_error) <= 1.0e-12
                and progress > best_progress
            )
        ):
            best_error = error
            best_progress = progress
    return best_progress, best_error


def calculate_curve_exit_metrics(
    reference_points: Sequence[Point],
    ground_truth_state_samples: Sequence[Sequence[float]],
    curvature_threshold: float = 0.40,
    post_curve_distance: float = 0.50,
    spacing: float = 0.025,
    curvature_window: float = 0.10,
) -> Dict[str, float]:
    """Measure physical tracking in curves and over 0.5 m after each exit."""
    if (
        curvature_threshold <= 0.0
        or post_curve_distance <= 0.0
        or spacing <= 0.0
        or curvature_window <= 0.0
    ):
        raise ValueError('curve metric parameters must be positive')
    path, distances, curvatures = _path_geometry(
        reference_points, spacing, curvature_window
    )
    base = {
        'planned_curve_exit_count': 0,
        'curve_tracking_sample_count': 0,
        'curve_tracking_rmse_m': 0.0,
        'curve_tracking_p95_m': 0.0,
        'curve_tracking_max_error_m': 0.0,
        'curve_exit_sample_count': 0,
        'curve_exit_tracking_rmse_m': 0.0,
        'curve_exit_tracking_p95_m': 0.0,
        'curve_exit_tracking_max_error_m': 0.0,
        'curve_exit_mean_abs_linear_mps': 0.0,
        'curve_exit_max_abs_linear_mps': 0.0,
        'curve_exit_max_abs_angular_radps': 0.0,
        'curve_exit_recovery_distance_m': post_curve_distance,
    }
    if len(path) < 2 or not ground_truth_state_samples:
        return base

    exit_distances: List[float] = []
    for index in range(1, len(curvatures)):
        if (
            abs(curvatures[index - 1]) >= curvature_threshold
            and abs(curvatures[index]) < curvature_threshold
            and (
                not exit_distances
                or distances[index] - exit_distances[-1]
                > 0.5 * curvature_window
            )
        ):
            exit_distances.append(distances[index])
    base['planned_curve_exit_count'] = len(exit_distances)

    progress = 0.0
    curve_errors: List[float] = []
    exit_errors: List[float] = []
    exit_linear_speeds: List[float] = []
    exit_angular_speeds: List[float] = []
    for sample in ground_truth_state_samples:
        if (
            len(sample) < 6
            or not all(math.isfinite(float(value)) for value in sample[:6])
        ):
            continue
        progress, error = _project_to_path(
            (float(sample[1]), float(sample[2])),
            path,
            distances,
            progress,
        )
        if not math.isfinite(error):
            continue
        curvature_index = min(
            range(len(distances)),
            key=lambda index: abs(distances[index] - progress),
        )
        if abs(curvatures[curvature_index]) >= curvature_threshold:
            curve_errors.append(error)
        if any(
            exit_distance <= progress
            <= exit_distance + post_curve_distance
            for exit_distance in exit_distances
        ):
            exit_errors.append(error)
            exit_linear_speeds.append(abs(float(sample[4])))
            exit_angular_speeds.append(abs(float(sample[5])))

    if curve_errors:
        base.update({
            'curve_tracking_sample_count': len(curve_errors),
            'curve_tracking_rmse_m': math.sqrt(
                fmean([error * error for error in curve_errors])
            ),
            'curve_tracking_p95_m': _percentile(curve_errors, 0.95),
            'curve_tracking_max_error_m': max(curve_errors),
        })
    if exit_errors:
        base.update({
            'curve_exit_sample_count': len(exit_errors),
            'curve_exit_tracking_rmse_m': math.sqrt(
                fmean([error * error for error in exit_errors])
            ),
            'curve_exit_tracking_p95_m': _percentile(exit_errors, 0.95),
            'curve_exit_tracking_max_error_m': max(exit_errors),
            'curve_exit_mean_abs_linear_mps': fmean(exit_linear_speeds),
            'curve_exit_max_abs_linear_mps': max(exit_linear_speeds),
            'curve_exit_max_abs_angular_radps': max(exit_angular_speeds),
        })
    return base
