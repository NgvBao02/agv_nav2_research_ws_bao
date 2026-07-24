# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Localization error and covariance metrics paired with Gazebo ground truth."""

import math
from statistics import fmean
from typing import Dict, List, Sequence, Tuple


# stamp, position error, yaw error, covariance x/y/yaw
LocalizationSample = Tuple[float, float, float, float, float, float]
PoseSample = Tuple[float, float, float, float]


def _percentile(values: List[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = fraction * float(len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    ratio = position - float(lower)
    return ordered[lower] + ratio * (ordered[upper] - ordered[lower])


def calculate_localization_metrics(
    samples: List[LocalizationSample],
) -> Dict[str, float]:
    """Summarize AMCL-vs-ground-truth error without hiding invalid samples."""
    valid = [
        tuple(float(value) for value in sample)
        for sample in samples
        if len(sample) == 6
        and all(math.isfinite(float(value)) for value in sample)
        and sample[1] >= 0.0
        and sample[2] >= 0.0
    ]
    if not valid:
        return {
            'localization_sample_count': 0,
            'localization_position_error_mean_m': 0.0,
            'localization_position_error_p95_m': 0.0,
            'localization_position_error_max_m': 0.0,
            'localization_position_error_final_m': 0.0,
            'localization_yaw_error_mean_rad': 0.0,
            'localization_yaw_error_p95_rad': 0.0,
            'localization_yaw_error_max_rad': 0.0,
            'localization_yaw_error_final_rad': 0.0,
            'localization_covariance_xy_final_m2': 0.0,
            'localization_covariance_yaw_final_rad2': 0.0,
        }

    position_errors = [sample[1] for sample in valid]
    yaw_errors = [sample[2] for sample in valid]
    final = valid[-1]
    return {
        'localization_sample_count': len(valid),
        'localization_position_error_mean_m': fmean(position_errors),
        'localization_position_error_p95_m': _percentile(
            position_errors, 0.95
        ),
        'localization_position_error_max_m': max(position_errors),
        'localization_position_error_final_m': final[1],
        'localization_yaw_error_mean_rad': fmean(yaw_errors),
        'localization_yaw_error_p95_rad': _percentile(yaw_errors, 0.95),
        'localization_yaw_error_max_rad': max(yaw_errors),
        'localization_yaw_error_final_rad': final[2],
        'localization_covariance_xy_final_m2': max(final[3], final[4]),
        'localization_covariance_yaw_final_rad2': final[5],
    }


def align_odometry_trace(
    odometry_samples: Sequence[Sequence[float]],
    reference_samples: Sequence[Sequence[float]],
) -> List[PoseSample]:
    """Rigidly align wheel odometry to the first ground-truth pose."""
    if not odometry_samples or not reference_samples:
        return []
    first_odom = odometry_samples[0]
    first_reference = reference_samples[0]
    if len(first_odom) < 4 or len(first_reference) < 4:
        return []
    values = [
        float(value)
        for value in (*first_odom[:4], *first_reference[:4])
    ]
    if not all(math.isfinite(value) for value in values):
        return []
    yaw_offset = float(first_reference[3]) - float(first_odom[3])
    cosine = math.cos(yaw_offset)
    sine = math.sin(yaw_offset)
    origin_x = float(first_odom[1])
    origin_y = float(first_odom[2])
    reference_x = float(first_reference[1])
    reference_y = float(first_reference[2])
    aligned: List[PoseSample] = []
    for sample in odometry_samples:
        if len(sample) < 4:
            continue
        stamp, x, y, yaw = (float(value) for value in sample[:4])
        if not all(math.isfinite(value) for value in (stamp, x, y, yaw)):
            continue
        relative_x = x - origin_x
        relative_y = y - origin_y
        aligned.append((
            stamp,
            reference_x + cosine * relative_x - sine * relative_y,
            reference_y + sine * relative_x + cosine * relative_y,
            math.atan2(
                math.sin(yaw + yaw_offset),
                math.cos(yaw + yaw_offset),
            ),
        ))
    return aligned


def calculate_pose_trace_error_metrics(
    reference_samples: Sequence[Sequence[float]],
    estimated_samples: Sequence[Sequence[float]],
    prefix: str,
) -> Dict[str, float]:
    """Pair two timestamped pose traces and summarize SE(2) error."""
    references = [
        tuple(float(value) for value in sample[:4])
        for sample in reference_samples
        if len(sample) >= 4
        and all(math.isfinite(float(value)) for value in sample[:4])
    ]
    estimates = [
        tuple(float(value) for value in sample[:4])
        for sample in estimated_samples
        if len(sample) >= 4
        and all(math.isfinite(float(value)) for value in sample[:4])
    ]
    base = {
        f'{prefix}_sample_count': 0,
        f'{prefix}_position_error_mean_m': 0.0,
        f'{prefix}_position_error_p95_m': 0.0,
        f'{prefix}_position_error_max_m': 0.0,
        f'{prefix}_position_error_final_m': 0.0,
        f'{prefix}_yaw_error_mean_rad': 0.0,
        f'{prefix}_yaw_error_p95_rad': 0.0,
        f'{prefix}_yaw_error_max_rad': 0.0,
        f'{prefix}_yaw_error_final_rad': 0.0,
    }
    if not references or not estimates:
        return base

    position_errors: List[float] = []
    yaw_errors: List[float] = []
    reference_index = 0
    for estimate in estimates:
        while (
            reference_index + 1 < len(references)
            and abs(references[reference_index + 1][0] - estimate[0])
            <= abs(references[reference_index][0] - estimate[0])
        ):
            reference_index += 1
        reference = references[reference_index]
        position_errors.append(math.hypot(
            estimate[1] - reference[1],
            estimate[2] - reference[2],
        ))
        yaw_errors.append(abs(math.atan2(
            math.sin(estimate[3] - reference[3]),
            math.cos(estimate[3] - reference[3]),
        )))

    base.update({
        f'{prefix}_sample_count': len(position_errors),
        f'{prefix}_position_error_mean_m': fmean(position_errors),
        f'{prefix}_position_error_p95_m': _percentile(position_errors, 0.95),
        f'{prefix}_position_error_max_m': max(position_errors),
        f'{prefix}_position_error_final_m': position_errors[-1],
        f'{prefix}_yaw_error_mean_rad': fmean(yaw_errors),
        f'{prefix}_yaw_error_p95_rad': _percentile(yaw_errors, 0.95),
        f'{prefix}_yaw_error_max_rad': max(yaw_errors),
        f'{prefix}_yaw_error_final_rad': yaw_errors[-1],
    })
    return base
