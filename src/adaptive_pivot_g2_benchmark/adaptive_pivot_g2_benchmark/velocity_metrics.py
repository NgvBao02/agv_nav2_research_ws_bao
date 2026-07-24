# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Command-space speed, acceleration, jerk, and wheel utilization metrics."""

import math
from statistics import fmean
from typing import Dict, List, Tuple


CommandSample = Tuple[float, float, float]
ShaperSample = Tuple[float, bool]


def _percentile(values: List[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = fraction * float(len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    ratio = position - float(lower)
    return ordered[lower] + ratio * (ordered[upper] - ordered[lower])


def calculate_velocity_metrics(
    samples: List[CommandSample],
    wheel_separation_m: float = 0.2548,
    cruise_speed_mps: float = 0.30,
) -> Dict[str, float]:
    """Calculate rate-robust dynamics metrics from timestamped Twist samples."""
    if wheel_separation_m <= 0.0 or cruise_speed_mps <= 0.0:
        raise ValueError('wheel separation and cruise speed must be positive')
    valid = [
        (float(stamp), float(linear), float(angular))
        for stamp, linear, angular in samples
        if all(math.isfinite(value) for value in (stamp, linear, angular))
    ]
    if not valid:
        return {
            'mean_abs_command_linear_mps': 0.0,
            'rms_command_linear_mps': 0.0,
            'p95_abs_command_linear_mps': 0.0,
            'max_command_linear_mps': 0.0,
            'p95_abs_command_angular_radps': 0.0,
            'max_command_angular_radps': 0.0,
            'p95_abs_command_wheel_linear_mps': 0.0,
            'max_command_wheel_linear_mps': 0.0,
            'max_abs_command_acceleration_mps2': 0.0,
            'p95_abs_command_acceleration_mps2': 0.0,
            'max_abs_command_angular_acceleration_radps2': 0.0,
            'p95_abs_command_angular_acceleration_radps2': 0.0,
            'max_abs_command_lateral_acceleration_mps2': 0.0,
            'p95_abs_command_lateral_acceleration_mps2': 0.0,
            'max_abs_command_jerk_mps3': 0.0,
            'p95_abs_command_jerk_mps3': 0.0,
            'moving_command_fraction': 0.0,
            'cruise_command_fraction': 0.0,
        }

    absolute_linear = [abs(sample[1]) for sample in valid]
    absolute_angular = [abs(sample[2]) for sample in valid]
    wheel_speeds = [
        max(
            abs(linear - 0.5 * wheel_separation_m * angular),
            abs(linear + 0.5 * wheel_separation_m * angular),
        )
        for _, linear, angular in valid
    ]
    lateral_accelerations = [
        abs(linear * angular) for _, linear, angular in valid
    ]
    accelerations: List[Tuple[float, float]] = []
    angular_accelerations: List[Tuple[float, float]] = []
    for first, last in zip(valid, valid[1:]):
        time_step = last[0] - first[0]
        if 1.0e-4 <= time_step <= 0.5:
            midpoint = 0.5 * (first[0] + last[0])
            accelerations.append(
                (midpoint, (last[1] - first[1]) / time_step)
            )
            angular_accelerations.append(
                (midpoint, (last[2] - first[2]) / time_step)
            )
    jerks: List[float] = []
    for first, last in zip(accelerations, accelerations[1:]):
        time_step = last[0] - first[0]
        if 1.0e-4 <= time_step <= 0.5:
            jerks.append((last[1] - first[1]) / time_step)
    absolute_acceleration = [abs(value[1]) for value in accelerations]
    absolute_angular_acceleration = [
        abs(value[1]) for value in angular_accelerations
    ]
    absolute_jerk = [abs(value) for value in jerks]

    return {
        'mean_abs_command_linear_mps': fmean(absolute_linear),
        'rms_command_linear_mps': math.sqrt(
            fmean(value * value for value in absolute_linear)
        ),
        'p95_abs_command_linear_mps': _percentile(absolute_linear, 0.95),
        'max_command_linear_mps': max(absolute_linear),
        'p95_abs_command_angular_radps': _percentile(
            absolute_angular, 0.95
        ),
        'max_command_angular_radps': max(absolute_angular),
        'p95_abs_command_wheel_linear_mps': _percentile(
            wheel_speeds, 0.95
        ),
        'max_command_wheel_linear_mps': max(wheel_speeds),
        'max_abs_command_acceleration_mps2': max(
            absolute_acceleration, default=0.0
        ),
        'p95_abs_command_acceleration_mps2': _percentile(
            absolute_acceleration, 0.95
        ),
        'max_abs_command_angular_acceleration_radps2': max(
            absolute_angular_acceleration, default=0.0
        ),
        'p95_abs_command_angular_acceleration_radps2': _percentile(
            absolute_angular_acceleration, 0.95
        ),
        'max_abs_command_lateral_acceleration_mps2': max(
            lateral_accelerations, default=0.0
        ),
        'p95_abs_command_lateral_acceleration_mps2': _percentile(
            lateral_accelerations, 0.95
        ),
        'max_abs_command_jerk_mps3': max(absolute_jerk, default=0.0),
        'p95_abs_command_jerk_mps3': _percentile(absolute_jerk, 0.95),
        'moving_command_fraction': sum(
            value >= 0.01 for value in absolute_linear
        ) / len(absolute_linear),
        'cruise_command_fraction': sum(
            value >= 0.90 * cruise_speed_mps for value in absolute_linear
        ) / len(absolute_linear),
    }


def calculate_shaper_metrics(
    samples: List[ShaperSample],
) -> Dict[str, float]:
    """Separate nominal jerk shaping from immediate safety overrides."""
    valid = [
        (abs(float(jerk)), bool(safety_override))
        for jerk, safety_override in samples
        if math.isfinite(float(jerk))
    ]
    nominal_jerk = [
        jerk for jerk, safety_override in valid if not safety_override
    ]
    override_jerk = [
        jerk for jerk, safety_override in valid if safety_override
    ]
    sample_count = len(valid)
    return {
        'adaptive_speed_telemetry_sample_count': sample_count,
        'adaptive_speed_nominal_jerk_sample_count': len(nominal_jerk),
        'adaptive_speed_nominal_p95_abs_jerk_mps3': _percentile(
            nominal_jerk, 0.95
        ),
        'adaptive_speed_nominal_max_abs_jerk_mps3': max(
            nominal_jerk, default=0.0
        ),
        'adaptive_speed_safety_override_fraction': (
            len(override_jerk) / sample_count if sample_count else 0.0
        ),
        'adaptive_speed_override_p95_abs_jerk_mps3': _percentile(
            override_jerk, 0.95
        ),
    }
