# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Run isolated Gazebo/Nav2 execution trials for every comparison method."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory


DEFAULT_METHODS = [
    'raw',
    'simple',
    'savitzky_golay',
    'constrained',
    'pstmo',
    'adaptive_hybrid',
]

INFRASTRUCTURE_ERROR_FRAGMENTS = (
    'nav2 did not reach the fully active state',
    'trial did not produce a result file',
    'gazebo ground-truth pose did not become available',
)

SUMMARY_OMITTED_FIELDS = {
    'aligned_odometry_state_trace',
    'command_velocity_trace',
    'controller_command_velocity_trace',
    'estimated_map_state_trace',
    'executed_path_xy',
    'ground_truth_state_trace',
    'localization_error_trace',
    'odometry_state_trace',
    'selected_path_poses',
    'selected_path_xy',
}

AGGREGATE_METRICS = [
    'execution_time_s',
    'controller_action_time_s',
    'physical_settle_time_s',
    'tracking_rmse_m',
    'tracking_max_error_m',
    'curve_tracking_rmse_m',
    'curve_tracking_p95_m',
    'curve_tracking_max_error_m',
    'curve_exit_tracking_rmse_m',
    'curve_exit_tracking_p95_m',
    'curve_exit_tracking_max_error_m',
    'curve_exit_mean_abs_linear_mps',
    'curve_exit_max_abs_linear_mps',
    'final_position_error_m',
    'final_yaw_error_rad',
    'final_estimated_position_error_m',
    'final_estimated_yaw_error_rad',
    'action_completion_position_error_m',
    'action_completion_yaw_error_rad',
    'action_completion_command_linear_mps',
    'action_completion_command_angular_radps',
    'action_completion_actual_linear_mps',
    'action_completion_actual_angular_radps',
    'post_action_travel_m',
    'post_action_yaw_change_rad',
    'post_action_max_command_linear_mps',
    'post_action_max_command_angular_radps',
    'post_action_max_actual_linear_mps',
    'post_action_max_actual_angular_radps',
    'nearest_ground_truth_goal_distance_m',
    'post_nearest_goal_travel_m',
    'final_actual_linear_mps',
    'final_actual_angular_radps',
    'traveled_distance_m',
    'executed_curvature_energy_1pm',
    'stopped_command_fraction',
    'mean_abs_command_linear_mps',
    'p95_abs_command_linear_mps',
    'max_command_linear_mps',
    'p95_abs_command_angular_radps',
    'max_command_angular_radps',
    'p95_abs_command_wheel_linear_mps',
    'max_command_wheel_linear_mps',
    'p95_abs_command_acceleration_mps2',
    'p95_abs_command_angular_acceleration_radps2',
    'p95_abs_command_lateral_acceleration_mps2',
    'p95_abs_command_jerk_mps3',
    'cruise_command_fraction',
    'actual_mean_abs_linear_mps',
    'actual_max_linear_mps',
    'actual_p95_abs_linear_mps',
    'actual_p95_abs_angular_radps',
    'actual_max_angular_radps',
    'actual_p95_abs_wheel_linear_mps',
    'actual_max_wheel_linear_mps',
    'actual_p95_abs_acceleration_mps2',
    'actual_p95_abs_angular_acceleration_radps2',
    'actual_p95_abs_lateral_acceleration_mps2',
    'actual_p95_abs_jerk_mps3',
    'controller_p95_abs_linear_mps',
    'controller_p95_abs_angular_radps',
    'controller_p95_abs_wheel_linear_mps',
    'controller_p95_abs_acceleration_mps2',
    'controller_p95_abs_angular_acceleration_radps2',
    'controller_p95_abs_lateral_acceleration_mps2',
    'controller_p95_abs_jerk_mps3',
    'collision_monitor_interventions',
    'planner_start_anchor_adjustment_m',
    'planner_goal_anchor_adjustment_m',
    'selected_start_anchor_adjustment_m',
    'selected_goal_anchor_adjustment_m',
    'localization_position_error_mean_m',
    'localization_position_error_p95_m',
    'localization_position_error_max_m',
    'localization_position_error_final_m',
    'localization_yaw_error_p95_rad',
    'localization_yaw_error_max_rad',
    'estimated_tracking_rmse_m',
    'estimated_tracking_max_error_m',
    'odometry_tracking_rmse_m',
    'odometry_tracking_max_error_m',
    'odometry_position_error_mean_m',
    'odometry_position_error_p95_m',
    'odometry_position_error_max_m',
    'estimated_pose_position_error_mean_m',
    'estimated_pose_position_error_p95_m',
    'estimated_pose_position_error_max_m',
]


def _run_launch(command, environment, timeout, log_path=None):
    """Run one isolated launch while retaining evidence for failed trials."""
    log_stream = (
        log_path.open('a', encoding='utf-8')
        if log_path is not None else open(os.devnull, 'w', encoding='utf-8')
    )
    process = subprocess.Popen(
        command,
        env=environment,
        start_new_session=True,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
    )
    try:
        return process, process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        return process, 124
    finally:
        log_stream.flush()
        log_stream.close()


def _process_group_exists(process_group_id):
    try:
        os.killpg(process_group_id, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _terminate_trial_process_group(process):
    """Terminate only children in the dedicated session made for this trial."""
    process_group_id = process.pid
    for requested_signal, grace_period in (
        (signal.SIGINT, 2.0),
        (signal.SIGTERM, 2.0),
        (signal.SIGKILL, 0.5),
    ):
        if not _process_group_exists(process_group_id):
            break
        try:
            os.killpg(process_group_id, requested_signal)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + grace_period
        while (
            _process_group_exists(process_group_id)
            and time.monotonic() < deadline
        ):
            process.poll()
            time.sleep(0.05)
    try:
        process.wait(timeout=0.5)
    except subprocess.TimeoutExpired:
        pass


def _stop_gazebo_server(environment):
    """Stop a detached Gazebo server inside this trial's private partition."""
    command = [
        'gz', 'service', '-s', '/server_control',
        '--reqtype', 'gz.msgs.ServerControl',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000', '--req', 'stop: true',
    ]
    try:
        subprocess.run(
            command,
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5.0,
            check=False,
        )
    except subprocess.TimeoutExpired:
        pass


def _arguments(args: Optional[List[str]] = None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--scenario-file',
        help=(
            'Scenario YAML to pass to each isolated trial. The file also '
            'selects its Gazebo environment via the top-level environment key.'
        ),
    )
    parser.add_argument('--scenario', default='lower_left_diagonal')
    parser.add_argument('--planner', default='ThetaStar')
    parser.add_argument(
        '--planners',
        nargs='+',
        help=(
            'Run multiple planner IDs. When supplied, this replaces the '
            'single --planner value.'
        ),
    )
    parser.add_argument('--methods', nargs='+', default=DEFAULT_METHODS)
    parser.add_argument('--output-dir', default='results/execution_matrix')
    parser.add_argument('--base-domain-id', type=int, default=120)
    parser.add_argument('--trial-timeout-s', type=float, default=240.0)
    parser.add_argument('--repetitions', type=int, default=1)
    parser.add_argument(
        '--resume',
        action='store_true',
        help=(
            'Reuse matching successful per-trial JSON files. Failed, corrupt, '
            'or configuration-mismatched trials are rerun.'
        ),
    )
    parser.add_argument(
        '--infrastructure-retries',
        type=int,
        default=1,
        help=(
            'Retry transient Gazebo/Nav2 setup failures this many times. '
            'Experimental controller failures are never retried.'
        ),
    )
    return parser.parse_args(args)


def _configuration_sha256(scenario_file):
    """Fingerprint the exact Nav2 parameters and scenario used by a matrix."""
    nav2_parameters = (
        Path(get_package_share_directory('vacuum_robot_gazebo')) /
        'config' / 'nav2_params.yaml'
    )
    if scenario_file:
        scenario_path = Path(scenario_file).resolve()
    else:
        scenario_path = (
            Path(get_package_share_directory('adaptive_pivot_g2_benchmark')) /
            'config' / 'research_scenarios.yaml'
        )
    digest = hashlib.sha256()
    for label, path in (
        ('nav2_params', nav2_parameters),
        ('scenario', scenario_path),
    ):
        digest.update(label.encode('utf-8'))
        digest.update(b'\0')
        digest.update(path.read_bytes())
        digest.update(b'\0')
    return digest.hexdigest()


def _matching_successful_record(
    result_path,
    scenario,
    planner,
    method,
    repetition,
    configuration_sha256=None,
):
    if not result_path.exists():
        return None
    try:
        with result_path.open(encoding='utf-8') as stream:
            record = json.load(stream)
    except (OSError, ValueError, TypeError):
        return None
    try:
        recorded_repetition = int(record.get('repetition', -1))
    except (TypeError, ValueError):
        return None
    if not (
        record.get('success', False)
        and record.get('scenario') == scenario
        and record.get('planner') == planner
        and record.get('method') == method
        and recorded_repetition == repetition
        and (
            configuration_sha256 is None or
            record.get('configuration_sha256') == configuration_sha256
        )
    ):
        return None
    record['repetition'] = repetition
    record['resumed'] = True
    return record


def _write_json(path, document):
    temporary_path = path.with_suffix(path.suffix + '.tmp')
    with temporary_path.open('w', encoding='utf-8') as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write('\n')
    temporary_path.replace(path)


def _is_infrastructure_failure(record):
    """Recognize setup failures that do not measure algorithm performance."""
    if record.get('success', False):
        return False
    error = str(record.get('error', '')).lower()
    return any(fragment in error for fragment in INFRASTRUCTURE_ERROR_FRAGMENTS)


def _compact_summary_record(record):
    """Remove duplicated high-rate traces from the matrix-level JSON."""
    compact = {
        key: value for key, value in record.items()
        if key not in SUMMARY_OMITTED_FIELDS
    }
    compact['omitted_trace_fields'] = sorted(
        key for key in record if key in SUMMARY_OMITTED_FIELDS
    )
    return compact


def _aggregate(records, methods):
    aggregates = {}
    for method in methods:
        method_records = [
            record for record in records if record.get('method') == method
        ]
        summary = {
            'trial_count': len(method_records),
            'success_count': sum(
                bool(record.get('success', False))
                for record in method_records
            ),
        }
        summary['success_rate'] = (
            summary['success_count'] / summary['trial_count']
            if summary['trial_count'] else 0.0
        )
        for metric in AGGREGATE_METRICS:
            values = [
                float(record[metric])
                for record in method_records
                if record.get('success', False)
                and isinstance(record.get(metric), (int, float))
                and math.isfinite(float(record[metric]))
            ]
            if not values:
                continue
            summary[f'{metric}_sample_count'] = len(values)
            summary[f'{metric}_mean'] = statistics.mean(values)
            summary[f'{metric}_stdev'] = (
                statistics.stdev(values) if len(values) > 1 else 0.0
            )
            summary[f'{metric}_min'] = min(values)
            summary[f'{metric}_max'] = max(values)
        aggregates[method] = summary
    return aggregates


def main(args: Optional[List[str]] = None) -> None:
    """Run each method in a clean process and aggregate its trial JSON."""
    options = _arguments(args)
    planners = options.planners or [options.planner]
    if len(set(planners)) != len(planners):
        raise ValueError('planner IDs must be unique')
    unknown = sorted(set(options.methods) - set(DEFAULT_METHODS))
    if unknown:
        raise ValueError(f'unknown methods: {unknown}')
    if options.repetitions < 1:
        raise ValueError('repetitions must be at least one')
    if options.infrastructure_retries < 0:
        raise ValueError('infrastructure retries must be non-negative')
    trial_count = (
        len(planners) * len(options.methods) * options.repetitions
    )
    if options.base_domain_id < 0 or options.base_domain_id + trial_count - 1 > 232:
        raise ValueError('execution-matrix ROS domain IDs must be within 0..232')
    output_dir = Path(options.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    configuration_sha256 = _configuration_sha256(options.scenario_file)
    records = []
    trial_index = 0
    for planner in planners:
        planner_slug = ''.join(
            character.lower() if character.isalnum() else '_'
            for character in planner
        ).strip('_')
        for method in options.methods:
            for repetition in range(1, options.repetitions + 1):
                suffix = (
                    f'_r{repetition:02d}'
                    if options.repetitions > 1 else ''
                )
                planner_prefix = (
                    f'{planner_slug}_' if len(planners) > 1 else ''
                )
                result_path = output_dir / (
                    f'{options.scenario}_{planner_prefix}'
                    f'{method}{suffix}.json'
                )
                resumed_record = (
                    _matching_successful_record(
                        result_path,
                        options.scenario,
                        planner,
                        method,
                        repetition,
                        configuration_sha256,
                    )
                    if options.resume else None
                )
                if resumed_record is not None:
                    records.append(resumed_record)
                    trial_index += 1
                    print(
                        f'[{trial_index}/{trial_count}] resumed '
                        f'{planner} / {method} '
                        f'(repetition {repetition}/'
                        f'{options.repetitions})',
                        flush=True,
                    )
                    continue
                environment = os.environ.copy()
                domain_id = options.base_domain_id + trial_index
                environment['ROS_DOMAIN_ID'] = str(domain_id)
                environment['GZ_PARTITION'] = (
                    f'pivot_matrix_{os.getpid()}_{domain_id}'
                )
                command = [
                    'ros2', 'launch', 'adaptive_pivot_g2_benchmark',
                    'execution_trial.launch.py',
                    f'scenario:={options.scenario}',
                    f'method:={method}',
                    f'planner:={planner}',
                    f'output_json:={result_path}',
                    'gui:=false',
                ]
                if options.scenario_file:
                    command.append(
                        'scenario_file:='
                        f'{Path(options.scenario_file).resolve()}'
                    )
                print(
                    f'[{trial_index + 1}/{trial_count}] running '
                    f'{planner} / {method} '
                    f'(repetition {repetition}/'
                    f'{options.repetitions})',
                    flush=True,
                )
                trial_started = time.monotonic()
                attempt_wall_time = 0.0
                record = {}
                return_code = 1
                infrastructure_attempt = 0
                for infrastructure_attempt in range(
                    1, options.infrastructure_retries + 2
                ):
                    result_path.unlink(missing_ok=True)
                    log_path = result_path.with_suffix('.log')
                    if infrastructure_attempt == 1:
                        log_path.unlink(missing_ok=True)
                    with log_path.open('a', encoding='utf-8') as log_stream:
                        log_stream.write(
                            f'=== infrastructure attempt '
                            f'{infrastructure_attempt} ===\n'
                        )
                    attempt_started = time.monotonic()
                    process, return_code = _run_launch(
                        command, environment, options.trial_timeout_s,
                        log_path,
                    )
                    _stop_gazebo_server(environment)
                    _terminate_trial_process_group(process)
                    attempt_wall_time = time.monotonic() - attempt_started
                    if result_path.exists():
                        with result_path.open(encoding='utf-8') as stream:
                            record = json.load(stream)
                    else:
                        record = {
                            'scenario': options.scenario,
                            'planner': planner,
                            'method': method,
                            'success': False,
                            'error': 'trial did not produce a result file',
                        }
                    if (
                        _is_infrastructure_failure(record) and
                        infrastructure_attempt <=
                        options.infrastructure_retries
                    ):
                        print(
                            '    transient setup failure; retrying '
                            f'({infrastructure_attempt}/'
                            f'{options.infrastructure_retries})',
                            flush=True,
                        )
                        continue
                    break
                wall_time = time.monotonic() - trial_started
                record['repetition'] = repetition
                record['configuration_sha256'] = configuration_sha256
                record['launch_return_code'] = return_code
                record['trial_wall_time_s'] = wall_time
                record['last_attempt_wall_time_s'] = attempt_wall_time
                record['infrastructure_attempt_count'] = (
                    infrastructure_attempt
                )
                record['trial_log'] = str(log_path)
                record['resumed'] = False
                _write_json(result_path, record)
                records.append(record)
                trial_index += 1
                print(
                    f'    success={record.get("success", False)} '
                    f'execution={record.get("execution_time_s", "n/a")}s '
                    f'wall={wall_time:.1f}s',
                    flush=True,
                )

    raw_hashes_by_planner = {}
    pairing_by_planner = {}
    planner_aggregates = {}
    for planner in planners:
        planner_records = [
            record for record in records
            if record.get('planner') == planner
        ]
        raw_hashes = {
            record.get('raw_path_sha256')
            for record in planner_records
            if record.get('raw_path_sha256')
        }
        raw_hashes_by_planner[planner] = sorted(raw_hashes)
        pairing_by_planner[planner] = (
            len(raw_hashes) == 1
            and all(record.get('raw_path_sha256') for record in planner_records)
        )
        planner_aggregates[planner] = _aggregate(
            planner_records, options.methods
        )
    paired_comparison_valid = all(pairing_by_planner.values())
    summary = {
        'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'scenario_file': (
            str(Path(options.scenario_file).resolve())
            if options.scenario_file else None
        ),
        'scenario': options.scenario,
        'configuration_sha256': configuration_sha256,
        'planner': planners[0] if len(planners) == 1 else None,
        'planners': planners,
        'methods': options.methods,
        'repetitions': options.repetitions,
        'all_successful': all(record.get('success', False) for record in records),
        'same_raw_path': paired_comparison_valid,
        'same_raw_path_by_planner': pairing_by_planner,
        'paired_comparison_valid': paired_comparison_valid,
        'comparison_warning': (
            '' if paired_comparison_valid else
            'Within one or more planners, raw path hashes differ or are '
            'missing; do not use those aggregate differences as paired '
            'smoother comparisons.'
        ),
        'raw_path_hashes': (
            raw_hashes_by_planner[planners[0]]
            if len(planners) == 1 else []
        ),
        'raw_path_hashes_by_planner': raw_hashes_by_planner,
        'aggregates': (
            planner_aggregates[planners[0]]
            if len(planners) == 1 else {}
        ),
        'planner_aggregates': planner_aggregates,
        'records': [_compact_summary_record(record) for record in records],
    }
    summary_path = output_dir / f'{options.scenario}_summary.json'
    _write_json(summary_path, summary)
    print(f'wrote matrix summary to {summary_path}', flush=True)


if __name__ == '__main__':
    main()
