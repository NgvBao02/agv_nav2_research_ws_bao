# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Run isolated Gazebo/Nav2 execution trials for every comparison method."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time
from typing import List, Optional


DEFAULT_METHODS = [
    'raw',
    'simple',
    'savitzky_golay',
    'constrained',
    'pivot_g2',
    'adaptive_hybrid',
]

AGGREGATE_METRICS = [
    'execution_time_s',
    'tracking_rmse_m',
    'tracking_max_error_m',
    'final_position_error_m',
    'final_yaw_error_rad',
    'traveled_distance_m',
    'executed_curvature_energy_1pm',
    'stopped_command_fraction',
]


def _run_launch(command, environment, timeout):
    process = subprocess.Popen(
        command,
        env=environment,
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    try:
        return process, process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        return process, 124


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
    parser.add_argument('--scenario', default='lower_left_diagonal')
    parser.add_argument('--planner', default='ThetaStar')
    parser.add_argument('--methods', nargs='+', default=DEFAULT_METHODS)
    parser.add_argument('--output-dir', default='results/execution_matrix')
    parser.add_argument('--base-domain-id', type=int, default=120)
    parser.add_argument('--trial-timeout-s', type=float, default=240.0)
    parser.add_argument('--repetitions', type=int, default=1)
    return parser.parse_args(args)


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
    unknown = sorted(set(options.methods) - set(DEFAULT_METHODS))
    if unknown:
        raise ValueError(f'unknown methods: {unknown}')
    if options.repetitions < 1:
        raise ValueError('repetitions must be at least one')
    trial_count = len(options.methods) * options.repetitions
    if options.base_domain_id < 0 or options.base_domain_id + trial_count - 1 > 232:
        raise ValueError('execution-matrix ROS domain IDs must be within 0..232')
    output_dir = Path(options.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    records = []
    trial_index = 0
    for method in options.methods:
        for repetition in range(1, options.repetitions + 1):
            suffix = (
                f'_r{repetition:02d}' if options.repetitions > 1 else ''
            )
            result_path = output_dir / (
                f'{options.scenario}_{method}{suffix}.json'
            )
            result_path.unlink(missing_ok=True)
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
                f'planner:={options.planner}',
                f'output_json:={result_path}',
                'gui:=false',
            ]
            print(
                f'[{trial_index + 1}/{trial_count}] running {method} '
                f'(repetition {repetition}/{options.repetitions})',
                flush=True,
            )
            started = time.monotonic()
            process, return_code = _run_launch(
                command, environment, options.trial_timeout_s
            )
            _stop_gazebo_server(environment)
            _terminate_trial_process_group(process)
            wall_time = time.monotonic() - started
            if result_path.exists():
                with result_path.open(encoding='utf-8') as stream:
                    record = json.load(stream)
            else:
                record = {
                    'scenario': options.scenario,
                    'planner': options.planner,
                    'method': method,
                    'success': False,
                    'error': 'trial did not produce a result file',
                }
            record['repetition'] = repetition
            record['launch_return_code'] = return_code
            record['trial_wall_time_s'] = wall_time
            records.append(record)
            trial_index += 1
            print(
                f'    success={record.get("success", False)} '
                f'execution={record.get("execution_time_s", "n/a")}s '
                f'wall={wall_time:.1f}s',
                flush=True,
            )

    raw_hashes = {
        record.get('raw_path_sha256')
        for record in records
        if record.get('raw_path_sha256')
    }
    summary = {
        'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'scenario': options.scenario,
        'planner': options.planner,
        'methods': options.methods,
        'repetitions': options.repetitions,
        'all_successful': all(record.get('success', False) for record in records),
        'same_raw_path': len(raw_hashes) == 1 and all(
            record.get('raw_path_sha256') for record in records
        ),
        'paired_comparison_valid': len(raw_hashes) == 1 and all(
            record.get('raw_path_sha256') for record in records
        ),
        'comparison_warning': (
            '' if len(raw_hashes) == 1 and all(
                record.get('raw_path_sha256') for record in records
            ) else
            'Raw path hashes differ or are missing; do not use aggregate '
            'differences as paired smoother comparisons.'
        ),
        'raw_path_hashes': sorted(raw_hashes),
        'aggregates': _aggregate(records, options.methods),
        'records': records,
    }
    summary_path = output_dir / f'{options.scenario}_summary.json'
    with summary_path.open('w', encoding='utf-8') as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write('\n')
    print(f'wrote matrix summary to {summary_path}', flush=True)


if __name__ == '__main__':
    main()
