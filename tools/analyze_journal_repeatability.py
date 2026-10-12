#!/usr/bin/env python3
"""Audit and summarize the focused PSTMO journal repeatability cohort."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import random
import statistics

from scipy import stats


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_COHORT = ROOT / 'results/pstmo_journal_repeatability_20261011'
METHODS = ('constrained', 'pstmo')
PRIMARY_METRICS = (
    'execution_time_s',
    'tracking_rmse_m',
    'executed_curvature_energy_1pm',
)
METRICS = {
    'execution_time_s': ('Execution time', 's', 'lower'),
    'tracking_rmse_m': ('Tracking RMSE', 'm', 'lower'),
    'executed_curvature_energy_1pm': (
        'Executed curvature energy', '1/m', 'lower'
    ),
    'planned_curvature_energy_1pm': (
        'Planned curvature energy', '1/m', 'lower'
    ),
    'actual_p95_abs_jerk_mps3': ('Actual jerk p95', 'm/s^3', 'lower'),
    'final_position_error_m': ('Final position error', 'm', 'lower'),
    'traveled_distance_m': ('Travelled distance', 'm', 'lower'),
    'smoothing_time_s': ('Smoothing time', 's', 'lower'),
    'planned_footprint_clearance_min_m': (
        'Minimum planned footprint clearance', 'm', 'higher'
    ),
}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def configuration_sha256(nav2_parameters, scenario_file):
    digest = hashlib.sha256()
    for label, path in (
        ('nav2_params', nav2_parameters),
        ('scenario', scenario_file),
    ):
        digest.update(label.encode('utf-8'))
        digest.update(b'\0')
        digest.update(path.read_bytes())
        digest.update(b'\0')
    return digest.hexdigest()


def atomic_json(path, document):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    temporary.replace(path)


def percentile(values, probability):
    ordered = sorted(values)
    if not ordered:
        return None
    position = probability * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def bootstrap_mean_ci(values, seed, samples=20000):
    if not values:
        return [None, None]
    generator = random.Random(seed)
    count = len(values)
    estimates = [
        statistics.fmean(values[generator.randrange(count)] for _ in range(count))
        for _ in range(samples)
    ]
    return [percentile(estimates, 0.025), percentile(estimates, 0.975)]


def exact_mcnemar(discordant_a, discordant_b):
    total = discordant_a + discordant_b
    if total == 0:
        return 1.0
    return float(stats.binomtest(
        min(discordant_a, discordant_b),
        total,
        p=0.5,
        alternative='two-sided',
    ).pvalue)


def wilcoxon_pvalue(values):
    nonzero = [value for value in values if abs(value) > 1.0e-12]
    if not nonzero:
        return 1.0
    return float(stats.wilcoxon(
        nonzero,
        alternative='two-sided',
        zero_method='wilcox',
        method='auto',
    ).pvalue)


def holm_adjust(raw):
    ordered = sorted(raw, key=raw.get)
    adjusted = {}
    running = 0.0
    count = len(ordered)
    for index, metric in enumerate(ordered):
        candidate = min(1.0, (count - index) * raw[metric])
        running = max(running, candidate)
        adjusted[metric] = running
    return adjusted


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cohort', default=str(DEFAULT_COHORT))
    return parser.parse_args()


def main():
    options = parse_args()
    cohort = Path(options.cohort).resolve()
    manifest_path = cohort / 'study_manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if manifest.get('status') != 'complete':
        raise RuntimeError('Study manifest is not complete')

    expected_trials = int(manifest['planned_trial_count'])
    trial_files = sorted(
        path for path in cohort.rglob('*.json')
        if path.name != 'study_manifest.json'
        and not path.name.endswith('_summary.json')
        and not path.name.endswith('.diagnostics.json')
        and path.parent.name != 'analysis'
        and 'interrupted_attempts' not in path.parts
        and 'infrastructure_attempts' not in path.parts
    )
    records = []
    issues = []
    for path in trial_files:
        try:
            record = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError) as error:
            issues.append(f'Cannot read {path.relative_to(ROOT)}: {error}')
            continue
        record['_source'] = str(path.relative_to(ROOT))
        records.append(record)
    if len(records) != expected_trials:
        issues.append(f'Expected {expected_trials} trials, found {len(records)}')

    keyed = {}
    for record in records:
        key = (
            record.get('scenario'),
            record.get('planner'),
            record.get('method'),
            int(record.get('repetition', -1)),
        )
        if key in keyed:
            issues.append(f'Duplicate trial key: {key}')
        keyed[key] = record

    expected_keys = {
        (scenario, planner, method, repetition)
        for scenario in manifest['scenarios']
        for planner in manifest['planners']
        for method in manifest['methods']
        for repetition in range(1, int(manifest['repetitions']) + 1)
    }
    missing = sorted(expected_keys - set(keyed))
    unexpected = sorted(set(keyed) - expected_keys)
    issues.extend(f'Missing trial key: {key}' for key in missing)
    issues.extend(f'Unexpected trial key: {key}' for key in unexpected)
    nav2_parameters = ROOT / 'src/vacuum_robot_gazebo/config/nav2_params.yaml'
    scenario_file = ROOT / manifest['scenario_file']
    expected_configuration = configuration_sha256(
        nav2_parameters, scenario_file
    )
    configuration_hashes = {
        record.get('configuration_sha256') for record in records
    }
    if configuration_hashes != {expected_configuration}:
        issues.append(
            'Configuration fingerprints do not match the current scenario '
            f'and Nav2 parameters: {sorted(configuration_hashes)}'
        )

    pairing = []
    paired_records = []
    for scenario in manifest['scenarios']:
        for planner in manifest['planners']:
            block_hashes = set()
            for repetition in range(1, int(manifest['repetitions']) + 1):
                pair = [keyed.get((scenario, planner, method, repetition)) for method in METHODS]
                if any(record is None for record in pair):
                    continue
                hashes = {record.get('raw_path_sha256') for record in pair}
                valid = None not in hashes and len(hashes) == 1
                if not valid:
                    issues.append(
                        'Raw-path pairing mismatch: '
                        f'{scenario}/{planner}/r{repetition:02d}'
                    )
                else:
                    block_hashes.update(hashes)
                paired_records.append((pair[0], pair[1]))
                pairing.append({
                    'scenario': scenario,
                    'planner': planner,
                    'repetition': repetition,
                    'valid': valid,
                    'raw_path_sha256': next(iter(hashes)) if len(hashes) == 1 else None,
                })
            if len(block_hashes) != 1:
                issues.append(
                    'Raw path changed across repetitions: '
                    f'{scenario}/{planner} ({len(block_hashes)} hashes)'
                )

    success = {
        method: sum(
            bool(record.get('success'))
            for record in records if record.get('method') == method
        )
        for method in METHODS
    }
    constrained_only = sum(
        bool(constrained.get('success')) and not bool(pstmo.get('success'))
        for constrained, pstmo in paired_records
    )
    pstmo_only = sum(
        bool(pstmo.get('success')) and not bool(constrained.get('success'))
        for constrained, pstmo in paired_records
    )

    metric_results = {}
    primary_raw_p = {}
    block_rows = []
    for metric_index, (metric, (label, unit, preferred)) in enumerate(METRICS.items()):
        block_differences = []
        trial_differences = []
        constrained_values = []
        pstmo_values = []
        for scenario in manifest['scenarios']:
            for planner in manifest['planners']:
                local = []
                for repetition in range(1, int(manifest['repetitions']) + 1):
                    constrained = keyed.get((scenario, planner, 'constrained', repetition))
                    pstmo = keyed.get((scenario, planner, 'pstmo', repetition))
                    if not constrained or not pstmo:
                        continue
                    if not constrained.get('success') or not pstmo.get('success'):
                        continue
                    left = constrained.get(metric)
                    right = pstmo.get(metric)
                    if not all(
                        isinstance(value, (int, float)) and math.isfinite(float(value))
                        for value in (left, right)
                    ):
                        continue
                    left = float(left)
                    right = float(right)
                    constrained_values.append(left)
                    pstmo_values.append(right)
                    trial_differences.append(right - left)
                    local.append(right - left)
                if local:
                    block_difference = statistics.fmean(local)
                    block_differences.append(block_difference)
                    block_rows.append({
                        'scenario': scenario,
                        'planner': planner,
                        'metric': metric,
                        'paired_repetitions': len(local),
                        'pstmo_minus_constrained': block_difference,
                    })
        raw_p = wilcoxon_pvalue(block_differences)
        if metric in PRIMARY_METRICS:
            primary_raw_p[metric] = raw_p
        mean_difference = (
            statistics.fmean(block_differences) if block_differences else None
        )
        baseline_mean = (
            statistics.fmean(constrained_values) if constrained_values else None
        )
        relative = (
            100.0 * mean_difference / baseline_mean
            if mean_difference is not None and baseline_mean not in (None, 0.0)
            else None
        )
        metric_results[metric] = {
            'label': label,
            'unit': unit,
            'preferred_direction': preferred,
            'paired_successful_trial_count': len(trial_differences),
            'independent_block_count': len(block_differences),
            'constrained_mean': baseline_mean,
            'pstmo_mean': statistics.fmean(pstmo_values) if pstmo_values else None,
            'mean_block_difference_pstmo_minus_constrained': mean_difference,
            'mean_block_difference_bootstrap_95_ci': bootstrap_mean_ci(
                block_differences,
                20261011 + metric_index,
            ),
            'median_block_difference_pstmo_minus_constrained': (
                statistics.median(block_differences) if block_differences else None
            ),
            'relative_difference_percent': relative,
            'blocks_pstmo_lower': sum(value < -1.0e-12 for value in block_differences),
            'blocks_equal': sum(abs(value) <= 1.0e-12 for value in block_differences),
            'blocks_pstmo_higher': sum(value > 1.0e-12 for value in block_differences),
            'wilcoxon_two_sided_p': raw_p,
        }

    adjusted = holm_adjust(primary_raw_p)
    for metric, value in adjusted.items():
        metric_results[metric]['holm_adjusted_p_primary_family'] = value

    diagnostics_files = sorted(
        path for path in cohort.rglob('*.diagnostics.json')
        if 'interrupted_attempts' not in path.parts
        and 'infrastructure_attempts' not in path.parts
    )
    diagnostic_events = []
    for path in diagnostics_files:
        try:
            document = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError) as error:
            issues.append(f'Cannot read diagnostics {path.relative_to(ROOT)}: {error}')
            continue
        events = document.get('events', [])
        if not events:
            issues.append(f'No PSTMO diagnostics event in {path.relative_to(ROOT)}')
            continue
        diagnostic_events.append(events[-1])
    expected_pstmo_trials = expected_trials // len(METHODS)
    if len(diagnostics_files) != expected_pstmo_trials:
        issues.append(
            f'Expected {expected_pstmo_trials} diagnostics files, '
            f'found {len(diagnostics_files)}'
        )
    diagnostics_audit = {
        'file_count': len(diagnostics_files),
        'event_count': len(diagnostic_events),
        'condition_only_count': sum(
            event.get('preprocessing_mode') == 'condition_only'
            for event in diagnostic_events
        ),
        'hierarchical_two_trim_count': sum(
            event.get('search_mode') == 'hierarchical_alpha_two_trim'
            for event in diagnostic_events
        ),
        'single_pipeline_count': sum(
            event.get('pipeline_execution_count') == 1
            for event in diagnostic_events
        ),
        'final_invariant_count': sum(
            bool(event.get('final_invariants_verified'))
            for event in diagnostic_events
        ),
        'total_dp_states': sum(
            int(event.get('dp_states', 0)) for event in diagnostic_events
        ),
        'total_compatible_edges': sum(
            int(event.get('compatible_edges', 0)) for event in diagnostic_events
        ),
    }

    analysis_dir = cohort / 'analysis'
    analysis_dir.mkdir(parents=True, exist_ok=True)
    scalar_columns = [
        'scenario', 'planner', 'method', 'repetition', 'success',
        'raw_path_sha256', 'configuration_sha256',
        *METRICS,
        'collision_monitor_interventions', 'planned_footprint_collision_sample_count',
        'infrastructure_attempt_count', '_source',
    ]
    with (analysis_dir / 'trial_metrics.csv').open(
        'w', encoding='utf-8', newline=''
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=scalar_columns)
        writer.writeheader()
        for record in sorted(
            records,
            key=lambda item: (
                item.get('scenario', ''), item.get('planner', ''),
                item.get('method', ''), int(item.get('repetition', -1)),
            ),
        ):
            writer.writerow({column: record.get(column) for column in scalar_columns})
    with (analysis_dir / 'block_differences.csv').open(
        'w', encoding='utf-8', newline=''
    ) as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                'scenario', 'planner', 'metric', 'paired_repetitions',
                'pstmo_minus_constrained',
            ],
        )
        writer.writeheader()
        writer.writerows(block_rows)

    output = {
        'cohort': str(cohort.relative_to(ROOT)),
        'manifest_sha256': sha256(manifest_path),
        'configuration_sha256': expected_configuration,
        'source_fingerprints': {
            str(path.relative_to(ROOT)): sha256(path)
            for path in [
                ROOT / 'tools/run_cross_aisle_study.py',
                ROOT / 'tools/run_journal_repeatability_study.py',
                ROOT / 'tools/analyze_journal_repeatability.py',
                ROOT / 'src/adaptive_pivot_g2_benchmark/adaptive_pivot_g2_benchmark/execution_matrix.py',
                ROOT / 'src/adaptive_pivot_g2_benchmark/adaptive_pivot_g2_benchmark/execution_trial.py',
                nav2_parameters,
                scenario_file,
            ]
        },
        'trial_count': len(records),
        'paired_trial_count': len(paired_records),
        'valid_raw_pair_count': sum(item['valid'] for item in pairing),
        'block_count': len(manifest['scenarios']) * len(manifest['planners']),
        'success': {
            **success,
            'constrained_only_success_pairs': constrained_only,
            'pstmo_only_success_pairs': pstmo_only,
            'mcnemar_exact_two_sided_p': exact_mcnemar(
                constrained_only, pstmo_only
            ),
        },
        'failures': [
            {
                key: record.get(key)
                for key in (
                    'scenario', 'planner', 'method', 'repetition', 'error',
                    'controller_action_succeeded', 'physically_settled',
                    'controller_status', 'controller_error_code',
                    'controller_error_msg',
                    'ground_truth_goal_reached', 'final_position_error_m',
                    'ground_truth_position_tolerance_m',
                    'final_yaw_error_rad', 'ground_truth_yaw_tolerance_rad',
                    'infrastructure_attempt_count', '_source',
                )
            }
            for record in records if not record.get('success')
        ],
        'metrics': metric_results,
        'diagnostics': diagnostics_audit,
        'infrastructure_retry_trial_count': sum(
            int(record.get('infrastructure_attempt_count', 1)) > 1
            for record in records
        ),
        'collision_monitor_intervention_total': sum(
            int(record.get('collision_monitor_interventions', 0))
            for record in records
        ),
        'planned_footprint_collision_sample_total': sum(
            int(record.get('planned_footprint_collision_sample_count', 0))
            for record in records
        ),
        'audit_issues': issues,
        'statistical_unit': (
            'Route-planner block (25 blocks); repetitions are averaged within '
            'each fixed-input block before Wilcoxon tests and cluster bootstrap.'
        ),
        'bootstrap': {
            'resamples': 20000,
            'seed_base': 20261011,
            'confidence_level': 0.95,
        },
        'multiplicity': (
            'Holm adjustment across the three primary continuous outcomes: '
            'execution time, tracking RMSE, and executed curvature energy.'
        ),
        'interpretation_limit': manifest['random_seed_note'],
    }
    atomic_json(analysis_dir / 'aggregate.json', output)

    lines = [
        '# PSTMO journal repeatability analysis',
        '',
        f"- Trials: {output['trial_count']} ({success['constrained']} Constrained successes; "
        f"{success['pstmo']} PSTMO successes).",
        f"- Paired Raw hashes: {output['valid_raw_pair_count']}/{output['paired_trial_count']}.",
        f"- Independent statistical units: {output['block_count']} route-planner blocks.",
        f"- Audit issues: {len(issues)}.",
        '',
        '| Metric | Constrained mean | PSTMO mean | Difference | 95% CI | Holm p |',
        '| --- | ---: | ---: | ---: | ---: | ---: |',
    ]
    for metric in PRIMARY_METRICS:
        result = metric_results[metric]
        lower, upper = result['mean_block_difference_bootstrap_95_ci']
        lines.append(
            f"| {result['label']} ({result['unit']}) | "
            f"{result['constrained_mean']:.6g} | {result['pstmo_mean']:.6g} | "
            f"{result['mean_block_difference_pstmo_minus_constrained']:.6g} | "
            f"[{lower:.6g}, {upper:.6g}] | "
            f"{result['holm_adjusted_p_primary_family']:.6g} |"
        )
    lines.extend([
        '',
        'Differences are PSTMO minus Constrained. Repetitions quantify fixed-input ',
        'execution repeatability; they are not independent random-seed or noise trials.',
        '',
    ])
    (analysis_dir / 'README.md').write_text('\n'.join(lines), encoding='utf-8')
    print(json.dumps(output, ensure_ascii=False, indent=2))
    if issues:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
