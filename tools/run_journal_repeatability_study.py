#!/usr/bin/env python3
"""Run the focused repeated cross-aisle study in isolated parallel slots."""

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import queue
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
SCENARIO_FILE = ROOT / 'docs/warehouse_cross_aisles_5_routes/scenarios.yaml'
DEFAULT_SCENARIOS = [
    'cross_aisle_transfer',
    'wide_diagonal_transfer',
    'south_u_transfer',
    'reverse_diagonal_transfer',
    'cross_to_picking_turn',
]
DEFAULT_PLANNERS = [
    'NavFnAStar',
    'NavFnDijkstra',
    'ThetaStar',
    'Smac2D',
    'SmacHybrid',
]
DEFAULT_METHODS = ['constrained', 'pstmo']
DOMAIN_BASES = [0, 58, 116, 174]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def configuration_sha256(nav2_params, scenario_file):
    """Match the execution-matrix fingerprint without importing ROS here."""
    digest = hashlib.sha256()
    for label, path in (
        ('nav2_params', nav2_params),
        ('scenario', scenario_file),
    ):
        digest.update(label.encode('utf-8'))
        digest.update(b'\0')
        digest.update(path.read_bytes())
        digest.update(b'\0')
    return digest.hexdigest()


def slug(value):
    return ''.join(
        character.lower() if character.isalnum() else '_'
        for character in value
    ).strip('_')


def atomic_json(path, document):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    temporary.replace(path)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output-root',
        default=str(ROOT / 'results/pstmo_journal_repeatability_20261011'),
    )
    parser.add_argument('--repetitions', type=int, default=5)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--scenarios', nargs='+', default=DEFAULT_SCENARIOS)
    parser.add_argument('--planners', nargs='+', default=DEFAULT_PLANNERS)
    parser.add_argument('--methods', nargs='+', default=DEFAULT_METHODS)
    parser.add_argument(
        '--resume',
        action='store_true',
        help=(
            'Continue an interrupted cohort after validating its manifest. '
            'Successful trial JSON files are preserved; any existing failed '
            'or mismatched trial stops the continuation for manual review.'
        ),
    )
    return parser.parse_args()


def expected_trial_path(output_root, scenario, planner, method, repetition):
    return (
        output_root / slug(planner) / scenario /
        f'{scenario}_{method}_r{repetition:02d}.json'
    )


def audit_existing_trials(output_root, jobs, methods, repetitions, config_hash):
    """Classify blocks without rewriting any previously recorded evidence."""
    complete = set()
    provisional = []
    for job in jobs:
        present = 0
        for method in methods:
            for repetition in range(1, repetitions + 1):
                path = expected_trial_path(
                    output_root, job['scenario'], job['planner'], method,
                    repetition,
                )
                if not path.exists():
                    continue
                try:
                    record = json.loads(path.read_text(encoding='utf-8'))
                except (OSError, ValueError, TypeError) as error:
                    raise RuntimeError(
                        f'Cannot resume with corrupt trial evidence: {path}'
                    ) from error
                base_identity = (
                    record.get('scenario') == job['scenario'] and
                    record.get('planner') == job['planner'] and
                    record.get('method') == method
                )
                if (
                    base_identity and
                    'repetition' not in record and
                    'configuration_sha256' not in record
                ):
                    provisional.append(path)
                    continue
                try:
                    recorded_repetition = int(record.get('repetition', -1))
                except (TypeError, ValueError):
                    recorded_repetition = -1
                if not (
                    base_identity and
                    recorded_repetition == repetition and
                    record.get('configuration_sha256') == config_hash
                ):
                    raise RuntimeError(
                        f'Cannot resume with mismatched trial evidence: {path}'
                    )
                if not record.get('success', False):
                    raise RuntimeError(
                        'Existing failed experimental trial must not be '
                        f'automatically rerun: {path}'
                    )
                if method == 'pstmo':
                    diagnostics = path.with_suffix('.diagnostics.json')
                    if not diagnostics.exists():
                        raise RuntimeError(
                            'Successful PSTMO trial is missing passive '
                            f'diagnostics: {diagnostics}'
                        )
                present += 1
        if present == len(methods) * repetitions:
            complete.add((job['scenario'], job['planner']))
    return complete, provisional


def archive_provisional_trials(output_root, paths):
    """Preserve interrupted, unfinalized attempts outside the trial matrix."""
    archived = []
    archive_root = output_root / 'interrupted_attempts'
    for path in paths:
        relative = path.relative_to(output_root)
        entry = {
            'reason': (
                'Parent matrix process ended before repetition/configuration '
                'metadata and atomic finalization; excluded from analysis.'
            ),
            'original_trial_path': str(relative),
            'archived_at_utc': datetime.now(timezone.utc).isoformat(),
            'artifacts': [],
        }
        companions = [
            path,
            path.with_suffix('.log'),
            path.with_suffix('.diagnostics.json'),
        ]
        for source in companions:
            if not source.exists():
                continue
            target = archive_root / source.relative_to(output_root)
            if target.exists():
                raise RuntimeError(
                    f'Interrupted-attempt archive already exists: {target}'
                )
            target.parent.mkdir(parents=True, exist_ok=True)
            artifact = {
                'source': str(source.relative_to(ROOT)),
                'archive': str(target.relative_to(ROOT)),
                'bytes': source.stat().st_size,
                'sha256': sha256(source),
            }
            source.replace(target)
            entry['artifacts'].append(artifact)
        archived.append(entry)
    return archived


def main():
    options = parse_args()
    if options.repetitions < 2:
        raise ValueError('The journal repeatability cohort requires at least two repetitions')
    if not 1 <= options.workers <= len(DOMAIN_BASES):
        raise ValueError(f'workers must be within 1..{len(DOMAIN_BASES)}')
    trial_count_per_job = len(options.methods) * options.repetitions
    if trial_count_per_job > 58:
        raise ValueError('Each job must fit inside its 58-domain isolation band')

    output_root = Path(options.output_root).resolve()
    if (
        output_root.exists() and any(output_root.rglob('*.json')) and
        not options.resume
    ):
        raise RuntimeError(
            f'Existing evidence in {output_root}; choose a new output root.'
        )
    output_root.mkdir(parents=True, exist_ok=True)
    manifest_path = output_root / 'study_manifest.json'
    jobs = [
        {'scenario': scenario, 'planner': planner}
        for scenario in options.scenarios
        for planner in options.planners
    ]
    git_commit = subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True
    ).strip()
    nav2_params = ROOT / 'src/vacuum_robot_gazebo/config/nav2_params.yaml'
    new_manifest = {
        'study': 'PSTMO journal repeatability validation',
        'status': 'running',
        'started_at_utc': datetime.now(timezone.utc).isoformat(),
        'output_root': str(output_root.relative_to(ROOT)),
        'scenario_file': str(SCENARIO_FILE.relative_to(ROOT)),
        'scenario_file_sha256': sha256(SCENARIO_FILE),
        'nav2_params_sha256': sha256(nav2_params),
        'git_commit': git_commit,
        'ros_distro': os.environ.get('ROS_DISTRO'),
        'platform': platform.platform(),
        'cpu_count': os.cpu_count(),
        'workers': options.workers,
        'domain_bases': DOMAIN_BASES[:options.workers],
        'scenarios': options.scenarios,
        'planners': options.planners,
        'methods': options.methods,
        'repetitions': options.repetitions,
        'planned_job_count': len(jobs),
        'planned_trial_count': len(jobs) * trial_count_per_job,
        'random_seed': None,
        'random_seed_note': (
            'No stochastic seed is exposed by this execution protocol; '
            'repetitions measure simulator/controller execution repeatability '
            'for fixed start, goal, planner, and configuration.'
        ),
        'jobs': [],
    }
    if options.resume:
        if not manifest_path.exists():
            raise RuntimeError(
                f'Cannot resume without an existing manifest: {manifest_path}'
            )
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        for field in (
            'scenario_file_sha256', 'nav2_params_sha256', 'git_commit',
            'scenarios', 'planners', 'methods', 'repetitions',
            'planned_job_count', 'planned_trial_count',
        ):
            if manifest.get(field) != new_manifest.get(field):
                raise RuntimeError(
                    f'Cannot resume: manifest field {field!r} no longer '
                    'matches the requested study.'
                )
        complete_keys, provisional = audit_existing_trials(
            output_root,
            jobs,
            options.methods,
            options.repetitions,
            configuration_sha256(nav2_params, SCENARIO_FILE),
        )
        if provisional:
            interrupted = archive_provisional_trials(
                output_root, provisional
            )
            manifest.setdefault('interrupted_attempts', []).extend(
                interrupted
            )
            complete_keys, remaining_provisional = audit_existing_trials(
                output_root,
                jobs,
                options.methods,
                options.repetitions,
                configuration_sha256(nav2_params, SCENARIO_FILE),
            )
            if remaining_provisional:
                raise RuntimeError(
                    'Interrupted attempts remain after archival: '
                    f'{remaining_provisional}'
                )
        completed = [
            item for item in manifest.get('jobs', [])
            if (item['scenario'], item['planner']) in complete_keys
        ]
        recorded_keys = {
            (item['scenario'], item['planner']) for item in completed
        }
        missing_manifest_keys = complete_keys - recorded_keys
        if missing_manifest_keys:
            raise RuntimeError(
                'Complete evidence exists without a completed manifest job; '
                f'manual audit required: {sorted(missing_manifest_keys)}'
            )
        jobs_to_run = [
            job for job in jobs
            if (job['scenario'], job['planner']) not in complete_keys
        ]
        manifest['status'] = 'running'
        manifest.pop('finished_at_utc', None)
        manifest['resumed_at_utc'] = datetime.now(timezone.utc).isoformat()
        manifest['resume_count'] = int(manifest.get('resume_count', 0)) + 1
        atomic_json(manifest_path, manifest)
    else:
        manifest = new_manifest
        completed = []
        jobs_to_run = jobs
        atomic_json(manifest_path, manifest)

    available_domains = queue.Queue()
    for domain in DOMAIN_BASES[:options.workers]:
        available_domains.put(domain)

    def run_job(job):
        domain = available_domains.get()
        planner_dir = output_root / slug(job['planner'])
        log_dir = output_root / 'batch_logs'
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"{job['scenario']}__{slug(job['planner'])}.log"
        command = [
            sys.executable,
            str(ROOT / 'tools/run_cross_aisle_study.py'),
            job['scenario'],
            str(domain),
            str(planner_dir),
            '--planners',
            job['planner'],
            '--methods',
            *options.methods,
            '--repetitions',
            str(options.repetitions),
        ]
        if options.resume:
            command.append('--resume')
        started = datetime.now(timezone.utc)
        try:
            log_mode = 'a' if options.resume else 'w'
            with log_path.open(log_mode, encoding='utf-8') as stream:
                if options.resume:
                    stream.write(
                        '\n=== interrupted cohort continuation '
                        f'{started.isoformat()} ===\n'
                    )
                    stream.flush()
                result = subprocess.run(
                    command,
                    cwd=ROOT,
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                    text=True,
                    check=False,
                )
            return {
                **job,
                'base_domain_id': domain,
                'return_code': result.returncode,
                'started_at_utc': started.isoformat(),
                'finished_at_utc': datetime.now(timezone.utc).isoformat(),
                'log': str(log_path.relative_to(ROOT)),
                'summary': str(
                    (planner_dir / job['scenario'] /
                     f"{job['scenario']}_summary.json").relative_to(ROOT)
                ),
            }
        finally:
            available_domains.put(domain)

    with ThreadPoolExecutor(max_workers=options.workers) as executor:
        futures = {
            executor.submit(run_job, job): job for job in jobs_to_run
        }
        for future in as_completed(futures):
            result = future.result()
            completed.append(result)
            manifest['jobs'] = sorted(
                completed,
                key=lambda item: (item['scenario'], item['planner']),
            )
            manifest['completed_job_count'] = len(completed)
            manifest['failed_job_count'] = sum(
                item['return_code'] != 0 for item in completed
            )
            atomic_json(manifest_path, manifest)
            print(
                f"[{len(completed)}/{len(jobs)}] "
                f"{result['scenario']} / {result['planner']} "
                f"return_code={result['return_code']}",
                flush=True,
            )

    manifest['status'] = (
        'complete' if all(item['return_code'] == 0 for item in completed)
        else 'complete_with_failed_jobs'
    )
    manifest['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
    atomic_json(manifest_path, manifest)
    if manifest['status'] != 'complete':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
