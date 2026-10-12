#!/usr/bin/env python3
"""Run the cross-aisle execution matrix with a passive diagnostics recorder."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'docs/warehouse_cross_aisles_5_routes'


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def archive_infrastructure_failures(output_dir, module):
    """Move finalized setup failures aside before a policy-safe retry."""
    archive_dir = output_dir / 'infrastructure_attempts'
    entries = []
    recovery = {}
    for result_path in sorted(output_dir.glob('*.json')):
        if (
            result_path.name.endswith('_summary.json') or
            result_path.name.endswith('.diagnostics.json')
        ):
            continue
        try:
            record = json.loads(result_path.read_text(encoding='utf-8'))
        except (OSError, ValueError, TypeError):
            continue
        if not module._is_infrastructure_failure(record):
            continue
        prior_count = int(record.get('infrastructure_attempt_count', 1))
        attempt = prior_count
        stem = result_path.stem
        entry = {
            'trial': {
                key: record.get(key)
                for key in ('scenario', 'planner', 'method', 'repetition')
            },
            'reason': record.get('error'),
            'archived_at_utc': datetime.now(timezone.utc).isoformat(),
            'prior_infrastructure_attempt_count': prior_count,
            'artifacts': [],
        }
        for source, label in (
            (result_path, 'trial_json'),
            (result_path.with_suffix('.log'), 'trial_log'),
            (result_path.with_suffix('.diagnostics.json'), 'diagnostics'),
        ):
            if not source.exists():
                continue
            target = archive_dir / (
                f'{stem}__infrastructure_attempt_{attempt:02d}'
                f'{source.name[len(stem):]}'
            )
            if target.exists():
                raise RuntimeError(
                    f'Infrastructure-attempt archive already exists: {target}'
                )
            target.parent.mkdir(parents=True, exist_ok=True)
            artifact = {
                'kind': label,
                'original_path': str(source.relative_to(ROOT)),
                'archive_path': str(target.relative_to(ROOT)),
                'bytes': source.stat().st_size,
                'sha256': sha256(source),
            }
            source.replace(target)
            entry['artifacts'].append(artifact)
        entries.append(entry)
        recovery[result_path.resolve()] = {
            'prior_count': prior_count,
            'archive': str(archive_dir.relative_to(ROOT)),
        }
    if not entries:
        raise RuntimeError(
            f'No finalized infrastructure failure found in {output_dir}'
        )
    manifest_path = archive_dir / 'recovery_manifest.json'
    existing = []
    if manifest_path.exists():
        existing = json.loads(
            manifest_path.read_text(encoding='utf-8')
        ).get('recoveries', [])
    temporary = manifest_path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps({
        'policy': (
            'Only setup/infrastructure failures are archived and rerun; '
            'matching finalized algorithm failures are preserved.'
        ),
        'recoveries': existing + entries,
    }, ensure_ascii=False, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    temporary.replace(manifest_path)
    return recovery


def record(path):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init()
    node = Node('cross_aisle_passive_recorder')
    events = []
    output = Path(path)
    def callback(msg):
        try:
            value = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if value.get('method') == 'pstmo' or value.get('search_mode') == 'hierarchical_alpha_two_trim':
            events.append(value)
            output.write_text(json.dumps({'events': events}, indent=2), encoding='utf-8')
    node.create_subscription(String, '/research/pstmo/diagnostics', callback, 20)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def matrix(
    scenario,
    domain,
    output_root=None,
    planners=None,
    methods=None,
    repetitions=1,
    resume=False,
    retry_infrastructure_only=False,
):
    from adaptive_pivot_g2_benchmark import execution_matrix as module
    output_dir = (Path(output_root).resolve() if output_root else ASSETS / 'execution') / scenario
    if (
        output_dir.exists() and any(output_dir.glob('*.json')) and
        not resume and not retry_infrastructure_only
    ):
        raise RuntimeError(f'Existing evidence in {output_dir}; choose a new output root as the third argument.')
    recovery = {}
    if retry_infrastructure_only:
        recovery = archive_infrastructure_failures(output_dir, module)
    original = module._run_launch
    def run(command, environment, timeout, log_path=None):
        observer = None
        if 'method:=pstmo' in command:
            output = log_path.with_suffix('.diagnostics.json')
            observer = subprocess.Popen(
                [sys.executable, str(Path(__file__).resolve()), 'record', str(output)],
                env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            time.sleep(0.5)
        try:
            return original(command, environment, timeout, log_path)
        finally:
            if observer is not None:
                observer.send_signal(signal.SIGINT)
                try:
                    observer.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    observer.terminate()
                    observer.wait(timeout=4)
    module._run_launch = run
    original_write = module._write_json
    def write(path, document):
        recovered = recovery.get(Path(path).resolve())
        if recovered and 'repetition' in document and 'success' in document:
            document['prior_infrastructure_attempt_count'] = recovered[
                'prior_count'
            ]
            document['infrastructure_attempt_count'] = (
                int(document.get('infrastructure_attempt_count', 1)) +
                recovered['prior_count']
            )
            document['infrastructure_recovery_archive'] = recovered['archive']
        original_write(path, document)
    module._write_json = write
    arguments = [
        '--scenario-file', str(ASSETS / 'scenarios.yaml'), '--scenario', scenario,
        '--planners', *(planners or [
            'NavFnAStar', 'NavFnDijkstra', 'ThetaStar', 'Smac2D', 'SmacHybrid',
        ]),
        '--methods', *(methods or [
            'raw', 'simple', 'savitzky_golay', 'constrained', 'pstmo',
        ]),
        '--output-dir', str(output_dir),
        '--base-domain-id', str(domain), '--trial-timeout-s', '360',
        '--repetitions', str(repetitions),
        '--infrastructure-retries', '1',
    ]
    if retry_infrastructure_only:
        arguments.append('--resume-preserve-failures')
    elif resume:
        arguments.append('--resume')
    module.main(arguments)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'record':
        record(sys.argv[2])
    else:
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument('scenario')
        parser.add_argument('base_domain_id', type=int)
        parser.add_argument('output_root', nargs='?')
        parser.add_argument('--planners', nargs='+')
        parser.add_argument('--methods', nargs='+')
        parser.add_argument('--repetitions', type=int, default=1)
        resume_group = parser.add_mutually_exclusive_group()
        resume_group.add_argument(
            '--resume',
            action='store_true',
            help=(
                'Reuse matching successful trial JSON files and run only '
                'missing trials. The caller must audit existing failures '
                'before using this option.'
            ),
        )
        resume_group.add_argument(
            '--retry-infrastructure-only',
            action='store_true',
            help=(
                'Archive and rerun only finalized setup failures while '
                'reusing matching successes and algorithm failures.'
            ),
        )
        options = parser.parse_args()
        matrix(
            options.scenario,
            options.base_domain_id,
            options.output_root,
            options.planners,
            options.methods,
            options.repetitions,
            options.resume,
            options.retry_infrastructure_only,
        )
