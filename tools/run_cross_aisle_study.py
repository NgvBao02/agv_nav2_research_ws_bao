#!/usr/bin/env python3
"""Run the unmodified execution matrix with a passive diagnostics recorder."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'docs/warehouse_cross_aisles_5_routes'


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


def matrix(scenario, domain, output_root=None):
    from adaptive_pivot_g2_benchmark import execution_matrix as module
    output_dir = (Path(output_root).resolve() if output_root else ASSETS / 'execution') / scenario
    if output_dir.exists() and any(output_dir.glob('*.json')):
        raise RuntimeError(f'Existing evidence in {output_dir}; choose a new output root as the third argument.')
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
    module.main([
        '--scenario-file', str(ASSETS / 'scenarios.yaml'), '--scenario', scenario,
        '--planners', 'NavFnAStar', 'NavFnDijkstra', 'ThetaStar', 'Smac2D', 'SmacHybrid',
        '--methods', 'raw', 'simple', 'savitzky_golay', 'constrained', 'pstmo',
        '--output-dir', str(output_dir),
        '--base-domain-id', str(domain), '--trial-timeout-s', '360',
        '--infrastructure-retries', '1',
    ])


if __name__ == '__main__':
    if sys.argv[1] == 'record':
        record(sys.argv[2])
    else:
        matrix(sys.argv[1], int(sys.argv[2]), sys.argv[3] if len(sys.argv)>3 else None)
