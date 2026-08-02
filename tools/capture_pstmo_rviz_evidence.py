#!/usr/bin/env python3

"""Capture one auditable five-method path comparison from a live RViz2 session."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile
import time
from typing import Dict

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


METHODS = ("raw", "simple", "savitzky_golay", "constrained", "pstmo")
VISIBILITY_METHODS = ("simple", "savitzky_golay", "constrained", "pstmo")
REQUIRED_METHODS = ("raw", "savitzky_golay", "constrained", "pstmo")


def latched_qos(depth: int = 1) -> QoSProfile:
    """Return the reliable transient-local QoS used by the comparison node."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def path_payload(message: PathMessage) -> dict:
    """Serialize a ROS Path and hash its exact planar pose sequence."""
    poses = [
        {
            "x": pose.pose.position.x,
            "y": pose.pose.position.y,
            "z": pose.pose.position.z,
            "qx": pose.pose.orientation.x,
            "qy": pose.pose.orientation.y,
            "qz": pose.pose.orientation.z,
            "qw": pose.pose.orientation.w,
        }
        for pose in message.poses
    ]
    canonical = json.dumps(poses, separators=(",", ":"), sort_keys=True)
    return {
        "frame_id": message.header.frame_id,
        "pose_count": len(poses),
        "sha256": hashlib.sha256(canonical.encode("utf-8")).hexdigest(),
        "poses": poses,
    }


class EvidenceCapture(Node):
    """Publish one request and retain the exact paths, metrics and diagnostics."""

    def __init__(self, planner: str, expected_preprocessing: str) -> None:
        super().__init__("pstmo_rviz_evidence_capture")
        self.planner = planner
        self.expected_preprocessing = expected_preprocessing
        self.active = False
        self.generation = None
        self.metrics: Dict[str, dict] = {}
        self.paths: Dict[str, PathMessage] = {}
        self.diagnostics = None
        self.planner_publisher = self.create_publisher(
            String, "/planner_selector", latched_qos()
        )
        self.visibility_publisher = self.create_publisher(
            String, "/research/smoother_visibility", latched_qos()
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped, "/research/goal_pose", QoSProfile(depth=1)
        )
        self.create_subscription(
            String, "/research/metrics", self.metrics_callback, latched_qos(32)
        )
        self.create_subscription(
            String,
            "/research/pstmo/diagnostics",
            self.diagnostics_callback,
            QoSProfile(depth=20),
        )
        for method in METHODS:
            self.create_subscription(
                PathMessage,
                f"/research/path/{method}",
                lambda message, selected=method: self.path_callback(selected, message),
                latched_qos(),
            )

    def metrics_callback(self, message: String) -> None:
        """Keep path-ready events from the generation requested by this tool."""
        if not self.active:
            return
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if payload.get("event") != "path_ready" or payload.get("planner") != self.planner:
            return
        method = payload.get("method")
        if method not in METHODS:
            return
        generation = payload.get("generation")
        if self.generation is None and method == "raw":
            self.generation = generation
        if generation != self.generation:
            return
        self.metrics[method] = payload

    def diagnostics_callback(self, message: String) -> None:
        """Keep only diagnostics of standalone hierarchical PSTMO."""
        if not self.active:
            return
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if (
            payload.get("search_mode") == "hierarchical_alpha_two_trim"
            and payload.get("preprocessing_mode") == self.expected_preprocessing
            and payload.get("los_executed") is (
                self.expected_preprocessing == "condition_then_los"
            )
            and payload.get("pipeline_execution_count") == 1
            and payload.get("final_invariants_verified") is True
        ):
            self.diagnostics = payload

    def path_callback(self, method: str, message: PathMessage) -> None:
        """Retain the latest nonempty path displayed by RViz2."""
        if self.active and message.poses:
            self.paths[method] = message

    def publish_visibility(self) -> None:
        """Show Raw plus the four conference comparison smoothers."""
        message = String()
        message.data = json.dumps({"methods": list(VISIBILITY_METHODS)})
        self.visibility_publisher.publish(message)

    def select_planner(self) -> None:
        """Select the requested global planner."""
        message = String()
        message.data = self.planner
        self.planner_publisher.publish(message)

    def publish_goal(self, x: float, y: float, yaw: float) -> None:
        """Publish the exact benchmark goal in the map frame."""
        import math

        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.orientation.w = math.cos(0.5 * yaw)
        self.goal_publisher.publish(message)

    def complete(self) -> bool:
        """Return whether every item needed to audit the screenshot is present."""
        return (
            set(REQUIRED_METHODS).issubset(self.metrics)
            and set(REQUIRED_METHODS).issubset(self.paths)
            and self.diagnostics is not None
        )


def spin_for(node: Node, seconds: float) -> None:
    """Spin a node for a bounded wall-clock duration."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))


def find_window_id(pattern: str) -> str:
    """Find one visible X11 client window matching a class/name expression."""
    tree = subprocess.run(
        ["xwininfo", "-root", "-tree"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    candidates = []
    for line in tree.splitlines():
        if re.search(pattern, line):
            match = re.match(r"\s*(0x[0-9a-fA-F]+)", line)
            geometry = re.search(r"\s(\d+)x(\d+)[+-]", line)
            if match and geometry:
                area = int(geometry.group(1)) * int(geometry.group(2))
                candidates.append((area, match.group(1)))
    if not candidates:
        raise RuntimeError(f"Could not find an X11 window matching {pattern!r}")
    return max(candidates)[1]


def capture_window(window_id: str, output: Path) -> None:
    """Capture one exact X11 client window and convert it losslessly to PNG."""
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="pstmo_xwd_") as directory:
        xwd = Path(directory) / "window.xwd"
        subprocess.run(
            ["xwd", "-silent", "-id", window_id, "-out", str(xwd)], check=True
        )
        subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                "-i",
                str(xwd),
                str(output),
            ],
            check=True,
        )


def parse_arguments() -> argparse.Namespace:
    """Parse one reproducible capture request."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--environment", required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--planner", required=True)
    parser.add_argument("--start-x", required=True, type=float)
    parser.add_argument("--start-y", required=True, type=float)
    parser.add_argument("--start-yaw", required=True, type=float)
    parser.add_argument("--goal-x", required=True, type=float)
    parser.add_argument("--goal-y", required=True, type=float)
    parser.add_argument("--goal-yaw", default=0.0, type=float)
    parser.add_argument(
        "--expected-preprocessing",
        choices=("condition_only", "condition_then_los"),
        default="condition_only",
    )
    parser.add_argument("--output-image", required=True, type=Path)
    parser.add_argument("--output-evidence", required=True, type=Path)
    parser.add_argument("--capture-gazebo", type=Path)
    parser.add_argument("--timeout", default=30.0, type=float)
    return parser.parse_args()


def main() -> None:
    """Capture one comparison image and its machine-readable evidence."""
    args = parse_arguments()
    rclpy.init()
    node = EvidenceCapture(args.planner, args.expected_preprocessing)
    try:
        spin_for(node, 1.0)
        node.publish_visibility()
        node.select_planner()
        spin_for(node, 2.0)
        node.metrics.clear()
        node.paths.clear()
        node.diagnostics = None
        node.generation = None
        node.active = True
        node.publish_goal(args.goal_x, args.goal_y, args.goal_yaw)
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.complete():
            raise RuntimeError(
                "Timed out waiting for all five paths, metrics and hierarchical diagnostics: "
                f"metrics={sorted(node.metrics)}, paths={sorted(node.paths)}, "
                f"diagnostics={node.diagnostics is not None}"
            )
        spin_for(node, 1.0)
        capture_window(
            find_window_id(r'\("rviz2" "rviz2"\)'), args.output_image
        )
        if args.capture_gazebo is not None:
            capture_window(
                find_window_id(r'\("gz-sim-gui" "Gazebo GUI"\)'),
                args.capture_gazebo,
            )
        evidence = {
            "source": "live Gazebo Harmonic and RViz2 ROS topics",
            "captured_at_unix_s": time.time(),
            "environment": args.environment,
            "scenario": args.scenario,
            "planner": args.planner,
            "start": [args.start_x, args.start_y, args.start_yaw],
            "goal": [args.goal_x, args.goal_y, args.goal_yaw],
            "expected_preprocessing": args.expected_preprocessing,
            "generation": node.generation,
            "metrics": node.metrics,
            "pstmo_diagnostics": node.diagnostics,
            "paths": {
                method: path_payload(node.paths[method]) for method in node.paths
            },
            "rviz_screenshot": str(args.output_image),
            "gazebo_screenshot": (
                str(args.capture_gazebo) if args.capture_gazebo is not None else None
            ),
        }
        args.output_evidence.parent.mkdir(parents=True, exist_ok=True)
        args.output_evidence.write_text(
            json.dumps(evidence, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(
            json.dumps(
                {
                    "environment": args.environment,
                    "scenario": args.scenario,
                    "planner": args.planner,
                    "generation": node.generation,
                    "search_mode": node.diagnostics["search_mode"],
                    "image": str(args.output_image),
                    "evidence": str(args.output_evidence),
                },
                ensure_ascii=False,
                sort_keys=True,
            )
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
