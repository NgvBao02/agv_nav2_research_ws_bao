#!/usr/bin/env python3

"""Capture the six latched comparison paths for one paper figure."""

import argparse
import json
import math
from pathlib import Path as FilePath
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


METHODS = (
    'raw',
    'simple',
    'savitzky_golay',
    'constrained',
    'pivot_g2',
    'adaptive_hybrid',
)


class PathCapture(Node):
    """Publish a goal until accepted and collect every comparison path."""

    def __init__(self, goal_x, goal_y, goal_yaw):
        super().__init__('paper_path_capture')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.paths = {}
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_yaw = goal_yaw
        self.publisher = self.create_publisher(
            PoseStamped, '/research/goal_pose', QoSProfile(depth=10)
        )
        self._path_subscriptions = []
        for method in METHODS:
            subscription = self.create_subscription(
                Path,
                f'/research/path/{method}',
                lambda message, selected=method: self._path_callback(
                    selected, message
                ),
                qos,
            )
            self._path_subscriptions.append(subscription)
        self.timer = self.create_timer(2.0, self._publish_goal)

    def _publish_goal(self):
        if 'raw' in self.paths:
            return
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.orientation.z = math.sin(0.5 * self.goal_yaw)
        goal.pose.orientation.w = math.cos(0.5 * self.goal_yaw)
        self.publisher.publish(goal)
        self.get_logger().info('Published paper-figure goal')

    def _path_callback(self, method, message):
        self.paths[method] = [
            {
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'yaw_z': pose.pose.orientation.z,
                'yaw_w': pose.pose.orientation.w,
            }
            for pose in message.poses
        ]
        self.get_logger().info(
            f'Captured {method}: {len(message.poses)} poses'
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--goal-x', type=float, default=-2.0)
    parser.add_argument('--goal-y', type=float, default=-0.5)
    parser.add_argument('--goal-yaw', type=float, default=0.6947382761967031)
    parser.add_argument('--timeout', type=float, default=90.0)
    options, ros_arguments = parser.parse_known_args()
    rclpy.init(args=ros_arguments)
    node = PathCapture(options.goal_x, options.goal_y, options.goal_yaw)
    deadline = time.monotonic() + options.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if set(node.paths) == set(METHODS):
                break
        missing = sorted(set(METHODS) - set(node.paths))
        if missing:
            raise RuntimeError(f'timed out waiting for paths: {missing}')
        payload = {
            'scenario': 'lower_left_diagonal',
            'goal': [options.goal_x, options.goal_y, options.goal_yaw],
            'methods': list(METHODS),
            'paths': node.paths,
        }
        output_path = FilePath(options.output).resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open('w', encoding='utf-8') as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        print(output_path)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
