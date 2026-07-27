#!/usr/bin/env python3
"""Observe a live headless run and enforce the 30-simulated-second stand gate."""

import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from .constants import JOINT_NAMES
from .safety import reorder_joint_state


class StandingSmokeTest(Node):
    def __init__(self) -> None:
        super().__init__('spot_standing_smoke_test')
        self.declare_parameter('duration', 30.0)
        self.declare_parameter('wall_timeout', 180.0)
        self.declare_parameter('minimum_height', 0.35)
        self.declare_parameter('maximum_tilt', 0.35)
        self.declare_parameter('maximum_drift', 0.50)
        self.declare_parameter('torque_limit', 60.0)
        self.declare_parameter('require_wbc_policy', False)
        self.duration = float(self.get_parameter('duration').value)
        self.wall_timeout = float(self.get_parameter('wall_timeout').value)
        self.minimum_height = float(
            self.get_parameter('minimum_height').value)
        self.maximum_tilt = float(self.get_parameter('maximum_tilt').value)
        self.maximum_drift = float(self.get_parameter('maximum_drift').value)
        self.torque_limit = float(self.get_parameter('torque_limit').value)
        self.require_wbc_policy = bool(
            self.get_parameter('require_wbc_policy').value)

        self.odom = None
        self.command = None
        self.joints_valid = False
        self.failure = None
        self.min_height_seen = math.inf
        self.max_tilt_seen = 0.0
        self.max_drift_seen = 0.0
        self.max_torque_seen = 0.0
        self.wbc_policy_seen = False
        self.create_subscription(
            Odometry, '/spot/odometry', self.on_odom, 10)
        self.create_subscription(
            JointState, '/spot/joint_states', self.on_joints, 10)
        self.create_subscription(
            JointTrajectory, '/spot/joint_trajectory', self.on_command, 10)
        self.create_subscription(
            String, '/spot/wbc/diagnostics', self.on_wbc_diagnostics, 10)

    def on_wbc_diagnostics(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if (
                payload.get('policy_active') is True
                and payload.get('current_mode') == 15
                and int(payload.get('qp_success_count', 0)) > 0):
            self.wbc_policy_seen = True

    def on_joints(self, msg: JointState) -> None:
        try:
            reorder_joint_state(msg.name, msg.position)
            reorder_joint_state(msg.name, msg.velocity)
            self.joints_valid = True
        except ValueError as error:
            self.failure = f'invalid joint state: {error}'

    def on_command(self, msg: JointTrajectory) -> None:
        if tuple(msg.joint_names) != JOINT_NAMES or len(msg.points) != 1:
            self.failure = 'backend command has invalid joint order or dimensions'
            return
        point = msg.points[0]
        if point.positions or point.velocities or len(point.effort) != 12:
            self.failure = 'backend did not publish effort-only 12-vector'
            return
        if not all(math.isfinite(value) for value in point.effort):
            self.failure = 'backend command contains non-finite effort'
            return
        self.max_torque_seen = max(
            self.max_torque_seen, *(abs(value) for value in point.effort))
        if self.max_torque_seen > self.torque_limit + 1.0e-6:
            self.failure = 'backend exceeded configured torque limit'
        self.command = msg

    def on_odom(self, msg: Odometry) -> None:
        values = (
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            msg.pose.pose.position.z, msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        if not all(math.isfinite(value) for value in values):
            self.failure = 'odometry contains a non-finite value'
            return
        self.odom = msg

    def run(self) -> bool:
        wall_deadline = time.monotonic() + self.wall_timeout
        while rclpy.ok() and time.monotonic() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.failure:
                self.get_logger().error(self.failure)
                return False
            if self.odom is not None and self.command is not None \
                    and self.joints_valid:
                break
        else:
            self.get_logger().error('timed out waiting for live controller data')
            return False

        start_sim = self.get_clock().now().nanoseconds * 1.0e-9
        start_x = self.odom.pose.pose.position.x
        start_y = self.odom.pose.pose.position.y
        while rclpy.ok() and time.monotonic() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.failure:
                self.get_logger().error(self.failure)
                return False
            if self.odom is None:
                continue
            pose = self.odom.pose.pose
            q = pose.orientation
            sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
            cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
            pitch = math.asin(sinp)
            tilt = max(abs(roll), abs(pitch))
            drift = math.hypot(
                pose.position.x - start_x, pose.position.y - start_y)
            self.min_height_seen = min(
                self.min_height_seen, pose.position.z)
            self.max_tilt_seen = max(self.max_tilt_seen, tilt)
            self.max_drift_seen = max(self.max_drift_seen, drift)
            if pose.position.z < self.minimum_height:
                self.failure = 'base fell below minimum standing height'
            elif tilt > self.maximum_tilt:
                self.failure = 'base tilt exceeded standing limit'
            elif drift > self.maximum_drift:
                self.failure = 'base drift exceeded standing limit'
            if self.failure:
                self.get_logger().error(self.failure)
                return False

            elapsed = self.get_clock().now().nanoseconds * 1.0e-9 - start_sim
            if elapsed >= self.duration:
                if self.require_wbc_policy and not self.wbc_policy_seen:
                    self.get_logger().error(
                        'no active four-foot WBC/OCS2 policy was observed')
                    return False
                self.get_logger().info(
                    f'PASS duration={elapsed:.3f}s '
                    f'min_height={self.min_height_seen:.3f}m '
                    f'max_tilt={self.max_tilt_seen:.3f}rad '
                    f'max_drift={self.max_drift_seen:.3f}m '
                    f'max_torque={self.max_torque_seen:.3f}Nm')
                return True
        self.get_logger().error('wall timeout before requested simulation time')
        return False


def main() -> None:
    rclpy.init()
    node = StandingSmokeTest()
    try:
        passed = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
