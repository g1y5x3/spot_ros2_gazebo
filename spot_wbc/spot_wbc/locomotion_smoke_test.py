#!/usr/bin/env python3
"""Run one bounded /cmd_vel axis and verify signed, upright motion."""

import json
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def yaw_from_odometry(message):
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def body_displacement(start, finish, yaw):
    dx = finish.pose.pose.position.x - start.pose.pose.position.x
    dy = finish.pose.pose.position.y - start.pose.pose.position.y
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def wrapped_angle_delta(start, finish):
    return math.atan2(math.sin(finish - start), math.cos(finish - start))


class LocomotionSmokeTest(Node):
    def __init__(self):
        super().__init__('spot_locomotion_smoke_test')
        self.declare_parameter('axis', 'forward')
        self.declare_parameter('command_duration', 6.0)
        self.declare_parameter('precommand_settle_duration', 20.0)
        self.declare_parameter('settle_duration', 3.0)
        self.declare_parameter('wall_timeout', 180.0)
        self.declare_parameter('minimum_height', 0.35)
        self.declare_parameter('maximum_tilt', 0.35)
        self.declare_parameter('forward_command', 0.03)
        self.declare_parameter('lateral_command', 0.03)
        self.declare_parameter('yaw_command', 0.05)
        self.declare_parameter('combined_forward_command', 0.02)
        self.declare_parameter('combined_lateral_command', 0.01)
        self.declare_parameter('combined_yaw_command', 0.03)
        self.declare_parameter('minimum_forward_motion', 5.0e-4)
        self.declare_parameter('minimum_lateral_motion', 5.0e-4)
        self.declare_parameter('minimum_yaw_motion', 2.0e-4)
        self.declare_parameter('minimum_combined_forward_motion', 2.0e-4)
        self.declare_parameter('minimum_combined_lateral_motion', 2.0e-4)
        self.declare_parameter('minimum_combined_yaw_motion', 1.0e-4)
        self.axis = str(self.get_parameter('axis').value)
        if self.axis not in ('forward', 'lateral', 'yaw', 'combined'):
            raise ValueError(
                'axis must be forward, lateral, yaw, or combined')
        self.command_duration = float(
            self.get_parameter('command_duration').value)
        self.precommand_settle_duration = float(
            self.get_parameter('precommand_settle_duration').value)
        self.settle_duration = float(
            self.get_parameter('settle_duration').value)
        self.wall_timeout = float(self.get_parameter('wall_timeout').value)
        self.minimum_height = float(
            self.get_parameter('minimum_height').value)
        self.maximum_tilt = float(
            self.get_parameter('maximum_tilt').value)
        self.thresholds = {
            'forward': float(
                self.get_parameter('minimum_forward_motion').value),
            'lateral': float(
                self.get_parameter('minimum_lateral_motion').value),
            'yaw': float(self.get_parameter('minimum_yaw_motion').value),
        }
        self.combined_thresholds = {
            'forward': float(self.get_parameter(
                'minimum_combined_forward_motion').value),
            'lateral': float(self.get_parameter(
                'minimum_combined_lateral_motion').value),
            'yaw': float(self.get_parameter(
                'minimum_combined_yaw_motion').value),
        }
        self.command = Twist()
        if self.axis == 'forward':
            self.command.linear.x = float(
                self.get_parameter('forward_command').value)
        elif self.axis == 'lateral':
            self.command.linear.y = float(
                self.get_parameter('lateral_command').value)
        elif self.axis == 'yaw':
            self.command.angular.z = float(
                self.get_parameter('yaw_command').value)
        else:
            self.command.linear.x = float(
                self.get_parameter('combined_forward_command').value)
            self.command.linear.y = float(
                self.get_parameter('combined_lateral_command').value)
            self.command.angular.z = float(
                self.get_parameter('combined_yaw_command').value)

        self.odom = None
        self.stationary_ready = False
        self.failure = None
        self.min_height_seen = math.inf
        self.max_tilt_seen = 0.0
        self.qp_failures_start = None
        self.qp_failures_end = None
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/spot/odometry', self.on_odometry, 10)
        self.create_subscription(
            String, '/spot/wbc/diagnostics', self.on_diagnostics, 10)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def on_odometry(self, message):
        self.odom = message
        pose = message.pose.pose
        q = pose.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)
        sinp = max(-1.0, min(
            1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sinp)
        height = float(pose.position.z)
        tilt = max(abs(roll), abs(pitch))
        self.min_height_seen = min(self.min_height_seen, height)
        self.max_tilt_seen = max(self.max_tilt_seen, tilt)
        if height < self.minimum_height:
            self.failure = 'base fell below locomotion height limit'
        elif tilt > self.maximum_tilt:
            self.failure = 'base exceeded locomotion tilt limit'

    def on_diagnostics(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        self.stationary_ready = (
            payload.get('current_mode') == 15
            and payload.get('policy_active') is True
            and str(payload.get('status', '')).startswith(
                'WBC policy tracking active'))
        failures = payload.get('qp_failure_count')
        if isinstance(failures, int):
            self.qp_failures_end = failures

    def wait_until(self, predicate, publish=None):
        deadline = time.monotonic() + self.wall_timeout
        last_publish = -math.inf
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.failure:
                return False
            now = self.now_seconds()
            if publish is not None and now - last_publish >= 0.02:
                self.publisher.publish(publish)
                last_publish = now
            if predicate():
                return True
        self.failure = 'wall timeout before simulation condition'
        return False

    def wait_sim_duration(self, duration, publish=None):
        start = self.now_seconds()
        return self.wait_until(
            lambda: self.now_seconds() - start >= duration, publish)

    def run(self):
        if not self.wait_until(
                lambda: self.odom is not None and self.stationary_ready
                and self.now_seconds() > 0.0):
            return False
        if not self.wait_sim_duration(self.precommand_settle_duration):
            return False
        start = self.odom
        start_yaw = yaw_from_odometry(start)
        self.qp_failures_start = self.qp_failures_end

        if not self.wait_sim_duration(
                self.command_duration, self.command):
            return False
        stop = Twist()
        if not self.wait_sim_duration(self.settle_duration, stop):
            return False
        if not self.wait_until(lambda: self.stationary_ready):
            return False

        finish = self.odom
        forward, lateral = body_displacement(start, finish, start_yaw)
        yaw = wrapped_angle_delta(
            start_yaw, yaw_from_odometry(finish))
        measurements = {
            'forward': forward, 'lateral': lateral, 'yaw': yaw}
        if self.axis == 'combined':
            for component, threshold in self.combined_thresholds.items():
                if measurements[component] < threshold:
                    self.failure = (
                        f'combined {component} motion '
                        f'{measurements[component]:.6f} did not exceed '
                        f'{threshold:.6f}')
                    return False
        else:
            signed_motion = measurements[self.axis]
            if signed_motion < self.thresholds[self.axis]:
                self.failure = (
                    f'{self.axis} motion {signed_motion:.6f} did not exceed '
                    f'{self.thresholds[self.axis]:.6f}')
                return False
        failure_delta = (
            None if self.qp_failures_start is None
            or self.qp_failures_end is None
            else self.qp_failures_end - self.qp_failures_start)
        self.get_logger().info(
            f'PASS axis={self.axis} forward={forward:.6f}m '
            f'lateral={lateral:.6f}m yaw={yaw:.6f}rad '
            f'min_height={self.min_height_seen:.3f}m '
            f'max_tilt={self.max_tilt_seen:.3f}rad '
            f'qp_failure_delta={failure_delta}')
        return True


def main():
    rclpy.init()
    node = LocomotionSmokeTest()
    try:
        passed = node.run()
        if not passed:
            node.get_logger().error(node.failure or 'locomotion test failed')
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
