#!/usr/bin/env python3
"""Watchdog-protected Gazebo JointTrajectory effort backend."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .constants import JOINT_NAMES
from .safety import TorqueLimiter


class EffortBackend(Node):
    def __init__(self, parameter_overrides=None) -> None:
        super().__init__(
            'spot_effort_backend', parameter_overrides=parameter_overrides)
        self.declare_parameter('input_topic', '/spot/effort_command')
        self.declare_parameter('output_topic', '/spot/joint_trajectory')
        for name in (
                'publish_rate', 'watchdog_timeout',
                'torque_limit', 'rate_limit'):
            self.declare_parameter(name, Parameter.Type.DOUBLE)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.watchdog_timeout = float(
            self.get_parameter('watchdog_timeout').value)
        self.limiter = TorqueLimiter(
            float(self.get_parameter('torque_limit').value),
            float(self.get_parameter('rate_limit').value))
        if self.publish_rate <= 0.0 or self.watchdog_timeout <= 0.0:
            raise ValueError('publish rate and watchdog timeout must be positive')

        self.desired = (0.0,) * len(JOINT_NAMES)
        self.last_command_time = None
        self.last_update_time = None
        self.in_fallback = True
        self.publisher = self.create_publisher(
            JointTrajectory, output_topic, 1)
        self.subscription = self.create_subscription(
            JointTrajectory, input_topic, self.on_command, 1)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.on_timer)
        self.get_logger().info(
            f'effort backend ready; watchdog={self.watchdog_timeout:.3f}s, '
            f'limit={self.limiter.torque_limit:.1f}Nm')

    def on_command(self, msg: JointTrajectory) -> None:
        if tuple(msg.joint_names) != JOINT_NAMES or len(msg.points) != 1:
            self.get_logger().error(
                'rejecting command: joint order or point count is invalid')
            return
        point = msg.points[0]
        if point.positions or point.velocities:
            self.get_logger().error(
                'rejecting command: position and velocity arrays must be empty')
            return
        effort = tuple(float(value) for value in point.effort)
        if len(effort) != len(JOINT_NAMES) or not all(
                math.isfinite(value) for value in effort):
            self.get_logger().error(
                'rejecting command: expected 12 finite efforts')
            return
        self.desired = effort
        self.last_command_time = self.get_clock().now().nanoseconds * 1.0e-9

    def publish(self, efforts) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.effort = list(efforts)
        msg.points = [point]
        self.publisher.publish(msg)

    def on_timer(self) -> None:
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if self.last_update_time is None or now <= self.last_update_time:
            dt = 1.0 / self.publish_rate
        else:
            dt = max(1.0e-4, min(0.05, now - self.last_update_time))
        self.last_update_time = now
        stale = (
            self.last_command_time is None
            or now - self.last_command_time > self.watchdog_timeout)
        target = (0.0,) * len(JOINT_NAMES) if stale else self.desired
        try:
            command = self.limiter.apply(target, dt)
        except ValueError as error:
            self.get_logger().error(f'safety limiter rejected command: {error}')
            self.limiter.reset()
            command = self.limiter.output
            stale = True

        if stale != self.in_fallback:
            if stale:
                self.get_logger().warn(
                    'command watchdog expired; ramping all efforts to zero')
            else:
                self.get_logger().info('valid torque command stream acquired')
            self.in_fallback = stale
        self.publish(command)


def main() -> None:
    rclpy.init()
    node = EffortBackend()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.limiter.reset()
        if rclpy.ok():
            try:
                node.publish(node.limiter.output)
            except RuntimeError:
                if rclpy.ok():
                    raise
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
