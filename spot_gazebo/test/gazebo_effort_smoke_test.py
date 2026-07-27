#!/usr/bin/env python3
"""Apply one small effort-only command and verify the joint velocity sign."""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class EffortSmokeTest(Node):
    def __init__(self) -> None:
        super().__init__('spot_effort_smoke_test')
        self.declare_parameter('joint_name', 'front_left_hip_x')
        self.declare_parameter('effort', 2.0)
        self.declare_parameter('duration', 0.20)
        self.declare_parameter('settle_duration', 0.25)
        self.declare_parameter('state_timeout', 5.0)
        self.declare_parameter('minimum_velocity_change', 0.01)
        self.declare_parameter('command_topic', '/spot/joint_trajectory')
        self.declare_parameter('state_topic', '/spot/joint_states')

        self.joint_name = str(self.get_parameter('joint_name').value)
        self.effort = float(self.get_parameter('effort').value)
        self.duration = float(self.get_parameter('duration').value)
        self.settle_duration = float(self.get_parameter('settle_duration').value)
        self.state_timeout = float(self.get_parameter('state_timeout').value)
        self.minimum_change = float(
            self.get_parameter('minimum_velocity_change').value)
        command_topic = str(self.get_parameter('command_topic').value)
        state_topic = str(self.get_parameter('state_topic').value)

        if not math.isfinite(self.effort) or abs(self.effort) > 5.0:
            raise ValueError('effort must be finite and no greater than 5 Nm')
        if self.effort == 0.0:
            raise ValueError('effort must be nonzero')
        if not 0.02 <= self.duration <= 1.0:
            raise ValueError('duration must be between 0.02 and 1.0 seconds')
        if self.state_timeout <= 0.0:
            raise ValueError('state timeout must be positive')

        self.publisher = self.create_publisher(
            JointTrajectory, command_topic, 1)
        self.subscription = self.create_subscription(
            JointState, state_topic, self.on_joint_state, 10)
        self.command_topic = command_topic
        self.last_velocity = None
        self.baseline_velocity = None
        self.extreme_velocity = None

    def on_joint_state(self, msg: JointState) -> None:
        try:
            index = msg.name.index(self.joint_name)
        except ValueError:
            return
        if index >= len(msg.velocity) or not math.isfinite(msg.velocity[index]):
            return
        self.last_velocity = float(msg.velocity[index])

    def publish_effort(self, effort: float) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [self.joint_name]
        point = JointTrajectoryPoint()
        # Intentionally leave positions and velocities empty: this exercises
        # only the Fortress JointTrajectoryPoint.effort actuator path.
        point.effort = [effort]
        msg.points = [point]
        self.publisher.publish(msg)

    def spin_for(self, duration: float, effort=None) -> None:
        deadline = time.monotonic() + duration
        next_publish = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.002)
            now = time.monotonic()
            if effort is not None and now >= next_publish:
                self.publish_effort(effort)
                next_publish = now + 0.01
            if self.last_velocity is not None:
                if self.extreme_velocity is None:
                    self.extreme_velocity = self.last_velocity
                elif self.effort > 0.0:
                    self.extreme_velocity = max(
                        self.extreme_velocity, self.last_velocity)
                else:
                    self.extreme_velocity = min(
                        self.extreme_velocity, self.last_velocity)

    def run(self) -> bool:
        state_deadline = time.monotonic() + self.state_timeout
        while rclpy.ok() and time.monotonic() < state_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_velocity is not None:
                break
        if self.last_velocity is None:
            self.get_logger().error(
                f'no finite velocity received for {self.joint_name}')
            return False
        self.spin_for(self.settle_duration)
        if self.count_publishers(self.command_topic) != 1:
            self.get_logger().error(
                'another ROS publisher is active on the command topic; '
                'stop it before running this test')
            return False

        self.baseline_velocity = self.last_velocity
        self.extreme_velocity = self.baseline_velocity
        self.spin_for(self.duration, self.effort)
        # Explicitly clear the persistent effort command, then continue
        # publishing zero briefly so bridge / transport latency is bounded.
        self.spin_for(0.10, 0.0)
        self.publish_effort(0.0)

        velocity_change = self.extreme_velocity - self.baseline_velocity
        expected_sign = math.copysign(1.0, self.effort)
        passed = (
            expected_sign * velocity_change >= self.minimum_change
            and math.isfinite(velocity_change)
        )
        report = (
            f'joint={self.joint_name} effort={self.effort:.3f}Nm '
            f'baseline_velocity={self.baseline_velocity:.6f}rad/s '
            f'extreme_velocity={self.extreme_velocity:.6f}rad/s '
            f'delta={velocity_change:.6f}rad/s')
        if passed:
            self.get_logger().info('PASS ' + report)
        else:
            self.get_logger().error('FAIL ' + report)
        return passed


def main() -> None:
    rclpy.init()
    node = None
    exit_code = 1
    try:
        node = EffortSmokeTest()
        exit_code = 0 if node.run() else 1
    except (TypeError, ValueError) as error:
        print(f'effort smoke test configuration error: {error}', file=sys.stderr)
    finally:
        if node is not None:
            node.publish_effort(0.0)
            node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
