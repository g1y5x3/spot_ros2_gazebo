#!/usr/bin/env python3
import json
import math

from geometry_msgs.msg import Twist
from ocs2_msgs.msg import MpcInput, MpcState, MpcTargetTrajectories
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from spot_state_interface.msg import CentroidalState
from std_msgs.msg import String

from .contracts import INPUT_DIMENSION, state_message_values
from .reference import ReferenceGenerator


class CmdVelReferenceNode(Node):
    def __init__(self):
        super().__init__('spot_cmd_vel_reference')
        for name in (
                'publish_rate', 'horizon', 'command_timeout',
                'max_vx', 'max_vy', 'max_wz',
                'max_ax', 'max_ay', 'max_aw', 'nominal_height'):
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        self.declare_parameter(
            'default_joint_position', Parameter.Type.DOUBLE_ARRAY)

        def value(name):
            return float(self.get_parameter(name).value)

        self.rate = value('publish_rate')
        self.horizon = value('horizon')
        self.timeout = value('command_timeout')
        self.generator = ReferenceGenerator(
            max_vx=value('max_vx'), max_vy=value('max_vy'),
            max_wz=value('max_wz'), max_ax=value('max_ax'),
            max_ay=value('max_ay'), max_aw=value('max_aw'),
            nominal_height=value('nominal_height'),
            default_joint_position=self.get_parameter(
                'default_joint_position').value)
        self.command = (0.0, 0.0, 0.0)
        self.last_command_time = None
        self.last_update_time = None
        self.state = None
        self.world_velocity = (0.0, 0.0, 0.0)

        self.target_pub = self.create_publisher(
            MpcTargetTrajectories, '/legged_robot_mpc_target', 1)
        self.diag_pub = self.create_publisher(
            String, '/spot/reference/diagnostics', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_subscription(
            CentroidalState, '/spot/ocs2_state', self.on_state, 10)
        self.create_timer(1.0 / self.rate, self.on_timer)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def on_command(self, message):
        command = (
            float(message.linear.x), float(message.linear.y),
            float(message.angular.z))
        if not all(math.isfinite(value) for value in command):
            self.get_logger().warn('rejected non-finite /cmd_vel')
            return
        self.command = command
        self.last_command_time = self.now_seconds()

    def on_state(self, message):
        try:
            self.state = state_message_values(message)
        except (TypeError, ValueError) as error:
            self.get_logger().warn(f'rejected estimator state: {error}')
            return
        if self.generator.x is None:
            self.generator.anchor(
                self.state[6], self.state[7], self.state[9])

    def make_target(self, now):
        current, future = self.generator.target_states(
            self.state, self.world_velocity, self.horizon)
        target = MpcTargetTrajectories()
        target.time_trajectory = [now, now + self.horizon]
        for values in (current, future):
            state = MpcState()
            state.value = list(values)
            target.state_trajectory.append(state)
            control = MpcInput()
            control.value = [0.0] * INPUT_DIMENSION
            target.input_trajectory.append(control)
        return target

    def on_timer(self):
        now = self.now_seconds()
        if self.state is None or self.generator.x is None:
            return
        if self.last_update_time is None or now < self.last_update_time:
            self.last_update_time = now
            return
        dt = min(0.2, now - self.last_update_time)
        self.last_update_time = now
        command_age = (
            math.inf if self.last_command_time is None
            else max(0.0, now - self.last_command_time))
        active = command_age <= self.timeout
        self.world_velocity = self.generator.update(
            self.command, self.state[9], dt, command_active=active)
        if not active and max(
                abs(self.generator.vx), abs(self.generator.vy),
                abs(self.generator.wz)) < 1.0e-4:
            # Do not retain the early-startup anchor after the safe standing
            # controller has settled. A later gait must start at the live
            # stationary pose rather than walking back toward spawn.
            self.generator.anchor(
                self.state[6], self.state[7], self.state[9])
        self.target_pub.publish(self.make_target(now))
        diagnostic = {
            'command_active': active,
            'command_age_sim_seconds': (
                command_age if math.isfinite(command_age) else None),
            'limited_body_velocity': [
                self.generator.vx, self.generator.vy, self.generator.wz],
            'world_velocity': list(self.world_velocity),
            'reference_xy_yaw': [
                self.generator.x, self.generator.y, self.generator.yaw],
            'nominal_height': self.generator.nominal_height,
        }
        message = String()
        message.data = json.dumps(diagnostic, separators=(',', ':'))
        self.diag_pub.publish(message)


def main():
    rclpy.init()
    node = CmdVelReferenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
