#!/usr/bin/env python3
"""Standalone safe standing controller, without OCS2."""

import math

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .constants import JOINT_NAMES
from .safety import reorder_joint_state
from .standing_core import StandingController


class StandingNode(Node):
    def __init__(self, parameter_overrides=None) -> None:
        super().__init__(
            'spot_standing_controller',
            parameter_overrides=parameter_overrides)
        default_urdf = (
            get_package_share_directory('spot_description')
            + '/models/spot/model.urdf')
        self.declare_parameter('state_topic', '/spot/joint_states')
        self.declare_parameter('output_topic', '/spot/effort_command')
        self.declare_parameter('urdf', default_urdf)
        for name in ('control_rate', 'state_timeout', 'transition_duration'):
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        self.declare_parameter(
            'initialization_samples', Parameter.Type.INTEGER)
        for name in ('nominal_joint_position', 'kp', 'kd'):
            self.declare_parameter(name, Parameter.Type.DOUBLE_ARRAY)

        state_topic = str(self.get_parameter('state_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        urdf = str(self.get_parameter('urdf').value)
        rate = float(self.get_parameter('control_rate').value)
        self.state_timeout = float(self.get_parameter('state_timeout').value)
        self.required_samples = int(
            self.get_parameter('initialization_samples').value)
        self.transition_duration = float(
            self.get_parameter('transition_duration').value)
        self.nominal_joint_position = self.parameter_vector(
            'nominal_joint_position')
        kp = self.parameter_vector('kp')
        kd = self.parameter_vector('kd')
        if (
                rate <= 0.0 or self.state_timeout <= 0.0
                or self.required_samples < 1
                or self.transition_duration <= 0.0):
            raise ValueError('invalid controller timing configuration')

        self.model = pin.buildModelFromUrdf(urdf)
        if self.model.nq != len(JOINT_NAMES) or tuple(
                self.model.names[1:]) != JOINT_NAMES:
            raise ValueError('Pinocchio URDF joint order does not match Spot')
        self.data = self.model.createData()
        self.core = StandingController(
            kp=kp, kd=kd, desired=self.nominal_joint_position)
        self.position = None
        self.velocity = None
        self.last_state_time = None
        self.valid_samples = 0
        self.enabled = False
        self.transition_start = None
        self.transition_position = None

        self.publisher = self.create_publisher(
            JointTrajectory, output_topic, 1)
        self.subscription = self.create_subscription(
            JointState, state_topic, self.on_state, 10)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'standing controller waiting for {self.required_samples} '
            'consistently ordered states')

    def parameter_vector(self, name):
        values = tuple(
            float(value) for value in self.get_parameter(name).value)
        if len(values) != len(JOINT_NAMES) or not all(
                math.isfinite(value) for value in values):
            raise ValueError(f'{name} must contain 12 finite values')
        return values

    def on_state(self, msg: JointState) -> None:
        try:
            position = reorder_joint_state(msg.name, msg.position)
            velocity = reorder_joint_state(msg.name, msg.velocity)
        except ValueError as error:
            self.valid_samples = 0
            self.get_logger().warn(
                f'rejecting invalid joint state: {error}',
                throttle_duration_sec=1.0)
            return
        self.position = position
        self.velocity = velocity
        self.last_state_time = self.get_clock().now().nanoseconds * 1.0e-9
        self.valid_samples = min(self.required_samples, self.valid_samples + 1)

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
        state_ready = (
            self.position is not None
            and self.velocity is not None
            and self.last_state_time is not None
            and now - self.last_state_time <= self.state_timeout
            and self.valid_samples >= self.required_samples)
        if not state_ready:
            if self.enabled:
                self.get_logger().error(
                    'joint state became stale; stopping torque commands so the '
                    'backend watchdog enters fallback')
            self.enabled = False
            self.transition_start = None
            return

        if self.transition_start is None:
            self.transition_start = now
            self.transition_position = self.position
        blend = max(
            0.0, min(1.0, (now - self.transition_start)
                     / self.transition_duration))
        # Cubic smoothstep has zero endpoint velocity and avoids a startup
        # torque impulse when Gazebo initializes joints away from nominal.
        blend = blend * blend * (3.0 - 2.0 * blend)
        self.core.desired = tuple(
            initial + blend * (nominal - initial)
            for initial, nominal in zip(
                self.transition_position, self.nominal_joint_position))

        gravity = tuple(float(value) for value in pin.computeGeneralizedGravity(
            self.model, self.data, np.asarray(self.position)))
        try:
            effort = self.core.compute(
                self.position, self.velocity, gravity)
        except ValueError as error:
            self.get_logger().error(f'controller rejected state: {error}')
            self.enabled = False
            return
        if not all(math.isfinite(value) for value in effort):
            self.get_logger().error('controller produced a non-finite torque')
            self.enabled = False
            return
        if not self.enabled:
            self.get_logger().info('valid state acquired; standing enabled')
            self.enabled = True
        self.publish(effort)


def main() -> None:
    rclpy.init()
    node = StandingNode()
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
