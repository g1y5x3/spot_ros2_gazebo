#!/usr/bin/env python3
import json
import math
import time

from ocs2_msgs.msg import (
    MpcFlattenedController, MpcInput, MpcObservation, MpcState,
    MpcTargetTrajectories)
from ocs2_msgs.srv import Reset
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from std_msgs.msg import Float64MultiArray, String

from .contracts import (
    INPUT_DIMENSION, STANCE_MODE, standing_target, validate_policy_dimensions,
    validate_state)


class ObservationBridge(Node):
    """Connect the replaceable Spot estimator to OCS2's standard ROS API."""

    def __init__(self):
        super().__init__('spot_ocs2_observation_bridge')
        for name in (
                'reset_retry_period', 'diagnostic_rate',
                'observation_rate'):
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        self.last_state = None
        self.last_state_time = None
        self.reset_pending = False
        self.initialized = False
        self.reset_error = 'waiting for live state and MPC reset service'
        self.policy_count = 0
        self.policy_age = math.inf
        self.policy_interval_ms = math.inf
        self.last_policy_wall = None
        self.policy_state_dim = 0
        self.policy_input_dim = 0
        self.policy_points = 0
        self.policy_cost = math.nan
        self.policy_valid = False

        self.observation_pub = self.create_publisher(
            MpcObservation, '/legged_robot_mpc_observation', 1)
        self.diagnostic_pub = self.create_publisher(
            String, '/spot/mpc/diagnostics', 10)
        self.reset_client = self.create_client(
            Reset, '/legged_robot_mpc_reset')
        self.create_subscription(
            Float64MultiArray, '/spot/ocs2_state', self.on_state, 10)
        policy_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            MpcFlattenedController, '/legged_robot_mpc_policy',
            self.on_policy, policy_qos)
        retry_period = float(
            self.get_parameter('reset_retry_period').value)
        diagnostic_rate = float(
            self.get_parameter('diagnostic_rate').value)
        observation_rate = float(
            self.get_parameter('observation_rate').value)
        self.create_timer(retry_period, self.try_reset)
        self.create_timer(1.0 / diagnostic_rate, self.publish_diagnostic)
        self.create_timer(1.0 / observation_rate, self.publish_observation)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def make_target_message(now, state):
        target_state = MpcState()
        target_state.value = list(standing_target(state))
        target_input = MpcInput()
        target_input.value = [0.0] * INPUT_DIMENSION
        target = MpcTargetTrajectories()
        # Two points make interpolation well-defined in all OCS2 consumers.
        target.time_trajectory = [now, now + 1.0]
        target.state_trajectory = [target_state, target_state]
        target.input_trajectory = [target_input, target_input]
        return target

    def try_reset(self):
        if self.initialized or self.reset_pending or self.last_state is None:
            return
        if not self.reset_client.service_is_ready():
            self.reset_error = 'waiting for /legged_robot_mpc_reset'
            return
        request = Reset.Request()
        request.reset = True
        request.target_trajectories = self.make_target_message(
            self.last_state_time, self.last_state)
        self.reset_pending = True
        future = self.reset_client.call_async(request)
        future.add_done_callback(self.on_reset)

    def on_reset(self, future):
        self.reset_pending = False
        try:
            response = future.result()
            self.initialized = bool(response.done)
            self.reset_error = (
                'MPC initialized from live state'
                if self.initialized else 'MPC reset service rejected request')
        except Exception as error:  # rclpy service errors are runtime-specific
            self.reset_error = f'MPC reset failed: {error}'
        if self.initialized:
            self.get_logger().info(self.reset_error)
        else:
            self.get_logger().error(self.reset_error)

    def on_state(self, msg):
        try:
            state = validate_state(msg.data)
        except (TypeError, ValueError) as error:
            self.reset_error = f'estimator state rejected: {error}'
            return
        now = self.now_seconds()
        self.last_state = state
        self.last_state_time = now

    def publish_observation(self):
        if not self.initialized or self.last_state is None:
            return
        now = self.now_seconds()
        observation = MpcObservation()
        observation.time = now
        observation.state.value = list(self.last_state)
        observation.input.value = [0.0] * INPUT_DIMENSION
        observation.mode = STANCE_MODE
        self.observation_pub.publish(observation)

    def on_policy(self, msg):
        wall_now = time.monotonic()
        if self.last_policy_wall is not None:
            self.policy_interval_ms = (
                wall_now - self.last_policy_wall) * 1000.0
        self.last_policy_wall = wall_now
        self.policy_count += 1
        self.policy_age = max(
            0.0, self.now_seconds() - msg.init_observation.time)
        self.policy_points = len(msg.time_trajectory)
        self.policy_state_dim = (
            len(msg.state_trajectory[0].value)
            if msg.state_trajectory else 0)
        self.policy_input_dim = (
            len(msg.input_trajectory[0].value)
            if msg.input_trajectory else 0)
        self.policy_cost = float(msg.performance_indices.cost)
        try:
            validate_policy_dimensions(
                [item.value for item in msg.state_trajectory],
                [item.value for item in msg.input_trajectory])
            self.policy_valid = all(
                math.isfinite(float(value))
                for item in msg.state_trajectory for value in item.value)
            self.policy_valid = self.policy_valid and all(
                math.isfinite(float(value))
                for item in msg.input_trajectory for value in item.value)
        except ValueError:
            self.policy_valid = False

    def publish_diagnostic(self):
        payload = {
            'initialized_from_live_state': self.initialized,
            'status': self.reset_error,
            'asynchronous_process_boundary': True,
            'policy_valid': self.policy_valid,
            'policy_count': self.policy_count,
            'policy_age_sim_seconds': (
                self.policy_age if math.isfinite(self.policy_age) else None),
            'policy_interval_wall_ms': (
                self.policy_interval_ms
                if math.isfinite(self.policy_interval_ms) else None),
            'state_dimension': self.policy_state_dim,
            'input_dimension': self.policy_input_dim,
            'contact_force_dimension': 12,
            'joint_velocity_dimension': 12,
            'contact_order': [
                'front_left_ee', 'front_right_ee',
                'rear_left_ee', 'rear_right_ee'],
            'trajectory_points': self.policy_points,
            'cost': self.policy_cost if math.isfinite(self.policy_cost) else None,
        }
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'))
        self.diagnostic_pub.publish(message)


def main():
    rclpy.init()
    node = ObservationBridge()
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
