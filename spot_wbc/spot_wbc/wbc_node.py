#!/usr/bin/env python3
import bisect
from concurrent.futures import ThreadPoolExecutor
import json
import math
import time

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import numpy as np
from ocs2_msgs.msg import MpcFlattenedController
import pinocchio as pin
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import JointState
from spot_state_interface.msg import CentroidalState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from spot_effort_controller.constants import JOINT_NAMES
from spot_effort_controller.safety import reorder_joint_state
from spot_ocs2_mpc.contracts import (
    state_message_time, state_message_values, validate_policy_dimensions)

from .qp import WholeBodyQp, stance_flags
from .safety import (
    apply_policy_correction, policy_is_fresh, select_bounded_torque)
from .startup import (
    StartupGate, smooth_posture_target, startup_posture_ready)


CONTACT_FRAMES = (
    'front_left_ee', 'front_right_ee', 'rear_left_ee', 'rear_right_ee')


class WholeBodyControllerNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('spot_wbc', parameter_overrides=parameter_overrides)
        urdf = (get_package_share_directory('spot_description')
                + '/models/spot/model.urdf')
        self.declare_parameter('urdf', urdf)
        self.declare_parameter(
            'enable_policy_tracking', Parameter.Type.BOOL)
        scalar_parameters = (
            'control_rate', 'state_timeout', 'qp_rate', 'stance_qp_rate',
            'policy_timeout',
            'initialization_samples',
            'torque_limit', 'policy_torque_delta_limit',
            'stance_policy_torque_delta_limit',
            'degraded_swing_kp', 'degraded_swing_kd',
            'degraded_swing_torque_delta', 'startup_transition_duration',
            'startup_minimum_height', 'startup_maximum_tilt',
            'startup_maximum_joint_speed', 'startup_stable_duration',
            'policy_blend_duration', 'stance_policy_blend_duration',
            'friction_coefficient',
            'maximum_contact_force', 'maximum_joint_acceleration',
            'joint_limit_horizon', 'qp_max_iterations', 'qp_tolerance',
            'force_tracking_weight', 'base_acceleration_weight',
            'joint_acceleration_weight', 'joint_velocity_tracking_gain',
            'swing_tracking_weight', 'acceleration_regularization',
            'torque_regularization', 'swing_position_gain',
            'swing_velocity_gain', 'maximum_base_acceleration',
        )
        for name in scalar_parameters:
            parameter_type = (
                Parameter.Type.INTEGER
                if name in ('initialization_samples', 'qp_max_iterations')
                else Parameter.Type.DOUBLE)
            self.declare_parameter(name, parameter_type)
        for name in (
                'feedback_kp', 'feedback_kd', 'fallback_kp', 'fallback_kd',
                'base_position_gain', 'base_velocity_gain',
                'base_orientation_gain', 'base_angular_velocity_gain',
                'nominal_joint_position'):
            self.declare_parameter(name, Parameter.Type.DOUBLE_ARRAY)

        def value(name):
            return float(self.get_parameter(name).value)

        self.control_rate = value('control_rate')
        self.qp_rate = value('qp_rate')
        self.stance_qp_rate = value('stance_qp_rate')
        if min(
                self.control_rate, self.qp_rate,
                self.stance_qp_rate) <= 0.0:
            raise ValueError('control and QP rates must be positive')
        self.required_joint_samples = int(
            self.get_parameter('initialization_samples').value)
        self.enable_policy_tracking = bool(
            self.get_parameter('enable_policy_tracking').value)
        self.state_timeout = value('state_timeout')
        self.policy_timeout = value('policy_timeout')
        self.transition_duration = value('startup_transition_duration')
        self.startup_minimum_height = value('startup_minimum_height')
        self.startup_maximum_tilt = value('startup_maximum_tilt')
        self.startup_maximum_joint_speed = value(
            'startup_maximum_joint_speed')
        self.startup_gate = StartupGate(value('startup_stable_duration'))
        self.policy_blend_duration = value('policy_blend_duration')
        self.stance_policy_blend_duration = value(
            'stance_policy_blend_duration')
        self.torque_limit = value('torque_limit')
        self.policy_torque_delta_limit = value(
            'policy_torque_delta_limit')
        self.stance_policy_torque_delta_limit = value(
            'stance_policy_torque_delta_limit')
        self.degraded_swing_kp = value('degraded_swing_kp')
        self.degraded_swing_kd = value('degraded_swing_kd')
        self.degraded_swing_torque_delta = value(
            'degraded_swing_torque_delta')
        self.swing_position_gain = value('swing_position_gain')
        self.swing_velocity_gain = value('swing_velocity_gain')
        self.maximum_base_acceleration = value(
            'maximum_base_acceleration')
        self.feedback_kp = self.parameter_vector('feedback_kp')
        self.feedback_kd = self.parameter_vector('feedback_kd')
        self.fallback_kp = self.parameter_vector('fallback_kp')
        self.fallback_kd = self.parameter_vector('fallback_kd')
        self.nominal_joint_position = self.parameter_vector(
            'nominal_joint_position')
        self.base_position_gain = self.parameter_vector(
            'base_position_gain', 3)
        self.base_velocity_gain = self.parameter_vector(
            'base_velocity_gain', 3)
        self.base_orientation_gain = self.parameter_vector(
            'base_orientation_gain', 3)
        self.base_angular_velocity_gain = self.parameter_vector(
            'base_angular_velocity_gain', 3)
        self.qp = WholeBodyQp(
            friction_coefficient=value('friction_coefficient'),
            torque_limit=self.torque_limit,
            maximum_contact_force=value('maximum_contact_force'),
            maximum_joint_acceleration=value('maximum_joint_acceleration'),
            joint_limit_horizon=value('joint_limit_horizon'),
            max_iterations=int(
                self.get_parameter('qp_max_iterations').value),
            tolerance=value('qp_tolerance'),
            force_tracking_weight=value('force_tracking_weight'),
            base_acceleration_weight=value('base_acceleration_weight'),
            joint_acceleration_weight=value('joint_acceleration_weight'),
            joint_velocity_tracking_gain=value(
                'joint_velocity_tracking_gain'),
            swing_tracking_weight=value('swing_tracking_weight'),
            acceleration_regularization=value(
                'acceleration_regularization'),
            torque_regularization=value('torque_regularization'))

        self.model = pin.buildModelFromUrdf(
            str(self.get_parameter('urdf').value),
            pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        self.fixed_model = pin.buildModelFromUrdf(
            str(self.get_parameter('urdf').value))
        self.fixed_data = self.fixed_model.createData()
        if self.model.nq != 19 or self.model.nv != 18:
            raise ValueError('expected a 12-joint free-flyer model')
        if tuple(self.model.names[2:]) != JOINT_NAMES:
            raise ValueError('Pinocchio joint order does not match Spot')
        self.contact_ids = [
            self.model.getFrameId(name) for name in CONTACT_FRAMES]
        if any(index >= len(self.model.frames) for index in self.contact_ids):
            raise ValueError('a configured contact frame is absent')

        self.state = None
        self.state_time = None
        self.contacts = None
        self.contacts_time = None
        self.position = None
        self.velocity = None
        self.joint_time = None
        self.valid_joint_samples = 0
        self.odom = None
        self.odom_time = None
        self.policy = None
        self.policy_time = None
        self.failure_count = 0
        self.solve_count = 0
        self.fallback_reason = 'waiting for state and policy'
        self.last_solve_ms = math.nan
        self.last_result = None
        self.transition_start = None
        self.transition_position = None
        self.startup_complete = False
        self.startup_ready = False
        self.startup_phase = 'waiting_for_state'
        self.policy_blend_start = None
        self.last_qp_time = None
        self.cached_policy_correction = None
        self.qp_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix='spot_wbc_qp')
        self.qp_future = None
        self.qp_future_started = None
        self.qp_future_policy_state = None
        self.qp_future_policy_input = None
        self.qp_future_mode = None
        self.policy_active = False
        self.current_mode = None
        self.max_commanded_torque = 0.0
        self.last_policy_torque_delta = 0.0

        self.publisher = self.create_publisher(
            JointTrajectory, '/spot/effort_command', 1)
        self.diag_pub = self.create_publisher(
            String, '/spot/wbc/diagnostics', 10)
        self.create_subscription(
            CentroidalState, '/spot/ocs2_state', self.on_state, 10)
        self.create_subscription(
            String, '/spot/state_adapter/diagnostics',
            self.on_state_diagnostics, 10)
        self.create_subscription(
            JointState, '/spot/joint_states', self.on_joints, 20)
        self.create_subscription(
            Odometry, '/spot/odometry', self.on_odom, 20)
        policy_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            MpcFlattenedController, '/legged_robot_mpc_policy',
            self.on_policy, policy_qos)
        self.create_timer(1.0 / self.control_rate, self.on_timer)
        self.create_timer(0.5, self.publish_diagnostics)

    def parameter_vector(self, name, size=12):
        values = np.asarray(self.get_parameter(name).value, dtype=float)
        if values.shape != (size,) or not np.all(np.isfinite(values)):
            raise ValueError(f'{name} must contain {size} finite values')
        return values

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def on_state(self, message):
        try:
            self.state = np.asarray(state_message_values(message))
            self.state_time = state_message_time(message)
        except (TypeError, ValueError):
            self.state = None
            self.state_time = None

    def on_state_diagnostics(self, message):
        try:
            payload = json.loads(message.data)
            contacts = payload['contacts_fl_fr_rl_rr']
            if (
                    len(contacts) != 4
                    or not all(type(value) is bool for value in contacts)):
                raise ValueError('expected four boolean contact flags')
            self.contacts = tuple(contacts)
            self.contacts_time = self.now_seconds()
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.contacts = None
            self.contacts_time = None

    def on_joints(self, message):
        try:
            self.position = np.asarray(
                reorder_joint_state(message.name, message.position))
            self.velocity = np.asarray(
                reorder_joint_state(message.name, message.velocity))
            self.joint_time = self.now_seconds()
            self.valid_joint_samples = min(
                self.required_joint_samples,
                self.valid_joint_samples + 1)
        except ValueError:
            self.position = None
            self.velocity = None
            self.valid_joint_samples = 0

    def on_odom(self, message):
        self.odom = message
        self.odom_time = self.now_seconds()

    def on_policy(self, message):
        try:
            validate_policy_dimensions(
                [item.value for item in message.state_trajectory],
                [item.value for item in message.input_trajectory])
            if not all(
                    math.isfinite(float(value))
                    for item in message.state_trajectory
                    for value in item.value):
                raise ValueError('non-finite policy state')
            if not all(
                    math.isfinite(float(value))
                    for item in message.input_trajectory
                    for value in item.value):
                raise ValueError('non-finite policy input')
        except ValueError:
            self.policy = None
            return
        self.policy = message
        self.policy_time = self.now_seconds()

    def current_policy(self, now):
        times = self.policy.time_trajectory
        index = max(0, min(
            len(times) - 1, bisect.bisect_right(times, now) - 1))
        state = np.asarray(self.policy.state_trajectory[index].value)
        control = np.asarray(self.policy.input_trajectory[index].value)
        mode = int(self.policy.init_observation.mode)
        schedule = self.policy.mode_schedule
        if schedule.mode_sequence:
            mode_index = bisect.bisect_right(schedule.event_times, now)
            mode = int(schedule.mode_sequence[
                min(mode_index, len(schedule.mode_sequence) - 1)])
        return state, control, mode

    def generalized_state(self):
        pose = self.odom.pose.pose
        twist = self.odom.twist.twist
        quaternion = np.asarray((
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w))
        quaternion /= np.linalg.norm(quaternion)
        q = np.concatenate((
            [pose.position.x, pose.position.y, pose.position.z],
            quaternion, self.position))
        v = np.concatenate((
            [twist.linear.x, twist.linear.y, twist.linear.z],
            [twist.angular.x, twist.angular.y, twist.angular.z],
            self.velocity))
        return q, v

    def dynamics_terms(self, q, v):
        pin.computeAllTerms(self.model, self.data, q, v)
        pin.forwardKinematics(
            self.model, self.data, q, v, np.zeros(self.model.nv))
        pin.updateFramePlacements(self.model, self.data)
        jacobians = np.zeros((12, 18))
        drift = np.zeros(12)
        for index, frame_id in enumerate(self.contact_ids):
            jacobian = pin.computeFrameJacobian(
                self.model, self.data, q, frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            acceleration = pin.getFrameClassicalAcceleration(
                self.model, self.data, frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            jacobians[3 * index:3 * index + 3] = jacobian[:3, :]
            drift[3 * index:3 * index + 3] = acceleration.linear
        return self.data.M.copy(), self.data.nle.copy(), jacobians, drift

    def swing_targets(self, q, v, policy_state, policy_input, mode):
        targets = np.zeros(12)
        flags = stance_flags(mode)
        if all(flags):
            return targets
        rotation = pin.rpy.rpyToMatrix(
            float(policy_state[11]), float(policy_state[10]),
            float(policy_state[9]))
        quaternion = pin.Quaternion(rotation).coeffs()
        q_desired = np.concatenate((
            policy_state[6:9], quaternion, policy_state[12:24]))
        v_desired = np.concatenate((
            np.zeros(6), policy_input[12:24]))
        desired_data = self.model.createData()
        pin.forwardKinematics(
            self.model, desired_data, q_desired, v_desired)
        pin.updateFramePlacements(self.model, desired_data)
        for index, stance in enumerate(flags):
            if stance:
                continue
            frame_id = self.contact_ids[index]
            current_position = self.data.oMf[frame_id].translation
            desired_position = desired_data.oMf[frame_id].translation
            current_velocity = pin.getFrameVelocity(
                self.model, self.data, frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
            desired_velocity = pin.getFrameVelocity(
                self.model, desired_data, frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
            targets[3 * index:3 * index + 3] = (
                self.swing_position_gain
                * (desired_position - current_position)
                + self.swing_velocity_gain
                * (desired_velocity - current_velocity))
        return targets

    def startup_target(self, now):
        if self.transition_start is None:
            self.transition_start = now
            self.transition_position = self.position.copy()
        elapsed = now - self.transition_start
        if not self.startup_complete:
            self.startup_phase = (
                'raise' if elapsed < self.transition_duration
                else 'stabilize')
        return smooth_posture_target(
            self.transition_position,
            self.nominal_joint_position,
            elapsed,
            self.transition_duration)

    def fallback(self, nonlinear, target):
        torque = (
            nonlinear[6:] + self.fallback_kp
            * (target - self.position)
            - self.fallback_kd * self.velocity)
        return np.clip(torque, -self.torque_limit, self.torque_limit)

    def fixed_base_fallback(self, target):
        gravity = pin.computeGeneralizedGravity(
            self.fixed_model, self.fixed_data, self.position)
        return np.clip(
            gravity + self.fallback_kp * (target - self.position)
            - self.fallback_kd * self.velocity,
            -self.torque_limit, self.torque_limit)

    def degraded_swing_torque(
            self, baseline, policy_state, policy_input, mode):
        correction = np.zeros(12)
        for leg_index, stance in enumerate(stance_flags(mode)):
            if stance:
                continue
            joints = slice(3 * leg_index, 3 * leg_index + 3)
            correction[joints] = (
                self.degraded_swing_kp
                * (policy_state[12:24][joints] - self.position[joints])
                + self.degraded_swing_kd
                * (policy_input[12:24][joints] - self.velocity[joints]))
        correction = np.clip(
            correction, -self.degraded_swing_torque_delta,
            self.degraded_swing_torque_delta)
        return np.clip(
            baseline + correction, -self.torque_limit, self.torque_limit)

    def desired_base_acceleration(self, policy_state, generalized_velocity):
        desired = np.zeros(6)
        desired[:3] = (
            self.base_position_gain
            * (policy_state[6:9] - self.state[6:9])
            + self.base_velocity_gain
            * (policy_state[0:3] - generalized_velocity[0:3]))
        # Generalized angular acceleration is X/Y/Z; the OCS2 pose is Z/Y/X.
        orientation_error_xyz = np.asarray((
            policy_state[11] - self.state[11],
            policy_state[10] - self.state[10],
            policy_state[9] - self.state[9]))
        desired[3:6] = (
            self.base_orientation_gain * orientation_error_xyz
            - self.base_angular_velocity_gain * generalized_velocity[3:6])
        return np.clip(
            desired, -self.maximum_base_acceleration,
            self.maximum_base_acceleration)

    def publish_torque(self, torque):
        torque = np.asarray(torque, dtype=float)
        if torque.shape != (12,) or not np.all(np.isfinite(torque)):
            self.failure_count += 1
            self.fallback_reason = (
                'invalid final torque rejected; publishing zero')
            torque = np.zeros(12)
        torque = np.clip(torque, -self.torque_limit, self.torque_limit)
        self.max_commanded_torque = max(
            self.max_commanded_torque, float(np.max(np.abs(torque))))
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.effort = [float(value) for value in torque]
        message.points = [point]
        self.publisher.publish(message)

    def on_timer(self):
        now = self.now_seconds()
        joint_state_ready = (
            self.position is not None and self.velocity is not None
            and self.joint_time is not None
            and self.valid_joint_samples >= self.required_joint_samples
            and now - self.joint_time <= self.state_timeout)
        if not joint_state_ready:
            self.fallback_reason = (
                'joint state stale or incomplete; output disabled')
            return
        startup_target = self.startup_target(now)
        dynamics_ready = (
            self.odom is not None and self.odom_time is not None
            and now - self.odom_time <= self.state_timeout)
        if not dynamics_ready:
            self.fallback_reason = (
                'odometry unavailable; fixed-base startup fallback')
            self.last_policy_torque_delta = 0.0
            self.publish_torque(self.fixed_base_fallback(startup_target))
            return
        estimator_ready = (
            self.state is not None and self.state_time is not None
            and policy_is_fresh(
                now, self.state_time, self.state_timeout))
        contacts_ready = (
            self.contacts is not None and self.contacts_time is not None
            and policy_is_fresh(
                now, self.contacts_time, self.state_timeout))
        transition_elapsed = now - self.transition_start
        if not self.startup_complete:
            # Keep policy corrections out until the belly-down joint ramp has
            # finished and measured state confirms a stable four-foot stand.
            self.policy_active = False
            self.current_mode = None
            self.policy_blend_start = None
            self.cached_policy_correction = None
            self.last_policy_torque_delta = 0.0
            self.startup_ready = (
                transition_elapsed >= self.transition_duration
                and estimator_ready
                and contacts_ready
                and startup_posture_ready(
                    base_height=self.state[8],
                    roll=self.state[11],
                    pitch=self.state[10],
                    joint_velocity=self.velocity,
                    contacts=self.contacts,
                    minimum_height=self.startup_minimum_height,
                    maximum_tilt=self.startup_maximum_tilt,
                    maximum_joint_speed=self.startup_maximum_joint_speed))
            if transition_elapsed < self.transition_duration:
                self.startup_gate.reset()
                self.fallback_reason = (
                    'startup belly-to-stand transition; '
                    'fixed-base fallback')
            elif not self.startup_gate.update(now, self.startup_ready):
                self.fallback_reason = (
                    'startup holding nominal posture; waiting for stable '
                    'height, attitude, velocity, and four-foot contact')
            else:
                self.startup_complete = True
                self.startup_phase = 'complete'
                self.get_logger().info(
                    'belly-down startup complete; policy handoff enabled')
            if not self.startup_complete:
                self.publish_torque(self.fixed_base_fallback(startup_target))
                return
        policy_ready = (
            self.enable_policy_tracking and estimator_ready
            and self.policy is not None
            and policy_is_fresh(
                now, self.policy_time, self.policy_timeout))
        if not policy_ready:
            if self.policy_active:
                # Re-anchor before returning to the nominal posture. This
                # prevents a stale-policy event from becoming a joint step.
                self.transition_start = now
                self.transition_position = self.position.copy()
                startup_target = self.position.copy()
            self.policy_active = False
            self.current_mode = None
            self.policy_blend_start = None
            self.cached_policy_correction = None
            self.fallback_reason = 'MPC policy stale; standalone fallback'
            self.last_policy_torque_delta = 0.0
            fallback, _ = select_bounded_torque(
                False, None, self.fixed_base_fallback(startup_target),
                self.torque_limit, self.policy_torque_delta_limit)
            self.publish_torque(fallback)
            return
        fallback_torque = self.fixed_base_fallback(startup_target)
        policy_state, policy_input, current_mode = self.current_policy(now)
        self.current_mode = current_mode
        if current_mode != 15:
            fallback_torque = self.degraded_swing_torque(
                fallback_torque, policy_state, policy_input, current_mode)
        if self.qp_future is not None and self.qp_future.done():
            try:
                result = self.qp_future.result()
            except Exception as error:
                result = None
                self.failure_count += 1
                self.fallback_reason = f'QP worker failure: {error}'
            self.last_solve_ms = (
                time.perf_counter() - self.qp_future_started) * 1000.0
            self.last_result = result
            if self.qp_future_mode != current_mode:
                self.policy_active = False
                self.policy_blend_start = None
                self.cached_policy_correction = None
                self.fallback_reason = (
                    'contact mode changed; stale QP result discarded')
            elif result is None or not result.success:
                self.policy_active = False
                if result is not None:
                    self.failure_count += 1
                self.policy_blend_start = None
                self.cached_policy_correction = None
                if result is not None:
                    self.fallback_reason = (
                        f'QP infeasible: {result.message}')
            else:
                self.solve_count += 1
                desired_q = self.qp_future_policy_state[12:24]
                desired_dq = self.qp_future_policy_input[12:24]
                candidate = (
                    result.torque + self.feedback_kp
                    * (desired_q - self.position)
                    + self.feedback_kd * (desired_dq - self.velocity))
                delta_limit = (
                    self.stance_policy_torque_delta_limit
                    if current_mode == 15
                    else self.policy_torque_delta_limit)
                candidate, selected = select_bounded_torque(
                    True, candidate, fallback_torque,
                    self.torque_limit, delta_limit)
                if selected:
                    # Cache only the policy correction. The high-gain posture
                    # torque must continue to follow live joint state at the
                    # 200 Hz control rate between slower QP solutions.
                    self.cached_policy_correction = (
                        candidate - fallback_torque)
                    if self.policy_blend_start is None:
                        self.policy_blend_start = now
                else:
                    self.policy_active = False
                    self.failure_count += 1
                    self.policy_blend_start = None
                    self.cached_policy_correction = None
                    self.fallback_reason = (
                        'non-finite WBC torque; standalone fallback')
            self.qp_future = None
        active_qp_rate = (
            self.stance_qp_rate if current_mode == 15 else self.qp_rate)
        qp_due = (
            self.qp_future is None
            and (
                self.last_qp_time is None
                or now - self.last_qp_time >= 1.0 / active_qp_rate)
        )
        if qp_due:
            self.last_qp_time = now
            try:
                q, v = self.generalized_state()
                mass, nonlinear, jacobians, drift = self.dynamics_terms(q, v)
                swing = self.swing_targets(
                    q, v, policy_state, policy_input, current_mode)
                desired_base = self.desired_base_acceleration(policy_state, v)
                self.qp_future_started = time.perf_counter()
                self.qp_future_policy_state = policy_state.copy()
                self.qp_future_policy_input = policy_input.copy()
                self.qp_future_mode = current_mode
                self.qp_future = self.qp_executor.submit(
                    self.qp.solve,
                    mass.copy(), nonlinear.copy(), jacobians.copy(),
                    drift.copy(), self.position.copy(), self.velocity.copy(),
                    self.model.lowerPositionLimit[7:].copy(),
                    self.model.upperPositionLimit[7:].copy(),
                    policy_state.copy(), policy_input.copy(), current_mode,
                    swing.copy(), desired_base.copy())
            except (ValueError, RuntimeError) as error:
                self.fallback_reason = f'Pinocchio failure: {error}'
        if self.cached_policy_correction is None:
            self.policy_active = False
            torque = fallback_torque
        else:
            blend_duration = (
                self.stance_policy_blend_duration
                if current_mode == 15
                else self.policy_blend_duration)
            blend = min(
                1.0, max(0.0, (now - self.policy_blend_start)
                         / blend_duration))
            blend = blend * blend * (3.0 - 2.0 * blend)
            torque = apply_policy_correction(
                fallback_torque, self.cached_policy_correction,
                blend, self.torque_limit)
            self.policy_active = True
            self.fallback_reason = (
                f'WBC policy tracking active; mode={current_mode}; '
                f'blend={blend:.3f}')
        self.last_policy_torque_delta = float(np.max(np.abs(
            np.asarray(torque) - fallback_torque)))
        self.publish_torque(torque)

    def publish_diagnostics(self):
        result = self.last_result
        payload = {
            'status': self.fallback_reason,
            'policy_active': self.policy_active,
            'current_mode': self.current_mode,
            'startup_complete': self.startup_complete,
            'startup_ready': self.startup_ready,
            'startup_phase': self.startup_phase,
            'startup_contacts_fl_fr_rl_rr': self.contacts,
            'policy_age_sim_seconds': (
                None if self.policy_time is None
                else max(0.0, self.now_seconds() - self.policy_time)),
            'qp_success_count': self.solve_count,
            'qp_failure_count': self.failure_count,
            'solve_time_wall_ms': (
                self.last_solve_ms
                if math.isfinite(self.last_solve_ms) else None),
            'decision_dimension': 42,
            'torque_dimension': 12,
            'control_rate_hz': self.control_rate,
            'qp_rate_hz': self.qp_rate,
            'stance_qp_rate_hz': self.stance_qp_rate,
            'torque_limit_nm': self.torque_limit,
            'maximum_commanded_torque': self.max_commanded_torque,
            'policy_torque_delta': self.last_policy_torque_delta,
            'valid_joint_samples': self.valid_joint_samples,
            'planned_contact_schedule': True,
            'measured_contacts_ready_for_future_use': True,
        }
        if result is not None:
            payload.update({
                'iterations': result.iterations,
                'dynamics_contact_equality_residual':
                    result.equality_residual,
                'minimum_friction_margin': result.minimum_friction_margin,
                'maximum_feedforward_torque': float(
                    np.max(np.abs(result.torque))),
                'maximum_contact_force': float(
                    np.max(np.abs(result.forces))),
            })
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'))
        self.diag_pub.publish(message)


def main():
    rclpy.init()
    node = WholeBodyControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.qp_executor.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
