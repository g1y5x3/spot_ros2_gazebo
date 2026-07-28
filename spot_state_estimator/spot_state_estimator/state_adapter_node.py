#!/usr/bin/env python3
import json
import math

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from spot_effort_controller.safety import reorder_joint_state
from spot_state_interface.msg import CentroidalState
from .math_utils import (
    normalized_quaternion, quaternion_to_yaw_pitch_roll, stamp_seconds,
    timestamps_consistent)


class StateAdapter(Node):
    """Simulation-only estimator using Gazebo ground-truth odometry."""

    def __init__(self):
        super().__init__('spot_state_adapter')
        urdf = (get_package_share_directory('spot_description') + '/models/spot/model.urdf')
        self.declare_parameter('urdf', urdf)
        self.declare_parameter('publish_rate', 100.0)
        # Maximum age of the oldest joint, IMU, or odometry sample.
        self.declare_parameter('max_age', 0.20)
        # Maximum timestamp separation between those three input streams.
        self.declare_parameter('max_skew', 0.11)
        self.max_age = float(self.get_parameter('max_age').value)
        self.max_skew = float(self.get_parameter('max_skew').value)

        self.model = pin.buildModelFromUrdf(str(self.get_parameter('urdf').value), pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        self.mass = float(pin.computeTotalMass(self.model))
        if self.model.nq != 19 or self.model.nv != 18: raise ValueError('expected 12-joint free-flyer model')

        self.joints = None
        self.imu = None
        self.odom = None
        self.contacts = [False] * 4

        self.state_pub = self.create_publisher(CentroidalState, '/spot/ocs2_state', 10)
        self.diag_pub = self.create_publisher(String, '/spot/state_adapter/diagnostics', 10)
        self.create_subscription(JointState, '/spot/joint_states', self.on_joints, 20)
        self.create_subscription(Imu, '/spot/imu', self.on_imu, 20)
        self.create_subscription(Odometry, '/spot/odometry', self.on_odom, 20)
        self.create_subscription(Contacts, '/spot/contact/front_left' , lambda msg: self.on_contact(0, msg), 10)
        self.create_subscription(Contacts, '/spot/contact/front_right', lambda msg: self.on_contact(1, msg), 10)
        self.create_subscription(Contacts, '/spot/contact/rear_left'  , lambda msg: self.on_contact(2, msg), 10)
        self.create_subscription(Contacts, '/spot/contact/rear_right' , lambda msg: self.on_contact(3, msg), 10)

        rate = float(self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        # Suppress repeated logs while still publishing diagnostics every cycle.
        self.last_reason = None
        self.get_logger().warn('using Gazebo ground-truth odometry (simulation only)')

    def on_joints(self, msg):
        self.joints = msg

    def on_imu(self, msg):
        self.imu = msg

    def on_odom(self, msg):
        self.odom = msg

    def on_contact(self, index, msg):
        self.contacts[index] = bool(msg.contacts)

    def diagnostic(self, ready, reason, stamps=()):
        payload = {
            'ready': ready,
            'reason': reason,
            'contacts_fl_fr_rl_rr': self.contacts,
            'source_stamps': list(stamps),
            'state_dimension': 24,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.diag_pub.publish(msg)
        if reason != self.last_reason:
            if ready:
                self.get_logger().info(reason)
            else:
                self.get_logger().warn(reason)
            self.last_reason = reason

    def on_timer(self):
        if self.joints is None or self.imu is None or self.odom is None:
            self.diagnostic(False, 'waiting for joint/IMU/odometry inputs')
            return
        now_time = self.get_clock().now()
        now = now_time.nanoseconds * 1.0e-9
        stamps = (
            stamp_seconds(self.joints.header.stamp),
            stamp_seconds(self.imu.header.stamp),
            stamp_seconds(self.odom.header.stamp)
        )
        if not timestamps_consistent(stamps, now, self.max_age, self.max_skew):
            self.diagnostic(False, 'inputs are stale or timestamp-skewed', stamps)
            return
        try:
            position = reorder_joint_state(self.joints.name, self.joints.position)
            velocity = reorder_joint_state(self.joints.name, self.joints.velocity)
            pose = self.odom.pose.pose
            quat = normalized_quaternion(pose.orientation.x,
                                         pose.orientation.y,
                                         pose.orientation.z,
                                         pose.orientation.w)
            yaw_pitch_roll = quaternion_to_yaw_pitch_roll(*quat)
            q = np.asarray((pose.position.x, pose.position.y, pose.position.z, *quat, *position))

            # Pinocchio's free-flyer velocity is local-frame linear, angular,
            # then joints. Gazebo odometry publishes base-frame twist.
            twist = self.odom.twist.twist
            v = np.asarray((twist.linear.x, twist.linear.y, twist.linear.z,
                            twist.angular.x, twist.angular.y, twist.angular.z,
                            *velocity))
            if not np.all(np.isfinite(q)) or not np.all(np.isfinite(v)):
                raise ValueError('pose/twist is non-finite')

            # Compute whole-body centroidal momentum hg = Ag(q) @ v for OCS2.
            pin.ccrba(self.model, self.data, q, v)
            momentum = self.data.hg.vector / self.mass
            normalized_momentum = (
                float(momentum[0]),  # normalized linear momentum x
                float(momentum[1]),  # normalized linear momentum y
                float(momentum[2]),  # normalized linear momentum z
                float(momentum[3]),  # normalized angular momentum x
                float(momentum[4]),  # normalized angular momentum y
                float(momentum[5]))  # normalized angular momentum z
            base_position = (
                pose.position.x, pose.position.y, pose.position.z)
            base_ypr = tuple(float(value) for value in yaw_pitch_roll)
            joint_position = tuple(float(value) for value in position)
            state = (
                normalized_momentum + base_position
                + base_ypr + joint_position)
            if not all(math.isfinite(value) for value in state):
                raise ValueError('assembled OCS2 state is invalid')

        except (ValueError, TypeError) as error:
            self.diagnostic(False, f'state rejected: {error}', stamps)
            return

        msg = CentroidalState()
        msg.header.stamp = now_time.to_msg()
        msg.header.frame_id = self.odom.header.frame_id
        msg.normalized_momentum = normalized_momentum
        msg.base_position = base_position
        msg.base_ypr = base_ypr
        msg.joint_position = joint_position
        self.state_pub.publish(msg)
        self.diagnostic(True, 'state adapter ready', stamps)


def main():
    rclpy.init()
    node = StateAdapter()
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
