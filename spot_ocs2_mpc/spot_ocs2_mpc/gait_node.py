#!/usr/bin/env python3
import json
import math

from geometry_msgs.msg import Twist
from ocs2_msgs.msg import ModeSchedule
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from .gait import gait_template, select_gait


class GaitManager(Node):
    """Select stance or the conservative trot from the live command state."""

    def __init__(self):
        super().__init__('spot_gait_manager')
        self.declare_parameter('command_timeout', Parameter.Type.DOUBLE)
        self.declare_parameter('motion_threshold', Parameter.Type.DOUBLE)
        self.declare_parameter('stationary_gait', Parameter.Type.STRING)
        self.declare_parameter('moving_gait', Parameter.Type.STRING)
        self.timeout = float(self.get_parameter('command_timeout').value)
        self.threshold = float(self.get_parameter('motion_threshold').value)
        self.stationary_gait = str(
            self.get_parameter('stationary_gait').value)
        self.moving_gait = str(self.get_parameter('moving_gait').value)
        gait_template(self.stationary_gait)
        gait_template(self.moving_gait)
        self.command = (0.0, 0.0, 0.0)
        self.command_time = None
        self.selected = None
        self.last_published = None
        self.publisher = self.create_publisher(
            ModeSchedule, '/legged_robot_mpc_mode_schedule', 1)
        self.diag_pub = self.create_publisher(
            String, '/spot/gait/diagnostics', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_timer(0.05, self.update)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def on_command(self, message):
        values = (
            float(message.linear.x), float(message.linear.y),
            float(message.angular.z))
        if not all(math.isfinite(value) for value in values):
            return
        self.command = values
        self.command_time = self.now_seconds()

    def update(self):
        now = self.now_seconds()
        command_age = (
            math.inf if self.command_time is None
            else max(0.0, now - self.command_time))
        selected = select_gait(
            self.command, command_age, self.timeout,
            self.moving_gait, self.stationary_gait, self.threshold)
        if selected != self.selected:
            self.selected = selected
            self.publish_selected()

    def publish_selected(self):
        times, modes = gait_template(self.selected)
        message = ModeSchedule()
        message.event_times = list(times)
        message.mode_sequence = list(modes)
        self.publisher.publish(message)
        diagnostic = String()
        diagnostic.data = json.dumps({
            'selected_gait': self.selected,
            'switching_times': list(times),
            'mode_sequence': list(modes),
            'contact_order': [
                'front_left', 'front_right', 'rear_left', 'rear_right'],
        }, separators=(',', ':'))
        self.diag_pub.publish(diagnostic)
        if self.selected != self.last_published:
            self.get_logger().info(f'selected gait: {self.selected}')
            self.last_published = self.selected


def main():
    rclpy.init()
    node = GaitManager()
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
