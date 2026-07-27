import math

from nav_msgs.msg import Odometry

from spot_wbc.locomotion_smoke_test import (
    body_displacement, wrapped_angle_delta, yaw_from_odometry)


def odometry(x=0.0, y=0.0, yaw=0.0):
    message = Odometry()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(0.5 * yaw)
    message.pose.pose.orientation.w = math.cos(0.5 * yaw)
    return message


def test_body_frame_projection_and_yaw():
    start = odometry(yaw=math.pi / 2.0)
    finish = odometry(x=-2.0, y=3.0, yaw=math.pi / 2.0 + 0.2)
    forward, lateral = body_displacement(start, finish, math.pi / 2.0)
    assert math.isclose(forward, 3.0, abs_tol=1.0e-12)
    assert math.isclose(lateral, 2.0, abs_tol=1.0e-12)
    assert math.isclose(
        yaw_from_odometry(finish), math.pi / 2.0 + 0.2,
        abs_tol=1.0e-12)


def test_wrapped_yaw_delta():
    delta = wrapped_angle_delta(math.pi - 0.1, -math.pi + 0.2)
    assert math.isclose(delta, 0.3, abs_tol=1.0e-12)
