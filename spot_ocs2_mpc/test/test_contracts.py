import math

import pytest
from spot_state_interface.msg import CentroidalState

from spot_ocs2_mpc.contracts import (
    CONTACT_FORCE_DIMENSION, INPUT_DIMENSION, JOINT_VELOCITY_DIMENSION,
    NOMINAL_HEIGHT, STATE_DIMENSION, state_message_time,
    state_message_values, standing_target,
    validate_policy_dimensions, validate_state)


def test_spot_centroidal_dimensions():
    assert STATE_DIMENSION == 24
    assert INPUT_DIMENSION == 24
    assert CONTACT_FORCE_DIMENSION == 4 * 3
    assert JOINT_VELOCITY_DIMENSION == 12
    assert validate_policy_dimensions(
        [[0.0] * STATE_DIMENSION], [[0.0] * INPUT_DIMENSION])


def test_centroidal_state_message_flattens_to_ocs2_order():
    message = CentroidalState()
    message.normalized_momentum = [float(value) for value in range(6)]
    message.base_position = [6.0, 7.0, 8.0]
    message.base_ypr = [9.0, 10.0, 11.0]
    message.joint_position = [float(value) for value in range(12, 24)]

    assert state_message_values(message) == tuple(
        float(value) for value in range(24))


def test_centroidal_state_message_exposes_header_time():
    message = CentroidalState()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 345000000

    assert state_message_time(message) == pytest.approx(12.345)


def test_standing_target_preserves_planar_pose_and_yaw():
    state = [float(index) for index in range(STATE_DIMENSION)]
    target = standing_target(state)
    assert target[:6] == (0.0,) * 6
    assert target[6:8] == (6.0, 7.0)
    assert target[8] == NOMINAL_HEIGHT
    assert target[9] == 9.0
    assert target[10:12] == (0.0, 0.0)
    assert len(target[12:]) == 12


def test_invalid_state_and_policy_are_rejected():
    with pytest.raises(ValueError):
        validate_state([0.0] * 23)
    bad = [0.0] * STATE_DIMENSION
    bad[2] = math.nan
    with pytest.raises(ValueError):
        validate_state(bad)
    with pytest.raises(ValueError):
        validate_policy_dimensions(
            [[0.0] * STATE_DIMENSION], [[0.0] * (INPUT_DIMENSION - 1)])
