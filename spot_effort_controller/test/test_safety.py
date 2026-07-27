import math

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from spot_effort_controller.constants import JOINT_NAMES, NOMINAL_STAND
from spot_effort_controller.safety import reorder_joint_state, TorqueLimiter
from spot_effort_controller.standing_core import StandingController
from spot_effort_controller.standing_node import StandingNode


def test_joint_reordering_is_authoritative():
    reversed_names = tuple(reversed(JOINT_NAMES))
    reversed_values = tuple(range(12))
    ordered = reorder_joint_state(reversed_names, reversed_values)
    assert ordered == tuple(reversed(range(12)))


def test_joint_mapping_rejects_missing_duplicate_and_nan():
    with pytest.raises(ValueError):
        reorder_joint_state(JOINT_NAMES[:-1], range(11))
    names = list(JOINT_NAMES)
    names[-1] = names[0]
    with pytest.raises(ValueError):
        reorder_joint_state(names, range(12))
    values = [0.0] * 12
    values[3] = math.nan
    with pytest.raises(ValueError):
        reorder_joint_state(JOINT_NAMES, values)


def test_torque_saturation_and_rate_limit():
    limiter = TorqueLimiter(torque_limit=10.0, rate_limit=20.0)
    output = limiter.apply([100.0] * 12, 0.1)
    assert output == (2.0,) * 12
    for _ in range(10):
        output = limiter.apply([100.0] * 12, 0.1)
    assert output == (10.0,) * 12


def test_torque_limiter_rejects_bad_dimensions_nan_and_dt():
    limiter = TorqueLimiter(10.0, 20.0)
    with pytest.raises(ValueError):
        limiter.apply([0.0] * 11, 0.01)
    bad = [0.0] * 12
    bad[0] = math.nan
    with pytest.raises(ValueError):
        limiter.apply(bad, 0.01)
    with pytest.raises(ValueError):
        limiter.apply([0.0] * 12, 0.0)


def test_watchdog_fallback_target_rate_limits_to_zero():
    limiter = TorqueLimiter(10.0, 20.0)
    assert limiter.apply([8.0] * 12, 0.1) == (2.0,) * 12
    assert limiter.apply([0.0] * 12, 0.05) == (1.0,) * 12
    assert limiter.apply([0.0] * 12, 0.05) == (0.0,) * 12


def test_standing_dimensions_finite_and_gravity_feedforward():
    controller = StandingController(kp=(1.0,) * 12, kd=(2.0,) * 12,
                                    desired=(0.0,) * 12)
    output = controller.compute((1.0,) * 12, (0.5,) * 12, (3.0,) * 12)
    assert output == (1.0,) * 12
    with pytest.raises(ValueError):
        controller.compute((0.0,) * 11, (0.0,) * 12, (0.0,) * 12)


def test_standing_initialization_disables_torque_until_valid_state():
    rclpy.init()
    node = StandingNode(parameter_overrides=[
        Parameter('control_rate', value=200.0),
        Parameter('state_timeout', value=0.10),
        Parameter('initialization_samples', value=10),
        Parameter('transition_duration', value=2.0),
        Parameter('nominal_joint_position', value=list(NOMINAL_STAND)),
        Parameter('kp', value=[400.0, 500.0, 500.0] * 4),
        Parameter('kd', value=[15.0, 20.0, 20.0] * 4),
    ])
    published = []
    node.publish = lambda efforts: published.append(tuple(efforts))
    try:
        node.on_timer()
        assert published == []
        assert not node.enabled

        state = JointState()
        state.name = list(JOINT_NAMES)
        state.position = list(NOMINAL_STAND)
        state.velocity = [0.0] * len(JOINT_NAMES)
        for _ in range(node.required_samples - 1):
            node.on_state(state)
        node.on_timer()
        assert published == []
        assert not node.enabled

        node.on_state(state)
        node.on_timer()
        assert len(published) == 1
        assert node.enabled
        assert len(published[0]) == len(JOINT_NAMES)
        assert all(math.isfinite(value) for value in published[0])
    finally:
        node.destroy_node()
        rclpy.shutdown()
