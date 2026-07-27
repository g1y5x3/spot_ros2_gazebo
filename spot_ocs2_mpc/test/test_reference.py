import math

import pytest

from spot_ocs2_mpc.reference import (
    ReferenceGenerator, body_to_world, move_toward)


def test_body_to_world_uses_estimated_yaw():
    vx, vy = body_to_world(1.0, 0.0, math.pi / 2.0)
    assert vx == pytest.approx(0.0, abs=1.0e-12)
    assert vy == pytest.approx(1.0)


def test_speed_and_acceleration_limits():
    generator = ReferenceGenerator(
        max_vx=0.2, max_vy=0.1, max_wz=0.4,
        max_ax=0.3, max_ay=0.2, max_aw=0.5)
    generator.anchor(0.0, 0.0, 0.0)
    generator.update((10.0, -10.0, 10.0), 0.0, 0.1)
    assert generator.vx == pytest.approx(0.03)
    assert generator.vy == pytest.approx(-0.02)
    assert generator.wz == pytest.approx(0.05)
    for _ in range(100):
        generator.update((10.0, -10.0, 10.0), 0.0, 0.1)
    assert generator.vx == pytest.approx(0.2)
    assert generator.vy == pytest.approx(-0.1)
    assert generator.wz == pytest.approx(0.4)


def test_timeout_smoothly_returns_velocity_to_zero():
    generator = ReferenceGenerator(max_ax=0.5, max_ay=0.5, max_aw=1.0)
    generator.anchor(0.0, 0.0, 0.0)
    generator.update((0.2, 0.1, 0.2), 0.0, 0.2)
    previous = (generator.vx, generator.vy, generator.wz)
    generator.update((0.2, 0.1, 0.2), 0.0, 0.1, command_active=False)
    assert 0.0 <= generator.vx < previous[0]
    assert 0.0 <= generator.vy < previous[1]
    assert 0.0 <= generator.wz < previous[2]
    assert generator.vx == pytest.approx(move_toward(previous[0], 0.0, 0.05))


def test_yaw_integration_and_continuous_trajectory():
    generator = ReferenceGenerator(max_aw=2.0)
    generator.anchor(1.0, 2.0, 0.3)
    velocity = generator.update((0.0, 0.0, 0.4), 0.3, 0.1)
    assert generator.yaw == pytest.approx(0.32)
    state = [0.0] * 24
    current, future = generator.target_states(state, velocity, 0.6)
    assert current[9] == pytest.approx(0.32)
    assert future[9] == pytest.approx(0.44)


def test_default_joint_posture_is_configurable():
    posture = tuple(0.01 * index for index in range(12))
    generator = ReferenceGenerator(default_joint_position=posture)
    generator.anchor(0.0, 0.0, 0.0)
    state = [0.0] * 24
    current, future = generator.target_states(
        state, (0.0, 0.0, 0.0), 0.6)
    assert current[12:24] == posture
    assert future[12:24] == posture
