import numpy as np

from spot_wbc.startup import (
    StartupGate, smooth_posture_target, startup_posture_ready)


def test_smooth_posture_target_has_zero_motion_endpoints():
    initial = np.asarray([0.45, 1.20, -2.75] * 4)
    nominal = np.asarray([0.15, 1.05, -1.68] * 4)

    assert np.allclose(
        smooth_posture_target(initial, nominal, -1.0, 5.0), initial)
    assert np.allclose(
        smooth_posture_target(initial, nominal, 2.5, 5.0),
        0.5 * (initial + nominal))
    assert np.allclose(
        smooth_posture_target(initial, nominal, 5.0, 5.0), nominal)


def test_startup_posture_requires_height_attitude_velocity_and_contacts():
    ready = dict(
        base_height=0.45,
        roll=0.02,
        pitch=-0.03,
        joint_velocity=np.zeros(12),
        contacts=(True, True, True, True),
        minimum_height=0.42,
        maximum_tilt=0.20,
        maximum_joint_speed=0.50,
    )
    assert startup_posture_ready(**ready)

    for key, value in (
            ('base_height', np.float64(0.30)),
            ('roll', 0.30),
            ('pitch', -0.30),
            ('joint_velocity', np.asarray([0.0] * 11 + [0.6])),
            ('contacts', (True, True, True, False))):
        rejected = ready.copy()
        rejected[key] = value
        result = startup_posture_ready(**rejected)
        assert type(result) is bool
        assert not result


def test_startup_gate_requires_continuous_stability():
    gate = StartupGate(stable_duration=0.50)
    assert not gate.update(1.00, True)
    assert not gate.update(1.49, True)
    assert gate.update(1.50, True)

    gate.reset()
    assert not gate.update(2.00, True)
    assert not gate.update(2.30, False)
    assert not gate.update(2.60, True)
    assert gate.update(3.10, True)
