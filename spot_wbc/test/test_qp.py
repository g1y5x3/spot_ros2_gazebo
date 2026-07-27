import math

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pinocchio as pin
import pytest

from spot_effort_controller.constants import NOMINAL_STAND
from spot_wbc.qp import WholeBodyQp, friction_margins, stance_flags
from spot_wbc.safety import (
    apply_policy_correction, policy_is_fresh, select_bounded_torque)


def test_contact_mode_order_and_friction_margins():
    assert stance_flags(15) == (True, True, True, True)
    assert stance_flags(9) == (True, False, False, True)
    forces = np.asarray([1.0, -2.0, 10.0] * 4)
    assert np.min(friction_margins(forces, 0.7)) >= 0.0
    with pytest.raises(ValueError):
        stance_flags(16)


def test_flying_qp_dimensions_finite_and_bounded():
    qp = WholeBodyQp(max_iterations=100)
    mass = np.eye(18)
    nonlinear = np.zeros(18)
    jacobians = np.zeros((12, 18))
    drift = np.zeros(12)
    q = np.zeros(12)
    dq = np.zeros(12)
    lower = np.full(12, -2.0)
    upper = np.full(12, 2.0)
    policy_state = np.zeros(24)
    policy_input = np.zeros(24)
    result = qp.solve(
        mass, nonlinear, jacobians, drift, q, dq, lower, upper,
        policy_state, policy_input, mode=0)
    assert result.success, result.message
    assert result.torque.shape == (12,)
    assert result.acceleration.shape == (18,)
    assert result.forces.shape == (12,)
    assert np.all(np.isfinite(result.torque))
    assert np.max(np.abs(result.torque)) <= 60.0


def test_invalid_qp_input_is_rejected():
    qp = WholeBodyQp()
    with pytest.raises(ValueError):
        qp.solve(
            np.eye(17), np.zeros(18), np.zeros((12, 18)), np.zeros(12),
            np.zeros(12), np.zeros(12), -np.ones(12), np.ones(12),
            np.zeros(24), np.zeros(24), 15)


def test_real_spot_stance_qp_satisfies_dynamics_and_bounds():
    urdf = (
        get_package_share_directory('spot_description')
        + '/models/spot/model.urdf')
    model = pin.buildModelFromUrdf(urdf, pin.JointModelFreeFlyer())
    data = model.createData()
    q = np.asarray((0.0, 0.0, 0.48, 0.0, 0.0, 0.0, 1.0,
                    *NOMINAL_STAND))
    velocity = np.zeros(18)
    pin.computeAllTerms(model, data, q, velocity)
    pin.forwardKinematics(model, data, q, velocity, np.zeros(18))
    pin.updateFramePlacements(model, data)
    jacobians = np.zeros((12, 18))
    drift = np.zeros(12)
    for index, name in enumerate((
            'front_left_ee', 'front_right_ee',
            'rear_left_ee', 'rear_right_ee')):
        frame_id = model.getFrameId(name)
        jacobians[3 * index:3 * index + 3] = pin.computeFrameJacobian(
            model, data, q, frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3]
        drift[3 * index:3 * index + 3] = (
            pin.getFrameClassicalAcceleration(
                model, data, frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear)
    state = np.asarray((
        *(0.0,) * 6, 0.0, 0.0, 0.48, 0.0, 0.0, 0.0, *NOMINAL_STAND))
    control = np.zeros(24)
    control[2:12:3] = pin.computeTotalMass(model) * 9.81 / 4.0
    solver = WholeBodyQp(max_iterations=100)
    result = solver.solve(
        data.M, data.nle, jacobians, drift,
        np.asarray(NOMINAL_STAND), np.zeros(12),
        model.lowerPositionLimit[7:], model.upperPositionLimit[7:],
        state, control, 15)
    assert result.success, result.message
    assert result.equality_residual < 1.0e-8
    assert result.minimum_friction_margin >= 0.0
    assert np.max(np.abs(result.torque)) < 60.0

    # Exercise the live stance-to-diagonal-support transition.  This is where
    # an old four-contact warm start must be replaced by a feasible seed.
    control[:] = 0.0
    support_force = pin.computeTotalMass(model) * 9.81 / 2.0
    control[2] = support_force
    control[11] = support_force
    result = solver.solve(
        data.M, data.nle, jacobians, drift,
        np.asarray(NOMINAL_STAND), np.zeros(12),
        model.lowerPositionLimit[7:], model.upperPositionLimit[7:],
        state, control, 9)
    assert result.success, result.message
    assert result.equality_residual < 1.0e-8
    assert result.minimum_friction_margin >= 0.0
    assert np.max(np.abs(result.torque)) < 60.0


def test_stale_or_invalid_policy_selects_bounded_fallback():
    fallback = np.linspace(-70.0, 70.0, 12)
    candidate = np.full(12, 40.0)
    torque, active = select_bounded_torque(
        False, candidate, fallback, 60.0, 20.0)
    assert not active
    assert np.max(np.abs(torque)) <= 60.0
    assert np.allclose(torque, np.clip(fallback, -60.0, 60.0))

    candidate[3] = math.nan
    torque, active = select_bounded_torque(
        True, candidate, fallback, 60.0, 20.0)
    assert not active
    assert np.all(np.isfinite(torque))
    assert not policy_is_fresh(10.0, 9.0, 0.25)
    assert policy_is_fresh(10.0, 9.9, 0.25)


def test_cached_policy_correction_preserves_live_feedback_updates():
    correction = np.linspace(-2.0, 2.0, 12)
    first_fallback = np.linspace(-10.0, 10.0, 12)
    second_fallback = first_fallback + 3.0
    first = apply_policy_correction(
        first_fallback, correction, 0.5, 60.0)
    second = apply_policy_correction(
        second_fallback, correction, 0.5, 60.0)
    # Regression guard: an old implementation cached the absolute first
    # command and unintentionally froze the 200 Hz posture feedback.
    assert np.allclose(second - first, second_fallback - first_fallback)
    assert np.allclose(
        apply_policy_correction(
            second_fallback, np.full(12, math.nan), 1.0, 60.0),
        second_fallback)
