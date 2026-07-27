from dataclasses import dataclass
import math
import warnings

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, linprog, minimize


GENERALIZED_VELOCITY_DIMENSION = 18
CONTACT_FORCE_DIMENSION = 12
TORQUE_DIMENSION = 12
DECISION_DIMENSION = 42


def stance_flags(mode):
    if not isinstance(mode, (int, np.integer)) or mode < 0 or mode > 15:
        raise ValueError('contact mode must be in [0, 15]')
    return tuple(bool(mode & (1 << index)) for index in range(4))


def friction_margins(forces, coefficient):
    values = np.asarray(forces, dtype=float).reshape(4, 3)
    margins = []
    for fx, fy, fz in values:
        margins.extend((
            coefficient * fz - fx,
            coefficient * fz + fx,
            coefficient * fz - fy,
            coefficient * fz + fy,
            fz))
    return np.asarray(margins)


@dataclass
class QpResult:
    success: bool
    torque: np.ndarray
    acceleration: np.ndarray
    forces: np.ndarray
    iterations: int
    equality_residual: float
    minimum_friction_margin: float
    message: str


class WholeBodyQp:
    """Convex QP with rigid dynamics, contact, friction, and actuator bounds."""

    def __init__(
            self, friction_coefficient=0.7, torque_limit=60.0,
            maximum_contact_force=250.0,
            maximum_joint_acceleration=100.0,
            joint_limit_horizon=0.05, force_tracking_weight=5.0,
            base_acceleration_weight=10.0,
            joint_acceleration_weight=1.0,
            joint_velocity_tracking_gain=5.0,
            swing_tracking_weight=10.0,
            acceleration_regularization=1.0e-3,
            torque_regularization=1.0e-4,
            max_iterations=40, tolerance=1.0e-4):
        self.mu = float(friction_coefficient)
        self.torque_limit = float(torque_limit)
        self.max_force = float(maximum_contact_force)
        self.max_joint_accel = float(maximum_joint_acceleration)
        self.limit_horizon = float(joint_limit_horizon)
        self.force_weight = float(force_tracking_weight)
        self.base_accel_weight = float(base_acceleration_weight)
        self.joint_accel_weight = float(joint_acceleration_weight)
        self.joint_velocity_gain = float(joint_velocity_tracking_gain)
        self.swing_weight = float(swing_tracking_weight)
        self.accel_regularization = float(acceleration_regularization)
        self.torque_regularization = float(torque_regularization)
        self.max_iterations = int(max_iterations)
        self.tolerance = float(tolerance)
        self.previous = None
        self.previous_mode = None
        positive = (
            self.mu, self.torque_limit, self.max_force,
            self.max_joint_accel, self.limit_horizon, self.force_weight,
            self.base_accel_weight, self.joint_accel_weight, self.swing_weight,
            self.joint_velocity_gain,
            self.accel_regularization, self.torque_regularization,
            self.tolerance)
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('QP configuration values must be finite/positive')

    @staticmethod
    def _validate(matrix, shape, name):
        value = np.asarray(matrix, dtype=float)
        if value.shape != shape or not np.all(np.isfinite(value)):
            raise ValueError(f'{name} must be finite with shape {shape}')
        return value

    def solve(
            self, mass_matrix, nonlinear_effects, contact_jacobians,
            contact_drift, joint_position, joint_velocity,
            joint_lower, joint_upper, policy_state, policy_input, mode,
            swing_acceleration=None, desired_base_acceleration=None):
        mass = self._validate(
            mass_matrix, (GENERALIZED_VELOCITY_DIMENSION,) * 2, 'mass')
        nonlinear = self._validate(
            nonlinear_effects, (GENERALIZED_VELOCITY_DIMENSION,),
            'nonlinear effects')
        jacobians = self._validate(
            contact_jacobians, (CONTACT_FORCE_DIMENSION,
                                GENERALIZED_VELOCITY_DIMENSION),
            'contact Jacobians')
        drift = self._validate(
            contact_drift, (CONTACT_FORCE_DIMENSION,), 'contact drift')
        q = self._validate(joint_position, (TORQUE_DIMENSION,), 'joint q')
        dq = self._validate(joint_velocity, (TORQUE_DIMENSION,), 'joint dq')
        lower_q = self._validate(
            joint_lower, (TORQUE_DIMENSION,), 'joint lower limits')
        upper_q = self._validate(
            joint_upper, (TORQUE_DIMENSION,), 'joint upper limits')
        self._validate(policy_state, (24,), 'policy state')
        control = self._validate(policy_input, (24,), 'policy input')
        swing = (
            np.zeros(CONTACT_FORCE_DIMENSION)
            if swing_acceleration is None else self._validate(
                swing_acceleration, (CONTACT_FORCE_DIMENSION,),
                'swing acceleration'))
        desired_base = (
            np.zeros(6) if desired_base_acceleration is None
            else self._validate(
                desired_base_acceleration, (6,),
                'desired base acceleration'))
        flags = stance_flags(mode)

        # x = [generalized acceleration(18), contact forces(12), torque(12)]
        accel_slice = slice(0, 18)
        force_slice = slice(18, 30)
        torque_slice = slice(30, 42)
        hessian = np.eye(DECISION_DIMENSION) * 1.0e-10
        gradient = np.zeros(DECISION_DIMENSION)
        hessian[accel_slice, accel_slice] += (
            2.0 * self.accel_regularization * np.eye(18))
        hessian[force_slice, force_slice] += (
            2.0 * self.force_weight * np.eye(12))
        gradient[force_slice] -= 2.0 * self.force_weight * control[:12]
        hessian[torque_slice, torque_slice] += (
            2.0 * self.torque_regularization * np.eye(12))
        hessian[0:6, 0:6] += (
            2.0 * self.base_accel_weight * np.eye(6))
        gradient[0:6] -= (
            2.0 * self.base_accel_weight * desired_base)

        desired_joint_accel = self.joint_velocity_gain * (control[12:] - dq)
        hessian[6:18, 6:18] += (
            2.0 * self.joint_accel_weight * np.eye(12))
        gradient[6:18] -= (
            2.0 * self.joint_accel_weight * desired_joint_accel)
        for index, stance in enumerate(flags):
            if stance:
                continue
            rows = slice(3 * index, 3 * index + 3)
            task = np.zeros((3, DECISION_DIMENSION))
            task[:, accel_slice] = jacobians[rows]
            desired = swing[rows] - drift[rows]
            hessian += 2.0 * self.swing_weight * (task.T @ task)
            gradient -= 2.0 * self.swing_weight * (task.T @ desired)

        selection = np.zeros((18, 12))
        selection[6:, :] = np.eye(12)
        dynamics = np.zeros((18, DECISION_DIMENSION))
        dynamics[:, accel_slice] = mass
        dynamics[:, force_slice] = -jacobians.T
        dynamics[:, torque_slice] = -selection
        equality_rows = [dynamics]
        equality_values = [-nonlinear]
        for index, stance in enumerate(flags):
            rows = slice(3 * index, 3 * index + 3)
            if stance:
                constraint = np.zeros((3, DECISION_DIMENSION))
                constraint[:, accel_slice] = jacobians[rows]
                equality_rows.append(constraint)
                equality_values.append(-drift[rows])
            else:
                constraint = np.zeros((3, DECISION_DIMENSION))
                constraint[:, slice(18 + 3 * index, 21 + 3 * index)] = (
                    np.eye(3))
                equality_rows.append(constraint)
                equality_values.append(np.zeros(3))
        equality_matrix = np.vstack(equality_rows)
        equality_target = np.concatenate(equality_values)

        friction_rows = []
        for index, stance in enumerate(flags):
            if not stance:
                continue
            fx, fy, fz = (
                18 + 3 * index, 19 + 3 * index, 20 + 3 * index)
            for tangent, sign in ((fx, -1.0), (fx, 1.0),
                                  (fy, -1.0), (fy, 1.0)):
                row = np.zeros(DECISION_DIMENSION)
                row[tangent] = sign
                row[fz] = self.mu
                friction_rows.append(row)

        lower = np.full(DECISION_DIMENSION, -np.inf)
        upper = np.full(DECISION_DIMENSION, np.inf)
        lower[accel_slice], upper[accel_slice] = -200.0, 200.0
        horizon = self.limit_horizon
        position_lower_accel = (
            2.0 * (lower_q - q - dq * horizon) / (horizon * horizon))
        position_upper_accel = (
            2.0 * (upper_q - q - dq * horizon) / (horizon * horizon))
        lower[6:18] = np.maximum(
            -self.max_joint_accel, position_lower_accel)
        upper[6:18] = np.minimum(
            self.max_joint_accel, position_upper_accel)
        lower[force_slice], upper[force_slice] = -self.max_force, self.max_force
        for index, stance in enumerate(flags):
            fz = 20 + 3 * index
            if stance:
                lower[fz] = 0.0
            else:
                lower[18 + 3 * index:21 + 3 * index] = 0.0
                upper[18 + 3 * index:21 + 3 * index] = 0.0
        lower[torque_slice], upper[torque_slice] = (
            -self.torque_limit, self.torque_limit)
        if np.any(lower > upper):
            return QpResult(
                False, np.zeros(12), np.zeros(18), np.zeros(12), 0,
                math.inf, -math.inf, 'joint limit acceleration infeasible')

        initial = (
            self.previous.copy()
            if self.previous is not None
            and self.previous.shape == (DECISION_DIMENSION,)
            else np.zeros(DECISION_DIMENSION))
        initial = np.minimum(upper, np.maximum(lower, initial))
        initial[force_slice] = np.minimum(
            upper[force_slice],
            np.maximum(lower[force_slice], control[:12]))

        # Rebuild a state- and mode-consistent feasible seed on every solve.
        # A previous solution ceases to satisfy rigid dynamics as the live
        # state changes, even when contact mode is unchanged. HiGHS keeps
        # SLSQP's starting point inside all physical constraints.
        friction_matrix = (
            np.vstack(friction_rows) if friction_rows
            else np.empty((0, DECISION_DIMENSION)))
        feasibility = linprog(
            np.zeros(DECISION_DIMENSION),
            A_ub=-friction_matrix if friction_rows else None,
            b_ub=np.zeros(len(friction_rows)) if friction_rows else None,
            A_eq=equality_matrix, b_eq=equality_target,
            bounds=list(zip(lower, upper)), method='highs')
        if not feasibility.success:
            return QpResult(
                False, np.zeros(12), np.zeros(18), np.zeros(12), 0,
                math.inf, -math.inf,
                f'contact-mode feasibility failed: '
                f'{feasibility.message}')
        initial = np.asarray(feasibility.x, dtype=float)

        constraints = [
            LinearConstraint(
                equality_matrix, equality_target, equality_target)]
        if friction_rows:
            friction_matrix = np.vstack(friction_rows)
            constraints.append(LinearConstraint(
                friction_matrix, np.zeros(len(friction_rows)),
                np.full(len(friction_rows), np.inf)))

        def objective(x):
            return 0.5 * x @ hessian @ x + gradient @ x

        def jacobian(x):
            return hessian @ x + gradient

        with warnings.catch_warnings():
            # SLSQP reports its documented internal trial-point clipping even
            # when the final solution satisfies the independently checked
            # bounds below. Keep runtime diagnostics focused on real failures.
            warnings.filterwarnings(
                'ignore',
                message='Values in x were outside bounds during a minimize step',
                category=RuntimeWarning,
                module='scipy.optimize')
            result = minimize(
                objective, initial, jac=jacobian, method='SLSQP',
                bounds=Bounds(lower, upper), constraints=constraints,
                options={
                    'maxiter': self.max_iterations,
                    'ftol': self.tolerance,
                    'disp': False,
                })
        solution = np.asarray(result.x, dtype=float)
        residual = float(np.max(np.abs(
            equality_matrix @ solution - equality_target)))
        margins = friction_margins(solution[force_slice], self.mu)
        finite = np.all(np.isfinite(solution))
        # Require both optimizer convergence and independently verified
        # physical constraints. A merely feasible intermediate point can be
        # far from the force/acceleration objective and is unsafe to actuate.
        success = bool(
            result.success and finite
            and residual <= 10.0 * self.tolerance
            and np.min(margins) >= -10.0 * self.tolerance
            and np.all(solution >= lower - 10.0 * self.tolerance)
            and np.all(solution <= upper + 10.0 * self.tolerance))
        if success:
            self.previous = solution
            self.previous_mode = mode
        return QpResult(
            success=success,
            torque=solution[torque_slice].copy(),
            acceleration=solution[accel_slice].copy(),
            forces=solution[force_slice].copy(),
            iterations=int(result.nit),
            equality_residual=residual,
            minimum_friction_margin=float(np.min(margins)),
            message=str(result.message))
