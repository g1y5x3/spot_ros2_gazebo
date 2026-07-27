"""Pure WBC safety selection independent of ROS and Gazebo."""

import math

import numpy as np


def policy_is_fresh(now, policy_time, timeout):
    values = (now, timeout)
    if not all(math.isfinite(float(value)) for value in values):
        return False
    if timeout <= 0.0 or policy_time is None:
        return False
    policy_time = float(policy_time)
    return (
        math.isfinite(policy_time)
        and 0.0 <= now - policy_time <= timeout)


def select_bounded_torque(
        policy_ready, candidate, fallback, torque_limit, delta_limit):
    fallback = np.asarray(fallback, dtype=float)
    if fallback.shape != (12,) or not np.all(np.isfinite(fallback)):
        fallback = np.zeros(12)
    fallback = np.clip(fallback, -torque_limit, torque_limit)
    if not policy_ready or candidate is None:
        return fallback, False
    candidate = np.asarray(candidate, dtype=float)
    if candidate.shape != (12,) or not np.all(np.isfinite(candidate)):
        return fallback, False
    candidate = np.clip(candidate, -torque_limit, torque_limit)
    candidate = np.clip(
        candidate, fallback - delta_limit, fallback + delta_limit)
    return candidate, True


def apply_policy_correction(
        fallback, correction, blend, torque_limit):
    """Add a cached policy correction to the current live fallback torque."""
    fallback = np.asarray(fallback, dtype=float)
    if fallback.shape != (12,) or not np.all(np.isfinite(fallback)):
        fallback = np.zeros(12)
    fallback = np.clip(fallback, -torque_limit, torque_limit)
    correction = np.asarray(correction, dtype=float)
    if correction.shape != (12,) or not np.all(np.isfinite(correction)):
        return fallback
    if not math.isfinite(float(blend)):
        return fallback
    blend = min(1.0, max(0.0, float(blend)))
    return np.clip(
        fallback + blend * correction, -torque_limit, torque_limit)
