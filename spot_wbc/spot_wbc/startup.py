"""Pure helpers for the guarded belly-down startup transition."""

import math

import numpy as np


def smooth_posture_target(initial, nominal, elapsed, duration):
    """Interpolate joint posture with zero velocity at both endpoints."""
    initial = np.asarray(initial, dtype=float)
    nominal = np.asarray(nominal, dtype=float)
    if (
            initial.shape != (12,) or nominal.shape != (12,)
            or not np.all(np.isfinite(initial))
            or not np.all(np.isfinite(nominal))):
        raise ValueError('startup postures must contain 12 finite values')
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError('startup duration must be finite and positive')
    if not math.isfinite(elapsed):
        raise ValueError('startup elapsed time must be finite')
    blend = min(1.0, max(0.0, elapsed / duration))
    blend = blend * blend * (3.0 - 2.0 * blend)
    return initial + blend * (nominal - initial)


def startup_posture_ready(
        base_height, roll, pitch, joint_velocity, contacts,
        minimum_height, maximum_tilt, maximum_joint_speed):
    """Return whether measured state is safe for policy handoff."""
    velocity = np.asarray(joint_velocity, dtype=float)
    contact_flags = tuple(bool(value) for value in contacts)
    scalars = (
        base_height, roll, pitch, minimum_height,
        maximum_tilt, maximum_joint_speed)
    if (
            velocity.shape != (12,)
            or len(contact_flags) != 4
            or not np.all(np.isfinite(velocity))
            or not all(math.isfinite(float(value)) for value in scalars)
            or minimum_height <= 0.0
            or maximum_tilt <= 0.0
            or maximum_joint_speed <= 0.0):
        return False
    return bool(
        base_height >= minimum_height
        and abs(roll) <= maximum_tilt
        and abs(pitch) <= maximum_tilt
        and float(np.max(np.abs(velocity))) <= maximum_joint_speed
        and all(contact_flags)
    )


class StartupGate:
    """Require startup readiness continuously before policy handoff."""

    def __init__(self, stable_duration):
        if not math.isfinite(stable_duration) or stable_duration <= 0.0:
            raise ValueError('stable duration must be finite and positive')
        self.stable_duration = float(stable_duration)
        self.stable_since = None

    def reset(self):
        self.stable_since = None

    def update(self, now, ready):
        if not math.isfinite(now):
            self.reset()
            return False
        if not ready:
            self.reset()
            return False
        if self.stable_since is None or now < self.stable_since:
            self.stable_since = now
        return now - self.stable_since >= self.stable_duration
