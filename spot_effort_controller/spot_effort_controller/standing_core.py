"""Transport-independent gravity-compensation plus joint-PD controller."""

import math
from typing import Iterable, Tuple

from .constants import JOINT_NAMES, NOMINAL_STAND


class StandingController:
    def __init__(
        self,
        kp,
        kd,
        desired=NOMINAL_STAND,
    ) -> None:
        self.kp = tuple(float(value) for value in kp)
        self.kd = tuple(float(value) for value in kd)
        self.desired = tuple(float(value) for value in desired)
        if not all(len(values) == len(JOINT_NAMES)
                   for values in (self.kp, self.kd, self.desired)):
            raise ValueError('standing vectors must have exactly 12 entries')
        if not all(math.isfinite(value)
                   for values in (self.kp, self.kd, self.desired)
                   for value in values):
            raise ValueError('standing vectors must be finite')

    def compute(
        self,
        position: Iterable[float],
        velocity: Iterable[float],
        gravity: Iterable[float],
    ) -> Tuple[float, ...]:
        position = tuple(float(value) for value in position)
        velocity = tuple(float(value) for value in velocity)
        gravity = tuple(float(value) for value in gravity)
        if not all(len(values) == len(JOINT_NAMES)
                   for values in (position, velocity, gravity)):
            raise ValueError('controller input vectors must have 12 entries')
        if not all(math.isfinite(value)
                   for values in (position, velocity, gravity)
                   for value in values):
            raise ValueError('controller input contains a non-finite value')
        return tuple(
            gravity_i + kp_i * (desired_i - position_i) - kd_i * velocity_i
            for gravity_i, kp_i, kd_i, desired_i, position_i, velocity_i in zip(
                gravity, self.kp, self.kd, self.desired, position, velocity))
