"""Pure torque safety logic, independent of ROS and Gazebo."""

import math
from typing import Iterable, Tuple

from .constants import JOINT_NAMES


def reorder_joint_state(names: Iterable[str], values: Iterable[float]) -> Tuple[float, ...]:
    name_list = list(names)
    value_list = list(values)
    if len(name_list) != len(value_list):
        raise ValueError('joint names and values have different lengths')
    if len(set(name_list)) != len(name_list):
        raise ValueError('duplicate joint name')
    mapping = dict(zip(name_list, value_list))
    try:
        result = tuple(float(mapping[name]) for name in JOINT_NAMES)
    except KeyError as error:
        raise ValueError(f'missing joint {error.args[0]}') from error
    if not all(math.isfinite(value) for value in result):
        raise ValueError('joint state contains a non-finite value')
    return result


class TorqueLimiter:
    def __init__(self, torque_limit: float, rate_limit: float) -> None:
        if torque_limit <= 0.0 or rate_limit <= 0.0:
            raise ValueError('torque and rate limits must be positive')
        self.torque_limit = float(torque_limit)
        self.rate_limit = float(rate_limit)
        self.output = (0.0,) * len(JOINT_NAMES)

    def reset(self) -> None:
        self.output = (0.0,) * len(JOINT_NAMES)

    def apply(self, desired: Iterable[float], dt: float) -> Tuple[float, ...]:
        desired = tuple(float(value) for value in desired)
        if len(desired) != len(JOINT_NAMES):
            raise ValueError('expected exactly 12 torque values')
        if not all(math.isfinite(value) for value in desired):
            raise ValueError('torque command contains a non-finite value')
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError('dt must be finite and positive')

        max_delta = self.rate_limit * dt
        next_output = []
        for previous, requested in zip(self.output, desired):
            saturated = max(-self.torque_limit,
                            min(self.torque_limit, requested))
            delta = max(-max_delta, min(max_delta, saturated - previous))
            next_output.append(previous + delta)
        self.output = tuple(next_output)
        return self.output
