import math

from spot_effort_controller.constants import NOMINAL_STAND


STATE_DIMENSION = 24
INPUT_DIMENSION = 24
CONTACT_FORCE_DIMENSION = 12
JOINT_VELOCITY_DIMENSION = 12
STANCE_MODE = 15
NOMINAL_HEIGHT = 0.48


def validate_state(values):
    state = tuple(float(value) for value in values)
    if len(state) != STATE_DIMENSION:
        raise ValueError(
            f'expected {STATE_DIMENSION} state values, got {len(state)}')
    if not all(math.isfinite(value) for value in state):
        raise ValueError('state contains a non-finite value')
    return state


def state_message_time(message):
    """Return a centroidal-state header timestamp in seconds."""
    value = (
        float(message.header.stamp.sec)
        + float(message.header.stamp.nanosec) * 1.0e-9)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError('state timestamp is invalid')
    return value


def state_message_values(message):
    """Flatten a typed centroidal-state message into OCS2 state order."""
    values = (
        tuple(message.normalized_momentum)
        + tuple(message.base_position)
        + tuple(message.base_ypr)
        + tuple(message.joint_position))
    return validate_state(values)


def standing_target(values):
    """Return a stationary target without changing current x, y, or yaw."""
    target = list(validate_state(values))
    target[0:6] = [0.0] * 6
    target[8] = NOMINAL_HEIGHT
    target[10:12] = [0.0, 0.0]  # pitch and roll
    target[12:24] = NOMINAL_STAND
    return tuple(target)


def validate_policy_dimensions(state_trajectory, input_trajectory):
    if not state_trajectory or not input_trajectory:
        raise ValueError('policy trajectories must be non-empty')
    if any(len(values) != STATE_DIMENSION for values in state_trajectory):
        raise ValueError('policy state dimension is not 24')
    if any(len(values) != INPUT_DIMENSION for values in input_trajectory):
        raise ValueError('policy input dimension is not 24')
    return True
