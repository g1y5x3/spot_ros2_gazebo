import math


STANCE_TEMPLATE = ((0.0, 1.0), (15,))
CONSERVATIVE_TROT_TEMPLATE = (
    (0.0, 0.50, 2.50, 3.00, 5.00),
    (9, 15, 6, 15),
)
CONSERVATIVE_CRAWL_TEMPLATE = (
    (0.0, 0.40, 2.00, 2.40, 4.00, 4.40, 6.00, 6.40, 8.00),
    (7, 15, 14, 15, 11, 15, 13, 15),
)


def command_requests_motion(command, threshold=1.0e-3):
    values = tuple(float(value) for value in command)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        return False
    return max(abs(value) for value in values) > threshold


def select_gait(
        command, command_age, timeout, moving_gait, stationary_gait,
        threshold=1.0e-3):
    if not math.isfinite(command_age) or command_age < 0.0:
        active = False
    else:
        active = (
            command_age <= timeout
            and command_requests_motion(command, threshold))
    selected = moving_gait if active else stationary_gait
    gait_template(selected)
    return selected


def gait_template(name):
    if name == 'stance':
        return STANCE_TEMPLATE
    if name == 'conservative_trot':
        return CONSERVATIVE_TROT_TEMPLATE
    if name == 'conservative_crawl':
        return CONSERVATIVE_CRAWL_TEMPLATE
    raise ValueError(f'unknown gait {name!r}')
