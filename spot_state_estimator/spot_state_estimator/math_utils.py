import math


def normalized_quaternion(x, y, z, w):
    values = tuple(float(v) for v in (x, y, z, w))
    if not all(math.isfinite(v) for v in values):
        raise ValueError('quaternion is non-finite')
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1.0e-9:
        raise ValueError('quaternion norm is zero')
    return tuple(v / norm for v in values)


def quaternion_to_yaw_pitch_roll(x, y, z, w):
    x, y, z, w = normalized_quaternion(x, y, z, w)
    yaw = math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    roll = math.atan2(2.0 * (w * x + y * z),
                      1.0 - 2.0 * (x * x + y * y))
    return yaw, pitch, roll


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def timestamps_consistent(stamps, now, max_age, max_skew):
    stamps = tuple(float(value) for value in stamps)
    return (
        len(stamps) > 0
        and all(math.isfinite(value) and value > 0.0 for value in stamps)
        and now - min(stamps) <= max_age
        and max(stamps) - min(stamps) <= max_skew
    )
