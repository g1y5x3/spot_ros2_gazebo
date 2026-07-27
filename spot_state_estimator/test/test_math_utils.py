import math

import pytest

from spot_state_estimator.math_utils import (
    normalized_quaternion, quaternion_to_yaw_pitch_roll,
    timestamps_consistent)


def test_quaternion_normalization_and_yaw():
    assert normalized_quaternion(0, 0, 0, 2) == (0, 0, 0, 1)
    yaw, pitch, roll = quaternion_to_yaw_pitch_roll(
        0, 0, math.sin(0.25), math.cos(0.25))
    assert yaw == pytest.approx(0.5)
    assert pitch == pytest.approx(0.0)
    assert roll == pytest.approx(0.0)


def test_bad_quaternion_rejected():
    with pytest.raises(ValueError):
        normalized_quaternion(0, 0, 0, 0)
    with pytest.raises(ValueError):
        normalized_quaternion(math.nan, 0, 0, 1)


def test_timestamp_consistency():
    assert timestamps_consistent((9.96, 9.98, 10.0), 10.0, 0.1, 0.05)
    assert not timestamps_consistent((9.0, 9.98, 10.0), 10.0, 0.1, 0.05)
    assert not timestamps_consistent((9.90, 10.0), 10.0, 0.2, 0.05)
