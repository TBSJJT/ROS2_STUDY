"""IMU 单位换算与角速度修正测试。"""

import math

import pytest

from gasrobot_base.imu import GRAVITY, ImuConverter
from gasrobot_base.models import RawFeedback


def _feedback(gyroscope_z: int) -> RawFeedback:
    return RawFeedback(
        linear_x=0.0,
        linear_y=0.0,
        angular_z=0.0,
        yaw=0.0,
        acceleration_raw=(4096, 0, -4096),
        gyroscope_raw=(131, -262, gyroscope_z),
    )


def test_converter_outputs_si_units():
    converter = ImuConverter(4096.0, 131.0, 1.0, 0.0, 0.0)
    sample = converter.convert(_feedback(393))

    assert sample.acceleration == pytest.approx((GRAVITY, 0.0, -GRAVITY))
    assert sample.gyroscope[0] == pytest.approx(math.radians(1.0))
    assert sample.gyroscope[1] == pytest.approx(math.radians(-2.0))
    assert sample.yaw_rate == pytest.approx(math.radians(3.0))


def test_converter_applies_sign_offset_and_deadband():
    converter = ImuConverter(
        4096.0,
        131.0,
        z_sign=-1.0,
        z_offset_radps=-0.01,
        z_deadband=0.01,
    )
    sample = converter.convert(_feedback(65))

    assert abs(sample.yaw_rate_before_deadband) < 0.01
    assert sample.yaw_rate == 0.0
