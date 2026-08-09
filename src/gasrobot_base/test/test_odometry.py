"""二维里程计积分测试。"""

import math

import pytest

from gasrobot_base.models import RawFeedback
from gasrobot_base.odometry import OdometryIntegrator, normalize_angle


def _feedback(
    linear_x: float = 1.0,
    linear_y: float = 0.0,
    angular_z: float = 0.0,
) -> RawFeedback:
    return RawFeedback(
        linear_x=linear_x,
        linear_y=linear_y,
        angular_z=angular_z,
        acceleration_raw=(0, 0, 0),
        gyroscope_raw=(0, 0, 0),
    )


def test_first_sample_only_initializes_time():
    integrator = OdometryIntegrator(use_imu_yaw_rate=True)
    sample = integrator.update(_feedback(), imu_yaw_rate=0.5, stamp=10.0)

    assert sample.x == 0.0
    assert sample.y == 0.0
    assert sample.yaw == 0.0
    assert sample.yaw_source == "IMU"


def test_midpoint_integration_uses_selected_imu_yaw_rate():
    integrator = OdometryIntegrator(
        use_imu_yaw_rate=True,
        max_interval=2.0,
    )
    integrator.update(_feedback(), imu_yaw_rate=1.0, stamp=0.0)
    sample = integrator.update(_feedback(), imu_yaw_rate=1.0, stamp=1.0)

    assert sample.x == pytest.approx(math.cos(0.5))
    assert sample.y == pytest.approx(math.sin(0.5))
    assert sample.yaw == pytest.approx(1.0)


def test_wheel_yaw_rate_and_reset_prevent_gap_integration():
    integrator = OdometryIntegrator(
        use_imu_yaw_rate=False,
        max_interval=1.0,
    )
    feedback = _feedback(angular_z=-0.25)
    integrator.update(feedback, imu_yaw_rate=1.0, stamp=1.0)
    sample = integrator.update(feedback, imu_yaw_rate=1.0, stamp=1.5)
    assert sample.angular_z == pytest.approx(-0.25)
    assert sample.yaw_source == "WHEEL"

    integrator.reset_time()
    after_reset = integrator.update(feedback, imu_yaw_rate=1.0, stamp=20.0)
    assert after_reset.yaw == pytest.approx(sample.yaw)


def test_normalize_angle_keeps_result_in_expected_range():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
