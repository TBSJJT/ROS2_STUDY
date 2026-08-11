"""二维里程计积分测试。"""

import math

import pytest

from gasrobot_base.models import RawFeedback
from gasrobot_base.odometry import OdometryIntegrator, normalize_angle


def _feedback(
    linear_x: float = 1.0,
    linear_y: float = 0.0,
    angular_z: float = 0.0,
    yaw: float = 0.0,
) -> RawFeedback:
    """构造只包含里程计测试所需字段的反馈样本。"""

    return RawFeedback(
        linear_x=linear_x,
        linear_y=linear_y,
        angular_z=angular_z,
        yaw=yaw,
        acceleration_raw=(0, 0, 0),
        gyroscope_raw=(0, 0, 0),
    )


def test_first_sample_initializes_time_and_adopts_stm32_yaw():
    """验证首帧不积分位置，但立即采用 STM32 航向。"""

    integrator = OdometryIntegrator(use_imu_angular_velocity=True)
    sample = integrator.update(
        _feedback(yaw=0.75),
        imu_angular_z=0.5,
        stamp=10.0,
    )

    assert sample.x == 0.0
    assert sample.y == 0.0
    assert sample.yaw == pytest.approx(0.75)
    assert sample.yaw_source == "STM32_YAW"


def test_midpoint_integration_uses_consecutive_stm32_yaw_values():
    """验证平移速度按照相邻两帧航向中点转换到 odom 坐标系。"""

    integrator = OdometryIntegrator(
        use_imu_angular_velocity=True,
        max_interval=2.0,
    )
    integrator.update(
        _feedback(yaw=0.0),
        imu_angular_z=1.0,
        stamp=0.0,
    )
    sample = integrator.update(
        _feedback(yaw=1.0),
        imu_angular_z=1.0,
        stamp=1.0,
    )

    assert sample.x == pytest.approx(math.cos(0.5))
    assert sample.y == pytest.approx(math.sin(0.5))
    assert sample.yaw == pytest.approx(1.0)


def test_yaw_midpoint_uses_short_path_across_wrap_boundary():
    """验证航向跨越正负 180 度时沿最短角差计算。"""

    integrator = OdometryIntegrator(
        use_imu_angular_velocity=False,
        max_interval=2.0,
    )
    integrator.update(
        _feedback(yaw=math.radians(179.0)),
        imu_angular_z=0.0,
        stamp=0.0,
    )
    sample = integrator.update(
        _feedback(yaw=math.radians(-179.0)),
        imu_angular_z=0.0,
        stamp=1.0,
    )

    assert sample.x == pytest.approx(-1.0, abs=1e-6)
    assert sample.y == pytest.approx(0.0, abs=1e-6)
    assert sample.yaw == pytest.approx(math.radians(-179.0))


def test_twist_source_and_reset_prevent_gap_translation():
    """验证角速度来源选择，以及断线重连期间不累计平移。"""

    integrator = OdometryIntegrator(
        use_imu_angular_velocity=False,
        max_interval=1.0,
    )
    feedback = _feedback(angular_z=-0.25, yaw=0.3)
    integrator.update(feedback, imu_angular_z=1.0, stamp=1.0)
    sample = integrator.update(feedback, imu_angular_z=1.0, stamp=1.5)
    assert sample.angular_z == pytest.approx(-0.25)
    assert sample.yaw_source == "STM32_YAW"

    integrator.reset_time()
    after_reset = integrator.update(
        _feedback(angular_z=-0.25, yaw=0.8),
        imu_angular_z=1.0,
        stamp=20.0,
    )
    assert after_reset.x == pytest.approx(sample.x)
    assert after_reset.y == pytest.approx(sample.y)
    assert after_reset.yaw == pytest.approx(0.8)


def test_normalize_angle_keeps_result_in_expected_range():
    """验证任意角度能够归一化到约定区间。"""

    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
