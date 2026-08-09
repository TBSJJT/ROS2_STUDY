"""ICM20602 原始反馈的单位换算与 Z 轴修正。"""

import math

from gasrobot_base.models import ImuSample, RawFeedback


GRAVITY = 9.80665


class ImuConverter:
    """将原始 IMU 计数转换为 ROS 使用的国际单位制数据。"""

    def __init__(
        self,
        acceleration_lsb_per_g: float,
        gyroscope_lsb_per_dps: float,
        z_sign: float,
        z_offset_radps: float,
        z_deadband: float,
    ) -> None:
        self.acceleration_lsb_per_g = acceleration_lsb_per_g
        self.gyroscope_lsb_per_dps = gyroscope_lsb_per_dps
        self.z_sign = -1.0 if z_sign < 0.0 else 1.0
        self.z_offset_radps = z_offset_radps
        self.z_deadband = z_deadband

    def convert(self, feedback: RawFeedback) -> ImuSample:
        """转换一帧反馈中的加速度计和陀螺仪数据。"""

        acceleration = tuple(
            raw / self.acceleration_lsb_per_g * GRAVITY
            for raw in feedback.acceleration_raw
        )
        gyroscope = tuple(
            math.radians(raw / self.gyroscope_lsb_per_dps)
            for raw in feedback.gyroscope_raw
        )
        yaw_rate_before_deadband = (
            self.z_sign * gyroscope[2] - self.z_offset_radps
        )
        yaw_rate = yaw_rate_before_deadband
        if abs(yaw_rate) < self.z_deadband:
            yaw_rate = 0.0

        return ImuSample(
            acceleration=acceleration,
            gyroscope=gyroscope,
            yaw_rate_before_deadband=yaw_rate_before_deadband,
            yaw_rate=yaw_rate,
        )
