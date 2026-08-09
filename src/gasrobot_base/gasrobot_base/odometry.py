"""麦克纳姆底盘二维里程计积分。"""

import math
from typing import Optional

from gasrobot_base.models import OdometrySample, RawFeedback


def normalize_angle(angle: float) -> float:
    """将角度归一化到负圆周率至正圆周率。"""

    return math.atan2(math.sin(angle), math.cos(angle))


class OdometryIntegrator:
    """使用底盘平移速度和可配置角速度源积分二维位姿。"""

    def __init__(
        self,
        use_imu_yaw_rate: bool,
        max_interval: float = 0.5,
    ) -> None:
        self.use_imu_yaw_rate = use_imu_yaw_rate
        self.max_interval = max_interval
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._last_stamp: Optional[float] = None

    def reset_time(self) -> None:
        """清除上次时间戳，避免断线时段被积分。"""

        self._last_stamp = None

    def update(
        self,
        feedback: RawFeedback,
        imu_yaw_rate: float,
        stamp: float,
    ) -> OdometrySample:
        """根据一帧反馈更新里程计并返回当前状态。"""

        angular_z = (
            imu_yaw_rate
            if self.use_imu_yaw_rate
            else feedback.angular_z
        )
        yaw_source = "IMU" if self.use_imu_yaw_rate else "WHEEL"

        interval = 0.0
        if self._last_stamp is not None:
            interval = stamp - self._last_stamp
            if interval <= 0.0 or interval > self.max_interval:
                interval = 0.0
        self._last_stamp = stamp

        if interval > 0.0:
            middle_yaw = self.yaw + angular_z * interval * 0.5
            world_linear_x = (
                feedback.linear_x * math.cos(middle_yaw)
                - feedback.linear_y * math.sin(middle_yaw)
            )
            world_linear_y = (
                feedback.linear_x * math.sin(middle_yaw)
                + feedback.linear_y * math.cos(middle_yaw)
            )
            self.x += world_linear_x * interval
            self.y += world_linear_y * interval
            self.yaw = normalize_angle(
                self.yaw + angular_z * interval
            )

        return OdometrySample(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            linear_x=feedback.linear_x,
            linear_y=feedback.linear_y,
            angular_z=angular_z,
            yaw_source=yaw_source,
        )
