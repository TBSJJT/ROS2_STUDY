"""麦克纳姆底盘二维里程计积分。"""

import math
from typing import Optional

from gasrobot_base.models import OdometrySample, RawFeedback


def normalize_angle(angle: float) -> float:
    """将角度归一化到负圆周率至正圆周率。"""

    return math.atan2(math.sin(angle), math.cos(angle))


class OdometryIntegrator:
    """使用底盘平移速度积分位置，并直接采用 STM32 的绝对航向。"""

    def __init__(
        self,
        use_imu_angular_velocity: bool,
        max_interval: float = 0.5,
    ) -> None:
        self.use_imu_angular_velocity = use_imu_angular_velocity
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
        imu_angular_z: float,
        stamp: float,
    ) -> OdometrySample:
        """根据一帧反馈更新里程计并返回当前状态。"""

        angular_z = (
            imu_angular_z
            if self.use_imu_angular_velocity
            else feedback.angular_z
        )

        interval = 0.0
        if self._last_stamp is not None:
            interval = stamp - self._last_stamp
            if interval <= 0.0 or interval > self.max_interval:
                interval = 0.0
        self._last_stamp = stamp

        if interval > 0.0:
            # 使用最短角差求区间中点，避免航向跨越正负圆周率时跳向错误方向。
            yaw_delta = normalize_angle(feedback.yaw - self.yaw)
            middle_yaw = normalize_angle(self.yaw + yaw_delta * 0.5)
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

        # 航向始终以本帧 STM32 绝对值为准，不对角速度进行二次积分。
        self.yaw = feedback.yaw

        return OdometrySample(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            linear_x=feedback.linear_x,
            linear_y=feedback.linear_y,
            angular_z=angular_z,
            yaw_source="STM32_YAW",
        )
