"""底盘桥接模块共享的数据模型。"""

from dataclasses import dataclass
from typing import Tuple


IntVector3 = Tuple[int, int, int]
FloatVector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class VelocityCommand:
    """ROS 坐标约定下的三轴速度指令。"""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class RawFeedback:
    """已经通过协议校验并完成速度单位换算的底盘反馈。"""

    linear_x: float
    linear_y: float
    angular_z: float
    yaw: float
    acceleration_raw: IntVector3
    gyroscope_raw: IntVector3


@dataclass(frozen=True)
class ImuSample:
    """转换为国际单位制后的 IMU 数据。"""

    acceleration: FloatVector3
    gyroscope: FloatVector3
    yaw_rate_before_deadband: float
    yaw_rate: float


@dataclass(frozen=True)
class OdometrySample:
    """一次二维里程计更新后的完整状态。"""

    x: float
    y: float
    yaw: float
    linear_x: float
    linear_y: float
    angular_z: float
    yaw_source: str
