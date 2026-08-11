"""底盘桥接模块共享的数据模型。"""

from dataclasses import dataclass
from typing import Tuple


IntVector3 = Tuple[int, int, int]
FloatVector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class VelocityCommand:
    """ROS 坐标约定下的三轴速度指令。"""

    # 麦克纳姆底盘允许前后、横向和原地旋转三个自由度同时运动。
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class RawFeedback:
    """已经通过协议校验并完成速度单位换算的底盘反馈。"""

    # 速度和航向已转换为 ROS 使用的 m/s、rad/s 和 rad。
    linear_x: float
    linear_y: float
    angular_z: float
    yaw: float

    # IMU 保留传感器原始计数，由 ImuConverter 统一转换为国际单位制。
    acceleration_raw: IntVector3
    gyroscope_raw: IntVector3


@dataclass(frozen=True)
class ImuSample:
    """转换为国际单位制后的 IMU 数据。"""

    acceleration: FloatVector3
    gyroscope: FloatVector3

    # 同时保留死区前后的 Z 轴角速度，便于标定和现场诊断。
    yaw_rate_before_deadband: float
    yaw_rate: float


@dataclass(frozen=True)
class OdometrySample:
    """一次二维里程计更新后的完整状态。"""

    # x、y、yaw 描述 odom 坐标系中的二维位姿。
    x: float
    y: float
    yaw: float

    # 以下速度均在机器人本体坐标系中表达。
    linear_x: float
    linear_y: float
    angular_z: float

    # 记录航向来源，状态日志可据此确认系统没有重复积分。
    yaw_source: str
