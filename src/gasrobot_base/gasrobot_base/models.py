#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘桥接模块共享的数据模型 (Data Models).

这个模块定义了在整个 gasrobot_base 包中传递的数据结构.
它们都是 "数据类" (dataclass) -- Python 3.7+ 引入的一种简化数据定义的方式.

================================================================================
什么是 dataclass (数据类)?

普通的 Python 类需要手写 __init__, __repr__, __eq__ 等方法.
例如一个简单的二维点类:
    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

使用 @dataclass 装饰器后, 同样的功能只需要:
    @dataclass
    class Point:
        x: float
        y: float

Python 会自动生成 __init__, __repr__, __eq__ 等方法.

frozen=True 的含义:
    创建的对象是 "不可变的" (immutable), 类似于 tuple 或 str.
    一旦创建, 就不能修改其属性值.
    好处: 线程安全, 可哈希 (可以作为字典的键), 防止意外修改.

================================================================================
数据流向概览:

    STM32 串口 ──> RawFeedback ──> ImuConverter.convert() ──> ImuSample
                      │                                         │
                      │                                         ▼
                      │                                   OdometryIntegrator
                      │                                   .update()
                      │                                         │
                      ▼                                         ▼
              VelocityCommand  <── /cmd_vel               OdometrySample
                  │                                            │
                  ▼                                            ▼
            protocol.encode()                          /odom + TF

================================================================================
"""

# dataclass: Python 的数据类装饰器, 自动生成 __init__ 等方法
# @dataclass(frozen=True) 表示创建不可变的数据类
from dataclasses import dataclass
# Tuple: 元组类型, 用于类型标注
# 元组是不可变的序列, 例如 (1, 2, 3)
from typing import Tuple


# =========================================================================
# 类型别名 (Type Aliases)
# =========================================================================
# 为了提高代码可读性, 定义两个 "类型别名":
#   IntVector3 = Tuple[int, int, int]
#   FloatVector3 = Tuple[float, float, float]
# 这样在后面的代码中, 可以用 IntVector3 代替 Tuple[int, int, int],
# 使意图更清晰: "这是一个包含 3 个整数的元组, 表示三维向量".

# IntVector3: 三个整数的元组, 用于 IMU 原始计数值 (X, Y, Z 三轴)
IntVector3 = Tuple[int, int, int]
# FloatVector3: 三个浮点数的元组, 用于转换后的国际单位制值
FloatVector3 = Tuple[float, float, float]


# =========================================================================
# VelocityCommand: 速度指令
# =========================================================================
@dataclass(frozen=True)
class VelocityCommand:
    """
    ROS 坐标约定下的三轴速度指令, 用于发送给 STM32 底盘.

    在 ROS 中, 机器人常用的坐标系约定 (REP 103):
    - X 轴: 指向前方 (机器人前进方向)
    - Y 轴: 指向左方 (面向 X 轴时的左边)
    - Z 轴: 指向上方 (垂直于地面)

    速度类型说明:
    - linear_x:  沿着 X 轴前进/后退的线速度, 正值前进, 负值后退, 单位 m/s
    - linear_y:  沿着 Y 轴横向平移的线速度, 正值左移, 负值右移, 单位 m/s
                  普通轮式机器人没有这个自由度, 麦克纳姆轮底盘才有
    - angular_z: 绕 Z 轴旋转的角速度, 正值逆时针 (从上方看), 负值顺时针, 单位 rad/s

    默认值均为 0.0, 表示停车状态.

    """

    # 各字段的默认值都是 0.0, 所以 VelocityCommand() 创建一个停车指令
    linear_x: float = 0.0    # X 轴线速度 (m/s), 前进为正
    linear_y: float = 0.0    # Y 轴线速度 (m/s), 左移为正
    angular_z: float = 0.0   # Z 轴角速度 (rad/s), 逆时针为正


# =========================================================================
# RawFeedback: 原始底盘反馈
# =========================================================================
@dataclass(frozen=True)
class RawFeedback:
    """
    已经通过协议校验并完成速度单位换算的底盘反馈数据.

    这个对象由 protocol.decode_feedback_frame() 创建.
    它表示从 STM32 底盘收到的一帧完整反馈数据, 已经过:
    1. 帧头/帧尾/校验和验证 (保证数据完整性)
    2. 速度单位从 mm/s 转换为 m/s
    3. 角速度方向从下位机约定转换为 ROS 约定 (取反)
    4. 航向从 0.01 度单位转换为弧度

    注意: acceleration_raw 和 gyroscope_raw 保留原始的传感器 LSB 计数值,
    因为它们的单位换算依赖于 ICM20602 的配置参数 (LSB/g, LSB/dps).
    换算工作由 ImuConverter 完成.

    字段说明:
        linear_x:        底盘本体 X 轴线速度 (m/s)
        linear_y:        底盘本体 Y 轴线速度 (m/s)
        angular_z:       底盘本体 Z 轴角速度 (rad/s)
        yaw:             底盘绝对航向角 (弧度, 由 STM32 计算)
        acceleration_raw: 三轴加速度原始值 (LSB 计数值), 元组 (ax, ay, az)
        gyroscope_raw:    三轴陀螺仪原始值 (LSB 计数值), 元组 (gx, gy, gz)

    """

    # 底盘运动数据 (已转换为国际单位制)
    linear_x: float     # X 线速度, m/s
    linear_y: float     # Y 线速度, m/s
    angular_z: float    # Z 角速度, rad/s
    yaw: float          # 绝对航向角, rad

    # IMU 原始传感器数据 (保留为 LSB 计数值, 由 ImuConverter 进一步转换)
    acceleration_raw: IntVector3   # (ax_raw, ay_raw, az_raw)
    gyroscope_raw: IntVector3      # (gx_raw, gy_raw, gz_raw)


# =========================================================================
# ImuSample: IMU 数据样本 (已转换单位)
# =========================================================================
@dataclass(frozen=True)
class ImuSample:
    """
    转换为国际单位制 (SI) 后的 IMU 数据.

    由 ImuConverter.convert() 从 RawFeedback 创建.
    所有传感器值已完成单位换算:
    - 加速度: LSB 计数值 -> m/s^2 (米每平方秒)
    - 角速度: LSB 计数值 -> rad/s  (弧度每秒)

    字段说明:
        acceleration:            三轴线加速度 (m/s^2), 元组 (ax, ay, az)
        gyroscope:               三轴角速度 (rad/s), 元组 (gx, gy, gz)
        yaw_rate_before_deadband: 死区修正前的 Z 轴角速度 (rad/s)
                                 保留此值便于传感器标定和现场诊断
        yaw_rate:                 死区修正后的 Z 轴角速度 (rad/s)
                                 这是实际用于里程计积分的值

    """

    acceleration: FloatVector3             # 三轴加速度, 单位 m/s^2
    gyroscope: FloatVector3                # 三轴角速度, 单位 rad/s

    # 死区前后的 Z 轴角速度: 保留两者方便标定和诊断
    # 例如日志中可以看出 "原始是 -0.005 rad/s, 被死区滤为 0"
    yaw_rate_before_deadband: float        # 死区修正前的 Z 轴角速度
    yaw_rate: float                        # 死区修正后的 Z 轴角速度


# =========================================================================
# OdometrySample: 里程计状态
# =========================================================================
@dataclass(frozen=True)
class OdometrySample:
    """
    一次二维里程计更新后的完整状态.

    由 OdometryIntegrator.update() 在每帧底盘反馈到达时更新.
    里程计用于估算机器人在世界坐标系 (odom 坐标系) 中的位姿.

    坐标系说明:
    - odom 坐标系: 里程计的参考坐标系, 原点在机器人启动时的位置
    - base_link 坐标系: 机器人的本体坐标系, 原点通常在底盘中心

    字段说明:
        x, y:      机器人质心在 odom 坐标系中的位置 (米)
        yaw:        机器人朝向角 (弧度), 0 表示朝向 odom 坐标系的 X 轴正方向
        linear_x, linear_y: 机器人本体坐标系下的线速度 (m/s)
        angular_z:  机器人本体坐标系下的角速度 (rad/s)
        yaw_source: 航向来源标识, 目前固定为 "STM32_YAW"
                    这个字段用于状态日志, 确认航向没有被重复积分

    """

    # 二维位姿 (相对于 odom 坐标系)
    x: float        # X 坐标, m
    y: float        # Y 坐标, m
    yaw: float      # 偏航角, rad

    # 速度 (相对于机器人本体坐标系)
    linear_x: float     # X 轴线速度, m/s
    linear_y: float     # Y 轴线速度, m/s
    angular_z: float    # Z 轴角速度, rad/s

    # 航向来源标识
    # "STM32_YAW" 表示航向直接来自 STM32 底盘, 没有经过二次积分
    yaw_source: str
