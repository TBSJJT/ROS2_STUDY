#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 传感器单位换算与角速度修正的单元测试。

本测试文件验证 ImuConverter 类的核心功能：
- 加速度原始计数值 → 国际单位制 m/s² 的转换
- 角速度原始计数值 → 国际单位制 rad/s 的转换
- Z 轴角速度的符号修正（处理安装方向翻转）
- Z 轴角速度的零偏补偿（消除陀螺仪静态漂移）
- Z 轴角速度的死区过滤（滤除静止时的小幅噪声）

ICM20602 是 STM32 底盘上使用的 6 轴惯性传感器（3 轴加速度 + 3 轴陀螺仪），
它通过 SPI 接口与 STM32 通信，原始数据以 LSB（最低有效位）计数值通过串口传回。
ROS 2 要求 IMU 数据使用国际单位制（m/s² 和 rad/s），所以需要做单位转换。
"""

# math 模块提供数学函数，这里用 math.radians() 做度到弧度的转换
import math

# pytest 提供测试框架和近似值比较
import pytest

# 导入被测试的类和常量
from gasrobot_base.imu import GRAVITY, ImuConverter
# RawFeedback：从串口协议解析出的原始底盘反馈数据
from gasrobot_base.models import RawFeedback


# -----------------------------------------------------------------------
# 辅助函数：构造测试用反馈数据
# -----------------------------------------------------------------------
def _feedback(gyroscope_z: int) -> RawFeedback:
    """
    构造一个用于测试的 RawFeedback 反馈帧。

    为了简化测试，这个函数：
    - 固定加速度原始值为 (4096, 0, -4096)
      - 4096 对应 +1g（如果有 accel_lsb_per_g=4096）
      - -4096 对应 -1g
      - 0 表示该轴没有加速度
    - 固定 X/Y 轴陀螺仪原始值为 (131, -262)
      - 131 对应 +1 dps（如果有 gyro_lsb_per_dps=131）
      - -262 对应 -2 dps
    - 只暴露 gyroscope_z 参数，方便测试 Z 轴的各种修正

    参数:
        gyroscope_z: Z 轴陀螺仪的原始计数值

    返回:
        一个 RawFeedback 对象，其中运动和航向字段填 0，IMU 字段按规则填充

    """
    return RawFeedback(
        # 底盘运动字段：测试中不使用，全部填 0
        linear_x=0.0,
        linear_y=0.0,
        angular_z=0.0,
        yaw=0.0,
        # 加速度计原始值：(X, Y, Z) = (4096, 0, -4096)
        # 即 X 轴测到 +1g，Z 轴测到 -1g（重力方向）
        acceleration_raw=(4096, 0, -4096),
        # 陀螺仪原始值：(X, Y, Z) = (131, -262, gyroscope_z)
        # X 轴正转 1dps，Y 轴反转 2dps，Z 轴可配置
        gyroscope_raw=(131, -262, gyroscope_z),
    )


# -----------------------------------------------------------------------
# 测试 1：验证国际单位制转换
# -----------------------------------------------------------------------
def test_converter_outputs_si_units():
    """
    验证加速度和角速度能正确换算为国际单位制（SI 单位）。

    国际单位制要求：
    - 线加速度用 m/s²（米每平方秒）
    - 角速度用 rad/s（弧度每秒）

    ICM20602 原始数据是 LSB 计数值，需要除以量程系数再乘以单位换算因子。

    加速度换算公式：
        m/s² = raw / accel_lsb_per_g × GRAVITY
    其中 GRAVITY = 9.80665 m/s²（标准重力加速度）

    角速度换算公式：
        rad/s = radians(raw / gyro_lsb_per_dps)
    即先把 LSB 转为 度/秒，再把度转为弧度

    测试参数：
        accel_lsb_per_g = 4096.0  →  4096 LSB = 1g
        gyro_lsb_per_dps = 131.0  →  131 LSB = 1 度/秒
        z_sign = 1.0（不反转）
        z_offset_radps = 0.0（不补偿零偏）
        z_deadband = 0.0（不用死区）

    """
    # 创建 ImuConverter，所有修正参数设为默认值（不做修正）
    # 这样我们只测试纯单位换算是否正确
    converter = ImuConverter(4096.0, 131.0, 1.0, 0.0, 0.0)

    # 用 gyroscope_z=393 创建反馈帧
    # 393 / 131 = 3 度/秒，这是 Z 轴角速度
    sample = converter.convert(_feedback(393))

    # --- 断言加速度 ---
    # pytest.approx() 是浮点数比较函数
    # 由于 IEEE 754 浮点数精度限制，直接用 == 比较可能失败
    # approx() 允许微小误差（默认 1e-6 相对误差）
    # X 轴：4096 / 4096 × GRAVITY = 1 × 9.80665 = 9.80665 m/s²
    # Y 轴：0 / 4096 × GRAVITY = 0 m/s²
    # Z 轴：-4096 / 4096 × GRAVITY = -9.80665 m/s²
    assert sample.acceleration == pytest.approx((GRAVITY, 0.0, -GRAVITY))

    # --- 断言角速度 ---
    # math.radians(1.0)：把 1 度转为弧度
    # X 轴：131 / 131 = 1 度/秒 → radians(1.0) rad/s
    assert sample.gyroscope[0] == pytest.approx(math.radians(1.0))
    # Y 轴：-262 / 131 = -2 度/秒 → radians(-2.0) rad/s
    assert sample.gyroscope[1] == pytest.approx(math.radians(-2.0))
    # Z 轴：393 / 131 = 3 度/秒 → radians(3.0) rad/s
    assert sample.yaw_rate == pytest.approx(math.radians(3.0))


# -----------------------------------------------------------------------
# 测试 2：验证 Z 轴修正链
# -----------------------------------------------------------------------
def test_converter_applies_sign_offset_and_deadband():
    """
    验证 Z 轴的方向符号、零偏补偿和静止死区三种修正依次生效。

    Z 轴修正的意义：
    1. 方向修正（z_sign = -1.0）：
       如果 IMU 安装方向与底盘坐标系 Z 轴相反（正转产生负读数），
       需要乘以 -1 来翻转符号。
    2. 零偏补偿（z_offset_radps = -0.01）：
       陀螺仪静止时也可能有微小输出（静态漂移），
       公式是 yaw_rate = z_sign × gyro_z - z_offset
       这里 z_offset = -0.01，相当于给 yaw_rate 加 0.01。
    3. 死区过滤（z_deadband = 0.01）：
       当修正后的角速度绝对值 < 死区阈值时，强制设为 0，
       防止微小噪声被积分累积导致底盘缓慢漂移。

    本测试中：
    - gyroscope_z = 65（对应约 0.496 度/秒 ≈ 0.00866 rad/s）
    - z_sign = -1.0：翻转后 ≈ -0.00866
    - z_offset = -0.01：修正后 yaw_rate_before_deadband = -0.00866 - (-0.01) ≈ 0.00134
    - deadband = 0.01：因为 |0.00134| < 0.01，最终 yaw_rate = 0.0

    """
    converter = ImuConverter(
        4096.0,       # 加速度量程系数
        131.0,        # 陀螺仪量程系数
        z_sign=-1.0,  # Z 轴方向反转
        z_offset_radps=-0.01,  # Z 轴零偏补偿值
        z_deadband=0.01,       # 死区阈值 0.01 rad/s
    )

    # 用 gyroscope_z=65 创建反馈帧
    sample = converter.convert(_feedback(65))

    # --- 断言死区前的值 ---
    # 65 / 131 × (π/180) ≈ 0.00866 rad/s（这是弧度制的角速度）
    # 经过符号翻转和零偏后的绝对值应该小于死区阈值
    # |yaw_rate_before_deadband| < 0.01
    assert abs(sample.yaw_rate_before_deadband) < 0.01

    # --- 断言死区起作用 ---
    # 因为绝对值小于死区阈值，yaw_rate 应该被强制设为 0.0
    # 这样就滤除了静止时的传感器噪声
    assert sample.yaw_rate == 0.0
