#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
为什么需要 ImuConverter？

STM32 通过 SPI 接口读取 ICM20602 的原始数据, 以 LSB (最低有效位) 计数值
通过串口传给 ROS 2. ROS 2 要求 IMU 数据使用国际单位制:
- 线加速度: m/s^2 (米每平方秒)
- 角速度: rad/s (弧度每秒)

ImuConverter 负责:
1. 单位换算: LSB 计数值 -> 物理单位
2. Z 轴修正: 方向符号 + 零偏补偿 + 死区过滤

陀螺仪的 Z 轴测量值经过三道修正:
1. 方向修正 (z_sign): 如果 IMU 安装方向与底盘 Z 轴相反, 乘以 -1 翻转
2. 零偏补偿 (z_offset): 减去静止时的漂移量
3. 死区过滤 (z_deadband): 绝对值小于阈值的信号强制设为 0, 消除微小噪声

修正后的值 (yaw_rate) 用于:
- 机器人航向角速度 (twist.angular.z, 当 use_imu_wz_for_twist=true 时)
- 状态显示和调试

修正前的值 (yaw_rate_before_deadband) 保留用于:
- 传感器标定 (帮助确定合适的零偏值)
- 现场诊断 (判断陀螺仪是否正常工作)

"""

# math 模块: 提供数学函数
# math.radians(): 度转弧度, 乘以 pi/180
import math

# 导入数据模型:
# ImuSample: IMU 转换后的输出数据
# RawFeedback: 从串口协议解析出的原始反馈
from gasrobot_base.models import ImuSample, RawFeedback


# =========================================================================
# 常量定义
# =========================================================================
# GRAVITY: 标准重力加速度 (m/s^2)
# ISO 标准值, 用于将 "g 单位" 的加速度转换为 m/s^2
# 例如: 1g = 9.80665 m/s^2, 2g = 19.6133 m/s^2
GRAVITY = 9.80665


# =========================================================================
# ImuConverter: IMU 数据转换器
# =========================================================================
class ImuConverter:
    """
    将 ICM20602 的原始 LSB 计数转换为 ROS 使用的国际单位制 (SI) 数据.

    这个类在初始化时保存量程系数和 Z 轴修正参数,
    然后通过 convert() 方法处理每一帧反馈中的 IMU 数据.

    使用示例:
        converter = ImuConverter(
            acceleration_lsb_per_g=4096.0,   # 4096 LSB = 1g
            gyroscope_lsb_per_dps=131.0,     # 131 LSB = 1 度/秒
            z_sign=1.0,                       # Z 轴方向不反转
            z_offset_radps=0.0,               # 不补偿零偏
            z_deadband=0.02,                  # 0.02 rad/s 以下的信号视为噪声
        )
        imu_sample = converter.convert(raw_feedback)

    """

    def __init__(
        self,
        acceleration_lsb_per_g: float,
        gyroscope_lsb_per_dps: float,
        z_sign: float,
        z_offset_radps: float,
        z_deadband: float,
    ) -> None:
        """
        保存传感器量程和 Z 轴修正参数.

        参数:
            acceleration_lsb_per_g: 加速度计量程系数 (LSB/g)
                                    即多少个 LSB 计数值对应 1g 的加速度
                                    例如 4096 意味着 4096 个计数 = 1g = 9.80665 m/s^2
                                    这个值取决于 ICM20602 的寄存器配置, 必须在
                                    STM32 固件和 ROS 端保持一致, 否则换算错误

            gyroscope_lsb_per_dps:  陀螺仪量程系数 (LSB/dps)
                                    即多少个 LSB 计数值对应 1 度/秒的角速度
                                    例如 131 意味着 131 个计数 = 1 度/秒

            z_sign:                 Z 轴角速度的方向修正符号
                                    1.0 表示不反转, -1.0 表示反转
                                    用于处理 IMU 芯片安装方向与底盘 Z 轴不一致的情况
                                    内部强制二值化: 正值为 1.0, 负值为 -1.0

            z_offset_radps:         Z 轴角速度的零偏补偿值 (rad/s)
                                    陀螺仪即使在静止时, 也可能输出微小的非零值
                                    (静态漂移). 这个值用于补偿该漂移:
                                    yaw_rate = z_sign * gyro_z - z_offset

            z_deadband:             Z 轴角速度的死区阈值 (rad/s)
                                    当修正后的角速度绝对值小于此阈值时, 强制输出 0
                                    用于滤除静止时的微小随机噪声
                                    如果为负值 (配置错误), 整个死区不生效

        """
        # 保存量程系数
        # 注意: 这些值必须与 STM32 端 ICM20602 寄存器的配置一致
        # 如果不一致, 转换出的物理值会有系统性的倍数误差
        self.acceleration_lsb_per_g = acceleration_lsb_per_g
        self.gyroscope_lsb_per_dps = gyroscope_lsb_per_dps

        # Z 轴修正参数
        # z_sign 被强制二值化: 正为 1.0, 负为 -1.0 (不允许 0)
        # 这样做可以避免配置错误导致 Z 轴信号被意外置零
        self.z_sign = -1.0 if z_sign < 0.0 else 1.0
        self.z_offset_radps = z_offset_radps
        self.z_deadband = z_deadband

    def convert(self, feedback: RawFeedback) -> ImuSample:
        """
        转换一帧反馈中的加速度计和陀螺仪数据.

        这是 ImuConverter 的核心方法, 每收到一帧底盘反馈就调用一次.
        转换过程: RawFeedback -> ImuSample

        参数:
            feedback: 从串口协议解析出的原始反馈帧
                      包含了三轴加速度和三轴陀螺仪的 LSB 原始计数值

        返回:
            ImuSample: 转换后的国际单位制 IMU 数据
                       包含转换后的加速度 (m/s^2), 角速度 (rad/s),
                       以及死区前后的 Z 轴角速度

        """
        # ---------- 步骤 1: 加速度换算 ----------
        # 公式: m/s^2 = raw_count / accel_lsb_per_g * GRAVITY
        #
        # 使用 "生成器表达式" (generator expression) 对三个轴逐一转换:
        #   for raw in feedback.acceleration_raw
        #     遍历原始加速度元组 (ax_raw, ay_raw, az_raw)
        #   raw / self.acceleration_lsb_per_g * GRAVITY
        #     把每个 LSB 值转换为 m/s^2
        # 外层的 tuple() 把生成器转换为不可变的元组
        #
        # 示例: LSB=4096, accel_lsb_per_g=4096.0
        #   4096 / 4096.0 * 9.80665 = 9.80665 m/s^2 (1g, 即静止时 Z 轴读数)
        acceleration = tuple(
            raw / self.acceleration_lsb_per_g * GRAVITY
            for raw in feedback.acceleration_raw
        )

        # ---------- 步骤 2: 陀螺仪换算 ----------
        # 公式: rad/s = radians(raw_count / gyro_lsb_per_dps)
        #
        # math.radians() 把角度从 "度" 转换为 "弧度"
        # 角度 = raw / gyro_lsb_per_dps (得到 度/秒)
        # radians(角度) = 角度 * pi / 180 (得到 弧度/秒)
        #
        # 示例: LSB=131, gyro_lsb_per_dps=131.0
        #   radians(131 / 131.0) = radians(1.0) = 0.01745 rad/s (约 1 度/秒)
        gyroscope = tuple(
            math.radians(raw / self.gyroscope_lsb_per_dps)
            for raw in feedback.gyroscope_raw
        )

        # ---------- 步骤 3: Z 轴角速度修正 ----------
        # 阶段 3a: 方向修正 + 零偏补偿
        # gyroscope[2] 是 Z 轴角速度 (Python 索引从 0 开始, 所以 [2] 是第三个元素)
        # z_sign:  翻转方向 (1.0 不变, -1.0 反转)
        # z_offset: 减去零偏漂移值
        yaw_rate_before_deadband = (
            self.z_sign * gyroscope[2] - self.z_offset_radps
        )

        # 阶段 3b: 死区过滤
        # 先用修正前的值初始化 (如果死区不生效, 就原样输出)
        yaw_rate = yaw_rate_before_deadband
        # abs() 取绝对值: 检查角速度大小是否小于死区阈值
        if abs(yaw_rate) < self.z_deadband:
            # 绝对值小于死区阈值 -> 认为是传感器噪声 -> 强制设为 0
            # 注意: 死区只影响用于运动控制的 yaw_rate,
            # 不影响 gyroscope 元组中的原始 Z 轴角速度
            yaw_rate = 0.0

        # ---------- 步骤 4: 创建并返回 ImuSample ----------
        # ImuSample 是不可变数据类 (frozen=True), 创建后不能修改
        return ImuSample(
            acceleration=acceleration,
            gyroscope=gyroscope,
            yaw_rate_before_deadband=yaw_rate_before_deadband,
            yaw_rate=yaw_rate,
        )
