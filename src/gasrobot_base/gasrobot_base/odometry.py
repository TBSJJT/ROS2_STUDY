#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
麦克纳姆底盘二维里程计积分.
中点法: 用 "上帧航向" 和 "本帧航向" 的中间角度来旋转
  yaw_mid = normalize_angle(yaw_prev + (yaw_current - yaw_prev) * 0.5)
  world_vx = vx_body * cos(yaw_mid) - vy_body * sin(yaw_mid)
  world_vy = vx_body * sin(yaw_mid) + vy_body * cos(yaw_mid)

为什么取中点?
  假设角速度恒定, 整个积分区间内航向是线性变化的.
  取中点的航向来旋转速度, 比取起点或终点的精度更高.
  这在数学上等价于"梯形积分法".
"""

import math
from typing import Optional

from gasrobot_base.models import OdometrySample, RawFeedback


def normalize_angle(angle: float) -> float:
    """
    将任意角度归一化到 [-pi, pi) 区间.

    数学原理:
        atan2(sin(angle), cos(angle))
    由于 sin(x + 2*pi) = sin(x) 且 cos(x + 2*pi) = cos(x),
    atan2 会自动返回 [-pi, pi) 范围内的 "主值" (principal value).

    """
    return math.atan2(math.sin(angle), math.cos(angle))


class OdometryIntegrator:
    """
    里程计积分器: 将 STM32 上报的线速度和角度积分为二维位置.
    """
    def __init__(
        self,
        use_imu_angular_velocity: bool,
        max_interval: float = 0.5,
    ) -> None:
        """
        初始化二维位置状态和角速度输出策略.
        参数:
            use_imu_angular_velocity: 角速度来源开关
                True  -> 使用 IMU 陀螺仪的 Z 轴角速度 (经过修正的值)
                False -> 使用底盘反馈的原始角速度
                这个开关只影响 twist.angular.z, 不参与 yaw 的计算

            max_interval: 最大积分时间间隔 (秒)
                如果两帧之间的时间差超过这个值, 跳过积分
                默认 0.5 秒, 对应串口 2Hz 的最低期望频率
                设得太大容易被异常的时间间隔导致位置跳跃
        """
        # 角速度来源开关
        self.use_imu_angular_velocity = use_imu_angular_velocity
        # 最大积分间隔
        self.max_interval = max_interval

        # 位置状态 (世界坐标系)
        # x, y: 机器人质心的 X/Y 坐标 (米)
        # 这两个值是 ROS 端唯一需要积分的状态量
        self.x = 0.0
        self.y = 0.0

        # 航向状态 (世界坐标系)
        # yaw 每帧被 STM32 报告的绝对值覆盖, 不参与积分
        self.yaw = 0.0

        # 上一帧的时间戳 (秒)
        # 初始为 None, 表示还没有收到过数据
        # Optional 是 typing 模块提供的泛型，等价于 Union[float, None]
        # 表示该变量可以是 float 类型，也可以是 None。
        # 当尚未记录时间戳时，值为 None。
        # 一旦赋值，通常为一个浮点数（如 time.time() 返回的秒数）。
        self._last_stamp: Optional[float] = None

    def reset_time(self) -> None:
        """
        清除上次时间戳, 避免断线时段被积分.

        当串口断线重连时调用.
        重连后第一帧没有有效的时间间隔, 只记录状态, 不做位置积分.
        这防止了断线几秒钟后产生一个巨大的位置跳跃.
        """
        self._last_stamp = None

    def update(
        self,
        feedback: RawFeedback,
        imu_angular_z: float,
        stamp: float,
    ) -> OdometrySample:
        """
        根据一帧反馈更新里程计并返回当前状态.

        这是里程计的核心方法, 每收到一帧底盘反馈就调用一次.

        处理流程:
        1. 根据开关选择角速度来源
        2. 计算与上一帧的时间间隔 Δt
        3. 检查 Δt 是否有效 (>0 且 <= max_interval)
        4. 如果有效, 用中点法积分位置
        5. 用 STM32 绝对值更新航向
        6. 返回 OdometrySample

        参数:
            feedback:      从串口协议解析出的原始反馈帧
            imu_angular_z: IMU Z 轴角速度 (rad/s, 已修正)
            stamp:         当前帧的时间戳 (秒, 通常来自 ROS 时钟)

        返回:
            OdometrySample: 积分后的最新里程计状态

        """
        # ---------- 步骤 1: 选择角速度来源 ----------
        # Python 的三元表达式: <true_value> if <condition> else <false_value>
        angular_z = (
            imu_angular_z                    # 使用 IMU 的值
            if self.use_imu_angular_velocity # 如果开关打开
            else feedback.angular_z           # 否则使用底盘反馈的值
        )

        # ---------- 步骤 2: 计算时间间隔 ----------
        interval = 0.0  # 默认为 0 (表示不积分)
        if self._last_stamp is not None:
            # 计算与上一帧的时间差
            interval = stamp - self._last_stamp

            # ---------- 步骤 3: 检查 Δt 有效性 ----------
            # 以下情况跳过积分:
            # 1. interval <= 0.0: 时间倒退 (时钟异常) 或同一帧处理了两次
            # 2. interval > max_interval: 间隔太大 (串口断线)
            if interval <= 0.0 or interval > self.max_interval:
                interval = 0.0  # 强制不积分

        # 更新上一帧时间戳 (无论是否积分)
        self._last_stamp = stamp

        # ---------- 步骤 4: 中点法积分 ----------
        if interval > 0.0:
            # --- 4a: 计算航向差 (走短边) ---
            # normalize_angle 确保航向跨越 -pi/pi 边界时走最短路径
            # 例如: yaw 从 179 度到 -179 度, 差值应该是 +2 度, 不是 -358 度
            yaw_delta = normalize_angle(feedback.yaw - self.yaw)

            # --- 4b: 计算中点航向 ---
            # 取上一帧航向和本帧航向的中间值
            # 再用 normalize_angle 确保中点也在 [-pi, pi) 范围内
            middle_yaw = normalize_angle(self.yaw + yaw_delta * 0.5)

            # --- 4c: 本体速度 -> 世界速度 (旋转坐标系) ---
            # 这是二维旋转矩阵的应用:
            #   [world_vx]   [cos(yaw)  -sin(yaw)] [body_vx]
            #   [world_vy] = [sin(yaw)   cos(yaw)] [body_vy]
            #
            # 为什么要旋转?
            # 底盘反馈的 linear_x, linear_y 是在机器人本体坐标系中的:
            # linear_x 是"机器人前方"的速度, linear_y 是"机器人左侧"的速度
            # 但里程计需要知道在世界坐标系 (固定不动) 中的位移
            # 用当前航向进行旋转, 把 "相对于机器人" 的速度转为 "相对于世界" 的速度
            world_linear_x = (
                feedback.linear_x * math.cos(middle_yaw)
                - feedback.linear_y * math.sin(middle_yaw)
            )
            world_linear_y = (
                feedback.linear_x * math.sin(middle_yaw)
                + feedback.linear_y * math.cos(middle_yaw)
            )

            # --- 4d: 速度 * 时间 = 位移 ---
            # 位移 = 速度 * Δt
            self.x += world_linear_x * interval
            self.y += world_linear_y * interval

        # ---------- 步骤 5: 更新航向 (绝对值覆盖) ----------
        # 航向始终以本帧 STM32 上报的绝对值为准
        # 不对角速度做二次积分, 避免累计误差
        self.yaw = feedback.yaw

        # ---------- 步骤 6: 返回当前状态 ----------
        return OdometrySample(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            linear_x=feedback.linear_x,   # 本体坐标系下的速度
            linear_y=feedback.linear_y,
            angular_z=angular_z,
            yaw_source="STM32_YAW",         # 标记航向来源为 STM32
        )
