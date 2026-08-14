#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
二维里程计积分器的单元测试。

本测试文件验证 OdometryIntegrator 类的核心功能：
- 首帧不积分位置（因为没有上一帧做时间参考）
- 平移速度按相邻两帧航向的中点转换坐标系
- 航向跨越 ±180 度（π/-π）时沿最短角差计算
- tan(角速度来源选择（IMU 或底盘实测）)
- 断线重连时重置时间戳，不累积平移

里程计（Odometry）是机器人定位的基础模块：
它根据底盘各方向的速度和方向，估算机器人在地图中的位置 (x, y, yaw)。
虽然里程计会随时间积累误差，但它是高频、低延迟的定位手段，
与 AMCL（概率定位）配合使用可以达到最佳效果。

我们的里程计策略（与本文件对应）：
- 位置 (x, y)：用底盘平移速度积分（梯形法 + 中点法）
- 航向 (yaw)：直接采用 STM32 的绝对值（不对角速度二次积分）
  因为 STM32 有更高精度的角度计算
- 角速度 (angular_z)：可选择使用 IMU Z 轴陀螺仪或底盘反馈
"""

import math

import pytest

from gasrobot_base.models import RawFeedback
from gasrobot_base.odometry import OdometryIntegrator, normalize_angle


# -----------------------------------------------------------------------
# 辅助函数：快速创建测试反馈
# -----------------------------------------------------------------------
def _feedback(
    linear_x: float = 1.0,
    linear_y: float = 0.0,
    angular_z: float = 0.0,
    yaw: float = 0.0,
) -> RawFeedback:
    """
    构造只包含里程计测试所需字段的 RawFeedback 样本。

    因为 RawFeedback 是不可变的 frozen dataclass（@dataclass(frozen=True)），
    所有字段在创建时都必须提供。但测试里程计时不需要真正的 IMU 数据，
    所以 acceleration_raw 和 gyroscope_raw 填零即可。

    参数:
        linear_x:  底盘本体 X 轴方向线速度 (m/s)
        linear_y:  底盘本体 Y 轴方向线速度 (m/s)
        angular_z: 底盘本体角速度 (rad/s)
        yaw:       STM32 报告的绝对航向角 (弧度)

    返回:
        一个新的 RawFeedback 对象

    """
    return RawFeedback(
        linear_x=linear_x,
        linear_y=linear_y,
        angular_z=angular_z,
        yaw=yaw,
        # IMU 字段在里程计测试中不使用，全部填 0
        acceleration_raw=(0, 0, 0),
        gyroscope_raw=(0, 0, 0),
    )


# -----------------------------------------------------------------------
# 测试 1：首帧不积分
# -----------------------------------------------------------------------
def test_first_sample_initializes_time_and_adopts_stm32_yaw():
    """
    验证首帧不积分位置，但立即采用 STM32 报告的绝对航向。

    为什么首帧不积分位置？
    - 积分需要时间间隔 Δt（两帧之间的时间差）
    - 第一帧只有当前速度，没有前一帧的时间戳
    - Δt 未知的情况下，不能计算位移
    - 所以第一帧只记录初始状态，不做位置更新

    为什么航向不需要积分？
    - 航向直接来自 STM32 的角度计算，是绝对值而非增量
    - 所以首帧也应该立即采用 STM32 的航向值

    """
    # 创建积分器：use_imu_angular_velocity=True 表示用 IMU 的 Z 轴角速度
    integrator = OdometryIntegrator(use_imu_angular_velocity=True)

    # 传入第一帧：yaw=0.75 弧度，imu_angular_z=0.5 rad/s，时间戳=10.0 秒
    # update() 返回 OdometrySample，包含积分后的完整状态
    sample = integrator.update(
        _feedback(yaw=0.75),
        imu_angular_z=0.5,
        stamp=10.0,
    )

    # --- 断言 ---
    # 首帧：位置不应该变化（因为没有 Δt）
    assert sample.x == 0.0
    assert sample.y == 0.0
    # 航向应该立即采用 STM32 的值
    assert sample.yaw == pytest.approx(0.75)
    # 航向来源应该标记为 "STM32_YAW"
    assert sample.yaw_source == "STM32_YAW"


# -----------------------------------------------------------------------
# 测试 2：中点法积分
# -----------------------------------------------------------------------
def test_midpoint_integration_uses_consecutive_stm32_yaw_values():
    """
    验证平移速度按照相邻两帧航向的中点转换到 odom 坐标系。

    问题背景：
    底盘反馈的是本体坐标系下的速度 (linear_x, linear_y)，
    但里程计需要在世界坐标系 (odom) 下积分。
    本体坐标相对于世界坐标的角度会不断变化，直接忽略角度变化
    会产生很大的积分误差。

    中点法（Midpoint Method）：
    1. 取上一帧航向和当前帧航向的中间值
    2. 以这个中间航向作为本体→世界的旋转角
    3. 把本体速度旋转到世界坐标系

    这样做比直接用上一帧或当前帧的角度更精确。

    测试场景：
    - 初始航向 0 弧度（指向 X 轴正方向）
    - 1 秒后航向变为 1 弧度（约 57.3 度）
    - 底盘始终以 1 m/s 沿本体 X 轴前进
    - 中值航向 = (0 + 1) / 2 = 0.5 弧度

    期望结果：
    - 位移 ≈ cos(0.5) 在 X 方向，sin(0.5) 在 Y 方向
    - 航向采用最新 STM32 值 1.0

    """
    # 最大时间间隔设为 2.0 秒，确保 1 秒间隔不会被当作异常
    integrator = OdometryIntegrator(
        use_imu_angular_velocity=True,
        max_interval=2.0,
    )

    # 第一帧：航向 = 0，时间戳 = 0.0
    integrator.update(
        _feedback(yaw=0.0),
        imu_angular_z=1.0,
        stamp=0.0,
    )

    # 第二帧：航向 = 1.0 弧度，时间戳 = 1.0 秒
    sample = integrator.update(
        _feedback(yaw=1.0),
        imu_angular_z=1.0,
        stamp=1.0,
    )

    # --- 断言 ---
    # cos(0.5) ≈ 0.8776：底盘以 1 m/s 前进 1 秒 → 约 0.8776 米在 X 方向
    assert sample.x == pytest.approx(math.cos(0.5))
    # sin(0.5) ≈ 0.4794：约 0.4794 米在 Y 方向
    assert sample.y == pytest.approx(math.sin(0.5))
    # 航向直接采用 STM32 最新值
    assert sample.yaw == pytest.approx(1.0)


# -----------------------------------------------------------------------
# 测试 3：航向跨越临界点
# -----------------------------------------------------------------------
def test_yaw_midpoint_uses_short_path_across_wrap_boundary():
    """
    验证航向跨越正负 180 度（π/-π 弧度）时沿最短角差计算。

    这是一个经典的"角度环绕"（Angle Wrap）问题：
    - 航向 179° (≈ π) 和 -179° (≈ -π) 在数学上只差 2°
    - 但简单平均 (π + (-π)) / 2 = 0 → 指向完全错误的方向！
    - 正确做法是"走短边"：从 179° 逆时针走 2° 到 -179°
      中值 = 179° + 1° = 180°（或 -180°）

    测试场景：
    - 上一帧航向 = 179°（接近 π）
    - 当前帧航向 = -179°（接近 -π）
    - 底盘以 1 m/s 沿本体 X 轴前进
    - 正确的中间航向为 ±180°（指向 X 轴负方向）

    如果代码有 bug，直接做数学平均：(179 + (-179)) / 2 = 0，
    则积分会指向 X 轴正方向，结果完全错误。

    """
    integrator = OdometryIntegrator(
        use_imu_angular_velocity=False,  # 使用底盘原始反馈角速度
        max_interval=2.0,
    )

    # 初始化为 179 度（接近 π）
    integrator.update(
        _feedback(yaw=math.radians(179.0)),  # math.radians 度→弧度
        imu_angular_z=0.0,
        stamp=0.0,
    )

    # 1 秒后航向变为 -179 度（接近 -π）
    # 实际只旋转了约 2 度
    sample = integrator.update(
        _feedback(yaw=math.radians(-179.0)),
        imu_angular_z=0.0,
        stamp=1.0,
    )

    # --- 断言 ---
    # 正确的中间航向为 ±180 度，底盘以 1 m/s 前进 1 秒
    # cos(±π) = -1.0, sin(±π) = 0.0
    # 所以 X 方向位移 ≈ -1.0 米（向后方走）
    assert sample.x == pytest.approx(-1.0, abs=1e-6)
    # Y 方向位移 ≈ 0
    assert sample.y == pytest.approx(0.0, abs=1e-6)
    # 航向采用最新 STM32 值
    assert sample.yaw == pytest.approx(math.radians(-179.0))


# -----------------------------------------------------------------------
# 测试 4：角速度来源与断线保护
# -----------------------------------------------------------------------
def test_twist_source_and_reset_prevent_gap_translation():
    """
    验证角速度来源选择，以及断线重连期间不累计平移。

    角速度来源选择：
    - use_imu_angular_velocity=True → 使用 IMU 陀螺仪的 Z 轴值
    - use_imu_angular_velocity=False → 使用底盘反馈的 angular_z 值
    IMU 的角速度通常更准（采样率更高、不受轮子打滑影响），
    但需要先经过零偏和死区修正。

    断线保护：
    - 串口断开期间，无法收到底盘反馈帧
    - 重新连接后，第一帧数据的 Δt 可能非常大（几秒甚至几十秒）
    - 如果直接用这个巨大的 Δt 积分，会产生巨大的位置跳跃
    - 所以断线后的第一帧不积分（reset_time() 清除时间戳）

    """
    # 使用底盘角速度（use_imu_angular_velocity=False）
    integrator = OdometryIntegrator(
        use_imu_angular_velocity=False,
        max_interval=1.0,  # 最大间隔 1 秒，超过则不积分
    )

    # 第一帧
    feedback = _feedback(angular_z=-0.25, yaw=0.3)
    integrator.update(feedback, imu_angular_z=1.0, stamp=1.0)

    # 第二帧：0.5 秒后
    sample = integrator.update(feedback, imu_angular_z=1.0, stamp=1.5)

    # 断言：使用的角速度来自底盘反馈（-0.25），不是 IMU（1.0）
    assert sample.angular_z == pytest.approx(-0.25)
    # 航向来源始终标记为 STM32_YAW
    assert sample.yaw_source == "STM32_YAW"

    # --- 模拟断线重连 ---
    # 断线时调用 reset_time() 清除 _last_stamp
    integrator.reset_time()

    # 重连后的第一帧（时间戳跳跃 18.5 秒）
    after_reset = integrator.update(
        _feedback(angular_z=-0.25, yaw=0.8),
        imu_angular_z=1.0,
        stamp=20.0,  # 时间从 1.5 跳到 20.0
    )

    # 断言：重连后位置不应该变化（没有积分）
    assert after_reset.x == pytest.approx(sample.x)
    assert after_reset.y == pytest.approx(sample.y)
    # 但航向应该更新为新值（STM32 报告的绝对值）
    assert after_reset.yaw == pytest.approx(0.8)


# -----------------------------------------------------------------------
# 测试 5：角度归一化函数
# -----------------------------------------------------------------------
def test_normalize_angle_keeps_result_in_expected_range():
    """
    验证 normalize_angle() 能将任意角度归一化到 [-π, π) 区间。

    角度归一化是机器人和图形学中非常常用的函数。
    例如 3π（540 度）在三角函数意义上等同于 π（180 度），
    而归一化到 [-π, π) 区间使角度便于比较和显示。

    数学原理：
    normalize_angle(x) = atan2(sin(x), cos(x))
    由于 sin(x + 2π)=sin(x) 且 cos(x + 2π)=cos(x)，
    atan2 会自动返回 [-π, π) 范围内的主值。

    测试用例：
    - 3π (540°) → 应该归一化到 π (180°)

    """
    # 3 * π = 3π，在数学上等同于 π（因为多转了完整一圈）
    # normalize_angle 应该返回 π
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
