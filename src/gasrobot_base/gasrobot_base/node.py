#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
它负责:
1. 从 /cmd_vel 话题接收速度指令 -> 编码为二进制协议帧 -> 通过串口发送给 STM32
2. 通过串口接收 STM32 反馈帧 -> 解码 -> 发布 /odom (里程计) 和 /imu (IMU) 话题
3. TF 广播: odom -> base_link 的坐标变换
本节点采用 "三层初始化" 架构, 将创建过程组织为:

第一层: 基础组件 (硬件无关)
  - BridgeConfig:  从 ROS 参数读取配置
  - SerialTransport:  串口通信 (不依赖 ROS)
  - FeedbackStreamParser: 协议帧解析 (不依赖 ROS)
  - ImuConverter:   IMU 数据转换 (不依赖 ROS)
  - OdometryIntegrator: 里程计积分 (不依赖 ROS)

第二层: 运行状态
  - target_command:    最新目标速度指令
  - latest_feedback:   最新收到的底盘反馈
  - latest_imu:        最新 IMU 样本
  - latest_odometry:   最新里程计状态
  - statistics:        通信统计计数器

第三层: ROS 接口
  - 订阅 /cmd_vel (速度指令)
  - 发布 /odom (里程计)
  - 发布 /imu/data_raw (IMU 数据)
  - TF 广播 odom -> base_link
  - 定时器: 发送循环、接收循环、状态输出

================================================================================
定时器设计

节点有三个定时器, 各自以不同的频率运行:

1. transmit_timer (50Hz):  周期发送速度指令
   - 如果串口未连接, 尝试重连
   - 如果指令超时 (超过 cmd_timeout 秒没有新指令), 发送零速度 (停车)
   - 安全保护: 不发送指令比发送错误指令更危险, 所以超时后持续发送停车帧

2. receive_timer (200Hz):  非阻塞读取串口数据
   - 高频轮询确保串口缓冲区中的数据及时被读取
   - 非阻塞读取 + 流式解析, 不假设每次读取恰好对应一帧

3. status_timer (1Hz):  周期输出运行状态日志
   - 汇总串口状态、指令状态、底盘速度和 IMU 数据

================================================================================
数据流向

  /cmd_vel (Twist)
      │
      ▼
  _command_callback() -> target_command
      │
      ▼ (50Hz)
  _transmit_loop() -> encode_velocity_command() -> SerialTransport.write()
      │
      ▼ USB Serial
  ═══════════════ STM32 底盘 ═══════════════
      │
      ▼ USB Serial
  SerialTransport.read_available()
      │
      ▼ (200Hz)
  _receive_loop() -> FeedbackStreamParser.feed()
      │
      ▼
  _handle_feedback() ──> ImuConverter.convert() ──> /imu/data_raw
      │
      └──────────────> OdometryIntegrator.update() ──> /odom + TF

"""

import math
import time
from dataclasses import dataclass
from typing import Optional

# TransformStamped: TF 坐标变换消息 (带有父子坐标系和时间戳的平移+旋转)
from geometry_msgs.msg import TransformStamped, Twist
# Twist: 速度指令消息 (linear.x/y/z 和 angular.x/y/z)
# Odometry: 里程计消息 (包含位姿和速度, 每个都有协方差矩阵)
from nav_msgs.msg import Odometry
from rclpy.node import Node
# Imu: IMU 惯性测量单元消息 (加速度+角速度+姿态, 各有协方差)
from sensor_msgs.msg import Imu
# SerialException: pyserial 串口异常基类 (通信错误)
from serial import SerialException
# TransformBroadcaster: TF 坐标变换发布器 (负责发布 odom->base_link 变换)
from tf2_ros import TransformBroadcaster

# =========================================================================
# 内部模块导入
# =========================================================================
# ImuConverter: IMU 原始数据 -> 国际单位制转换器
from gasrobot_base.imu import ImuConverter

# 数据模型: 各模块间传递的数据结构
from gasrobot_base.models import (
    ImuSample,         # 一份 IMU 数据样本 (已转换单位)
    OdometrySample,    # 一次里程计更新的完整状态
    RawFeedback,       # 从串口协议解析出的原始底盘反馈
    VelocityCommand,   # 三轴速度指令
)

# OdometryIntegrator: 里程计积分器 (位置积分+航向管理)
from gasrobot_base.odometry import OdometryIntegrator

# BridgeConfig: 节点配置 (参数读取+校验)
from gasrobot_base.parameters import BridgeConfig

# 协议模块: 控制帧编码、反馈帧解码、流式解析
from gasrobot_base.protocol import (
    COMMAND_FRAME_SIZE,       # 控制帧长度: 9 字节
    FeedbackStreamParser,     # 流式帧解析器
    encode_velocity_command,  # 速度指令 -> 二进制帧编码
)

# SerialTransport: 串口通信封装 (打开/关闭/读写)
from gasrobot_base.serial_transport import SerialTransport


# =========================================================================
# _BridgeStatistics: 通信统计计数器
# =========================================================================
@dataclass
class _BridgeStatistics:
    """
    记录串口收发和故障次数, 用于周期状态诊断.

    这是一个普通的 (非 frozen) 数据类, 因为计数器需要被修改.
    前缀 _ 表示这是模块内部使用的类, 不应被外部直接引用.

    """

    # transmitted: 累计发送的帧数
    # 每成功发送一帧速度指令 +1
    transmitted: int = 0

    # received: 累计接收的有效帧数
    # 每成功解码一帧反馈 +1 (坏帧不计入)
    received: int = 0

    # serial_errors: 累计串口异常次数
    # 包括: 连接失败、写入失败、读取异常等
    serial_errors: int = 0


# =========================================================================
# STM32BridgeNode: 核心编排节点
# =========================================================================
class STM32BridgeNode(Node):
    """
    连接 ROS 2 话题与 STM32 底盘串口的编排节点.

    这是整个 gasrobot_base 包的核心类. 它:
    - 继承自 rclpy.node.Node (获得 ROS 2 节点的全部能力)
    - 管理串口连接生命周期 (打开/重连/关闭)
    - 管理通信协议 (编码指令/解码反馈)
    - 管理数据流 (指令->底盘->反馈->里程计+IMU+TF)

    """

    def __init__(self) -> None:
        """
        初始化配置、功能组件、ROS 接口和周期任务.

        初始化三阶段:
        1. 第一层: 创建非 ROS 组件 (配置、串口、协议、算法)
        2. 第二层: 初始化运行状态变量
        3. 第三层: 创建 ROS 接口 (话题、TF、定时器)
        最后: 尝试打开串口

        """
        super().__init__("stm32_bridge")

        # ================================================================
        # 第一层: 读取参数并构造与 ROS 无关的协议、串口和算法组件
        # ================================================================
        # BridgeConfig.from_node(self):
        #   在 self (本节点) 上声明所有参数, 读取值, 并校验
        #   返回一个不可变的 BridgeConfig 对象
        self.config = BridgeConfig.from_node(self)

        # SerialTransport: 串口通信传输层
        # 不依赖 ROS, 只依赖 pyserial
        # 这里只保存参数, 不立即连接硬件
        self.transport = SerialTransport(
            port=self.config.port,
            baud=self.config.baud,
            startup_delay=self.config.startup_delay,
        )

        # FeedbackStreamParser: 流式帧解析器
        # 从连续的字节流中提取完整的反馈帧
        self.parser = FeedbackStreamParser()

        # ImuConverter: IMU 数据转换器
        # 原始 LSB 计数 -> 国际单位制 (m/s^2 和 rad/s)
        self.imu_converter = ImuConverter(
            acceleration_lsb_per_g=self.config.accel_lsb_per_g,
            gyroscope_lsb_per_dps=self.config.gyro_lsb_per_dps,
            z_sign=self.config.imu_z_sign,
            z_offset_radps=self.config.gyro_z_offset_radps,
            z_deadband=self.config.gyro_z_deadband,
        )

        # OdometryIntegrator: 里程计积分器
        # use_imu_wz_for_twist 选择角速度来源
        self.odometry_integrator = OdometryIntegrator(
            use_imu_angular_velocity=self.config.use_imu_wz_for_twist,
        )

        # ================================================================
        # 第二层: 保存指令、反馈和通信时刻, 供超时保护与状态输出使用
        # ================================================================
        # target_command: 最新的目标速度指令
        # 初始为零速度 (停车状态)
        self.target_command = VelocityCommand()

        # effective_command: 发送循环经过安全门控后实际选用的指令。
        # 它与 target_command 分开保存，避免状态日志在看门狗停车后仍把
        # 上游最后一次非零请求误显示成正在生效的速度。
        self.effective_command = VelocityCommand()

        # last_command_time: 上次收到 /cmd_vel 消息的时间
        # 初始为 None (还没收到过指令)
        # 用于指令看门狗: 超过 cmd_timeout 后自动停车
        self.last_command_time: Optional[float] = None

        # last_feedback_time: 上次收到有效反馈帧的时间
        # 用于状态显示和反馈看门狗安全门控
        self.last_feedback_time: Optional[float] = None

        # last_reconnect_attempt: 上次尝试重连的时间
        # 用于控制重连频率 (不超过 reconnect_period)
        self.last_reconnect_attempt = 0.0

        # latest_feedback: 最新的有效反馈帧
        # None 表示还没有收到过反馈
        self.latest_feedback: Optional[RawFeedback] = None

        # latest_imu: 最新的 IMU 样本
        # None 表示还没有 IMU 数据
        self.latest_imu: Optional[ImuSample] = None

        # latest_odometry: 最新的里程计状态
        # 初始化为全零 + "NONE" 来源标记
        self.latest_odometry = OdometrySample(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_x=0.0,
            linear_y=0.0,
            angular_z=0.0,
            yaw_source="NONE",  # 初始状态: 没有数据源
        )

        # statistics: 通信统计计数器
        self.statistics = _BridgeStatistics()

        # ================================================================
        # 第三层: 建立 ROS 话题、TF 广播器以及不同职责的定时器
        # ================================================================
        # --- 订阅者 (Subscription) ---
        # 订阅 /cmd_vel 话题, 类型为 Twist (速度指令)
        # 收到消息时回调 self._command_callback
        # Queue Size = 20: 缓存最多 20 条未处理的消息
        self.command_subscription = self.create_subscription(
            Twist,
            self.config.cmd_vel_topic,
            self._command_callback,
            20,
        )

        # --- 发布者 (Publisher) ---
        # 发布 /odom 话题 (里程计)
        self.odometry_publisher = self.create_publisher(
            Odometry,
            self.config.odom_topic,
            20,  # Queue Size
        )

        # 发布 /imu/data_raw 话题 (IMU 数据)
        self.imu_publisher = self.create_publisher(
            Imu,
            self.config.imu_topic,
            20,
        )

        # --- TF 广播器 (Transform Broadcaster) ---
        # 如果配置要求发布 odom->base_link TF, 创建广播器
        # 如果配置不要求, 设为 None (不发布 TF)
        self.tf_broadcaster = (
            TransformBroadcaster(self)
            if self.config.publish_odom_tf
            else None
        )

        # --- 定时器 (Timer) ---
        # 1. 发送定时器: 按 tx_rate 频率发送速度指令
        #    1.0 / tx_rate 计算周期: 50Hz -> 0.02 秒
        self.transmit_timer = self.create_timer(
            1.0 / self.config.tx_rate,
            self._transmit_loop,
        )

        # 2. 接收定时器: 200Hz (0.005 秒) 高频轮询串口
        #    高频是为了尽量减少串口缓冲区中的数据积压
        self.receive_timer = self.create_timer(0.005, self._receive_loop)

        # 3. 状态定时器: 按 status_period 频率输出状态
        self.status_timer = self.create_timer(
            self.config.status_period,
            self._print_status,
        )

        # 启动信息
        self.get_logger().info(
            f"STM32 底盘桥接节点已启动: "
            f"{self.config.port} @ {self.config.baud}"
        )

        # --- 最后一步: 尝试打开串口 ---
        # 放在初始化的最后, 因为打开串口可能需要一点时间
        # 在此之前 ROS 接口已经创建好了, 即使串口暂时连不上也能工作
        self._open_serial()

    # =====================================================================
    # 串口连接管理
    # =====================================================================
    def _open_serial(self) -> bool:
        """
        连接串口并发送启动停车帧, 成功时返回 True.

        连接成功后:
        1. 清空协议解析器的缓冲区 (丢弃断线前残留数据)
        2. 重置里程计的时间戳 (断线期间的间隔不积分)
        3. 连续发送多帧零速度指令 (清除下位机可能保留的旧指令)

        连接失败时:
        1. 增加串口错误计数
        2. 关闭串口 (释放资源)
        3. 记录错误日志
        4. 返回 False (节点继续运行, 后续自动重连)

        返回:
            True: 串口连接成功并发送了停车帧
            False: 连接失败

        """
        # 记录本次尝试的时间 (防止高频重连)
        self.last_reconnect_attempt = time.monotonic()

        try:
            # 步骤 1: 建立串口连接
            self.transport.connect()

            # 步骤 2: 清理状态
            self.parser.clear()                 # 清空协议缓冲区
            self.odometry_integrator.reset_time()  # 重置里程计时间戳
            # 新连接必须先收到本次连接上的有效反馈，才能解除零速门控。
            self.last_feedback_time = None
            self.effective_command = VelocityCommand()

            self.get_logger().info(
                f"串口已连接: {self.config.port} @ {self.config.baud}"
            )

            # 步骤 3: 发送多帧停车指令
            # 为什么要发送多帧?
            # - 上位机重启期间, 下位机可能保留了最后收到的运动指令
            # - 如果直接开始发送新指令, 下位机可能"惯性"执行旧指令
            # - 连续多帧零速度确保下位机明确进入停车状态
            for _ in range(self.config.startup_stop_frames):
                # VelocityCommand() 无参数 -> 所有速度默认 0 (停车)
                if not self._write_velocity(VelocityCommand()):
                    return False  # 写入失败 -> 连接失败
                time.sleep(0.02)  # 帧间间隔 20ms, 让下位机有处理时间

            return True

        except (OSError, SerialException) as exc:
            # 连接失败:
            # - OSError: 设备文件不存在 (/dev/ttyUSB0 不存在)
            # - SerialException: 串口配置错误 (权限不足、波特率不支持等)
            self.statistics.serial_errors += 1
            self.transport.close()
            self.last_feedback_time = None
            self.effective_command = VelocityCommand()
            self.get_logger().error(
                f"无法打开串口 {self.config.port}: {exc}"
            )
            return False

    def _mark_serial_disconnected(self, exc: Exception) -> None:
        """
        统一记录通信异常, 并清理依赖连续数据的解析和积分状态.

        当串口读写过程中发生异常时调用 (不是连接阶段).
        这个时候串口已经打开了, 但通信中断了.

        清理操作:
        - 增加错误计数
        - 记录错误日志
        - 关闭串口 (释放文件描述符)
        - 清空协议缓冲区 (丢弃损坏的数据)
        - 重置里程计时间戳 (断线时段不积分)

        参数:
            exc: 导致断线的异常对象

        """
        self.statistics.serial_errors += 1
        self.get_logger().error(f"串口通信错误: {exc}")
        self.transport.close()
        self.parser.clear()
        self.odometry_integrator.reset_time()
        self.last_feedback_time = None
        self.effective_command = VelocityCommand()

    def _try_reconnect(self) -> None:
        """
        按配置周期尝试重连, 避免故障时高频打开串口.

        重连策略:
        - 如果已经连上了, 什么都不做
        - 如果距离上次重连尝试不到 reconnect_period 秒, 跳过
        - 否则, 调用 _open_serial() 尝试连接

        这样确保在串口持续不可用时, 不会疯狂尝试打开 (浪费资源).
        """
        if self.transport.is_open:
            return  # 已连接, 不需要重连

        now = time.monotonic()
        # 检查距离上次尝试的时间是否超过重连间隔
        if now - self.last_reconnect_attempt >= self.config.reconnect_period:
            self.last_reconnect_attempt = now
            self._open_serial()

    # =====================================================================
    # 安全过滤
    # =====================================================================
    @staticmethod
    def _finite_or_zero(value: float) -> float:
        """
        过滤 ROS 消息中可能存在的 NaN (非数字) 和 Infinity (无穷大) 值.

        为什么需要过滤?
        ROS 2 消息中的 float 字段可能包含特殊值:
        - NaN (Not a Number): 未初始化的值, 或 0/0 的结果
        - +Infinity: 除以零等数学异常
        - -Infinity: 同上

        这些值如果被编码发送到底盘:
        - NaN 在整数转换时可能变成任意值 -> 底盘失控
        - Infinity 被截断为 32767 或 -32768 -> 全速运动

        math.isfinite(x) 对 NaN, Inf, -Inf 都返回 False.

        参数:
            value: 待过滤的浮点数

        返回:
            有限的值原样返回, NaN/Inf 返回安全的 0.0

        """
        return value if math.isfinite(value) else 0.0

    # =====================================================================
    # /cmd_vel 订阅回调
    # =====================================================================
    def _command_callback(self, message: Twist) -> None:
        """
        缓存最新的速度指令, 并刷新指令看门狗的时间戳.

        这个回调在每次收到 /cmd_vel 消息时被调用 (由 ROS 执行器触发).
        注意: 这个回调不做任何阻塞操作! 只更新缓存值和时间戳.

        参数:
            message: Twist 消息, 包含 linear.x/y/z 和 angular.x/y/z

        """
        # 从 Twist 消息中提取三轴速度, 同时过滤 NaN/Inf
        self.target_command = VelocityCommand(
            linear_x=self._finite_or_zero(float(message.linear.x)),
            linear_y=self._finite_or_zero(float(message.linear.y)),
            angular_z=self._finite_or_zero(float(message.angular.z)),
        )

        # 刷新指令时间戳, 重置看门狗
        # time.monotonic() 是单调时钟 (不受系统时间调整影响)
        self.last_command_time = time.monotonic()

    # =====================================================================
    # 速度指令写入
    # =====================================================================
    def _write_velocity(self, command: VelocityCommand) -> bool:
        """
        编码并发送一帧速度指令, 发送失败时切换为断线状态.

        流程:
        1. 检查串口是否打开
        2. 用协议编码器把 VelocityCommand 编码为 9 字节二进制帧
        3. 通过串口写入帧
        4. 检查写入字节数是否正确 (应为 9 字节)
        5. 更新统计计数

        参数:
            command: 要发送的速度指令 (包含三轴速度值)

        返回:
            True: 发送成功
            False: 发送失败 (串口未打开或写入异常)

        """
        # 串口未打开时直接返回失败
        if not self.transport.is_open:
            return False

        # 步骤 1: 编码速度指令
        # encode_velocity_command 会做限幅和单位转换:
        #   m/s -> mm/s, rad/s -> mrad/s, 旋转方向取反
        frame = encode_velocity_command(
            command,
            self.config.velocity_limits,  # 速度限幅值
        )

        try:
            # 步骤 2: 写入串口
            # transport.write() 返回实际写入的字节数
            written = self.transport.write(frame)

            # 步骤 3: 检查是否完整写入
            if written != COMMAND_FRAME_SIZE:
                # 写入不完整 -> 串口可能处于异常状态
                raise SerialException(
                    f"仅写入 {written}/{COMMAND_FRAME_SIZE} 字节"
                )

            # 步骤 4: 更新统计
            self.statistics.transmitted += 1

            # 调试模式: 打印每帧发送的数据
            if self.config.debug_tx:
                # frame.hex(' ') 把二进制帧转为可读的十六进制字符串
                # 例如: "7b 05 dc ff 06 fd 14 37 7d"
                # {variable:+.3f}: 格式化浮点数, 始终显示符号, 3 位小数
                self.get_logger().info(
                    f"发送 {frame.hex(' ')} | "
                    f"vx={command.linear_x:+.3f}, "
                    f"vy={command.linear_y:+.3f}, "
                    f"wz={command.angular_z:+.3f}"
                )

            return True

        except (OSError, SerialException) as exc:
            # 写入过程中任何异常都视为断线
            # _mark_serial_disconnected 会关闭串口并清理状态
            self._mark_serial_disconnected(exc)
            return False

    # =====================================================================
    # 发送循环 (50Hz 定时器回调)
    # =====================================================================
    def _transmit_loop(self) -> None:
        """
        周期发送有效的速度指令; 指令超时后自动发送零速度指令 (停车).

        这个回调以 tx_rate (默认 50Hz) 的频率被 ROS 执行器调用.

        安全机制:
        1. 指令看门狗 (Command Watchdog)
        - 如果超过 cmd_timeout (默认 0.3 秒) 没有收到新的 /cmd_vel,
        - 自动发送零速度指令 (而不是停止发送)
        - 持续发送停车帧让下位机明确知道应该保持静止

        2. 反馈看门狗 (Feedback Watchdog)
        - 在收到第一帧有效反馈前不允许发送非零速度;
        - 有效反馈超过 feedback_timeout 未更新时强制发送零速度;
        - 防止串口仍可写、但底盘状态反馈已经失效时继续开环运动。

        为什么 "持续发送停车帧" 比 "停止发送" 更好?
        - 停止发送 -> 下位机不知道上位机是 crash 了还是故意不动
        - 持续发送停车帧 -> 下位机明确收到"保持静止"的指令
        - 下位机也可能有自己的看门狗: 如果一段时间没收到任何帧, 自动停车

        """
        # 串口未打开 -> 尝试重连
        if not self.transport.is_open:
            self._try_reconnect()
            return

        now = time.monotonic()

        # 判断指令是否超时:
        # - 从未收到过指令 (last_command_time is None)
        # - 距离上次收到指令超过 cmd_timeout 秒
        command_expired = (
            self.last_command_time is None
            or now - self.last_command_time > self.config.cmd_timeout
        )

        # 反馈丢失时即使上游仍在连续发布 /cmd_vel，也不允许继续发送非零
        # 指令。反馈恢复后，新的且未超时的速度请求会自动恢复生效。
        feedback_expired = (
            self.last_feedback_time is None
            or now - self.last_feedback_time > self.config.feedback_timeout
        )

        # 根据超时状态选择要发送的指令:
        # - 超时: 发送零速度指令 (停车)
        # - 未超时: 发送最新的目标指令
        command = (
            VelocityCommand()
            if command_expired or feedback_expired
            else self.target_command
        )
        self.effective_command = command

        # 发送指令
        self._write_velocity(command)

    # =====================================================================
    # 接收循环 (200Hz 定时器回调)
    # =====================================================================
    def _receive_loop(self) -> None:
        """
        非阻塞读取串口, 并处理本轮拆出的全部完整反馈帧.

        这个回调以 200Hz 的频率被 ROS 执行器调用.
        高频轮询确保串口缓冲区中的数据及时被读取和解析.

        处理流程:
        1. 非阻塞读取串口缓冲区中的所有数据
        2. 将数据喂入流式解析器 (FeedbackStreamParser)
        3. 解析器返回本次拆出的所有完整反馈帧
        4. 对每个完整帧调用 _handle_feedback() 处理

        """
        # 串口未打开时跳过
        if not self.transport.is_open:
            return

        try:
            # 步骤 1: 非阻塞读取
            # read_available() 不会阻塞线程, 缓冲区为空时返回 b""
            data = self.transport.read_available()

            # 步骤 2-4: 喂入解析器并处理所有完整帧
            # parser.feed(data) 返回一个列表: 可能包含 0 个、1 个或多个帧
            for feedback in self.parser.feed(data):
                self._handle_feedback(feedback)

        except (OSError, SerialException) as exc:
            # 读取过程中任何异常都视为断线
            self._mark_serial_disconnected(exc)

    # =====================================================================
    # 反馈帧处理
    # =====================================================================
    def _handle_feedback(self, feedback: RawFeedback) -> None:
        """
        转换一帧底盘反馈并同步发布 IMU、里程计和 TF.

        每收到一帧完整的反馈帧, 这个函数:
        1. 缓存原始反馈数据
        2. 转换 IMU 数据 (LSB -> 国际单位制)
        3. 更新里程计 (积分平移 + 更新航向)
        4. 发布 IMU 消息
        5. 发布里程计消息 + TF 变换

        关键设计: 同一帧派生的 IMU、里程计和 TF 共用一个 ROS 时间戳.
        这确保了数据的一致性 -- 导航栈不会看到"里程计是新帧但 IMU 是旧帧"
        的情况.

        参数:
            feedback: 解码后的原始反馈数据

        """
        # 缓存最新反馈数据 (用于状态日志)
        self.latest_feedback = feedback

        # 转换 IMU 数据: LSB 计数 -> 国际单位制
        self.latest_imu = self.imu_converter.convert(feedback)

        # 更新统计计数
        self.statistics.received += 1
        self.last_feedback_time = time.monotonic()

        # 获取当前 ROS 时间戳
        # self.get_clock().now() 返回 rclpy.time.Time 对象
        stamp = self.get_clock().now()

        # 更新里程计
        # stamp.nanoseconds / 1e9: 把 ROS 时间戳转换为 float 秒 (UNIX 风格)
        self.latest_odometry = self.odometry_integrator.update(
            feedback=feedback,
            imu_angular_z=self.latest_imu.yaw_rate,  # 修正后的 IMU Z 轴角速度
            stamp=stamp.nanoseconds / 1e9,
        )

        # 发布 IMU 和里程计
        self._publish_imu(stamp, self.latest_imu)
        self._publish_odometry(stamp, self.latest_odometry)

        # 调试模式: 打印每帧收到的数据
        if self.config.debug_rx:
            self.get_logger().info(
                f"收到反馈 | vx={feedback.linear_x:+.3f}, "
                f"vy={feedback.linear_y:+.3f}, "
                f"wz={feedback.angular_z:+.3f}, "
                f"yaw={math.degrees(feedback.yaw):+.2f} deg, "
                f"imu_wz={self.latest_imu.yaw_rate:+.4f}"
            )

    # =====================================================================
    # IMU 消息发布
    # =====================================================================
    def _publish_imu(self, stamp, sample: ImuSample) -> None:
        """
        发布符合 ROS sensor_msgs/Imu 规范的国际单位制 IMU 数据.

        sensor_msgs/Imu 消息结构:
        - header: 时间戳 + 坐标系
        - orientation: 姿态四元数 (我们不提供, 设为未知)
        - angular_velocity: 三轴角速度 (rad/s)
        - linear_acceleration: 三轴线加速度 (m/s^2)
        - 每个量都有对应的 3x3 协方差矩阵 (9 个 float)

        参数:
            stamp:  ROS 时间戳对象
            sample: 已转换单位的 IMU 样本

        """
        # 创建 Imu 消息对象
        message = Imu()

        # 设置消息头
        # stamp.to_msg(): 把 rclpy Time 对象转为 ROS 消息格式
        message.header.stamp = stamp.to_msg()
        # frame_id: 标注数据所在的坐标系 (如 "imu_link")
        message.header.frame_id = self.config.imu_frame

        # 姿态 (orientation): 没有完整姿态数据
        # covariance[0] = -1 是 ROS 约定: 表示该数据不可用
        message.orientation_covariance[0] = -1.0

        # 线加速度 (三轴)
        # 从 ImuSample 的 acceleration 元组中提取
        # 元组索引: [0]=X, [1]=Y, [2]=Z
        message.linear_acceleration.x = sample.acceleration[0]
        message.linear_acceleration.y = sample.acceleration[1]
        message.linear_acceleration.z = sample.acceleration[2]

        # 角速度 (三轴)
        # gyroscope 是原始三轴角速度, yaw_rate 是修正后的 Z 轴角速度
        message.angular_velocity.x = sample.gyroscope[0]
        message.angular_velocity.y = sample.gyroscope[1]
        # 使用修正后的 Z 轴角速度 (已经过方向修正、零偏补偿和死区过滤)
        message.angular_velocity.z = sample.yaw_rate

        # 协方差矩阵: 3x3 矩阵, 按行主序存储在一维数组中
        # 索引: 0 1 2
        #       3 4 5
        #       6 7 8
        # 对角元素 ([0], [4], [8]) 表示各轴的方差
        # 非对角元素为 0 (假设各轴独立)

        # 角速度协方差
        message.angular_velocity_covariance[0] = 0.02   # X 轴方差
        message.angular_velocity_covariance[4] = 0.02   # Y 轴方差
        message.angular_velocity_covariance[8] = 0.01   # Z 轴方差 (更小, 因为经过了修正)

        # 线加速度协方差
        message.linear_acceleration_covariance[0] = 0.10  # X 轴方差
        message.linear_acceleration_covariance[4] = 0.10  # Y 轴方差
        message.linear_acceleration_covariance[8] = 0.10  # Z 轴方差

        # 发布消息
        self.imu_publisher.publish(message)

    # =====================================================================
    # 里程计消息发布 + TF 广播
    # =====================================================================
    def _publish_odometry(
        self,
        stamp,
        sample: OdometrySample,
    ) -> None:
        """
        发布二维里程计 (nav_msgs/Odometry) 消息, 并按配置广播 TF 变换.

        nav_msgs/Odometry 消息结构:
        - header: 时间戳 + 参考坐标系
        - child_frame_id: 子坐标系 (通常是 "base_link")
        - pose: 子坐标系在参考坐标系中的位姿 (位置 + 四元数)
        - twist: 子坐标系在参考坐标系中的速度 (线速度 + 角速度)
        - 位姿和速度各有 6x6 协方差矩阵

        TF 变换说明:
        - odom -> base_link: 里程计坐标系 -> 机器人本体坐标系
        - 导航栈依赖这个 TF 来知道机器人的当前位置

        参数:
            stamp:  ROS 时间戳对象
            sample: 里程计状态

        """
        # ----- 偏航角 -> 四元数 -----
        # 二维平面只需要 Z 和 W 两个分量
        # 公式: z = sin(yaw/2), w = cos(yaw/2)
        # X=0, Y=0 (绕 Z 轴旋转, X 和 Y 分量为 0)
        half_yaw = sample.yaw * 0.5
        quaternion_z = math.sin(half_yaw)
        quaternion_w = math.cos(half_yaw)

        # ----- 构造 Odometry 消息 -----
        message = Odometry()

        # 消息头
        message.header.stamp = stamp.to_msg()
        # frame_id: 参考坐标系 (odom 坐标系)
        message.header.frame_id = self.config.odom_frame
        # child_frame_id: 被描述的坐标系 (机器人本体坐标系)
        message.child_frame_id = self.config.base_frame

        # 位姿 (Pose): 位置 + 姿态
        message.pose.pose.position.x = sample.x
        message.pose.pose.position.y = sample.y
        # position.z 默认为 0 (二维平面)
        message.pose.pose.orientation.z = quaternion_z
        message.pose.pose.orientation.w = quaternion_w
        # orientation.x 和 .y 默认为 0 (绕 Z 轴旋转)

        # 速度 (Twist): 线速度 + 角速度
        message.twist.twist.linear.x = sample.linear_x
        message.twist.twist.linear.y = sample.linear_y
        # linear.z 默认为 0 (二维平面, 没有垂直方向速度)
        message.twist.twist.angular.z = sample.angular_z
        # angular.x 和 .y 默认为 0 (只绕 Z 轴旋转)

        # ----- 协方差矩阵 -----
        # 6x6 协方差矩阵, 按行主序存储在一维数组 (36 个 float) 中
        # 对角线索引: 0, 7, 14, 21, 28, 35
        # 前 3 个对角元素对应位姿 (x, y, z)
        # 后 3 个对角元素对应姿态 (roll, pitch, yaw)

        # 位姿协方差
        message.pose.covariance[0] = 0.03   # X 坐标的方差 (m^2)
        message.pose.covariance[7] = 0.03   # Y 坐标的方差
        message.pose.covariance[14] = 1e6   # Z 坐标的方差 -> 极大 = 不可知 (二维不观测)
        message.pose.covariance[21] = 1e6   # Roll 方差 -> 不观测
        message.pose.covariance[28] = 1e6   # Pitch 方差 -> 不观测
        message.pose.covariance[35] = 0.08  # Yaw 的方差 (rad^2)

        # 速度协方差
        message.twist.covariance[0] = 0.04   # X 线速度方差
        message.twist.covariance[7] = 0.04   # Y 线速度方差
        message.twist.covariance[14] = 1e6   # Z 线速度方差 -> 不观测
        message.twist.covariance[21] = 1e6   # Roll 角速度方差 -> 不观测
        message.twist.covariance[28] = 1e6   # Pitch 角速度方差 -> 不观测
        message.twist.covariance[35] = 0.10  # Yaw 角速度方差 (rad^2/s^2)

        # 发布里程计消息
        self.odometry_publisher.publish(message)

        # ----- TF 广播 (可选) -----
        # 如果配置了 publish_odom_tf, 同时发布 TF 变换
        if self.tf_broadcaster is not None:
            # TF 消息与 /odom 使用相同的位姿和时间戳
            # 这很重要: 如果 TF 和 /odom 的时间戳不一致,
            # 导航栈可能会出现瞬时定位错位
            transform = TransformStamped()
            transform.header.stamp = stamp.to_msg()
            transform.header.frame_id = self.config.odom_frame   # 父坐标系
            transform.child_frame_id = self.config.base_frame    # 子坐标系
            transform.transform.translation.x = sample.x          # X 平移
            transform.transform.translation.y = sample.y          # Y 平移
            transform.transform.rotation.z = quaternion_z          # Z 旋转分量
            transform.transform.rotation.w = quaternion_w          # W 旋转分量

            # 发送 TF 变换
            self.tf_broadcaster.sendTransform(transform)

    # =====================================================================
    # 状态日志输出 (1Hz 定时器回调)
    # =====================================================================
    def _print_status(self) -> None:
        """
        周期汇总通信、指令、底盘、里程计和 IMU 运行状态.

        这个回调每秒触发一次, 打印综合性的状态信息, 便于:
        - 现场调试 (一眼看出所有关键数据)
        - 日志记录 (可以用 ros2 bag 或日志文件分析)
        - 问题诊断 (迅速发现串口断线、超时等问题)

        状态信息的格式设计为一行或多行, 便于 grep 和解析.

        """
        now = time.monotonic()

        # 串口状态
        serial_state = "已连接" if self.transport.is_open else "未连接"

        # 指令状态
        # _age_state 返回 (状态文字, 数据年龄) 例如 ("有效", "0.15s")
        command_state, command_age = self._age_state(
            now,
            self.last_command_time,
            self.config.cmd_timeout,
            "超时停车",
        )

        # 反馈状态
        feedback_state, feedback_age = self._age_state(
            now,
            self.last_feedback_time,
            self.config.feedback_timeout,
            "超时",
        )

        # 缓存最新数据引用
        feedback = self.latest_feedback
        imu = self.latest_imu
        odometry = self.latest_odometry

        # 组装状态字符串
        # f-string 中使用 :+.3f 格式化: 正数显示 +, 3 位小数
        status = (
            f"串口={serial_state}  指令={command_state}/{command_age}  "
            f"反馈={feedback_state}/{feedback_age}\n"
            f"  请求: vx={self.target_command.linear_x:+.3f} m/s  "
            f"vy={self.target_command.linear_y:+.3f} m/s  "
            f"wz={self.target_command.angular_z:+.3f} rad/s\n"
            f"  生效: vx={self.effective_command.linear_x:+.3f} m/s  "
            f"vy={self.effective_command.linear_y:+.3f} m/s  "
            f"wz={self.effective_command.angular_z:+.3f} rad/s\n"
            f"  底盘: vx={self._value(feedback, 'linear_x'):+.3f} m/s  "
            f"vy={self._value(feedback, 'linear_y'):+.3f} m/s  "
            f"wz={self._value(feedback, 'angular_z'):+.3f} rad/s\n"
            f"  里程: 源={odometry.yaw_source}  x={odometry.x:+.3f} m  "
            f"y={odometry.y:+.3f} m  "
            f"yaw={math.degrees(odometry.yaw):+.2f} deg\n"
            f"  统计: 发送={self.statistics.transmitted}  "
            f"接收={self.statistics.received}  "
            f"坏帧={self.parser.bad_frame_count}  "
            f"串口错误={self.statistics.serial_errors}"
        )

        # 如果有 IMU 数据, 追加 IMU 状态
        if imu is not None:
            status += (
                f"\n  IMU: gx={imu.gyroscope[0]:+.4f}  "
                f"gy={imu.gyroscope[1]:+.4f}  "
                f"gz={imu.gyroscope[2]:+.4f} rad/s  "
                f"使用值={imu.yaw_rate:+.4f} rad/s"
            )

        # 如果配置要求打印 IMU 原始值, 追加原始数据
        if self.config.print_imu_raw and feedback is not None:
            status += (
                f"\n  原始值: 加速度={feedback.acceleration_raw}  "
                f"陀螺仪={feedback.gyroscope_raw}"
            )

        self.get_logger().info(status)

    @staticmethod
    def _age_state(
        now: float,
        last_time: Optional[float],
        timeout: float,
        timeout_label: str,
    ):
        """
        根据最近更新时间返回 (状态文字, 数据年龄) 元组.

        这是一个通用的状态判断函数, 同时用于指令和反馈.

        参数:
            now:           当前时间 (time.monotonic())
            last_time:     上次更新时间 (可能为 None)
            timeout:       超时阈值 (秒)
            timeout_label: 超时时的状态描述文字

        返回:
            (status: str, age: str) 元组
            例如 ("有效", "0.25s") 或 ("超时", "1.50s")

        """
        # 从未收到过数据
        if last_time is None:
            return "未收到", "无"

        # 计算数据年龄 (从上次更新到现在的时间)
        age = now - last_time

        # 判断是否超时
        state = timeout_label if age > timeout else "有效"

        return state, f"{age:.2f}s"

    @staticmethod
    def _value(item, name: str) -> float:
        """
        安全读取可选反馈对象中的数值字段.

        如果反馈对象为 None (还没收到过数据), 返回 0.0.
        这避免了状态日志中因访问 None 属性而崩溃.

        参数:
            item: 可能为 None 的数据对象
            name: 属性名字符串

        返回:
            属性值 (如果 item 不为 None), 否则 0.0

        """
        # getattr(item, name): 相当于 item.name, 但 name 可以是变量
        return float(getattr(item, name)) if item is not None else 0.0

    # =====================================================================
    # 安全退出
    # =====================================================================
    def stop_and_close(self) -> None:
        """
        退出前发送多帧停止指令并关闭串口.

        这个方法在节点被销毁前调用 (由 stm32_bridge.py 的 main 函数).
        确保:
        1. STM32 底盘收到明确的停车指令
        2. 串口资源被正确释放

        即使串口已经断开, 也会尽力关闭 (避免资源泄漏).

        """
        try:
            # 如果串口仍然打开, 发送多帧停车指令
            if self.transport.is_open:
                for _ in range(5):
                    # 发送零速度指令
                    if not self._write_velocity(VelocityCommand()):
                        break  # 写入失败 -> 停止发送
                    time.sleep(0.02)  # 帧间间隔
        finally:
            # 无论发送是否成功, 都关闭串口
            self.transport.close()
