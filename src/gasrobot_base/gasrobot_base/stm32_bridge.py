#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 与 STM32 麦克纳姆底盘串口桥接节点。

============================================================
一、ROS 2 -> STM32 控制帧，共 9 字节
============================================================

    byte[0]      : 0x7B，帧头
    byte[1:3]    : Vx，有符号 int16，大端，单位 mm/s
    byte[3:5]    : Vy，有符号 int16，大端，单位 mm/s
    byte[5:7]    : Wz，有符号 int16，大端，单位 mrad/s
    byte[7]      : byte[1] ~ byte[6] 累加和的低 8 位
    byte[8]      : 0x7D，帧尾

ROS 坐标约定：

    linear.x  > 0：向前
    linear.y  > 0：向左
    angular.z > 0：从上往下看逆时针旋转

下位机当前在调用 Drive_Motor() 前对 Move_Y 取反，因此上位机不要再次
对 linear.y 取反，否则会造成两次反向。

============================================================
二、STM32 -> ROS 2 反馈帧，共 21 字节
============================================================

    byte[0]      : 0x7B，帧头
    byte[1:3]    : Vx，有符号 int16，大端，单位 mm/s
    byte[3:5]    : Vy，有符号 int16，大端，单位 mm/s
    byte[5:7]    : Wz，有符号 int16，大端，单位 mrad/s
    byte[7:9]    : 加速度计 X，int16
    byte[9:11]   : 加速度计 Y，int16
    byte[11:13]  : 加速度计 Z，int16
    byte[13:15]  : 陀螺仪 X，int16
    byte[15:17]  : 陀螺仪 Y，int16
    byte[17:19]  : 陀螺仪 Z，int16
    byte[19]     : byte[1] ~ byte[18] 累加和的低 8 位
    byte[20]     : 0x7D，帧尾

============================================================
三、IMU 与 yaw 说明
============================================================

下位机已经在上电时调用 icm20602_gyro_calibrate() 计算陀螺仪零偏。

推荐下位机发送“已经减去零偏、但单位仍为原始 LSB 的数据”。

本节点不会再次做动态零偏标定，避免重复减零偏。这里只做：

    1. LSB -> deg/s
    2. deg/s -> rad/s
    3. 可选固定补偿 gyro_z_offset_radps
    4. 小角速度死区过滤
    5. 对 Z 轴角速度积分得到 yaw

============================================================
四、节点功能
============================================================

订阅：
    /cmd_vel

发布：
    /odom
    /imu/data_raw
    odom -> base_link TF

安全功能：
    - /cmd_vel 超时自动发送零速度
    - 串口断开自动重连
    - 启动时发送多帧停止指令
    - 节点退出时发送多帧停止指令
    - 检查帧头、帧尾、长度和累加校验
    - 速度限幅
    - 过滤 NaN 和 Inf

调试打印：
    - ROS 目标速度
    - 编码器反馈速度
    - IMU 原始值
    - IMU Z 轴角速度
    - yaw 角度与弧度
    - odom x、y
    - 串口 TX、RX、坏帧和错误计数
"""

import math
import time
from typing import Optional

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from serial import SerialException
from tf2_ros import TransformBroadcaster


# ============================================================
# 串口协议常量
# ============================================================

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D

COMMAND_FRAME_SIZE = 9
FEEDBACK_FRAME_SIZE = 21

GRAVITY = 9.80665


# ============================================================
# 工具函数
# ============================================================

def clamp(value: float, lower: float, upper: float) -> float:
    """将 value 限制在 [lower, upper] 范围内。"""
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    """将角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def read_i16_be(data: bytes, offset: int) -> int:
    """从 data[offset:offset+2] 读取大端有符号 int16。"""
    return int.from_bytes(
        data[offset:offset + 2],
        byteorder="big",
        signed=True,
    )


def write_i16_be(
    buffer: bytearray,
    offset: int,
    value: int,
) -> None:
    """向 buffer[offset:offset+2] 写入大端有符号 int16。"""
    value = max(-32768, min(32767, int(value)))

    encoded = value.to_bytes(
        2,
        byteorder="big",
        signed=True,
    )

    buffer[offset] = encoded[0]
    buffer[offset + 1] = encoded[1]


# ============================================================
# ROS 2 节点
# ============================================================

class STM32Bridge(Node):
    """STM32 麦克纳姆底盘串口桥接节点。"""

    def __init__(self) -> None:
        super().__init__("stm32_bridge")

        # --------------------------------------------------------
        # 参数默认值
        # --------------------------------------------------------

        defaults = {
            # 串口
            "port": "/dev/ttyUSB0",
            "baud": 115200,
            "reconnect_period": 1.0,
            "startup_delay": 0.2,
            "startup_stop_frames": 5,

            # ROS 话题
            "cmd_vel_topic": "/cmd_vel",
            "odom_topic": "/odom",
            "imu_topic": "/imu/data_raw",

            # ROS 坐标系
            "odom_frame": "odom",
            "base_frame": "base_link",
            "imu_frame": "imu_link",

            # 是否由本节点发布 odom -> base_link
            "publish_odom_tf": True,

            # 控制发送和超时
            "tx_rate": 50.0,
            "cmd_timeout": 0.3,
            "feedback_timeout": 0.5,

            # ROS 指令限幅
            "max_linear_x": 0.5,
            "max_linear_y": 0.5,
            "max_angular_z": 1.2,

            # ICM20602 当前量程换算
            # ±8 g       -> 4096 LSB/g
            # ±250 deg/s -> 131 LSB/(deg/s)
            "accel_lsb_per_g": 4096.0,
            "gyro_lsb_per_dps": 131.0,

            # yaw 来源：
            # True  -> IMU Z 轴角速度
            # False -> 编码器正运动学得到的 Wz
            "use_imu_wz_for_odom": True,

            # IMU Z 轴方向修正：
            # ROS 要求逆时针为正。
            # 若实车逆时针转动时 yaw 减小，将其改为 -1.0。
            "imu_z_sign": 1.0,

            # 固定的 Z 轴角速度补偿，单位 rad/s。
            # 下位机已做零偏标定时保持 0.0。
            "gyro_z_offset_radps": 0.0,

            # 静止死区，单位 rad/s。
            # 仅过滤小抖动，不属于二次零偏标定。
            "gyro_z_deadband": 0.02,

            # 状态打印
            "status_period": 1.0,
            "print_imu_raw": True,

            # 每一帧十六进制打印，正常使用时保持 False
            "debug_tx": False,
            "debug_rx": False,
        }

        for name, default in defaults.items():
            self.declare_parameter(name, default)

        def get(name):
            return self.get_parameter(name).value

        # --------------------------------------------------------
        # 读取参数
        # --------------------------------------------------------

        self.port = str(get("port"))
        self.baud = int(get("baud"))
        self.reconnect_period = float(get("reconnect_period"))
        self.startup_delay = float(get("startup_delay"))
        self.startup_stop_frames = int(get("startup_stop_frames"))

        self.cmd_vel_topic = str(get("cmd_vel_topic"))
        self.odom_topic = str(get("odom_topic"))
        self.imu_topic = str(get("imu_topic"))

        self.odom_frame = str(get("odom_frame"))
        self.base_frame = str(get("base_frame"))
        self.imu_frame = str(get("imu_frame"))
        self.publish_odom_tf = bool(get("publish_odom_tf"))

        self.tx_rate = float(get("tx_rate"))
        self.cmd_timeout = float(get("cmd_timeout"))
        self.feedback_timeout = float(get("feedback_timeout"))

        self.max_linear_x = float(get("max_linear_x"))
        self.max_linear_y = float(get("max_linear_y"))
        self.max_angular_z = float(get("max_angular_z"))

        self.accel_lsb_per_g = float(get("accel_lsb_per_g"))
        self.gyro_lsb_per_dps = float(get("gyro_lsb_per_dps"))

        self.use_imu_wz_for_odom = bool(
            get("use_imu_wz_for_odom")
        )

        self.imu_z_sign = (
            -1.0
            if float(get("imu_z_sign")) < 0.0
            else 1.0
        )

        self.gyro_z_offset_radps = float(
            get("gyro_z_offset_radps")
        )

        self.gyro_z_deadband = float(
            get("gyro_z_deadband")
        )

        self.status_period = float(get("status_period"))
        self.print_imu_raw = bool(get("print_imu_raw"))
        self.debug_tx = bool(get("debug_tx"))
        self.debug_rx = bool(get("debug_rx"))

        self._validate_parameters()

        # --------------------------------------------------------
        # 串口状态
        # --------------------------------------------------------

        self.ser: Optional[serial.Serial] = None
        self.rx_buffer = bytearray()
        self.last_reconnect_attempt = 0.0

        # --------------------------------------------------------
        # ROS 目标速度
        # --------------------------------------------------------

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        self.last_cmd_time: Optional[float] = None

        # --------------------------------------------------------
        # STM32 底盘速度反馈
        # --------------------------------------------------------

        self.measured_vx = 0.0
        self.measured_vy = 0.0
        self.wheel_wz = 0.0

        # --------------------------------------------------------
        # IMU 数据
        # --------------------------------------------------------

        self.acc_raw = [0, 0, 0]
        self.gyro_raw = [0, 0, 0]

        self.acc_mps2 = [0.0, 0.0, 0.0]
        self.gyro_radps = [0.0, 0.0, 0.0]

        # 方向修正、固定补偿和死区前
        self.imu_wz_before_deadband = 0.0

        # 最终用于发布和 yaw 积分的角速度
        self.imu_wz = 0.0

        # --------------------------------------------------------
        # 里程计状态
        # --------------------------------------------------------

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.current_odom_wz = 0.0
        self.yaw_source = "NONE"

        self.last_feedback_time: Optional[float] = None
        self.last_feedback_stamp = None

        # --------------------------------------------------------
        # 串口统计
        # --------------------------------------------------------

        self.tx_count = 0
        self.rx_count = 0
        self.bad_frame_count = 0
        self.serial_error_count = 0

        # --------------------------------------------------------
        # ROS 订阅、发布、TF
        # --------------------------------------------------------

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            20,
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            20,
        )

        self.imu_pub = self.create_publisher(
            Imu,
            self.imu_topic,
            20,
        )

        self.tf_broadcaster: Optional[
            TransformBroadcaster
        ] = (
            TransformBroadcaster(self)
            if self.publish_odom_tf
            else None
        )

        # --------------------------------------------------------
        # 定时器
        # --------------------------------------------------------

        self.tx_timer = self.create_timer(
            1.0 / self.tx_rate,
            self.tx_loop,
        )

        self.rx_timer = self.create_timer(
            0.005,
            self.read_loop,
        )

        self.status_timer = self.create_timer(
            self.status_period,
            self.print_status,
        )

        # --------------------------------------------------------
        # 启动日志
        # --------------------------------------------------------

        self.get_logger().info(
            f"STM32 mecanum bridge started: "
            f"{self.port} @ {self.baud}"
        )

        self.get_logger().info(
            f"topics: cmd={self.cmd_vel_topic}, "
            f"odom={self.odom_topic}, "
            f"imu={self.imu_topic}"
        )

        self.get_logger().info(
            f"yaw source preference="
            f"{'IMU' if self.use_imu_wz_for_odom else 'WHEEL'}, "
            f"imu_z_sign={self.imu_z_sign:+.0f}, "
            f"gyro_z_offset={self.gyro_z_offset_radps:+.5f}rad/s, "
            f"deadband={self.gyro_z_deadband:.3f}rad/s"
        )

        self.open_serial()

    # ============================================================
    # 参数检查
    # ============================================================

    def _validate_parameters(self) -> None:
        """检查会导致节点无法正常工作的参数。"""

        if self.baud <= 0:
            raise ValueError("baud 必须大于 0")

        if self.reconnect_period <= 0.0:
            raise ValueError("reconnect_period 必须大于 0")

        if self.tx_rate <= 0.0:
            raise ValueError("tx_rate 必须大于 0")

        if self.cmd_timeout <= 0.0:
            raise ValueError("cmd_timeout 必须大于 0")

        if self.feedback_timeout <= 0.0:
            raise ValueError("feedback_timeout 必须大于 0")

        if self.max_linear_x <= 0.0:
            raise ValueError("max_linear_x 必须大于 0")

        if self.max_linear_y <= 0.0:
            raise ValueError("max_linear_y 必须大于 0")

        if self.max_angular_z <= 0.0:
            raise ValueError("max_angular_z 必须大于 0")

        if self.accel_lsb_per_g <= 0.0:
            raise ValueError("accel_lsb_per_g 必须大于 0")

        if self.gyro_lsb_per_dps <= 0.0:
            raise ValueError("gyro_lsb_per_dps 必须大于 0")

        if self.gyro_z_deadband < 0.0:
            raise ValueError("gyro_z_deadband 不能小于 0")

        if self.status_period <= 0.0:
            raise ValueError("status_period 必须大于 0")

    # ============================================================
    # 串口管理
    # ============================================================

    def serial_ready(self) -> bool:
        """判断串口对象是否存在并已打开。"""
        return (
            self.ser is not None
            and self.ser.is_open
        )

    def open_serial(self) -> bool:
        """打开串口并在启动后发送多帧停止指令。"""

        self.close_serial()

        try:
            ser = serial.Serial(
                port=None,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,
                write_timeout=0.1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )

            ser.port = self.port

            # 尽量避免部分 USB 转串口在 DTR/RTS 切换时复位设备。
            try:
                ser.dtr = False
                ser.rts = False
            except (OSError, SerialException):
                pass

            ser.open()

            try:
                ser.dtr = False
                ser.rts = False
            except (OSError, SerialException):
                pass

            self.ser = ser

            time.sleep(max(0.0, self.startup_delay))

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.rx_buffer.clear()

            # 串口重连后避免使用跨断线时间积分。
            self.last_feedback_stamp = None

            self.get_logger().info(
                f"Serial connected: "
                f"{self.port} @ {self.baud}"
            )

            for _ in range(
                max(1, self.startup_stop_frames)
            ):
                self.write_velocity(
                    0.0,
                    0.0,
                    0.0,
                )
                time.sleep(0.02)

            return True

        except (OSError, SerialException) as exc:
            self.ser = None
            self.serial_error_count += 1

            self.get_logger().error(
                f"Cannot open serial port "
                f"{self.port}: {exc}"
            )

            return False

    def close_serial(self) -> None:
        """关闭串口。"""

        if self.ser is None:
            return

        try:
            if self.ser.is_open:
                self.ser.close()

        except (OSError, SerialException):
            pass

        finally:
            self.ser = None

    def mark_serial_disconnected(
        self,
        exc: Exception,
    ) -> None:
        """记录串口错误并进入重连状态。"""

        self.serial_error_count += 1

        self.get_logger().error(
            f"Serial communication error: {exc}"
        )

        self.close_serial()
        self.rx_buffer.clear()
        self.last_feedback_stamp = None

    def try_reconnect(self) -> None:
        """按 reconnect_period 周期尝试重新连接。"""

        if self.serial_ready():
            return

        now = time.monotonic()

        if (
            now - self.last_reconnect_attempt
            >= self.reconnect_period
        ):
            self.last_reconnect_attempt = now
            self.open_serial()

    # ============================================================
    # ROS /cmd_vel -> STM32
    # ============================================================

    def cmd_callback(self, msg: Twist) -> None:
        """保存最新的 ROS 三轴速度指令。"""

        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)

        self.target_vx = (
            vx if math.isfinite(vx) else 0.0
        )

        self.target_vy = (
            vy if math.isfinite(vy) else 0.0
        )

        self.target_wz = (
            wz if math.isfinite(wz) else 0.0
        )

        self.last_cmd_time = time.monotonic()

    def build_command_frame(
        self,
        vx_mps: float,
        vy_mps: float,
        wz_radps: float,
    ) -> bytearray:
        """将 ROS 三轴速度构造成 9 字节 STM32 控制帧。"""

        vx_mps = clamp(
            vx_mps,
            -self.max_linear_x,
            self.max_linear_x,
        )

        vy_mps = clamp(
            vy_mps,
            -self.max_linear_y,
            self.max_linear_y,
        )

        wz_radps = clamp(
            wz_radps,
            -self.max_angular_z,
            self.max_angular_z,
        )

        # m/s -> mm/s
        vx_mmps = int(round(vx_mps * 1000.0))
        vy_mmps = int(round(vy_mps * 1000.0))

        # ROS 规定 angular.z > 0 为逆时针左转，
        # 但当前 STM32 运动学中 Wz > 0 表现为顺时针右转，
        # 因此发送给下位机前对旋转指令取反。
        wz_mradps = int(round(-wz_radps * 1000.0))
        frame = bytearray(COMMAND_FRAME_SIZE)

        frame[0] = FRAME_HEADER

        write_i16_be(frame, 1, vx_mmps)
        write_i16_be(frame, 3, vy_mmps)
        write_i16_be(frame, 5, wz_mradps)

        frame[7] = sum(frame[1:7]) & 0xFF
        frame[8] = FRAME_TAIL

        return frame

    def write_velocity(
        self,
        vx_mps: float,
        vy_mps: float,
        wz_radps: float,
    ) -> bool:
        """向 STM32 写入一帧速度指令。"""

        if not self.serial_ready():
            return False

        frame = self.build_command_frame(
            vx_mps,
            vy_mps,
            wz_radps,
        )

        try:
            written = self.ser.write(frame)

            if written != COMMAND_FRAME_SIZE:
                raise SerialException(
                    f"Only wrote {written}/"
                    f"{COMMAND_FRAME_SIZE} bytes"
                )

            self.tx_count += 1

            if self.debug_tx:
                self.get_logger().info(
                    f"TX {frame.hex(' ')} | "
                    f"vx={vx_mps:+.3f}, "
                    f"vy={vy_mps:+.3f}, "
                    f"wz={wz_radps:+.3f}"
                )

            return True

        except (OSError, SerialException) as exc:
            self.mark_serial_disconnected(exc)
            return False

    def tx_loop(self) -> None:
        """定时发送目标速度；指令超时则发送零速度。"""

        if not self.serial_ready():
            self.try_reconnect()
            return

        now = time.monotonic()

        command_expired = (
            self.last_cmd_time is None
            or now - self.last_cmd_time
            > self.cmd_timeout
        )

        if command_expired:
            self.write_velocity(
                0.0,
                0.0,
                0.0,
            )

        else:
            self.write_velocity(
                self.target_vx,
                self.target_vy,
                self.target_wz,
            )

    # ============================================================
    # STM32 反馈接收与协议解析
    # ============================================================

    def read_loop(self) -> None:
        """读取当前串口缓冲区中的全部数据。"""

        if not self.serial_ready():
            return

        try:
            waiting = self.ser.in_waiting

            if waiting <= 0:
                return

            data = self.ser.read(waiting)

            if data:
                self.rx_buffer.extend(data)
                self.parse_rx_buffer()

        except (OSError, SerialException) as exc:
            self.mark_serial_disconnected(exc)

    def parse_rx_buffer(self) -> None:
        """
        从接收缓冲区中搜索并解析完整的 21 字节反馈帧。

        遇到错误帧时只丢弃一个字节，继续搜索下一个帧头，
        避免一次错误导致后续全部错位。
        """

        header = bytes([FRAME_HEADER])

        while True:
            header_index = self.rx_buffer.find(header)

            if header_index < 0:
                if len(self.rx_buffer) > 2048:
                    self.rx_buffer.clear()

                return

            if header_index > 0:
                del self.rx_buffer[:header_index]
                self.bad_frame_count += 1

            if len(self.rx_buffer) < FEEDBACK_FRAME_SIZE:
                return

            frame = bytes(
                self.rx_buffer[:FEEDBACK_FRAME_SIZE]
            )

            if frame[20] != FRAME_TAIL:
                del self.rx_buffer[0]
                self.bad_frame_count += 1
                continue

            expected_checksum = (
                sum(frame[1:19]) & 0xFF
            )

            if frame[19] != expected_checksum:
                del self.rx_buffer[0]
                self.bad_frame_count += 1
                continue

            del self.rx_buffer[:FEEDBACK_FRAME_SIZE]
            self.handle_feedback(frame)

    def handle_feedback(self, frame: bytes) -> None:
        """解析一帧已经通过校验的 STM32 反馈。"""

        # --------------------------------------------------------
        # 底盘三轴速度
        # --------------------------------------------------------

        # mm/s -> m/s
        self.measured_vx = (
            read_i16_be(frame, 1) / 1000.0
        )

        self.measured_vy = (
            read_i16_be(frame, 3) / 1000.0
        )

        # mrad/s -> rad/s
        self.wheel_wz = (
            -read_i16_be(frame, 5) / 1000.0
        )

        # --------------------------------------------------------
        # ICM20602 原始数据
        # --------------------------------------------------------

        self.acc_raw = [
            read_i16_be(frame, 7),
            read_i16_be(frame, 9),
            read_i16_be(frame, 11),
        ]

        self.gyro_raw = [
            read_i16_be(frame, 13),
            read_i16_be(frame, 15),
            read_i16_be(frame, 17),
        ]

        # --------------------------------------------------------
        # 转换为 ROS 使用的 SI 单位
        # --------------------------------------------------------

        self.acc_mps2 = [
            raw
            / self.accel_lsb_per_g
            * GRAVITY
            for raw in self.acc_raw
        ]

        self.gyro_radps = [
            math.radians(
                raw / self.gyro_lsb_per_dps
            )
            for raw in self.gyro_raw
        ]

        # 下位机已经完成零偏标定，这里不做二次自动标定。
        # 仅进行方向修正、可选固定补偿和静止死区。
        self.imu_wz_before_deadband = (
            self.imu_z_sign
            * self.gyro_radps[2]
            - self.gyro_z_offset_radps
        )

        if (
            abs(self.imu_wz_before_deadband)
            < self.gyro_z_deadband
        ):
            self.imu_wz = 0.0

        else:
            self.imu_wz = (
                self.imu_wz_before_deadband
            )

        # --------------------------------------------------------
        # 发布 ROS 数据
        # --------------------------------------------------------

        self.rx_count += 1
        self.last_feedback_time = time.monotonic()

        stamp = self.get_clock().now()

        self.publish_imu(stamp)
        self.update_and_publish_odometry(stamp)

        if self.debug_rx:
            self.get_logger().info(
                f"RX {frame.hex(' ')} | "
                f"wheel=("
                f"{self.measured_vx:+.3f},"
                f"{self.measured_vy:+.3f},"
                f"{self.wheel_wz:+.3f}) | "
                f"imu_wz={self.imu_wz:+.4f}"
            )

    # ============================================================
    # IMU 发布
    # ============================================================

    def publish_imu(self, stamp) -> None:
        """发布 sensor_msgs/Imu。"""

        msg = Imu()

        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.imu_frame

        # 下位机反馈帧没有姿态四元数。
        # covariance[0] = -1 表示 orientation 不可用。
        msg.orientation_covariance[0] = -1.0

        msg.linear_acceleration.x = self.acc_mps2[0]
        msg.linear_acceleration.y = self.acc_mps2[1]
        msg.linear_acceleration.z = self.acc_mps2[2]

        msg.angular_velocity.x = self.gyro_radps[0]
        msg.angular_velocity.y = self.gyro_radps[1]
        msg.angular_velocity.z = self.imu_wz

        # 这些值需要后续根据实测噪声进一步标定。
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.01

        msg.linear_acceleration_covariance[0] = 0.10
        msg.linear_acceleration_covariance[4] = 0.10
        msg.linear_acceleration_covariance[8] = 0.10

        self.imu_pub.publish(msg)

    # ============================================================
    # 里程计与 yaw
    # ============================================================

    def update_and_publish_odometry(self, stamp) -> None:
        """
        根据底盘 Vx、Vy 和角速度积分二维里程计。

        x、y 使用 STM32 编码器正运动学反馈。
        yaw 默认使用 IMU Z 轴角速度积分。
        """

        dt = 0.0

        if self.last_feedback_stamp is not None:
            dt = (
                stamp - self.last_feedback_stamp
            ).nanoseconds / 1e9

            # 时间异常或串口断流重连后，本帧不积分。
            if dt <= 0.0 or dt > 0.5:
                dt = 0.0

        self.last_feedback_stamp = stamp

        if self.use_imu_wz_for_odom:
            odom_wz = self.imu_wz
            self.yaw_source = "IMU"

        else:
            odom_wz = self.wheel_wz
            self.yaw_source = "WHEEL"

        self.current_odom_wz = odom_wz

        if dt > 0.0:
            # 中点法：
            # 用当前周期中间时刻的 yaw 将车体速度转换到 odom 坐标系。
            middle_yaw = (
                self.odom_yaw
                + odom_wz * dt * 0.5
            )

            world_vx = (
                self.measured_vx
                * math.cos(middle_yaw)
                - self.measured_vy
                * math.sin(middle_yaw)
            )

            world_vy = (
                self.measured_vx
                * math.sin(middle_yaw)
                + self.measured_vy
                * math.cos(middle_yaw)
            )

            self.odom_x += world_vx * dt
            self.odom_y += world_vy * dt

            self.odom_yaw = normalize_angle(
                self.odom_yaw
                + odom_wz * dt
            )

        # 平面 yaw -> 四元数
        half_yaw = self.odom_yaw * 0.5
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        # --------------------------------------------------------
        # 发布 /odom
        # --------------------------------------------------------

        odom = Odometry()

        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.measured_vx
        odom.twist.twist.linear.y = self.measured_vy
        odom.twist.twist.angular.z = odom_wz

        # 平面底盘只使用 x、y、yaw。
        odom.pose.covariance[0] = 0.03
        odom.pose.covariance[7] = 0.03
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.08

        odom.twist.covariance[0] = 0.04
        odom.twist.covariance[7] = 0.04
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.10

        self.odom_pub.publish(odom)

        # --------------------------------------------------------
        # 发布 odom -> base_link
        # --------------------------------------------------------

        if self.tf_broadcaster is not None:
            transform = TransformStamped()

            transform.header.stamp = stamp.to_msg()
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame

            transform.transform.translation.x = self.odom_x
            transform.transform.translation.y = self.odom_y
            transform.transform.translation.z = 0.0

            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(
                transform
            )

    # ============================================================
    # 状态打印
    # ============================================================

    def print_status(self) -> None:
        """定时打印调试和标定所需信息。"""

        now = time.monotonic()

        serial_state = (
            "CONNECTED"
            if self.serial_ready()
            else "DISCONNECTED"
        )

        if self.last_cmd_time is None:
            cmd_state = "NO_CMD"
            cmd_age = "none"

        else:
            age = now - self.last_cmd_time

            cmd_state = (
                "TIMEOUT_STOP"
                if age > self.cmd_timeout
                else "ACTIVE"
            )

            cmd_age = f"{age:.2f}s"

        if self.last_feedback_time is None:
            feedback_state = "NO_FEEDBACK"
            feedback_age = "none"

        else:
            age = now - self.last_feedback_time

            feedback_state = (
                "TIMEOUT"
                if age > self.feedback_timeout
                else "ACTIVE"
            )

            feedback_age = f"{age:.2f}s"

        yaw_deg = math.degrees(
            self.odom_yaw
        )

        status = (
            f"Serial={serial_state}  "
            f"CMD={cmd_state}/{cmd_age}  "
            f"FB={feedback_state}/{feedback_age}\n"

            f"  target : "
            f"vx={self.target_vx:+.3f} m/s  "
            f"vy={self.target_vy:+.3f} m/s  "
            f"wz={self.target_wz:+.3f} rad/s\n"

            f"  wheel  : "
            f"vx={self.measured_vx:+.3f} m/s  "
            f"vy={self.measured_vy:+.3f} m/s  "
            f"wz={self.wheel_wz:+.3f} rad/s\n"

            f"  imu    : "
            f"gx={self.gyro_radps[0]:+.4f}  "
            f"gy={self.gyro_radps[1]:+.4f}  "
            f"gz={self.gyro_radps[2]:+.4f} rad/s  "
            f"gz_before_deadband="
            f"{self.imu_wz_before_deadband:+.4f}  "
            f"gz_used={self.imu_wz:+.4f} rad/s\n"

            f"  odom   : "
            f"source={self.yaw_source}  "
            f"x={self.odom_x:+.3f} m  "
            f"y={self.odom_y:+.3f} m  "
            f"yaw={yaw_deg:+.2f} deg  "
            f"yaw_rad={self.odom_yaw:+.4f}  "
            f"wz={self.current_odom_wz:+.4f} rad/s\n"

            f"  serial : "
            f"TX={self.tx_count}  "
            f"RX={self.rx_count}  "
            f"BAD={self.bad_frame_count}  "
            f"ERR={self.serial_error_count}"
        )

        if self.print_imu_raw:
            status += (
                f"\n"
                f"  raw    : "
                f"acc={tuple(self.acc_raw)}  "
                f"gyro={tuple(self.gyro_raw)}"
            )

        self.get_logger().info(status)

    # ============================================================
    # 节点退出
    # ============================================================

    def stop_and_close(self) -> None:
        """退出前发送多帧停止指令并关闭串口。"""

        try:
            if self.serial_ready():
                for _ in range(5):
                    if not self.write_velocity(
                        0.0,
                        0.0,
                        0.0,
                    ):
                        break

                    time.sleep(0.02)

        finally:
            self.close_serial()


# ============================================================
# 程序入口
# ============================================================

def main(args=None) -> None:
    """ROS 2 节点入口。"""

    rclpy.init(args=args)

    node: Optional[STM32Bridge] = None

    try:
        node = STM32Bridge()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.stop_and_close()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
