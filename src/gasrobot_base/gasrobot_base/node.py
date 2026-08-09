"""STM32 底盘桥接的 ROS 2 编排节点。"""

import math
import time
from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from serial import SerialException
from tf2_ros import TransformBroadcaster

from gasrobot_base.imu import ImuConverter
from gasrobot_base.models import (
    ImuSample,
    OdometrySample,
    RawFeedback,
    VelocityCommand,
)
from gasrobot_base.odometry import OdometryIntegrator
from gasrobot_base.parameters import BridgeConfig
from gasrobot_base.protocol import (
    COMMAND_FRAME_SIZE,
    FeedbackStreamParser,
    encode_velocity_command,
)
from gasrobot_base.serial_transport import SerialTransport


@dataclass
class _BridgeStatistics:
    transmitted: int = 0
    received: int = 0
    serial_errors: int = 0


class STM32BridgeNode(Node):
    """连接 ROS 2 话题与 STM32 底盘串口的编排节点。"""

    def __init__(self) -> None:
        super().__init__("stm32_bridge")
        self.config = BridgeConfig.from_node(self)
        self.transport = SerialTransport(
            port=self.config.port,
            baud=self.config.baud,
            startup_delay=self.config.startup_delay,
        )
        self.parser = FeedbackStreamParser()
        self.imu_converter = ImuConverter(
            acceleration_lsb_per_g=self.config.accel_lsb_per_g,
            gyroscope_lsb_per_dps=self.config.gyro_lsb_per_dps,
            z_sign=self.config.imu_z_sign,
            z_offset_radps=self.config.gyro_z_offset_radps,
            z_deadband=self.config.gyro_z_deadband,
        )
        self.odometry_integrator = OdometryIntegrator(
            use_imu_yaw_rate=self.config.use_imu_wz_for_odom,
        )

        self.target_command = VelocityCommand()
        self.last_command_time: Optional[float] = None
        self.last_feedback_time: Optional[float] = None
        self.last_reconnect_attempt = 0.0
        self.latest_feedback: Optional[RawFeedback] = None
        self.latest_imu: Optional[ImuSample] = None
        self.latest_odometry = OdometrySample(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_x=0.0,
            linear_y=0.0,
            angular_z=0.0,
            yaw_source="NONE",
        )
        self.statistics = _BridgeStatistics()

        self.command_subscription = self.create_subscription(
            Twist,
            self.config.cmd_vel_topic,
            self._command_callback,
            20,
        )
        self.odometry_publisher = self.create_publisher(
            Odometry,
            self.config.odom_topic,
            20,
        )
        self.imu_publisher = self.create_publisher(
            Imu,
            self.config.imu_topic,
            20,
        )
        self.tf_broadcaster = (
            TransformBroadcaster(self)
            if self.config.publish_odom_tf
            else None
        )

        self.transmit_timer = self.create_timer(
            1.0 / self.config.tx_rate,
            self._transmit_loop,
        )
        self.receive_timer = self.create_timer(0.005, self._receive_loop)
        self.status_timer = self.create_timer(
            self.config.status_period,
            self._print_status,
        )

        self.get_logger().info(
            f"STM32 底盘桥接节点已启动："
            f"{self.config.port} @ {self.config.baud}"
        )
        self._open_serial()

    def _open_serial(self) -> bool:
        self.last_reconnect_attempt = time.monotonic()
        try:
            self.transport.connect()
            self.parser.clear()
            self.odometry_integrator.reset_time()
            self.get_logger().info(
                f"串口已连接：{self.config.port} @ {self.config.baud}"
            )
            for _ in range(self.config.startup_stop_frames):
                if not self._write_velocity(VelocityCommand()):
                    return False
                time.sleep(0.02)
            return True
        except (OSError, SerialException) as exc:
            self.statistics.serial_errors += 1
            self.transport.close()
            self.get_logger().error(
                f"无法打开串口 {self.config.port}：{exc}"
            )
            return False

    def _mark_serial_disconnected(self, exc: Exception) -> None:
        self.statistics.serial_errors += 1
        self.get_logger().error(f"串口通信错误：{exc}")
        self.transport.close()
        self.parser.clear()
        self.odometry_integrator.reset_time()

    def _try_reconnect(self) -> None:
        if self.transport.is_open:
            return
        now = time.monotonic()
        if now - self.last_reconnect_attempt >= self.config.reconnect_period:
            self.last_reconnect_attempt = now
            self._open_serial()

    @staticmethod
    def _finite_or_zero(value: float) -> float:
        return value if math.isfinite(value) else 0.0

    def _command_callback(self, message: Twist) -> None:
        self.target_command = VelocityCommand(
            linear_x=self._finite_or_zero(float(message.linear.x)),
            linear_y=self._finite_or_zero(float(message.linear.y)),
            angular_z=self._finite_or_zero(float(message.angular.z)),
        )
        self.last_command_time = time.monotonic()

    def _write_velocity(self, command: VelocityCommand) -> bool:
        if not self.transport.is_open:
            return False

        frame = encode_velocity_command(
            command,
            self.config.velocity_limits,
        )
        try:
            written = self.transport.write(frame)
            if written != COMMAND_FRAME_SIZE:
                raise SerialException(
                    f"仅写入 {written}/{COMMAND_FRAME_SIZE} 字节"
                )
            self.statistics.transmitted += 1
            if self.config.debug_tx:
                self.get_logger().info(
                    f"发送 {frame.hex(' ')} | "
                    f"vx={command.linear_x:+.3f}, "
                    f"vy={command.linear_y:+.3f}, "
                    f"wz={command.angular_z:+.3f}"
                )
            return True
        except (OSError, SerialException) as exc:
            self._mark_serial_disconnected(exc)
            return False

    def _transmit_loop(self) -> None:
        if not self.transport.is_open:
            self._try_reconnect()
            return

        now = time.monotonic()
        command_expired = (
            self.last_command_time is None
            or now - self.last_command_time > self.config.cmd_timeout
        )
        command = VelocityCommand() if command_expired else self.target_command
        self._write_velocity(command)

    def _receive_loop(self) -> None:
        if not self.transport.is_open:
            return
        try:
            data = self.transport.read_available()
            for feedback in self.parser.feed(data):
                self._handle_feedback(feedback)
        except (OSError, SerialException) as exc:
            self._mark_serial_disconnected(exc)

    def _handle_feedback(self, feedback: RawFeedback) -> None:
        self.latest_feedback = feedback
        self.latest_imu = self.imu_converter.convert(feedback)
        self.statistics.received += 1
        self.last_feedback_time = time.monotonic()

        stamp = self.get_clock().now()
        self.latest_odometry = self.odometry_integrator.update(
            feedback=feedback,
            imu_yaw_rate=self.latest_imu.yaw_rate,
            stamp=stamp.nanoseconds / 1e9,
        )
        self._publish_imu(stamp, self.latest_imu)
        self._publish_odometry(stamp, self.latest_odometry)

        if self.config.debug_rx:
            self.get_logger().info(
                f"收到反馈 | vx={feedback.linear_x:+.3f}, "
                f"vy={feedback.linear_y:+.3f}, "
                f"wz={feedback.angular_z:+.3f}, "
                f"imu_wz={self.latest_imu.yaw_rate:+.4f}"
            )

    def _publish_imu(self, stamp, sample: ImuSample) -> None:
        message = Imu()
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = self.config.imu_frame
        message.orientation_covariance[0] = -1.0
        message.linear_acceleration.x = sample.acceleration[0]
        message.linear_acceleration.y = sample.acceleration[1]
        message.linear_acceleration.z = sample.acceleration[2]
        message.angular_velocity.x = sample.gyroscope[0]
        message.angular_velocity.y = sample.gyroscope[1]
        message.angular_velocity.z = sample.yaw_rate
        message.angular_velocity_covariance[0] = 0.02
        message.angular_velocity_covariance[4] = 0.02
        message.angular_velocity_covariance[8] = 0.01
        message.linear_acceleration_covariance[0] = 0.10
        message.linear_acceleration_covariance[4] = 0.10
        message.linear_acceleration_covariance[8] = 0.10
        self.imu_publisher.publish(message)

    def _publish_odometry(
        self,
        stamp,
        sample: OdometrySample,
    ) -> None:
        half_yaw = sample.yaw * 0.5
        quaternion_z = math.sin(half_yaw)
        quaternion_w = math.cos(half_yaw)

        message = Odometry()
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = self.config.odom_frame
        message.child_frame_id = self.config.base_frame
        message.pose.pose.position.x = sample.x
        message.pose.pose.position.y = sample.y
        message.pose.pose.orientation.z = quaternion_z
        message.pose.pose.orientation.w = quaternion_w
        message.twist.twist.linear.x = sample.linear_x
        message.twist.twist.linear.y = sample.linear_y
        message.twist.twist.angular.z = sample.angular_z

        message.pose.covariance[0] = 0.03
        message.pose.covariance[7] = 0.03
        message.pose.covariance[14] = 1e6
        message.pose.covariance[21] = 1e6
        message.pose.covariance[28] = 1e6
        message.pose.covariance[35] = 0.08
        message.twist.covariance[0] = 0.04
        message.twist.covariance[7] = 0.04
        message.twist.covariance[14] = 1e6
        message.twist.covariance[21] = 1e6
        message.twist.covariance[28] = 1e6
        message.twist.covariance[35] = 0.10
        self.odometry_publisher.publish(message)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp.to_msg()
            transform.header.frame_id = self.config.odom_frame
            transform.child_frame_id = self.config.base_frame
            transform.transform.translation.x = sample.x
            transform.transform.translation.y = sample.y
            transform.transform.rotation.z = quaternion_z
            transform.transform.rotation.w = quaternion_w
            self.tf_broadcaster.sendTransform(transform)

    def _print_status(self) -> None:
        now = time.monotonic()
        serial_state = "已连接" if self.transport.is_open else "未连接"
        command_state, command_age = self._age_state(
            now,
            self.last_command_time,
            self.config.cmd_timeout,
            "超时停车",
        )
        feedback_state, feedback_age = self._age_state(
            now,
            self.last_feedback_time,
            self.config.feedback_timeout,
            "超时",
        )
        feedback = self.latest_feedback
        imu = self.latest_imu
        odometry = self.latest_odometry

        status = (
            f"串口={serial_state}  指令={command_state}/{command_age}  "
            f"反馈={feedback_state}/{feedback_age}\n"
            f"  目标：vx={self.target_command.linear_x:+.3f} m/s  "
            f"vy={self.target_command.linear_y:+.3f} m/s  "
            f"wz={self.target_command.angular_z:+.3f} rad/s\n"
            f"  底盘：vx={self._value(feedback, 'linear_x'):+.3f} m/s  "
            f"vy={self._value(feedback, 'linear_y'):+.3f} m/s  "
            f"wz={self._value(feedback, 'angular_z'):+.3f} rad/s\n"
            f"  里程：源={odometry.yaw_source}  x={odometry.x:+.3f} m  "
            f"y={odometry.y:+.3f} m  "
            f"yaw={math.degrees(odometry.yaw):+.2f} deg\n"
            f"  统计：发送={self.statistics.transmitted}  "
            f"接收={self.statistics.received}  "
            f"坏帧={self.parser.bad_frame_count}  "
            f"串口错误={self.statistics.serial_errors}"
        )
        if imu is not None:
            status += (
                f"\n  IMU：gx={imu.gyroscope[0]:+.4f}  "
                f"gy={imu.gyroscope[1]:+.4f}  "
                f"gz={imu.gyroscope[2]:+.4f} rad/s  "
                f"使用值={imu.yaw_rate:+.4f} rad/s"
            )
        if self.config.print_imu_raw and feedback is not None:
            status += (
                f"\n  原始值：加速度={feedback.acceleration_raw}  "
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
        if last_time is None:
            return "未收到", "无"
        age = now - last_time
        state = timeout_label if age > timeout else "有效"
        return state, f"{age:.2f}s"

    @staticmethod
    def _value(item, name: str) -> float:
        return float(getattr(item, name)) if item is not None else 0.0

    def stop_and_close(self) -> None:
        """退出前发送多帧停止指令并关闭串口。"""

        try:
            if self.transport.is_open:
                for _ in range(5):
                    if not self._write_velocity(VelocityCommand()):
                        break
                    time.sleep(0.02)
        finally:
            self.transport.close()
