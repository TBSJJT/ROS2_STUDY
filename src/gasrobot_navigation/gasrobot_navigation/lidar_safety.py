#!/usr/bin/env python3
"""
LaserScan 速度安全过滤器。

输入：
    /cmd_vel       （通常来自遥控节点或 Nav2）
输出：
    /cmd_vel_safe  （STM32 桥接节点应订阅此话题）

本节点不会主动驱动机器人。只有在激光扫描数据有效且前方扇区无障碍物时，
才会转发未超时的速度指令。
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class LidarSafety(Node):
    def __init__(self) -> None:
        super().__init__("lidar_safety")

        defaults = {
            "scan_topic": "/scan",
            "input_cmd_topic": "/cmd_vel",
            "output_cmd_topic": "/cmd_vel_safe",
            "front_angle_deg": 30.0,
            "stop_distance": 0.30,
            "required_stop_points": 2,
            "scan_timeout": 0.5,
            "cmd_timeout": 0.5,
            "allow_straight_reverse": True,
            "publish_rate": 20.0,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        get = lambda name: self.get_parameter(name).value

        self.scan_topic = str(get("scan_topic"))
        self.input_cmd_topic = str(get("input_cmd_topic"))
        self.output_cmd_topic = str(get("output_cmd_topic"))
        self.front_angle_deg = float(get("front_angle_deg"))
        self.stop_distance = float(get("stop_distance"))
        self.required_stop_points = int(get("required_stop_points"))
        self.scan_timeout = float(get("scan_timeout"))
        self.cmd_timeout = float(get("cmd_timeout"))
        self.allow_straight_reverse = bool(get("allow_straight_reverse"))
        self.publish_rate = float(get("publish_rate"))

        if self.input_cmd_topic == self.output_cmd_topic:
            raise ValueError("input_cmd_topic and output_cmd_topic must differ")
        if self.stop_distance <= 0.0:
            raise ValueError("stop_distance must be positive")
        if self.required_stop_points < 1:
            raise ValueError("required_stop_points must be at least 1")
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, scan_qos
        )
        self.cmd_sub = self.create_subscription(
            Twist, self.input_cmd_topic, self.cmd_callback, 20
        )
        self.cmd_pub = self.create_publisher(
            Twist, self.output_cmd_topic, 20
        )

        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.last_scan_time = None
        self.front_blocked = True
        self.front_min = math.inf
        self.last_state = None

        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_safe_command
        )

        self.get_logger().info(
            f"Lidar safety: {self.input_cmd_topic} -> "
            f"{self.output_cmd_topic}, front +/-"
            f"{self.front_angle_deg:.1f}deg, stop < "
            f"{self.stop_distance:.2f}m"
        )

    @staticmethod
    def copy_twist(source: Twist) -> Twist:
        output = Twist()
        output.linear.x = source.linear.x
        output.linear.y = source.linear.y
        output.linear.z = source.linear.z
        output.angular.x = source.angular.x
        output.angular.y = source.angular.y
        output.angular.z = source.angular.z
        return output

    def cmd_callback(self, msg: Twist) -> None:
        self.last_cmd = self.copy_twist(msg)
        self.last_cmd_time = time.monotonic()

    def scan_callback(self, msg: LaserScan) -> None:
        stop_count = 0
        front_min = math.inf
        limit = math.radians(self.front_angle_deg)

        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(angle) > limit:
                continue
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            front_min = min(front_min, distance)
            if distance < self.stop_distance:
                stop_count += 1

        self.front_min = front_min
        self.front_blocked = stop_count >= self.required_stop_points
        self.last_scan_time = time.monotonic()

    def publish_safe_command(self) -> None:
        now = time.monotonic()

        cmd_fresh = (
            self.last_cmd_time is not None
            and now - self.last_cmd_time <= self.cmd_timeout
        )
        scan_fresh = (
            self.last_scan_time is not None
            and now - self.last_scan_time <= self.scan_timeout
        )

        output = Twist()

        if not cmd_fresh:
            state = "CMD_TIMEOUT_STOP"
        elif not scan_fresh:
            state = "SCAN_TIMEOUT_STOP"
        elif self.front_blocked:
            reverse_escape = (
                self.allow_straight_reverse
                and self.last_cmd.linear.x < 0.0
                and abs(self.last_cmd.linear.y) < 1e-3
                and abs(self.last_cmd.angular.z) < 1e-3
            )

            if reverse_escape:
                output = self.copy_twist(self.last_cmd)
                state = "REVERSE_ESCAPE"
            else:
                state = "OBSTACLE_STOP"
        else:
            output = self.copy_twist(self.last_cmd)
            state = "PASS"

        self.cmd_pub.publish(output)

        if state != self.last_state:
            distance = (
                "inf" if math.isinf(self.front_min)
                else f"{self.front_min:.3f}m"
            )
            self.get_logger().info(f"{state}, front_min={distance}")
            self.last_state = state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarSafety()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
