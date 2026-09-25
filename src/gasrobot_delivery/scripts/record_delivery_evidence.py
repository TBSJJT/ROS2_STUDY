#!/usr/bin/env python3
"""记录一批实际配送的真值、规划路径和双窗口截图，不发送任何控制命令。

在开始新批次前运行。Gazebo ModelStates 没有 header，CSV 中时间明确采用接收时
ROS 时钟；状态话题另有采样时刻。真值按已保存的 world→map 刚体标定变换后，
实时发布为 RViz 累计轨迹。记录器不得用于定位或控制。
"""
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import time

from PyQt5.QtWidgets import QApplication
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import yaml


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, output, stations, app):
        super().__init__(
            "delivery_evidence", parameter_overrides=[Parameter("use_sim_time", value=True)]
        )
        self.output, self.app = output, app
        self.book = yaml.safe_load(stations.read_text())
        (output / "stations.yaml").write_text(stations.read_text())
        self.tx, self.ty, self.angle = self.book["registration"]["world_to_map"]
        self.status = None
        self.run = None
        self.leg = None
        self.legs = []
        self.truth = None
        self.previous = None
        self.seen = set()
        self.done = False
        self.terminal_wall = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.trace = RosPath()
        self.trace.header.frame_id = "map"
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(RosPath, "/delivery_evidence/actual_path", qos)
        self.create_subscription(String, "/delivery_manager/status", self.on_status, qos)
        self.create_subscription(
            ModelStates, "/gazebo/model_states", self.on_truth, qos_profile_sensor_data
        )
        self.create_subscription(RosPath, "/plan", self.on_plan, 10)
        self.stream = (output / "actual_samples.csv").open("w", newline="")
        self.csv = csv.writer(self.stream)
        self.csv.writerow(
            [
                "sample_index",
                "receipt_ros_time",
                "status_ros_time",
                "leg",
                "source",
                "target",
                "state",
                "world_x",
                "world_y",
                "world_yaw",
                "truth_map_x",
                "truth_map_y",
                "amcl_x",
                "amcl_y",
                "amcl_yaw",
            ]
        )
        self.plans = (output / "plans.jsonl").open("w")
        self.statuses = (output / "statuses.jsonl").open("w")
        self.samples = 0
        self.last_publish = 0
        self.get_logger().info("READY: 等待新的配送批次；不采纳上一批终态")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        status = json.loads(msg.data)
        self.status = status
        if self.run is None:
            if status["state"] not in ("NAVIGATING", "RETURNING"):
                return
            self.run = status["run_dir"]
            (self.output / "run_reference.json").write_text(json.dumps(status, indent=2))
        if status["run_dir"] != self.run:
            return
        self.statuses.write(json.dumps(status) + "\n")
        self.statuses.flush()
        if status["state"] in ("NAVIGATING", "RETURNING") and (
            self.leg is None or self.leg["target"] != status["target"]
        ):
            source = self.leg["target"] if self.leg else self.book["depot"]
            self.leg = dict(
                index=len(self.legs) + 1,
                source=source,
                target=status["target"],
                start_ros=status["ros_time"],
                initial_plan_length=None,
                traveled_m=0.0,
                midpoint_captured=False,
            )
            self.legs.append(self.leg)
            self.previous = None
        if status["state"] in ("COMPLETED", "FAILED", "CANCELLED") and self.terminal_wall is None:
            self.terminal_wall = time.monotonic()

    def on_plan(self, msg):
        if (
            self.run is None
            or self.leg is None
            or self.status["state"] not in ("NAVIGATING", "RETURNING")
        ):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # 只绑定本段的新规划，防止上一目标的末帧路径成为新行程的长度基准。
        if msg.header.frame_id != "map" or stamp < self.leg["start_ros"]:
            return
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        if self.leg["initial_plan_length"] is None and length > 0.5:
            self.leg["initial_plan_length"] = length
        self.plans.write(
            json.dumps(
                dict(
                    receipt_ros_time=self.now(),
                    stamp=stamp,
                    leg=self.leg["index"],
                    length_m=length,
                    points=points,
                )
            )
            + "\n"
        )
        self.plans.flush()

    def on_truth(self, msg):
        if self.run is None or self.leg is None or "gasrobot" not in msg.name:
            return
        p = msg.pose[msg.name.index("gasrobot")]
        x, y = p.position.x, p.position.y
        c, s = math.cos(self.angle), math.sin(self.angle)
        mx, my = c * x - s * y + self.tx, s * x + c * y + self.ty
        if self.previous is not None and self.status["state"] in ("NAVIGATING", "RETURNING"):
            self.leg["traveled_m"] += math.dist((x, y), self.previous)
        self.previous = (x, y)
        amcl = ("", "", "")
        try:
            t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time()).transform
            amcl = (t.translation.x, t.translation.y, yaw(t.rotation))
        except Exception:
            pass  # 缺失估计保留空值，绝不用真值补造 AMCL 数据。
        self.samples += 1
        now = self.now()
        self.truth = dict(
            sample_index=self.samples,
            receipt_ros_time=now,
            world_x=x,
            world_y=y,
            truth_map_x=mx,
            truth_map_y=my,
        )
        self.csv.writerow(
            [
                self.samples,
                now,
                self.status["ros_time"],
                self.leg["index"],
                self.leg["source"],
                self.leg["target"],
                self.status["state"],
                x,
                y,
                yaw(p.orientation),
                mx,
                my,
                *amcl,
            ]
        )
        self.stream.flush()
        # 显示轨迹约 10 Hz 更新；CSV 保留收到的全部约 20 Hz 真值采样。
        if now - self.last_publish >= 0.1:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = mx, my
            pose.pose.orientation.w = 1.0
            self.trace.poses.append(pose)
            self.trace.header.stamp = pose.header.stamp
            self.publisher.publish(self.trace)
            self.last_publish = now

    def capture(self, label):
        if label in self.seen:
            return
        filename = f"{len(self.seen)+1:02d}_{label}.png"
        pix = self.app.primaryScreen().grabWindow(0)
        if pix.isNull() or not pix.save(str(self.output / filename)):
            raise RuntimeError("主屏幕截图失败")
        record = dict(
            file=filename,
            captured_local=datetime.now().astimezone().isoformat(),
            width=pix.width(),
            height=pix.height(),
            status=self.status,
            truth_sample=self.truth,
            leg=dict(self.leg),
            timing_note="ROS status, ModelStates receipt and GUI rendering are asynchronous",
        )
        (self.output / filename.replace(".png", ".json")).write_text(json.dumps(record, indent=2))
        self.seen.add(label)
        self.get_logger().info(f"CAPTURE {filename}")

    def poll(self):
        if self.run is None or self.truth is None:
            return
        state = self.status["state"]
        if state in ("NAVIGATING", "RETURNING"):
            length = self.leg["initial_plan_length"]
            target = self.book["stations"][self.leg["target"]]
            remaining = math.dist(
                (self.truth["truth_map_x"], self.truth["truth_map_y"]), (target["x"], target["y"])
            )
            # 用实际运动累积距离达到本段初始规划长度的一半来触发，排除原地转向。
            # 再要求距离终点至少 0.6 m，确保截图发生在两目标之间而非到站时。
            if (
                length
                and self.leg["traveled_m"] >= length * 0.5
                and remaining > 0.6
                and not self.leg["midpoint_captured"]
            ):
                self.capture(f"midway_{self.leg['source']}_to_{self.leg['target']}")
                self.leg["midpoint_captured"] = True
        elif state == "SERVICING" and self.status["service_remaining_sec"] < 3.8:
            self.capture(f"servicing_{self.leg['target']}")
        elif self.terminal_wall is not None and time.monotonic() - self.terminal_wall > 1.0:
            self.capture(state.lower())
            (self.output / "legs.json").write_text(json.dumps(self.legs, indent=2))
            self.done = True

    def close(self):
        self.stream.close()
        self.plans.close()
        self.statuses.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--stations", type=Path, default=Path("src/gasrobot_delivery/config/stations.yaml")
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if any(args.output.iterdir()):
        parser.error("请使用新的空输出目录，避免覆盖实验记录")
    app = QApplication([])
    rclpy.init()
    node = Recorder(args.output, args.stations, app)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.01)
            app.processEvents()
            node.poll()
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
