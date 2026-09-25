#!/usr/bin/env python3
"""Simulation-only acceptance runner. Ground truth is observed, never controls motion.

Run from the workspace after sourcing ROS and install/setup.bash. Navigation mode
uses Nav2 goals; batch mode uses the delivery manager's public Trigger services.
Artifacts are kept under runs/validation_* including failed attempts.
"""
import argparse
import csv
from datetime import datetime
import json
import hashlib
import uuid
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger, Empty
from tf2_ros import Buffer, TransformListener
import yaml


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def overlaps(x, y, a, box):
    bx, by, hx, hy = box
    c, s = math.cos(a), math.sin(a)
    # SAT：保守包络包含前部雷达/相机、底盘和车轮；接触噪声容差 5 mm。
    for ux, uy in [(1, 0), (0, 1), (c, s), (-s, c)]:
        radius = 0.17 * abs(ux * c + uy * s) + 0.153 * abs(-ux * s + uy * c)
        radius += hx * abs(ux) + hy * abs(uy)
        if abs((x - bx) * ux + (y - by) * uy) >= radius - 0.005:
            return False
    return True


class Validator(Node):
    def __init__(self, book):
        super().__init__(
            "delivery_acceptance", parameter_overrides=[Parameter("use_sim_time", value=True)]
        )
        self.stations = book["stations"]
        self.registration = book["registration"]["world_to_map"]
        self.error = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.status = None
        self.truth = None
        self.truth_samples = 0
        self.collisions = []
        self.cmd = Twist()
        self.rows = []
        self.run_start = time.monotonic()
        self.artifact = Path("runs") / (
            "validation_" + datetime.now().strftime("%Y%m%dT%H%M%S") + "_" + uuid.uuid4().hex[:6]
        )
        self.artifact.mkdir(parents=True)
        self.boxes = [(x, y, 1, 0.8) for x in [3.5, 6.5, 9.5] for y in [3, 7]]
        self.boxes += [
            (6, -0.075, 6.15, 0.075),
            (6, 10.075, 6.15, 0.075),
            (-0.075, 5, 0.075, 5),
            (12.075, 5, 0.075, 5),
        ]
        # Rack shelves overlap the robot height; use its full horizontal envelope.
        self.boxes += [(1.5, 0.45, 0.7, 0.3)]
        self.create_subscription(
            ModelStates, "/gazebo/model_states", self.on_truth, qos_profile_sensor_data
        )
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(
            String,
            "/delivery_manager/status",
            self.on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def on_truth(self, msg):
        if "gasrobot" not in msg.name:
            return
        pose = msg.pose[msg.name.index("gasrobot")]
        self.truth = [pose.position.x, pose.position.y, yaw(pose.orientation)]
        self.truth_samples += 1
        if any(overlaps(*self.truth, box) for box in self.boxes):
            if not self.collisions or self.now() - self.collisions[-1][0] > 1:
                self.collisions.append([self.now(), *self.truth])

    def on_cmd(self, msg):
        self.cmd = msg

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait(self, future, timeout=240):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise RuntimeError("Wall-clock timeout")
        return future.result()

    def ready(self):
        """等待生命周期激活及首帧数据，避免将启动中的动作服务误判为就绪。"""
        client = self.create_client(GetState, "/bt_navigator/get_state")
        deadline = time.monotonic() + 90
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if not client.service_is_ready():
                    continue
                state = self.wait(client.call_async(GetState.Request()), 5)
                if state.current_state.id != 3 or self.now() <= 0 or self.truth is None:
                    continue
                if self.tf.can_transform("map", "base_footprint", rclpy.time.Time()):
                    if self.client.server_is_ready():
                        self.settle(3)
                        return
            raise RuntimeError("Nav2 lifecycle / clock / TF / ground truth not ready")
        finally:
            self.destroy_client(client)

    def settle(self, seconds=2):
        until = self.now() + seconds
        deadline = time.monotonic() + 20
        while self.now() < until and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def save(self):
        data = dict(
            error=self.error,
            results=self.rows,
            truth_samples=self.truth_samples,
            collisions=self.collisions,
            wall_elapsed_sec=time.monotonic() - self.run_start,
        )
        (self.artifact / "results.json").write_text(json.dumps(data, indent=2))

    def navigate(self, name):
        p = self.stations[name]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = p["x"], p["y"]
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(
            p["yaw"] / 2
        ), math.cos(p["yaw"] / 2)
        start = self.now()
        last = [0]

        def feedback(msg):
            if time.monotonic() - last[0] > 20:
                last[0] = time.monotonic()
                print(
                    "NAV", name, "remaining", round(msg.feedback.distance_remaining, 2), flush=True
                )

        handle = self.wait(self.client.send_goal_async(goal, feedback_callback=feedback))
        if not handle.accepted:
            raise RuntimeError("Goal rejected")
        try:
            result = self.wait(handle.get_result_async(), 300)
        except Exception:
            self.wait(handle.cancel_goal_async(), 10)
            raise
        self.settle()
        p_actual = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time()).transform
        xy = math.hypot(p_actual.translation.x - p["x"], p_actual.translation.y - p["y"])
        ya = abs(angle(yaw(p_actual.rotation) - p["yaw"]))
        row = dict(
            station=name,
            status=result.status,
            elapsed_sim_sec=self.now() - start,
            xy_error=xy,
            yaw_error=ya,
            truth_pose=self.truth,
            cmd_linear=self.cmd.linear.x,
            cmd_angular=self.cmd.angular.z,
        )
        registration = self.registration
        if self.truth:
            tx, ty, a = registration
            gx = math.cos(a) * self.truth[0] - math.sin(a) * self.truth[1] + tx
            gy = math.sin(a) * self.truth[0] + math.cos(a) * self.truth[1] + ty
            row["localization_xy_error"] = math.hypot(
                p_actual.translation.x - gx, p_actual.translation.y - gy
            )
            row["localization_yaw_error"] = abs(angle(yaw(p_actual.rotation) - self.truth[2] - a))
        self.rows.append(row)
        self.save()
        print("RESULT", json.dumps(row), flush=True)
        assert result.status == 4 and xy <= 0.10 and ya <= 0.10, "Navigation arrival failed"
        assert abs(self.cmd.linear.x) < 0.01 and abs(self.cmd.angular.z) < 0.01, "Not stopped"
        assert self.truth_samples > 0 and not self.collisions, "Collision or missing ground truth"

    def service(self, name):
        client = self.create_client(Trigger, "/delivery_manager/" + name)
        if not client.wait_for_service(timeout_sec=30):
            raise RuntimeError("Manager service unavailable")
        response = self.wait(client.call_async(Trigger.Request()), 10)
        self.destroy_client(client)
        return response

    def wait_state(self, states, timeout=240):
        """等待公开状态，而不是把服务应答误认为动作已经完成。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.status and self.status["state"] in states:
                return dict(self.status)
        raise RuntimeError(f"Manager did not reach {states}: {self.status}")

    def physics(self, name):
        client = self.create_client(Empty, "/" + name)
        try:
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError(f"Gazebo {name} unavailable")
            self.wait(client.call_async(Empty.Request()), 10)
        finally:
            self.destroy_client(client)

    def controls(self):
        """真实 Gazebo 异常操作验收；结束后返回物料站，另起干净批次。"""
        assert self.service("start").success
        assert not self.service("start").success, "Duplicate start accepted"
        self.wait_state({"NAVIGATING"})
        self.settle(2)
        assert self.service("pause").success
        self.wait_state({"PAUSED"})
        assert self.service("resume").success
        self.wait_state({"NAVIGATING"})
        self.settle(2)
        assert self.service("cancel").success
        self.wait_state({"CANCELLED"})
        self.settle(2)
        assert not self.status["navigation_pending"]
        assert abs(self.cmd.linear.x) < 0.01 and abs(self.cmd.angular.z) < 0.01
        self.rows.append(
            {"control_check": "duplicate start / navigation pause-resume-cancel", "passed": True}
        )
        self.navigate("depot")

        assert self.service("start").success
        self.wait_state({"SERVICING"})
        assert self.service("pause").success
        paused = self.wait_state({"PAUSED"})
        remaining = paused["service_remaining_sec"]
        self.settle(3)
        assert abs(self.status["service_remaining_sec"] - remaining) < 1e-6
        assert not self.status["completed"], "Arrival was counted as delivery"
        assert self.service("resume").success
        self.wait_state({"SERVICING"})
        self.physics("pause_physics")
        try:
            # Gazebo 暂停期间 ROS 定时器也停止；使用墙钟读取消息，确认仿真时间不推进。
            until = time.monotonic() + 1
            while time.monotonic() < until:
                rclpy.spin_once(self, timeout_sec=0.05)
            frozen = self.now()
            until = time.monotonic() + 2
            while time.monotonic() < until:
                rclpy.spin_once(self, timeout_sec=0.05)
            assert abs(self.now() - frozen) < 1e-6, "Paused Gazebo clock advanced"
        finally:
            self.physics("unpause_physics")
        self.wait_state({"NAVIGATING", "FAILED"})
        assert self.status["state"] == "NAVIGATING" and self.status["completed"] == ["S1"]
        assert self.service("cancel").success
        self.wait_state({"CANCELLED"})
        self.rows.append(
            {
                "control_check": "service pause / Gazebo clock pause / completion after service",
                "passed": True,
            }
        )
        self.navigate("depot")
        self.save()

    def configure_batch(self, path):
        """空闲时通过公开参数接口选择批次，验证启动时使用的正式配置。"""
        client = self.create_client(SetParameters, "/delivery_manager/set_parameters")
        try:
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError("Manager parameter service unavailable")
            request = SetParameters.Request()
            request.parameters = [
                Parameter("batch_file", value=str(Path(path).resolve())).to_parameter_msg()
            ]
            result = self.wait(client.call_async(request), 10)
            if not result.results or not all(item.successful for item in result.results):
                raise RuntimeError(str(result.results))
        finally:
            self.destroy_client(client)

    def audit_batch_log(self, directory):
        """从已落盘事件复核卸货时间与交付顺序，不仅依赖最后一条状态消息。"""
        directory = Path(directory)
        batch = yaml.safe_load((directory / "batch.yaml").read_text())
        summary = json.loads((directory / "summary.json").read_text())
        with (directory / "events.csv").open() as stream:
            events = list(csv.DictReader(stream))
        arrivals = {}
        delivered = []
        durations = []
        for event in events:
            if event["event"] == "arrived":
                arrivals[event["station"]] = float(event["ros_time"])
            elif event["event"] == "service_completed":
                duration = float(event["ros_time"]) - arrivals[event["station"]]
                assert duration + 1e-6 >= batch["service_sec"], "Service ended early"
                delivered.append(event["station"])
                durations.append(duration)
            elif event["event"] == "navigation_result":
                detail = json.loads(event["details"])
                if detail["nav_status"] == 4:
                    assert detail["xy_error"] <= 0.10 and detail["yaw_error"] <= 0.10
        assert delivered == batch["route"] == summary["completed_stations"]
        assert summary["success"] and summary["returned_to_depot"]
        assert sum(event["event"] == "returned" for event in events) == 1
        assert (directory / "trajectory.csv").stat().st_size > 100
        return {
            "summary": summary,
            "service_durations_sec": durations,
            "retries": sum(event["event"] == "retry" for event in events),
        }

    def batch(self):
        previous_run = self.status.get("run_dir") if self.status else None
        response = self.service("start")
        if not response.success:
            raise RuntimeError(response.message)
        start = self.now()
        last = None
        deadline = time.monotonic() + 1600
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                not self.status
                or self.status["ros_time"] < start
                or self.status["run_dir"] == previous_run
            ):
                continue
            state = (self.status["state"], self.status["target"])
            if state != last:
                print("TASK", state, flush=True)
                last = state
            if self.status["state"] in ["COMPLETED", "FAILED", "CANCELLED"]:
                self.settle()
                row = dict(self.status, truth_pose=self.truth)
                if self.status["state"] == "COMPLETED":
                    row["log_audit"] = self.audit_batch_log(self.status["run_dir"])
                self.rows.append(row)
                self.save()
                assert self.status["state"] == "COMPLETED", "Batch did not complete"
                assert (
                    self.truth_samples > 0 and not self.collisions
                ), "Collision or no ground truth"
                return
        self.service("cancel")
        raise RuntimeError("Batch wall-clock timeout")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode", choices=["navigation", "batch", "controls"], default="navigation"
    )
    parser.add_argument("--stations", default="src/gasrobot_delivery/config/stations.yaml")
    parser.add_argument("--route", nargs="*")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--batch-file", help="在批次验收前设置管理器 batch_file")
    args = parser.parse_args()
    rclpy.init()
    node = Validator(yaml.safe_load(Path(args.stations).read_text()))
    metadata = {"arguments": vars(args), "sha256": {}}
    for path in [Path(args.stations), Path("src/gasrobot_delivery/config/nav2_workshop.yaml")]:
        metadata["sha256"][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    (node.artifact / "metadata.json").write_text(json.dumps(metadata, indent=2))
    try:
        node.ready()
        if args.mode == "navigation":
            if not node.client.wait_for_server(timeout_sec=60):
                raise RuntimeError("Nav2 unavailable")
            route = args.route or [s for i in range(1, 7) for s in (f"S{i}", "depot")] + [
                "S1",
                "S5",
                "depot",
            ]
            for station in route:
                node.navigate(station)
        else:
            if args.batch_file:
                node.configure_batch(args.batch_file)
            if args.mode == "controls":
                node.controls()
            else:
                for _ in range(args.repeat):
                    node.batch()
        print("PASS", node.artifact, flush=True)
    except BaseException as exc:
        node.error = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
