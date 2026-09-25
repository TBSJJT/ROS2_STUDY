#!/usr/bin/env python3
"""在独立 ROS_DOMAIN_ID 下用受控 Action 服务验证真实 ROS 适配层。

这不是 Gazebo 导航验收：它专门制造延迟接受、取消和不可达响应，检查
异步时序及日志。必须设置 ROS_DOMAIN_ID=42，避免向真实仿真发送消息。
"""
from datetime import datetime
import json
import math
import os
from pathlib import Path
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.clock import Clock as NodeClock, ClockType
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import yaml
from gasrobot_delivery.manager import DeliveryManager


class FakeNavigation(Node):
    def __init__(self, depot):
        super().__init__("protocol_navigation")
        self.pose = dict(depot)
        self.clock = self.create_publisher(Clock, "/clock", 10)
        self.odom = self.create_publisher(Odometry, "/odom", 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.elapsed = 1.0
        self.abort_remaining = 0
        self.accept_delay = 0.2
        self.active = 0
        self.max_active = 0
        group = ReentrantCallbackGroup()
        self.create_timer(
            0.02,
            self.publish,
            callback_group=group,
            clock=NodeClock(clock_type=ClockType.STEADY_TIME),
        )
        self.create_service(
            GetState, "/bt_navigator/get_state", self.get_state, callback_group=group
        )
        self.action = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            self.execute,
            goal_callback=self.accept,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group,
        )
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "map", "odom"
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform(t)

    def accept(self, _goal):
        from rclpy.action import GoalResponse

        time.sleep(self.accept_delay)
        return GoalResponse.ACCEPT

    def get_state(self, _request, response):
        response.current_state.id = 3
        response.current_state.label = "active"
        return response

    def publish(self):
        self.elapsed += 0.1
        stamp = rclpy.time.Time(seconds=self.elapsed).to_msg()
        self.clock.publish(Clock(clock=stamp))
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "odom", "base_footprint"
        t.header.stamp = stamp
        t.transform.translation.x, t.transform.translation.y = self.pose["x"], self.pose["y"]
        t.transform.rotation.z, t.transform.rotation.w = math.sin(self.pose["yaw"] / 2), math.cos(
            self.pose["yaw"] / 2
        )
        self.tf.sendTransform(t)
        o = Odometry()
        o.header.frame_id, o.child_frame_id, o.header.stamp = "odom", "base_footprint", stamp
        o.pose.pose.position.x, o.pose.pose.position.y = self.pose["x"], self.pose["y"]
        o.pose.pose.orientation = t.transform.rotation
        self.odom.publish(o)

    def execute(self, handle):
        self.active += 1
        self.max_active = max(self.max_active, self.active)
        try:
            deadline = time.monotonic() + 0.7
            while time.monotonic() < deadline:
                if handle.is_cancel_requested:
                    handle.canceled()
                    return NavigateToPose.Result()
                time.sleep(0.02)
            if self.abort_remaining:
                self.abort_remaining -= 1
                handle.abort()
            else:
                p = handle.request.pose.pose
                self.pose = {
                    "x": p.position.x,
                    "y": p.position.y,
                    "yaw": math.atan2(
                        2 * p.orientation.w * p.orientation.z, 1 - 2 * p.orientation.z**2
                    ),
                }
                handle.succeed()
            return NavigateToPose.Result()
        finally:
            self.active -= 1


def wait_until(predicate, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("Protocol condition timeout")


def main():
    assert (
        os.environ.get("ROS_DOMAIN_ID") == "42"
    ), "Use ROS_DOMAIN_ID=42 for isolated protocol testing"
    root = Path.cwd()
    out = root / "runs" / ("protocol_" + datetime.now().strftime("%Y%m%dT%H%M%S"))
    stations = root / "src/gasrobot_delivery/config/stations.yaml"
    config = root / "src/gasrobot_delivery/config"
    depot = yaml.safe_load(stations.read_text())["stations"]["depot"]
    args = ["--ros-args"]
    for k, v in {
        "stations_file": stations,
        "map_file": root / "src/gasrobot_gas_mapping/maps/workshop_delivery_v1.yaml",
        "batch_file": config / "batch_short.yaml",
        "params_file": config / "nav2_workshop.yaml",
        "output_dir": out,
        # 同时覆盖 Humble 字符串数组参数声明和额外配置快照。
        "snapshot_files": json.dumps([str(config / "nav2_workshop.yaml")]),
        "use_sim_time": "true",
    }.items():
        args += ["-p", f"{k}:={v}"]
    rclpy.init(args=args)
    fake, manager, client_node = FakeNavigation(depot), DeliveryManager(), Node("protocol_client")
    executor = MultiThreadedExecutor(num_threads=6)
    for node in [fake, manager, client_node]:
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    clients = {
        name: client_node.create_client(Trigger, "/delivery_manager/" + name)
        for name in ["start", "pause", "resume", "cancel"]
    }

    def call(name):
        assert clients[name].wait_for_service(timeout_sec=10)
        f = clients[name].call_async(Trigger.Request())
        wait_until(f.done)
        return f.result()

    checks = []
    try:
        wait_until(lambda: manager.ready()[0])
        assert call("start").success
        assert not call("start").success
        assert call("pause").success
        wait_until(lambda: manager.engine.state == "PAUSED")
        assert manager.engine.completed == []
        assert call("resume").success
        assert call("cancel").success
        wait_until(lambda: manager.engine.state == "CANCELLED")
        assert manager.engine.completed == []
        checks.append("delayed acceptance / duplicate start / navigation pause-resume-cancel")
        fake.pose = dict(depot)
        wait_until(lambda: manager.ready()[0])
        assert call("start").success
        wait_until(lambda: manager.engine.state == "SERVICING")
        assert call("pause").success
        remaining = manager.engine.remaining_service
        time.sleep(0.5)
        assert manager.engine.remaining_service == remaining
        assert call("resume").success
        wait_until(lambda: manager.engine.state == "COMPLETED")
        assert manager.engine.completed == ["S1", "S3", "S5"]
        checks.append("service pause preserves remaining / short batch and return")
        wait_until(lambda: manager.ready()[0])
        fake.abort_remaining = 2
        assert call("start").success
        wait_until(lambda: manager.engine.state == "FAILED")
        assert manager.engine.completed == []
        checks.append("unreachable retries exactly once and fails without false delivery")
        assert fake.max_active == 1
        out.mkdir(exist_ok=True)
        (out / "protocol_result.json").write_text(
            json.dumps({"passed": checks, "max_active_goals": fake.max_active}, indent=2)
        )
        print("PASS", out, checks, flush=True)
    finally:
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        for node in [client_node, manager, fake]:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
