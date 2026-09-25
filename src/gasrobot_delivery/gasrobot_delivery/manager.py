#!/usr/bin/env python3
"""配送状态机的 ROS 适配层。

本节点只发送 NavigateToPose 目标，不发布 /cmd_vel，不订阅 Gazebo 真值。
所有回调由单线程 executor 串行执行，状态机无需锁；Action 请求是异步的。
状态机令牌覆盖发送、接受、执行、取消全过程，用于拒绝上一目标的迟到回调。
"""
import json
import math
import yaml
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from gasrobot_delivery.config import load_batch, load_stations
from gasrobot_delivery.engine import DeliveryEngine
from gasrobot_delivery.logger import RunLogger


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class DeliveryManager(Node):
    def __init__(self):
        super().__init__("delivery_manager")
        self.paths = {}
        for key in ("stations_file", "batch_file", "map_file", "params_file"):
            self.paths[key] = self.declare_parameter(key, "").value
        # Humble 把空列表推断为 BYTE_ARRAY；明确声明字符串数组，兼容启动文件覆盖。
        self.snapshot_files = (
            self.declare_parameter("snapshot_files", Parameter.Type.STRING_ARRAY).value or []
        )
        self.output_dir = self.declare_parameter("output_dir", str(Path.cwd() / "runs")).value
        self.book = load_stations(self.paths["stations_file"], self.paths["map_file"])
        self.stations = self.book["stations"]
        self.batch = load_batch(self.paths["batch_file"], self.stations, self.book["depot"])
        self.engine = DeliveryEngine(self.batch, self.book["depot"])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.handles = {}
        self.cancel_pending = set()
        self.pending_results = {}
        self.add_on_set_parameters_callback(self.parameters_changed)
        self.lifecycle_client = self.create_client(GetState, "/bt_navigator/get_state")
        self.lifecycle_future = None
        self.nav_active = False
        self.lifecycle_checked = -10.0
        self.logger = None
        self.last_run = ""
        self.odom = None
        self.odom_received = 0.0
        self.previous_odom = None
        self.distance = 0.0
        self.last_status = -1.0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "~/status", qos)
        self.marker_pub = self.create_publisher(MarkerArray, "~/stations", qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        for name in ("start", "pause", "resume", "cancel"):
            self.create_service(Trigger, "~/" + name, self.service(name))
        self.create_timer(0.05, self.tick)
        self.create_timer(2.0, self.markers)
        self.markers()
        self.get_logger().info("Ready for explicit start; no delivery starts automatically")

    def parameters_changed(self, parameters):
        for parameter in parameters:
            if parameter.name != "batch_file":
                return SetParametersResult(
                    successful=False, reason="Restart to change this parameter"
                )
            if self.engine.busy:
                return SetParametersResult(
                    successful=False, reason="Cannot change batch while active"
                )
            try:
                load_batch(parameter.value, self.stations, self.book["depot"])
            except (OSError, ValueError, TypeError) as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose(self):
        t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        if abs(self.now() - stamp) > 2.0:
            raise RuntimeError("stale map-to-robot transform")
        p = t.transform
        return p.translation.x, p.translation.y, yaw(p.rotation)

    def ready(self):
        """出发前检查生命周期、定位新鲜度、静止状态及装货地点。"""
        if not self.nav_active:
            return False, "Nav2 lifecycle is not active"
        if not self.client.server_is_ready():
            return False, "Nav2 action server is not ready"
        try:
            x, y, _ = self.pose()
        except Exception as exc:
            return False, f"Localization unavailable: {exc}"
        if self.odom is None or self.now() - self.odom_received > 2.0:
            return False, "Fresh odometry required"
        v = self.odom.twist.twist
        if abs(v.linear.x) > 0.02 or abs(v.angular.z) > 0.04:
            return False, "Robot must be stopped"
        p = self.stations[self.book["depot"]]
        if math.hypot(x - p["x"], y - p["y"]) > 0.35:
            return False, "Start requires robot at depot (within 0.35 m)"
        return True, "ready"

    def service(self, name):
        """生成 Trigger 服务回调；start 是一次装货确认，不是自动启动开关。"""

        def callback(_request, response):
            now = self.now()
            if name == "start":
                if self.engine.busy:
                    response.success, response.message = False, "Task already active"
                    return response
                ok, reason = self.ready()
                if not ok:
                    response.success, response.message = False, reason
                    return response
                try:
                    # Reload only while idle, allowing a different batch between runs.
                    self.paths["batch_file"] = self.get_parameter("batch_file").value
                    batch = load_batch(self.paths["batch_file"], self.stations, self.book["depot"])
                    files = {
                        "stations.yaml": self.paths["stations_file"],
                        "batch.yaml": self.paths["batch_file"],
                        "map.yaml": self.paths["map_file"],
                        "nav2_params.yaml": self.paths["params_file"],
                    }
                    # 同时保留仿真动力学与世界文件，便于解释同一地图上的行为差异。
                    for extra in self.snapshot_files:
                        files["simulation_" + Path(extra).name] = extra
                    meta = yaml.safe_load(Path(self.paths["map_file"]).read_text())
                    files[Path(meta["image"]).name] = str(
                        Path(self.paths["map_file"]).parent / meta["image"]
                    )
                    self.logger = RunLogger(self.output_dir, files, batch, now)
                    self.last_run = str(self.logger.path)
                    self.engine.batch = self.batch = batch
                    self.distance = 0.0
                    self.previous_odom = None
                except (OSError, ValueError, TypeError) as exc:
                    response.success, response.message = False, str(exc)
                    return response
            response.success, response.message = getattr(self.engine, name)(now)
            self.flush()
            return response

        return callback

    def on_odom(self, msg):
        """积分连续轮式位移；不使用 AMCL 修正后的 map 跳变计算距离。"""
        self.odom = msg
        self.odom_received = self.now()
        p = msg.pose.pose.position
        if self.logger and not self.logger.closed:
            if self.previous_odom is not None:
                self.distance += math.hypot(
                    p.x - self.previous_odom[0], p.y - self.previous_odom[1]
                )
            self.previous_odom = (p.x, p.y)

    def dispatch(self, token, station):
        """异步发目标；暂停可能发生在服务器接受目标之前。"""
        p = self.stations[station]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = p["x"], p["y"]
        goal.pose.pose.orientation.z = math.sin(p["yaw"] / 2)
        goal.pose.pose.orientation.w = math.cos(p["yaw"] / 2)
        future = self.client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.accepted(token, station, f))

    def accepted(self, token, station, future):
        """保存目标句柄；若发送期间已收到取消，接受后立刻补发取消。"""
        try:
            handle = future.result()
        except Exception as exc:
            if token != self.engine.active:
                return  # 旧批次的异常同样不能改变当前任务。
            self.engine.finish("FAILED", self.now(), f"Goal transport error: {exc}")
            self.flush()
            return
        if not handle.accepted:
            self.cancel_pending.discard(token)
            self.engine.nav_result(token, False, self.now(), "goal rejected")
        else:
            self.handles[token] = handle
            handle.get_result_async().add_done_callback(lambda f: self.result(token, station, f))
            if token in self.cancel_pending or token != self.engine.active:
                handle.cancel_goal_async()
        self.flush()

    def result(self, token, station, future):
        """接收 Action 终态，先等待物理停车，再允许卸货或后续动作。"""
        self.handles.pop(token, None)
        self.cancel_pending.discard(token)
        try:
            status = future.result().status
        except Exception as exc:
            if token != self.engine.active:
                return
            self.engine.finish("FAILED", self.now(), f"Result transport error: {exc}")
            self.flush()
            return
        if token == self.engine.active:
            self.pending_results[token] = (station, status, None)
        self.finish_results()
        self.flush()

    def finish_results(self):
        """连续静止 0.25 秒后检查实际 map 位姿，提交最终导航结果。"""
        for token, (station, status, stopped_since) in list(self.pending_results.items()):
            if token != self.engine.active:
                self.pending_results.pop(token)
                continue
            # Goal results and cancellation callbacks can precede physical stopping.
            v = self.odom.twist.twist if self.odom else None
            stopped = (
                v is not None
                and self.now() - self.odom_received < 1.0
                and abs(v.linear.x) < 0.02
                and abs(v.angular.z) < 0.04
            )
            if not stopped:
                self.pending_results[token] = (station, status, None)
                continue
            if stopped_since is None:
                self.pending_results[token] = (station, status, self.now())
                continue
            if self.now() - stopped_since < 0.25:
                continue
            self.pending_results.pop(token)
            self.cancel_pending.discard(token)
            success = status == GoalStatus.STATUS_SUCCEEDED
            reason = f"Nav2 status {status}"
            try:
                x, y, a = self.pose()
                p = self.stations[station]
                xy = math.hypot(x - p["x"], y - p["y"])
                angle = abs(math.atan2(math.sin(a - p["yaw"]), math.cos(a - p["yaw"])))
                self.engine.event(
                    "navigation_result",
                    self.now(),
                    nav_status=status,
                    xy_error=xy,
                    yaw_error=angle,
                )
                if success and (xy > 0.10 or angle > 0.10):
                    success, reason = False, "arrival pose outside 0.10 m / 0.10 rad tolerance"
            except Exception:
                success, reason = False, "no fresh localization at arrival"
            self.engine.nav_result(token, success, self.now(), reason)

    def flush(self):
        """执行状态机命令并落盘事件；终态写 summary，失败记录也保留。"""
        while self.engine.commands:
            kind, token, station = self.engine.commands.pop(0)
            if kind == "navigate":
                self.dispatch(token, station)
            else:
                self.cancel_pending.add(token)
                if token in self.handles:
                    self.handles[token].cancel_goal_async()
        while self.engine.events:
            event = self.engine.events.pop(0)
            self.get_logger().info(json.dumps(event, ensure_ascii=False))
            if self.logger and not self.logger.closed:
                self.logger.event(event)
        if self.logger and not self.logger.closed and self.engine.state in self.engine.TERMINAL:
            self.logger.finish(self.engine, self.now(), self.distance)

    def tick(self):
        """以仿真时钟推进状态机，周期检查导航生命周期并发布状态。"""
        now = self.now()
        if now < self.last_status:
            # 时钟回退后仍应及时发布失败状态，不能等旧时间戳追平。
            self.last_status = -1.0
            self.lifecycle_checked = -10.0
            self.nav_active = False
        if now - self.lifecycle_checked > 1 and self.lifecycle_future is None:
            self.lifecycle_checked = now
            if self.lifecycle_client.service_is_ready():
                self.lifecycle_future = self.lifecycle_client.call_async(GetState.Request())
        if self.lifecycle_future is not None and self.lifecycle_future.done():
            try:
                self.nav_active = self.lifecycle_future.result().current_state.id == 3
            except Exception:
                self.nav_active = False
            self.lifecycle_future = None
        self.finish_results()
        self.engine.tick(now)
        self.flush()
        if now - self.last_status < 0.5:
            return
        self.last_status = now
        p = None
        try:
            p = self.pose()
        except Exception:
            pass
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "state": self.engine.state,
                        "batch_id": self.batch.batch_id,
                        "target": self.engine.target,
                        "completed": self.engine.completed,
                        "navigation_pending": self.engine.active is not None,
                        "service_remaining_sec": self.engine.remaining_service,
                        "run_dir": self.last_run,
                        "map_pose": p,
                        "ros_time": now,
                    }
                )
            )
        )
        if self.logger and not self.logger.closed and self.odom and p:
            o = self.odom.pose.pose.position
            v = self.odom.twist.twist
            self.logger.pose([now, *p, o.x, o.y, v.linear.x, v.angular.z])

    def markers(self):
        """用箭头显示交付朝向，文字显示工位 ID；地标只用于可视化。"""
        array = MarkerArray()
        for index, (name, p) in enumerate(self.stations.items()):
            for text in (False, True):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns, marker.id = "stations", index * 2 + int(text)
                marker.type = Marker.TEXT_VIEW_FACING if text else Marker.ARROW
                marker.action = Marker.ADD
                marker.pose.position.x, marker.pose.position.y = p["x"], p["y"]
                marker.pose.position.z = 0.45 if text else 0.05
                marker.pose.orientation.z, marker.pose.orientation.w = math.sin(
                    p["yaw"] / 2
                ), math.cos(p["yaw"] / 2)
                marker.scale.x, marker.scale.y, marker.scale.z = (
                    (0.4, 0.07, 0.09) if not text else (0.0, 0.0, 0.25)
                )
                marker.color.r = 0.1 if name == self.book["depot"] else 1.0
                marker.color.g, marker.color.b, marker.color.a = 0.8, 0.1, 1.0
                marker.text = name
                array.markers.append(marker)
        self.marker_pub.publish(array)


def main():
    import signal
    import time
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = [False]

    def stop(_signum, _frame):
        stopping[0] = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = DeliveryManager()
    try:
        while rclpy.ok() and not stopping[0]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node.engine.busy:
            node.engine.cancel(node.now())
            node.flush()
            deadline = time.monotonic() + 5
            while node.engine.active is not None and time.monotonic() < deadline and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
            if node.engine.state not in node.engine.TERMINAL:
                node.engine.finish("FAILED", node.now(), "shutdown before cancellation confirmed")
                node.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
