#!/usr/bin/env python3
"""独占 Gazebo/Nav2 的时间标定器：每段原始样本保留，失败即停止。

只允许仿真。车辆由 NavigateToPose 驱动，Gazebo 真值只记录/检测碰撞，
不用于修正定位或反馈控制。连续采样采用到站固定朝向+停车+固定等待，
不传送机器人，不清除定位历史来隐藏失败。连续样本可能相关，记录轮次和种子。
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import signal
import time

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters, ListParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
import yaml

from delivery_tasks.config import load_stations, number
from delivery_planner.calibration import schedule, summarize
from delivery_planner.costs import digest, load_distance_cache, write_json


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def overlaps(x, y, a, box):
    """与现有验收相同的车体保守包络 SAT；只评价，不参与路径规划。"""
    bx, by, hx, hy = box
    c, s = math.cos(a), math.sin(a)
    for ux, uy in [(1, 0), (0, 1), (c, s), (-s, c)]:
        radius = 0.17 * abs(ux * c + uy * s) + 0.153 * abs(-ux * s + uy * c)
        radius += hx * abs(ux) + hy * abs(uy)
        if abs((x - bx) * ux + (y - by) * uy) >= radius - 0.005:
            return False
    return True


def flatten(raw, prefix=""):
    result = {}
    for name, value in raw.items():
        key = prefix + name
        if isinstance(value, dict):
            result.update(flatten(value, key + "."))
        else:
            result[key] = value
    return result


class Calibration(Node):
    def __init__(self, args, cache, book):
        super().__init__(
            "delivery_time_calibration",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.args, self.cache, self.book = args, cache, book
        self.output = args.output
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.odom, self.truth = None, None
        self.odom_received = self.truth_received = -1.0
        self.cmd, self.cmd_received = Twist(), -1.0
        self.collisions, self.samples = [], []
        self.current = None
        self.active_handle = None
        self.sending = None
        self.stop_requested = False
        self.last_clock = 0.0
        self.boxes = [(x, y, 1, 0.8) for x in [3.5, 6.5, 9.5] for y in [3, 7]]
        self.boxes += [
            (6, -0.075, 6.15, 0.075),
            (6, 10.075, 6.15, 0.075),
            (-0.075, 5, 0.075, 5),
            (12.075, 5, 0.075, 5),
            (1.5, 0.45, 0.7, 0.3),
        ]
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            ModelStates, "/gazebo/model_states", self.on_truth, qos_profile_sensor_data
        )
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.stream = (self.output / "trajectory.csv").open("w", newline="")
        self.writer = csv.writer(self.stream)
        self.writer.writerow(
            [
                "ros_time",
                "sample",
                "phase",
                "from",
                "to",
                "map_x",
                "map_y",
                "map_yaw",
                "odom_x",
                "odom_y",
                "linear_x",
                "angular_z",
                "world_x",
                "world_y",
                "world_yaw",
            ]
        )
        self.create_timer(0.1, self.trace)
        self.data = {
            "version": 1,
            "stations": cache["stations"],
            "depot": cache["depot"],
            "time_source": "independent_gazebo_navigation_ros_time_mean",
            "distance_content_sha256": cache["content_sha256"],
            "config_sha256": cache["config_sha256"],
            "samples": self.samples,
            "minimum_repeats": args.repeats,
            "seed": args.seed,
            "inter_leg_hold_sec": args.hold_sec,
            "timeout_sec": args.timeout_sec,
            "timing": "goal dispatch through success and >=0.25 s stopped; excludes service",
            "sampling": "continuous Euler circuits; fixed station headings; no teleport; correlated samples possible",
            "error": None,
            "collisions": self.collisions,
            "finished": False,
        }
        self.save()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        self.odom, self.odom_received = msg, self.now()

    def on_cmd(self, msg):
        self.cmd, self.cmd_received = msg, self.now()

    def on_truth(self, msg):
        if "delivery_robot" not in msg.name:
            return
        p = msg.pose[msg.name.index("delivery_robot")]
        self.truth, self.truth_received = [
            p.position.x,
            p.position.y,
            yaw(p.orientation),
        ], self.now()
        if any(overlaps(*self.truth, box) for box in self.boxes):
            if not self.collisions or self.now() - self.collisions[-1]["ros_time"] > 1:
                self.collisions.append(
                    {
                        "ros_time": self.now(),
                        "world_pose": self.truth,
                        "sample": None if self.current is None else self.current["sample"],
                    }
                )

    def pose(self):
        t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        if abs(self.now() - stamp) > 1:
            raise RuntimeError("stale localization transform")
        p = t.transform
        return [p.translation.x, p.translation.y, yaw(p.rotation)]

    def trace(self):
        if not self.current or self.odom is None:
            return
        try:
            pose = self.pose()
        except Exception:
            pose = ["", "", ""]
        r = self.current
        o, v = self.odom.pose.pose.position, self.odom.twist.twist
        self.writer.writerow(
            [
                self.now(),
                r["sample"],
                r["phase"],
                r["from"],
                r["to"],
                *pose,
                o.x,
                o.y,
                v.linear.x,
                v.angular.z,
                *(self.truth or ["", "", ""]),
            ]
        )
        self.stream.flush()

    def spin(self):
        rclpy.spin_once(self, timeout_sec=0.05)
        now = self.now()
        if now + 0.001 < self.last_clock:
            raise RuntimeError("ROS time jumped backwards; calibration invalid")
        self.last_clock = now
        if self.stop_requested:
            raise InterruptedError("operator requested stop")
        if self.collisions:
            raise RuntimeError("conservative ground-truth envelope overlaps obstacle")

    def wait_future(self, future, seconds=10):
        deadline = time.monotonic() + seconds
        while not future.done() and time.monotonic() < deadline:
            self.spin()
        if not future.done():
            raise TimeoutError("service/action handshake wall timeout")
        return future.result()

    def service(self, kind, name, request):
        client = self.create_client(kind, name)
        try:
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError(f"service unavailable: {name}")
            return self.wait_future(client.call_async(request))
        finally:
            self.destroy_client(client)

    def stopped(self):
        if self.odom is None or self.now() - self.odom_received > 1:
            return False
        v = self.odom.twist.twist
        return abs(v.linear.x) < 0.02 and abs(v.angular.z) < 0.04

    def station_error(self, name):
        p, desired = self.pose(), self.book["stations"][name]
        return math.hypot(p[0] - desired["x"], p[1] - desired["y"]), abs(
            angle(p[2] - desired["yaw"])
        )

    def ready(self):
        deadline = time.monotonic() + 120
        client = self.create_client(GetState, "/bt_navigator/get_state")
        try:
            while time.monotonic() < deadline:
                self.spin()
                if not client.service_is_ready() or self.truth is None or self.odom is None:
                    continue
                state = self.wait_future(client.call_async(GetState.Request()))
                if state.current_state.id == 3 and self.client.server_is_ready():
                    try:
                        self.pose()
                        return
                    except Exception:
                        pass
            raise RuntimeError("Nav2/TF/odom/Gazebo not ready")
        finally:
            self.destroy_client(client)

    def verify_runtime(self):
        """核验关键生效参数，而不只相信启动时传入的文件名。"""
        if any(name == "delivery_manager" for name, _ in self.get_node_names_and_namespaces()):
            raise RuntimeError("launch with enable_delivery:=false to avoid competing goals")
        raw = yaml.safe_load((self.args.distance.parent / "snapshots/navigation.yaml").read_text())
        runtime = {}
        for node in (
            "planner_server",
            "controller_server",
            "velocity_smoother",
            "global_costmap/global_costmap",
            "local_costmap/local_costmap",
            "map_server",
            "amcl",
        ):
            response = self.service(
                ListParameters, "/" + node + "/list_parameters", ListParameters.Request()
            )
            keys = sorted(response.result.names)
            response = self.service(
                GetParameters, "/" + node + "/get_parameters", GetParameters.Request(names=keys)
            )
            runtime[node] = dict(
                zip(keys, [parameter_value_to_python(p) for p in response.values])
            )
            if runtime[node].get("use_sim_time") is not True:
                raise RuntimeError(f"{node}: simulation time required")
            if node in ("map_server", "amcl"):
                continue
            section = raw
            for part in node.split("/"):
                section = section[part]
            section = section["ros__parameters"]
            # 配置保留了未启用的 static_layer；Nav2 不会声明未实例化插件的参数。
            # 仅排除显式不在 plugins 列表中的插件子树，其余生效参数仍逐项严格比对。
            inactive = {
                key
                for key, value in section.items()
                if isinstance(value, dict)
                and "plugin" in value
                and "plugins" in section
                and key not in section["plugins"]
            }
            for key, expected in flatten(section).items():
                if key.split(".")[0] in inactive:
                    continue
                if runtime[node].get(key) != expected:
                    raise RuntimeError(
                        f"{node}.{key}: active={runtime[node].get(key)!r}, expected={expected!r}"
                    )
        active_map = Path(runtime["map_server"]["yaml_filename"])
        if hashlib.sha256(active_map.read_bytes()).hexdigest() != self.book["map_yaml_sha256"]:
            raise RuntimeError("active map differs from station calibration")
        write_json(self.output / "runtime_parameters.json", runtime)
        self.data["runtime_parameters_sha256"] = digest(runtime)

    def hold(self, seconds):
        until, wall = self.now() + seconds, time.monotonic() + 600
        while self.now() < until:
            self.spin()
            if time.monotonic() > wall:
                raise TimeoutError("simulation clock stalled during hold")
            if not self.stopped():
                raise RuntimeError("robot moved during inter-leg hold")

    def cancel(self):
        """即使发生错误/外部中断也等待取消终态，之后不再发送任何目标。"""
        if self.sending is not None:
            # 中断可能发生于服务器接受之前；先接住句柄，再取消，避免留下迟到目标。
            future, self.sending = self.sending, None
            rclpy.spin_until_future_complete(self, future, timeout_sec=10)
            if not future.done():
                raise RuntimeError("goal acceptance unconfirmed; stop simulation before reuse")
            late_handle = future.result()
            if late_handle.accepted:
                self.active_handle = late_handle
        if self.active_handle is None:
            return
        handle, self.active_handle = self.active_handle, None
        future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=10)
        if not result.done():
            raise RuntimeError("cancel terminal state unconfirmed; stop the simulation launch")

    def navigate(self, source, target, round_index, phase="measure"):
        xy, ya = self.station_error(source)
        if xy > 0.10 or ya > 0.10 or not self.stopped():
            raise RuntimeError(f"origin {source} not stationary/in tolerance: {xy}, {ya}")
        start = self.now()
        record = {
            "sample": len(self.samples),
            "round": round_index,
            "phase": phase,
            "from": source,
            "to": target,
            "status": "in_progress",
            "start_ros_time": start,
            "start_map_pose": self.pose(),
            "start_world_pose": self.truth,
        }
        self.current = record
        self.samples.append(record)
        self.save()
        p = self.book["stations"][target]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = p["x"], p["y"]
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(
            p["yaw"] / 2
        ), math.cos(p["yaw"] / 2)
        try:
            self.sending = self.client.send_goal_async(goal)
            handle = self.wait_future(self.sending)
            self.sending = None
            if not handle.accepted:
                raise RuntimeError("goal rejected")
            self.active_handle = handle
            future = handle.get_result_async()
            wall, next_report = time.monotonic() + 900, time.monotonic() + 30
            while not future.done():
                self.spin()
                if self.now() - self.truth_received > 1 or self.now() - self.odom_received > 1:
                    raise RuntimeError("odometry or truth observation missing")
                if self.now() - start > self.args.timeout_sec:
                    raise TimeoutError("navigation ROS-time timeout")
                if time.monotonic() > wall:
                    raise TimeoutError("simulation stalled or too slow (wall watchdog)")
                if time.monotonic() > next_report:
                    print(
                        f"PROGRESS round={round_index} {source}->{target} elapsed={self.now()-start:.1f}s",
                        flush=True,
                    )
                    next_report = time.monotonic() + 30
            result = future.result()
            self.active_handle = None
            record["action_status"] = result.status
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f"navigation action failed: {result.status}")
            # 与配送管理器一致：成功回调之后连续静止 >=0.25 ROS 秒，再检查目标误差。
            stopped_since = None
            settle_deadline = self.now() + 10
            while True:
                self.spin()
                if self.now() > settle_deadline:
                    raise RuntimeError("did not stop after navigation success")
                if not self.stopped():
                    stopped_since = None
                elif stopped_since is None:
                    stopped_since = self.now()
                elif self.now() - stopped_since >= 0.25:
                    break
            xy, ya = self.station_error(target)
            if xy > 0.10 or ya > 0.10:
                raise RuntimeError(f"arrival outside tolerance: {xy}, {ya}")
            record.update(
                status="succeeded",
                travel_time_sec=self.now() - start,
                end_ros_time=self.now(),
                xy_error_m=xy,
                yaw_error_rad=ya,
                end_map_pose=self.pose(),
                end_world_pose=self.truth,
            )
            print(
                f'SAMPLE {record["sample"]} {source}->{target} {record["travel_time_sec"]:.3f}s',
                flush=True,
            )
        except BaseException as exc:
            record.update(
                status="failed",
                reason=str(exc),
                end_ros_time=self.now(),
                travel_time_sec=None,
                observed_elapsed_sec=self.now() - start,
                failure_kind=(
                    "navigation_timeout" if str(exc) == "navigation ROS-time timeout" else "other"
                ),
                completion_time_right_censored=(str(exc) == "navigation ROS-time timeout"),
            )
            self.cancel()
            raise
        finally:
            self.current = None
            self.save()

    def save(self):
        statistics = summarize(self.cache["stations"], self.samples, self.args.repeats)
        self.data.update(statistics)
        self.data["complete"] = (
            statistics["complete"]
            and self.data["finished"]
            and self.data["error"] is None
            and not self.collisions
        )
        self.data.pop("content_sha256", None)
        self.data["content_sha256"] = digest(self.data)
        write_json(self.output / "time_matrix.json", self.data)

    def run(self):
        self.ready()
        self.verify_runtime()
        self.hold(self.args.hold_sec)
        depot = self.book["depot"]
        if self.args.pilot:
            trials = [[depot, "S1", depot] for _ in range(self.args.repeats)]
        else:
            trials = [
                schedule(self.cache["stations"], depot, self.args.seed + k)
                for k in range(self.args.repeats)
            ]
        self.data["scheduled_routes"] = trials
        self.save()
        for k, route in enumerate(trials):
            for source, target in zip(route, route[1:]):
                self.navigate(source, target, k)
                self.hold(self.args.hold_sec)
        self.data["finished"] = True
        self.save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--distance", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--seed", type=int, default=20261011)
    parser.add_argument("--hold-sec", type=float, default=5.0)
    parser.add_argument("--timeout-sec", type=float, default=180.0)
    parser.add_argument(
        "--pilot", action="store_true", help="only depot-S1-depot, not a complete cache"
    )
    args, ros_args = parser.parse_known_args()
    if args.repeats < 2:
        parser.error("--repeats must be >=2")
    number(args.hold_sec, "hold_sec")
    number(args.timeout_sec, "timeout_sec", strict=True)
    args.distance = args.distance.resolve()
    cache = load_distance_cache(args.distance)
    # 全部输入绑定到距离归档；运行前再核对安装版本，以免旧归档配新模型。
    root = args.distance.parent
    book = load_stations(
        root / "snapshots/stations.yaml", root / "snapshots/map/workshop_delivery_v1.yaml"
    )
    files = {
        "stations.yaml": Path(get_package_share_directory("delivery_tasks"))
        / "config/stations.yaml",
        "navigation.yaml": Path(get_package_share_directory("delivery_tasks"))
        / "config/nav2_workshop.yaml",
    }
    for p in (root / "snapshots/description").rglob("*"):
        if p.is_file():
            files[str(p.relative_to(root / "snapshots"))] = Path(
                get_package_share_directory("delivery_description")
            ) / p.relative_to(root / "snapshots/description")
    for p in (root / "snapshots/simulation").rglob("*"):
        if p.is_file():
            files[str(p.relative_to(root / "snapshots"))] = Path(
                get_package_share_directory("delivery_simulation")
            ) / p.relative_to(root / "snapshots/simulation")
    for relative, current in files.items():
        if (root / "snapshots" / relative).read_bytes() != current.read_bytes():
            raise ValueError(f"current source differs from distance cache: {relative}")
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=ros_args)
    node = Calibration(args, cache, book)

    def stop(_sig, _frame):
        node.stop_requested = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    code = 0
    try:
        node.run()
    except BaseException as exc:
        node.data["error"] = str(exc)
        node.get_logger().error(str(exc))
        code = 1
    finally:
        try:
            node.cancel()
        finally:
            node.stream.flush()
            node.data["trajectory_sha256"] = hashlib.sha256(
                (args.output / "trajectory.csv").read_bytes()
            ).hexdigest()
            node.save()
            node.stream.close()
            node.destroy_node()
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
