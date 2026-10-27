#!/usr/bin/env python3
"""采集 42 个有向 Nav2 规划结果；只发送 ComputePathToPose，不导航。

Humble 的 ComputePathToPose 结果没有新版本中的 error_code 字段，故使用
动作终态、路径坐标和首尾点检查判断成功。所有失败保留，不能以直线补齐。
运行过程保存原始路径和代价栅格，规划耗时与行驶时间是不同量。
"""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from delivery_tasks.config import mapping
from delivery_planner.costs import (
    assemble_matrix,
    digest,
    path_length,
    write_json,
    check_plan_trials,
)


class Builder(Node):
    def __init__(self, output, book, planner_id):
        super().__init__("delivery_cost_matrix_builder")
        self.output, self.book, self.planner_id = output, book, planner_id
        self.client = ActionClient(self, ComputePathToPose, "/cost_matrix/compute_path_to_pose")
        self.lifecycle = self.create_client(GetState, "/cost_matrix/planner_server/get_state")
        self.costmap_client = self.create_client(
            GetCostmap, "/cost_matrix/global_costmap/get_costmap"
        )
        manifest = json.loads((output / "provenance.json").read_text())
        self.names = [book["depot"], *[name for name in book["stations"] if name != book["depot"]]]
        self.data = {
            "version": 1,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "stations": self.names,
            "depot": book["depot"],
            "frame_id": "map",
            "distance_source": "nav2_static_planned_path",
            "distance_unit": "m",
            "time_source": None,
            "travel_time_sec": None,
            "planner_id": planner_id,
            "config_sha256": manifest["config_sha256"],
            "complete": False,
            "all_reachable": False,
            "planning_consistency": {
                "repeats": 3,
                "epsilon_m": 1e-6,
                "require_identical_paths": True,
            },
            "pairs": [],
            "error": None,
        }
        self.save()

    def wait(self, future, seconds=30):
        deadline = time.monotonic() + seconds
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError("wall-clock timeout waiting for static planner")
        return future.result()

    def ready(self):
        """服务可见不等于激活；静态规划用墙钟，不等待不存在的 Gazebo 时钟。"""
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.lifecycle.service_is_ready():
                continue
            state = self.wait(self.lifecycle.call_async(GetState.Request()), 5)
            if (
                state.current_state.id == 3
                and self.client.server_is_ready()
                and self.costmap_client.service_is_ready()
            ):
                return
        raise TimeoutError("static planner lifecycle did not become active")

    def read_costmap(self):
        """排除时间戳后哈希整个栅格；采集期间代价变化将使整次缓存无效。"""
        costmap = self.wait(self.costmap_client.call_async(GetCostmap.Request())).map
        meta, origin = costmap.metadata, costmap.metadata.origin
        if costmap.header.frame_id != "map" or not meta.size_x or not meta.size_y:
            raise ValueError("static costmap is empty or has wrong frame")
        return {
            "frame_id": costmap.header.frame_id,
            "resolution": meta.resolution,
            "size_x": meta.size_x,
            "size_y": meta.size_y,
            "origin": [
                origin.position.x,
                origin.position.y,
                origin.position.z,
                origin.orientation.x,
                origin.orientation.y,
                origin.orientation.z,
                origin.orientation.w,
            ],
            "data": list(costmap.data),
        }

    def pose(self, name):
        point = self.book["stations"][name]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = point["x"], point["y"]
        pose.pose.orientation.z = math.sin(point["yaw"] / 2)
        pose.pose.orientation.w = math.cos(point["yaw"] / 2)
        return pose

    def plan(self, source, target):
        goal = ComputePathToPose.Goal()
        goal.start, goal.goal = self.pose(source), self.pose(target)
        goal.use_start, goal.planner_id = True, self.planner_id
        record = {"from": source, "to": target, "status": "rejected", "path_xy_m": []}
        handle = self.wait(self.client.send_goal_async(goal))
        if not handle.accepted:
            return record
        try:
            result = self.wait(handle.get_result_async())
        except TimeoutError:
            # 超时后先请求取消；若取消不确认则抛异常结束整个专用规划器，不再发目标。
            self.wait(handle.cancel_goal_async(), 5)
            raise
        record["action_status"] = result.status
        record["status"] = "planning_failed"
        path = result.result.path
        duration = result.result.planning_time
        record["planning_wall_sec"] = duration.sec + duration.nanosec * 1e-9
        record["path_xy_m"] = [[p.pose.position.x, p.pose.position.y] for p in path.poses]
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return record
        try:
            if path.header.frame_id != "map":
                raise ValueError("path frame is not map")
            length = path_length(record["path_xy_m"])
            for name, xy in [(source, record["path_xy_m"][0]), (target, record["path_xy_m"][-1])]:
                p = self.book["stations"][name]
                if math.dist(xy, [p["x"], p["y"]]) > 0.15:
                    raise ValueError("path endpoint differs from requested station by >0.15 m")
            record.update(status="reachable", distance_m=length)
        except ValueError as exc:
            record.update(status="invalid_path", reason=str(exc))
        return record

    def save(self):
        self.data["distance_m"] = assemble_matrix(self.names, self.data["pairs"])
        # 自身哈希不参与哈希计算，避免递归定义。
        self.data.pop("content_sha256", None)
        self.data["content_sha256"] = digest(self.data)
        write_json(self.output / "matrix.json", self.data)

    def run(self):
        self.ready()
        costmap = self.read_costmap()
        write_json(self.output / "costmap.json", costmap)
        self.data["costmap_sha256"] = digest(costmap)
        for source in self.names:
            for target in self.names:
                if source == target:
                    continue
                if digest(self.read_costmap()) != self.data["costmap_sha256"]:
                    raise RuntimeError(
                        "costmap changed during collection; discard incomplete cache"
                    )
                trials = []
                for _ in range(3):
                    if digest(self.read_costmap()) != self.data["costmap_sha256"]:
                        raise RuntimeError("costmap changed between planning repeats")
                    trials.append(self.plan(source, target))
                checked = check_plan_trials(trials)
                record = dict(trials[0])
                record.update(planning_trials=trials, consistency=checked)
                if not checked["passed"]:
                    record["status"] = "inconsistent_or_failed_planning"
                    record.pop("distance_m", None)
                self.data["pairs"].append(record)
                self.save()
                print(
                    f'{source} -> {target}: {record["status"]} '
                    f'{record.get("distance_m", "null")} m',
                    flush=True,
                )
        if digest(self.read_costmap()) != self.data["costmap_sha256"]:
            raise RuntimeError("costmap changed after final planning request")
        self.data["complete"] = True
        self.data["all_reachable"] = all(p["status"] == "reachable" for p in self.data["pairs"])
        self.save()
        with (self.output / "distance_m.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["from/to", *self.names])
            for name, row in zip(self.names, self.data["distance_m"]):
                writer.writerow([name, *["unavailable" if v is None else v for v in row]])
        print(f"Matrix saved: {self.output / 'matrix.json'}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stations", type=Path, required=True)
    parser.add_argument("--planner-id", default="GridBased")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Builder(args.output, mapping(args.stations), args.planner_id)
    try:
        node.run()
        return 0 if node.data["all_reachable"] else 2
    except Exception as exc:
        node.data["error"] = str(exc)
        node.data["complete"] = False
        node.save()
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
