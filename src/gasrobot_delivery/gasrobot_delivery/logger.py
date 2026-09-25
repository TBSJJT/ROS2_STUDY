"""每批独立的实验记录。

配置在开始时复制，避免随后修改批次文件影响结果解释；事件每条 flush，
即使任务失败也保留过程记录。summary 仅在终态写入，不提前声明任务成功。
"""

import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import uuid


class RunLogger:
    def __init__(self, root, files, batch, started):
        self.path = Path(root) / (
            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S") + "_" + uuid.uuid4().hex[:8]
        )
        self.path.mkdir(parents=True, exist_ok=False)
        hashes = {}
        for name, path in files.items():
            path = Path(path)
            hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
            shutil.copyfile(path, self.path / name)
        (self.path / "metadata.json").write_text(
            json.dumps(
                {
                    "batch_id": batch.batch_id,
                    "started_ros_time": started,
                    "sha256": hashes,
                    "clock": "ROS simulation time",
                    "distance_source": "wheel odometry increments",
                },
                indent=2,
            )
        )
        self.events_file = (self.path / "events.csv").open("w", newline="")
        self.events = csv.DictWriter(
            self.events_file,
            fieldnames=["batch_id", "ros_time", "elapsed", "station", "event", "state", "details"],
        )
        self.events.writeheader()
        self.trajectory_file = (self.path / "trajectory.csv").open("w", newline="")
        self.trajectory = csv.writer(self.trajectory_file)
        self.trajectory.writerow(
            ["ros_time", "map_x", "map_y", "map_yaw", "odom_x", "odom_y", "linear_x", "angular_z"]
        )
        self.batch = batch
        self.closed = False

    def event(self, entry):
        row = {key: entry[key] for key in ["ros_time", "elapsed", "station", "event", "state"]}
        row["batch_id"] = self.batch.batch_id
        row["details"] = json.dumps({k: v for k, v in entry.items() if k not in row})
        self.events.writerow(row)
        self.events_file.flush()

    def pose(self, row):
        self.trajectory.writerow(row)
        self.trajectory_file.flush()

    def finish(self, engine, now, distance):
        (self.path / "summary.json").write_text(
            json.dumps(
                {
                    "batch_id": self.batch.batch_id,
                    "state": engine.state,
                    "success": engine.state == "COMPLETED",
                    "completed_stations": engine.completed,
                    "elapsed_sec": max(0, now - engine.started),
                    "odom_distance_m": distance,
                    "returned_to_depot": engine.state == "COMPLETED",
                },
                indent=2,
            )
        )
        self.events_file.close()
        self.trajectory_file.close()
        self.closed = True
