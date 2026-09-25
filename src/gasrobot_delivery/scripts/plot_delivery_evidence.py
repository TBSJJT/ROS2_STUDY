#!/usr/bin/env python3
"""从逐段原始 Gazebo 采样合并整批轨迹；不平滑、不按工位插值造路径。

输出每段 CSV/图、合并 CSV/图、AMCL 对照图以及逐段首帧规划图。
规划图明确标为 planned；真实行驶图只读取 actual_samples.csv 的真值列。
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/gasrobot_delivery_matplotlib")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
import numpy as np
from PIL import Image
import yaml

# 禁止绘图库简化折线，逐点保留输入采样。连线仅连接相邻实测点。
plt.rcParams["path.simplify"] = False
COLORS = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#882255"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    out = args.directory
    run = Path(json.loads((out / "run_reference.json").read_text())["run_dir"])
    # 素材归档搬到另一台机器后，优先用同包配置快照。
    if (out / "run_record").exists():
        run = out / "run_record"
    legs = json.loads((out / "legs.json").read_text())
    with (out / "actual_samples.csv").open() as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames
        rows = list(reader)
    assert rows, "No measured samples"
    indices = [int(row["sample_index"]) for row in rows]
    assert indices == list(range(1, len(rows) + 1)), "Missing or reordered samples"
    times = np.array([float(r["receipt_ros_time"]) for r in rows])
    assert np.all(np.diff(times) >= 0), "ROS clock rolled back: do not bridge trajectories"
    meta = yaml.safe_load((run / "map.yaml").read_text())
    book = yaml.safe_load((run / "stations.yaml").read_text())
    grid = np.asarray(Image.open(run / Path(meta["image"]).name))
    width, height = grid.shape[1] * meta["resolution"], grid.shape[0] * meta["resolution"]
    ox, oy, angle = meta["origin"]
    transform = Affine2D().rotate(angle).translate(ox, oy)

    def canvas(title):
        fig, ax = plt.subplots(figsize=(11, 9), constrained_layout=True)
        ax.imshow(
            grid,
            cmap="gray",
            vmin=0,
            vmax=255,
            origin="upper",
            extent=(0, width, 0, height),
            transform=transform + ax.transData,
        )
        for name, pose in book["stations"].items():
            ax.plot(pose["x"], pose["y"], "o", color="#333333", markersize=4)
            ax.annotate(
                name,
                (pose["x"], pose["y"]),
                xytext=(5, 8),
                textcoords="offset points",
                fontweight="bold",
            )
        # imshow 的仿射变换不自动更新数据范围，必须用变换后的四角设置边界。
        # 否则 map 原点为负时，自动缩放会裁掉左侧及下侧墙体。
        corners = transform.transform([[0, 0], [width, 0], [0, height], [width, height]])
        ax.set_xlim(corners[:, 0].min(), corners[:, 0].max())
        ax.set_ylim(corners[:, 1].min(), corners[:, 1].max())
        ax.set_aspect("equal")
        ax.set(xlabel="map x (m)", ylabel="map y (m)", title=title)
        return fig, ax

    def measured(ax, part, color, label):
        # 采样缺口超过 0.25 ROS 秒则断线，不能悄悄跨缺口补造连续运动。
        x, y = [], []
        previous = None
        for r in part:
            now = float(r["receipt_ros_time"])
            if previous is not None and now - previous > 0.25:
                x.append(float("nan"))
                y.append(float("nan"))
            x.append(float(r["truth_map_x"]))
            y.append(float(r["truth_map_y"]))
            previous = now
        ax.plot(x, y, color=color, linewidth=1.5, label=label)

    segment_dir = out / "segments"
    segment_dir.mkdir(exist_ok=True)
    merged = []
    stats = []
    full_fig, full_ax = canvas("Measured Gazebo trajectory — seven recorded legs merged")
    first_plans = {}
    for line in (out / "plans.jsonl").read_text().splitlines():
        plan = json.loads(line)
        first_plans.setdefault(plan["leg"], plan)
    plan_fig, plan_ax = canvas(
        "First recorded Nav2 plan per leg — planned, not executed trajectory"
    )
    for leg in legs:
        number = leg["index"]
        part = [r for r in rows if int(r["leg"]) == number]
        assert part, f"No measured samples for leg {number}"
        label = f"{number}: {leg['source']} → {leg['target']}"
        stem = f"{number:02d}_{leg['source']}_to_{leg['target']}"
        csv_path = segment_dir / (stem + ".csv")
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(part)
        # 整批结果从刚保存的七份段文件按原序读回，而非另造一条整批折线。
        with csv_path.open() as f:
            reread = list(csv.DictReader(f))
        assert reread == part
        merged.extend(reread)
        color = COLORS[(number - 1) % len(COLORS)]
        fig, ax = canvas("Measured Gazebo trajectory — " + label)
        measured(ax, reread, color, label)
        ax.legend(loc="upper center", fontsize=9)
        fig.savefig(segment_dir / (stem + ".png"), dpi=200)
        plt.close(fig)
        measured(full_ax, reread, color, label)
        points = [(float(r["world_x"]), float(r["world_y"])) for r in part]
        stats.append(
            dict(
                index=number,
                source=leg["source"],
                target=leg["target"],
                sample_count=len(part),
                first_sample=int(part[0]["sample_index"]),
                last_sample=int(part[-1]["sample_index"]),
                within_leg_world_distance_m=sum(
                    math.dist(a, b) for a, b in zip(points, points[1:])
                ),
                csv_sha256=hashlib.sha256(csv_path.read_bytes()).hexdigest(),
            )
        )
        if number in first_plans:
            xy = np.array(first_plans[number]["points"])
            plan_ax.plot(xy[:, 0], xy[:, 1], color=color, linewidth=1.3, label=label)
    assert merged == rows, "Merged leg records do not match the original continuous log"
    with (out / "merged_actual_trajectory.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(merged)
    for suffix in ("png", "pdf"):
        full_ax.legend(loc="upper center", ncol=3, fontsize=8)
        full_fig.savefig(out / ("actual_trajectory_merged." + suffix), dpi=220)
    # 对照图明确区分真实位置经固定配准后的轨迹与 AMCL 定位估计。
    amcl = [r for r in rows if r["amcl_x"] and r["amcl_y"]]
    full_ax.plot(
        [float(r["amcl_x"]) for r in amcl],
        [float(r["amcl_y"]) for r in amcl],
        color="#333333",
        linestyle="--",
        linewidth=0.8,
        alpha=0.7,
        label="AMCL estimate",
    )
    full_ax.set_title("Registered Gazebo measurements vs AMCL position estimates")
    full_ax.legend(loc="upper center", ncol=3, fontsize=8)
    full_fig.savefig(out / "truth_vs_amcl.png", dpi=220)
    plt.close(full_fig)
    plan_ax.legend(loc="upper center", ncol=3, fontsize=8)
    plan_fig.savefig(out / "planned_segments.png", dpi=220)
    plt.close(plan_fig)
    points = [(float(r["world_x"]), float(r["world_y"])) for r in rows]
    report = dict(
        source="gazebo/model_states, model gasrobot",
        time_basis="ROS clock on receipt; ModelStates has no header",
        world_to_map=book["registration"]["world_to_map"],
        sample_count=len(rows),
        first_receipt_ros_time=float(times[0]),
        last_receipt_ros_time=float(times[-1]),
        max_sample_gap_sec=float(np.max(np.diff(times))),
        gaps_over_025_sec=int(np.count_nonzero(np.diff(times) > 0.25)),
        measured_world_distance_m=sum(math.dist(a, b) for a, b in zip(points, points[1:])),
        leg_count=len(legs),
        legs=stats,
        merged_rows_equal_original=True,
        original_sha256=hashlib.sha256((out / "actual_samples.csv").read_bytes()).hexdigest(),
        merged_sha256=hashlib.sha256(
            (out / "merged_actual_trajectory.csv").read_bytes()
        ).hexdigest(),
        smoothing=False,
        synthetic_station_connections=False,
    )
    (out / "trajectory_provenance.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
