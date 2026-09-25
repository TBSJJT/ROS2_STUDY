#!/usr/bin/env python3
"""将同一地图上的运行轨迹叠加为 PNG，供验收记录和后续实验复查。

轨迹来自 AMCL 的 map 位姿，不能用这条折线代替轮式里程计距离或 Gazebo 真值。
例如：python3 src/gasrobot_delivery/scripts/plot_runs.py runs/批次目录 --output /tmp/run.png
"""
import argparse
import csv
import json
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/gasrobot_delivery_matplotlib")

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.transforms import Affine2D  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
import yaml  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    first = args.runs[0]
    hashes = json.loads((first / "metadata.json").read_text())["sha256"]
    meta = yaml.safe_load((first / "map.yaml").read_text())
    stations = yaml.safe_load((first / "stations.yaml").read_text())["stations"]
    grid = np.asarray(Image.open(first / Path(meta["image"]).name).convert("L"))
    width, height = grid.shape[1] * meta["resolution"], grid.shape[0] * meta["resolution"]
    ox, oy, yaw = meta["origin"]
    transform = Affine2D().rotate(yaw).translate(ox, oy)
    fig, ax = plt.subplots(figsize=(10, 8), constrained_layout=True)
    ax.imshow(
        grid,
        cmap="gray",
        vmin=0,
        vmax=255,
        origin="upper",
        extent=(0, width, 0, height),
        transform=transform + ax.transData,
    )
    for index, directory in enumerate(args.runs, start=1):
        current = json.loads((directory / "metadata.json").read_text())["sha256"]
        if any(current[key] != hashes[key] for key in ("map.yaml", "stations.yaml")):
            raise ValueError("Overlay requires identical map and station configurations")
        with (directory / "trajectory.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        summary = json.loads((directory / "summary.json").read_text())
        ax.plot(
            [float(row["map_x"]) for row in rows],
            [float(row["map_y"]) for row in rows],
            linewidth=1.3,
            alpha=0.85,
            label=f"Batch {index}: {summary['state']}, {summary['elapsed_sec']:.1f} s",
        )
    for name, pose in stations.items():
        ax.plot(pose["x"], pose["y"], "o", color="#b2182b", markersize=4)
        ax.annotate(
            name,
            (pose["x"], pose["y"]),
            xytext=(5, 7),
            textcoords="offset points",
            fontsize=9,
            fontweight="bold",
        )
        ax.arrow(
            pose["x"],
            pose["y"],
            0.28 * np.cos(pose["yaw"]),
            0.28 * np.sin(pose["yaw"]),
            width=0.012,
            color="#b2182b",
            zorder=4,
        )
    corners = transform.transform([[0, 0], [width, 0], [0, height], [width, height]])
    ax.set_xlim(corners[:, 0].min(), corners[:, 0].max())
    ax.set_ylim(corners[:, 1].min(), corners[:, 1].max())
    ax.set_aspect("equal")
    ax.set(xlabel="map x (m)", ylabel="map y (m)", title="Workshop delivery — AMCL trajectories")
    ax.legend(loc="upper center", fontsize=8, framealpha=0.95)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)
    print(args.output)


if __name__ == "__main__":
    main()
