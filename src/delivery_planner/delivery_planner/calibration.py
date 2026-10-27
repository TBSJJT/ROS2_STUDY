"""时间标定的调度、统计和数据绑定，核心不依赖 ROS。

每轮使用有向完全图的欧拉回路：每条边恰好走一次且最终回到物料站。
固定种子让采样顺序可复查，各轮更换顺序减少固定次序的影响；并不声称
连续运行的样本独立同分布。任何失败均保留，正式缓存默认要求每边五次成功且无失败。
"""

import json
import math
import hashlib
import random
import statistics
from pathlib import Path
from delivery_planner.costs import digest


def schedule(names, depot, seed):
    if len(names) < 2 or len(names) != len(set(names)) or depot not in names:
        raise ValueError("invalid stations for calibration")
    rng = random.Random(seed)
    edges = {a: [b for b in names if b != a] for a in names}
    for row in edges.values():
        rng.shuffle(row)
    stack, trail = [depot], []
    while stack:
        if edges[stack[-1]]:
            stack.append(edges[stack[-1]].pop())
        else:
            trail.append(stack.pop())
    return list(reversed(trail))


def summarize(names, samples, minimum_repeats=5):
    if type(minimum_repeats) is not int or minimum_repeats < 2:
        raise ValueError("at least two repeats are required to report sample standard deviation")
    if len(set(names)) != len(names) or len(names) < 2:
        raise ValueError("invalid station list")
    identifiers = set()
    for trial in samples:
        if trial["from"] not in names or trial["to"] not in names or trial["from"] == trial["to"]:
            raise ValueError("invalid directed sample pair")
        key = (trial.get("round"), trial["from"], trial["to"])
        if key in identifiers:
            raise ValueError("duplicate sample in same round")
        identifiers.add(key)
        if trial["status"] not in ("succeeded", "failed", "in_progress"):
            raise ValueError("unknown trial status")
        if trial["status"] == "succeeded":
            value = trial.get("travel_time_sec")
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError("successful samples need positive finite ROS durations")
    mean = [[0.0 if a == b else None for b in names] for a in names]
    rows = []
    for i, a in enumerate(names):
        for j, b in enumerate(names):
            if a == b:
                continue
            trials = [r for r in samples if r["from"] == a and r["to"] == b]
            values = [r["travel_time_sec"] for r in trials if r["status"] == "succeeded"]
            failures = sum(r["status"] == "failed" for r in trials)
            pending = sum(r["status"] == "in_progress" for r in trials)
            finished = len(values) + failures
            ordered = sorted(values)
            # 经验分位数采用线性插值：h=(n-1)*p，即常见的 type-7 定义。
            # 只有成功完成的时长可参与；超时不当成完成时长，也不抹掉失败。
            h = (len(ordered) - 1) * 0.95 if ordered else 0
            p95 = (
                (
                    ordered[math.floor(h)]
                    + (h - math.floor(h)) * (ordered[math.ceil(h)] - ordered[math.floor(h)])
                )
                if ordered
                else None
            )
            row = {
                "from": a,
                "to": b,
                "attempts": len(trials),
                "successes": len(values),
                "failures": failures,
                "pending": pending,
                "n": len(values),
                "completed_attempts": finished,
                "success_rate": len(values) / finished if finished else None,
                "p95_sec": p95,
                "quantile_method": "linear_type7",
                "duration_population": "successful_completions_only",
                "timeouts": sum(
                    r.get("failure_kind") == "navigation_timeout"
                    or r.get("reason") == "navigation ROS-time timeout"
                    for r in trials
                ),
                "mean_sec": statistics.mean(values) if values else None,
                "stddev_sec": statistics.stdev(values) if len(values) >= 2 else None,
                "min_sec": min(values) if values else None,
                "max_sec": max(values) if values else None,
            }
            row["usable"] = len(values) >= minimum_repeats and failures == 0 and pending == 0
            if row["usable"]:
                mean[i][j] = row["mean_sec"]
            rows.append(row)
    return {
        "travel_time_sec": mean,
        "pairs": rows,
        "complete": all(r["usable"] for r in rows),
        "minimum_repeats": minimum_repeats,
    }


def load_time_cache(path, distance_cache):
    path = Path(path)
    data = json.loads(path.read_text())
    checksum = data.pop("content_sha256")
    if digest(data) != checksum:
        raise ValueError("time cache content hash mismatch")
    data["content_sha256"] = checksum
    if (
        data.get("version") != 1
        or data.get("time_source") != "independent_gazebo_navigation_ros_time_mean"
    ):
        raise ValueError("unsupported time cache source/version")
    if data["distance_content_sha256"] != distance_cache["content_sha256"]:
        raise ValueError("time and distance caches are not bound to the same collection")
    if (
        data["stations"] != distance_cache["stations"]
        or not data["complete"]
        or not data["finished"]
        or data["error"]
        or data["collisions"]
    ):
        raise ValueError("incomplete time cache or station order mismatch")
    runtime = json.loads((path.parent / "runtime_parameters.json").read_text())
    if digest(runtime) != data["runtime_parameters_sha256"]:
        raise ValueError("runtime parameter snapshot hash mismatch")
    if (
        hashlib.sha256((path.parent / "trajectory.csv").read_bytes()).hexdigest()
        != data["trajectory_sha256"]
    ):
        raise ValueError("trajectory hash mismatch")
    actual = summarize(data["stations"], data["samples"], data["minimum_repeats"])
    # 旧归档只有基本统计；新字段存在时逐项核对，兼容历史记录但不补写它们。
    for saved, calculated in zip(data["pairs"], actual["pairs"]):
        for key, value in saved.items():
            if key in calculated and value != calculated[key]:
                raise ValueError("time statistics differ from raw samples")
    if len(data["pairs"]) != len(actual["pairs"]):
        raise ValueError("missing pair statistics")
    if actual["travel_time_sec"] != data["travel_time_sec"] or not actual["complete"]:
        raise ValueError("time matrix differs from raw sample statistics")
    return data
