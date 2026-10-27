"""有向距离矩阵与缓存校验，不依赖 ROS 消息类型。

距离只对 Nav2 返回的原始折线路径逐段求和。失败点对以 null 保存，
不使用欧氏距离替代，也不通过转置补齐反方向。JSON 禁止 NaN/Infinity，
这样后续优化程序必须明确处理不可达点对。
"""

import hashlib
import json
import math
from pathlib import Path


def digest(value):
    """规范 JSON 哈希不依赖字典顺序，用于绑定配置和矩阵内容。"""
    raw = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(raw.encode()).hexdigest()


def write_json(path, value):
    """每完成一个点对即原子替换进度，异常退出仍保留已完成记录。"""
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    temporary.replace(path)


def path_length(points):
    """输入二维米制坐标；跨工位成功路径至少要有两个有效点。"""
    if len(points) < 2:
        raise ValueError("path has fewer than two points")
    for p in points:
        if len(p) != 2 or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
            for v in p
        ):
            raise ValueError("path contains invalid coordinates")
    return math.fsum(math.dist(a, b) for a, b in zip(points, points[1:]))


def assemble_matrix(names, records):
    """只允许每个有向点对出现一次；记录包含失败状态也属于覆盖完成。"""
    if len(set(names)) != len(names) or len(names) < 2:
        raise ValueError("station names must be unique and include at least two points")
    index = {name: i for i, name in enumerate(names)}
    matrix = [[0.0 if i == j else None for j in range(len(names))] for i in range(len(names))]
    seen = set()
    for row in records:
        a, b = row["from"], row["to"]
        if a not in index or b not in index or a == b or (a, b) in seen:
            raise ValueError("unknown, diagonal or duplicate pair")
        seen.add((a, b))
        if row["status"] == "reachable":
            matrix[index[a]][index[b]] = path_length(row["path_xy_m"])
    return matrix


def load_distance_cache(path, expected_config_sha256=None):
    """离线读取时同时核对内容和来源快照，拒绝截断或混用缓存。

    expected_config_sha256 供使用另一套当前配置的调用者传入；不给定时只验证
    该归档自身的一致性，不代表自动确认它与任意外部运行中的 Nav2 一致。
    """
    path = Path(path)
    data = json.loads(path.read_text())
    checksum = data.pop("content_sha256")
    if digest(data) != checksum:
        raise ValueError("matrix content hash mismatch")
    data["content_sha256"] = checksum
    if data["version"] != 1 or data["distance_source"] != "nav2_static_planned_path":
        raise ValueError("unsupported distance cache")
    if not data["complete"]:
        raise ValueError("incomplete distance cache")
    if digest(json.loads((path.parent / "costmap.json").read_text())) != data["costmap_sha256"]:
        raise ValueError("costmap hash mismatch")
    provenance = json.loads((path.parent / "provenance.json").read_text())
    if digest(provenance["sha256"]) != data["config_sha256"]:
        raise ValueError("configuration manifest mismatch")
    for relative, sha in provenance["sha256"].items():
        snapshot = (path.parent / relative).resolve()
        if not snapshot.is_relative_to(path.parent.resolve()):
            raise ValueError("snapshot escapes cache directory")
        if hashlib.sha256(snapshot.read_bytes()).hexdigest() != sha:
            raise ValueError(f"snapshot changed: {relative}")
    if expected_config_sha256 is not None and data["config_sha256"] != expected_config_sha256:
        raise ValueError("cache does not match requested configuration")
    n = len(data["stations"])
    if len(data["pairs"]) != n * (n - 1):
        raise ValueError("missing pair records")
    if assemble_matrix(data["stations"], data["pairs"]) != data["distance_m"]:
        raise ValueError("distance matrix differs from recorded paths")
    if "planning_consistency" in data:
        for row in data["pairs"]:
            checked = check_plan_trials(
                row["planning_trials"], data["planning_consistency"]["epsilon_m"]
            )
            if checked != row["consistency"]:
                raise ValueError("planning trial consistency metadata mismatch")
            if row["status"] == "reachable" and (
                not checked["passed"] or row["path_xy_m"] != row["planning_trials"][0]["path_xy_m"]
            ):
                raise ValueError("unverified or averaged planning path")
    return data


def check_plan_trials(trials, epsilon_m=1e-6):
    """不平均规划结果。保留全部原始 Path，只在一致时选第一条作为固定代价。

    相同长度也可能对应不同路线，因此同时严格检查路径点序列。固定 Navfn
    场景采用这一更严格标准；随机规划器需要另行声明随机种子和生成协议。
    """
    if len(trials) < 3 or not math.isfinite(epsilon_m) or epsilon_m < 0:
        raise ValueError("at least three planning trials and finite nonnegative epsilon required")
    lengths = [path_length(r["path_xy_m"]) if r["status"] == "reachable" else None for r in trials]
    valid = all(v is not None for v in lengths)
    spread = max(lengths) - min(lengths) if valid else None
    same_path = valid and all(r["path_xy_m"] == trials[0]["path_xy_m"] for r in trials)
    return {
        "trial_count": len(trials),
        "epsilon_m": epsilon_m,
        "distance_spread_m": spread,
        "identical_paths": same_path,
        "passed": valid and spread <= epsilon_m and same_path,
        "selection": "first_trial_not_average",
    }
