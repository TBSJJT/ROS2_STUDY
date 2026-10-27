"""冻结优化输入：只复制通过校验的归档，后续读取核对共同输入指纹。

冻结是版本/内容约束，并非修改系统文件权限。保留原始标定证据；任何编辑
都导致内容校验失败，需要生成新的实验版本。执行验证不能回填这个目录。
"""

import hashlib
import json
from pathlib import Path
import shutil
from delivery_planner.costs import digest, load_distance_cache, write_json
from delivery_planner.calibration import load_time_cache
from delivery_planner.model import load_instance


def validate_inputs(distance_file, time_file, instance_file):
    distance = load_distance_cache(distance_file)
    if not distance.get("planning_consistency") or not distance["all_reachable"]:
        raise ValueError("freeze requires verified repeated planning for every pair")
    time = load_time_cache(time_file, distance)
    if time["minimum_repeats"] < 5:
        raise ValueError("formal frozen input requires at least five measurements per pair")
    load_instance(instance_file, distance["stations"], distance["depot"])
    return {
        "distance": distance["content_sha256"],
        "time": time["content_sha256"],
        "instance": hashlib.sha256(Path(instance_file).read_bytes()).hexdigest(),
        "model": "completion_tardiness_mean_time_v1",
    }


def freeze(distance_file, time_file, instance_file, output):
    distance_file, time_file, instance_file, output = map(
        lambda p: Path(p).resolve(), (distance_file, time_file, instance_file, output)
    )
    fingerprint = validate_inputs(distance_file, time_file, instance_file)
    # 防止递归把输出复制到输入之中，且永不覆盖已有冻结版本。
    if any(output.is_relative_to(p.parent) for p in (distance_file, time_file)):
        raise ValueError("output must be outside input archive directories")
    output.mkdir(parents=True, exist_ok=False)
    shutil.copytree(distance_file.parent, output / "distance")
    shutil.copytree(time_file.parent, output / "time")
    shutil.copyfile(instance_file, output / "instance.yaml")
    if (
        validate_inputs(
            output / "distance" / distance_file.name,
            output / "time" / time_file.name,
            output / "instance.yaml",
        )
        != fingerprint
    ):
        raise ValueError("input changed during freeze; discard incomplete output")
    manifest = {
        "version": 1,
        "inputs": fingerprint,
        "evaluation_input_sha256": digest(fingerprint),
        "distance_file": "distance/" + distance_file.name,
        "time_file": "time/" + time_file.name,
        "instance_file": "instance.yaml",
        "objective_semantics": "planned_distance_and_tardiness_at_mean_times",
        "actual_execution_data_may_update_inputs": False,
    }
    write_json(output / "experiment.json", manifest)
    return manifest


def load_bundle(folder):
    folder = Path(folder).resolve()
    manifest = json.loads((folder / "experiment.json").read_text())
    paths = [
        (folder / manifest[key]).resolve()
        for key in ("distance_file", "time_file", "instance_file")
    ]
    if manifest["version"] != 1 or any(not p.is_relative_to(folder) for p in paths):
        raise ValueError("invalid experiment manifest paths/version")
    fingerprint = validate_inputs(*paths)
    if (
        fingerprint != manifest["inputs"]
        or digest(fingerprint) != manifest["evaluation_input_sha256"]
    ):
        raise ValueError("frozen experiment input hash mismatch")
    return paths
