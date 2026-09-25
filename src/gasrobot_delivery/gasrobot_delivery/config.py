"""配送配置读取与静态检查。

工位与地图图像哈希绑定，防止换地图后误用旧坐标；所有停车点必须有
0.30 米已知自由空间。此检查仅排除静态几何风险，不能代替实际导航验收。
"""

from dataclasses import dataclass
from pathlib import Path
import hashlib
import math
import yaml
import numpy as np
from PIL import Image


def number(value, field, minimum=0.0, strict=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{field}: expected finite number")
    if value < minimum or (strict and value == minimum):
        raise ValueError(f"{field}: outside allowed range")
    return float(value)


class UniqueKeyLoader(yaml.SafeLoader):
    """拒绝重复键，防止 YAML 后一工位坐标静默覆盖前一工位。"""

    def construct_mapping(self, node, deep=False):
        self.flatten_mapping(node)
        result = {}
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            if not isinstance(key, str):
                raise ValueError("configuration keys must be strings")
            if key in result:
                raise ValueError(f"duplicate YAML key: {key}")
            result[key] = self.construct_object(value_node, deep=deep)
        return result


def mapping(path):
    try:
        value = yaml.load(Path(path).read_text(), Loader=UniqueKeyLoader)
    except yaml.YAMLError as exc:
        raise ValueError(f"{path}: invalid YAML: {exc}") from exc
    if not isinstance(value, dict) or (
        type(value.get("version")) is not int or value.get("version") != 1
    ):
        raise ValueError(f"{path}: expected version 1 mapping")
    return value


@dataclass(frozen=True)
class Batch:
    batch_id: str
    route: tuple
    service_sec: float
    navigation_timeout_sec: float
    max_retries: int


def load_batch(path, stations, depot):
    """读取一批互不重复的工位任务；物料站回程由状态机统一追加。"""
    raw = mapping(path)
    route = raw.get("route")
    if not isinstance(route, list) or not route or any(not isinstance(x, str) for x in route):
        raise ValueError("route must be a nonempty list of station IDs")
    if len(set(route)) != len(route):
        raise ValueError("duplicate station in route")
    if any(x not in stations or x == depot for x in route):
        raise ValueError("route contains unknown station or depot")
    retries = raw.get("max_retries", 1)
    if type(retries) is not int or retries < 0:
        raise ValueError("max_retries must be a nonnegative integer")
    batch_id = raw.get("batch_id")
    if not isinstance(batch_id, str) or not batch_id.strip():
        raise ValueError("batch_id must be a nonempty string")
    return Batch(
        batch_id,
        tuple(route),
        number(raw.get("service_sec", 5), "service_sec"),
        number(raw.get("navigation_timeout_sec", 180), "navigation_timeout_sec", strict=True),
        retries,
    )


def load_stations(path, map_file):
    """读取 map 位姿并按地图原点、旋转和分辨率验证停车点净空。"""
    raw = mapping(path)
    if raw.get("frame_id") != "map":
        raise ValueError("stations must use map frame")
    stations = raw.get("stations")
    if not isinstance(stations, dict) or raw.get("depot") not in stations:
        raise ValueError("stations must include depot")
    for name, pose in stations.items():
        if not isinstance(name, str) or not name.strip() or not isinstance(pose, dict):
            raise ValueError("invalid station")
        for field in ("x", "y", "yaw"):
            number(pose.get(field), f"{name}.{field}", minimum=-math.inf)
    if hashlib.sha256(Path(map_file).read_bytes()).hexdigest() != raw.get("map_yaml_sha256"):
        raise ValueError("station calibration does not match map metadata hash")
    meta = yaml.safe_load(Path(map_file).read_text())
    image_path = Path(map_file).parent / meta["image"]
    if hashlib.sha256(image_path.read_bytes()).hexdigest() != raw.get("map_image_sha256"):
        raise ValueError("station calibration does not match map image hash")
    image = np.array(Image.open(image_path).convert("L"))
    probability = image / 255.0 if meta.get("negate", 0) else 1.0 - image / 255.0
    free = probability < meta["free_thresh"]
    ox, oy, yaw = meta["origin"]
    res = meta["resolution"]
    radius = math.ceil(0.30 / res)
    for name, pose in stations.items():
        dx, dy = pose["x"] - ox, pose["y"] - oy
        cx = math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy) / res)
        cy = image.shape[0] - 1 - math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy) / res)
        for j in range(-radius, radius + 1):
            for i in range(-radius, radius + 1):
                if math.hypot(i * res, j * res) > 0.30:
                    continue
                x, y = cx + i, cy + j
                if not (0 <= y < free.shape[0] and 0 <= x < free.shape[1] and free[y, x]):
                    raise ValueError(f"{name}: less than 0.30 m known free clearance")
    return raw
