#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""根据 Nav2 静态地图校验初始化位姿和巡检航点。"""

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import yaml

from gasrobot_navigation.route_config import RouteBook, RouteConfigError


@dataclass(frozen=True)
class MapMetadata:
    """巡检路线校验所需的二维地图元数据。"""

    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    negate: bool
    occupied_threshold: float
    free_threshold: float


@dataclass(frozen=True)
class PgmOccupancyMap:
    """不依赖图像库的 PGM 栅格地图读取结果。"""

    width: int
    height: int
    maximum_value: int
    pixels: bytes
    metadata: MapMetadata

    def world_to_image(self, x: float, y: float) -> Tuple[int, int]:
        """把 map 坐标转换为 PGM 图像的列、行索引。"""

        delta_x = x - self.metadata.origin_x
        delta_y = y - self.metadata.origin_y
        cosine = math.cos(self.metadata.origin_yaw)
        sine = math.sin(self.metadata.origin_yaw)

        # 地图原点带旋转时，先变换到地图自身的局部坐标系。
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        grid_x = math.floor(local_x / self.metadata.resolution)
        grid_y = math.floor(local_y / self.metadata.resolution)

        # OccupancyGrid 从左下角计数，PGM 图像从左上角计数。
        return grid_x, self.height - 1 - grid_y

    def cell_state(self, column: int, row: int) -> str:
        """按照 Nav2 trinary 模式返回 free、unknown 或 occupied。"""

        if not (0 <= column < self.width and 0 <= row < self.height):
            return "outside"

        pixel = self.pixels[row * self.width + column]
        shade = pixel / self.maximum_value
        occupancy = shade if self.metadata.negate else 1.0 - shade
        if occupancy > self.metadata.occupied_threshold:
            return "occupied"
        if occupancy < self.metadata.free_threshold:
            return "free"
        return "unknown"

    def validate_pose(
        self,
        x: float,
        y: float,
        minimum_clearance: float,
        context: str,
    ) -> None:
        """确认位姿及机器人周围安全半径全部位于已知自由栅格。"""

        column, row = self.world_to_image(x, y)
        state = self.cell_state(column, row)
        if state != "free":
            raise RouteConfigError(
                f"{context} 不在已知自由区域: 坐标=({x:.3f}, {y:.3f}), "
                f"地图状态={state}"
            )

        radius_cells = math.ceil(
            minimum_clearance / self.metadata.resolution
        )
        radius_squared = minimum_clearance * minimum_clearance
        for offset_y in range(-radius_cells, radius_cells + 1):
            for offset_x in range(-radius_cells, radius_cells + 1):
                distance_squared = (
                    (offset_x * self.metadata.resolution) ** 2
                    + (offset_y * self.metadata.resolution) ** 2
                )
                if distance_squared > radius_squared:
                    continue
                if self.cell_state(
                    column + offset_x,
                    row + offset_y,
                ) != "free":
                    raise RouteConfigError(
                        f"{context} 距障碍物、未知区或地图边界不足 "
                        f"{minimum_clearance:.2f} 米: "
                        f"坐标=({x:.3f}, {y:.3f})"
                    )


def _next_pgm_token(data: bytes, index: int) -> Tuple[bytes, int]:
    """读取 PGM 头部的下一个字段，并跳过空白和注释。"""

    length = len(data)
    while index < length:
        if data[index:index + 1] == b"#":
            newline = data.find(b"\n", index)
            if newline < 0:
                raise RouteConfigError("PGM 文件注释没有正常结束")
            index = newline + 1
            continue
        if chr(data[index]).isspace():
            index += 1
            continue
        break

    start = index
    while index < length:
        character = data[index:index + 1]
        if character == b"#" or chr(data[index]).isspace():
            break
        index += 1
    if start == index:
        raise RouteConfigError("PGM 文件头不完整")
    return data[start:index], index


def _load_pgm(path: Path, metadata: MapMetadata) -> PgmOccupancyMap:
    """读取 Nav2 地图使用的 8 位二进制 PGM 文件。"""

    try:
        data = path.read_bytes()
    except OSError as exc:
        raise RouteConfigError(f"无法读取地图图像: {path}") from exc

    index = 0
    magic, index = _next_pgm_token(data, index)
    width_raw, index = _next_pgm_token(data, index)
    height_raw, index = _next_pgm_token(data, index)
    maximum_raw, index = _next_pgm_token(data, index)
    if magic != b"P5":
        raise RouteConfigError("路线安全校验仅支持二进制 PGM(P5) 地图")

    try:
        width = int(width_raw)
        height = int(height_raw)
        maximum_value = int(maximum_raw)
    except ValueError as exc:
        raise RouteConfigError("PGM 文件头包含非法数字") from exc
    if width <= 0 or height <= 0 or not 0 < maximum_value <= 255:
        raise RouteConfigError("PGM 地图尺寸或最大灰度值非法")

    # 最大灰度字段之后必须有一个空白分隔符；兼容 Windows 的 CRLF。
    if index >= len(data) or not chr(data[index]).isspace():
        raise RouteConfigError("PGM 文件头与像素数据之间缺少分隔符")
    if data[index:index + 2] == b"\r\n":
        index += 2
    else:
        index += 1

    expected_size = width * height
    pixels = data[index:index + expected_size]
    if len(pixels) != expected_size:
        raise RouteConfigError("PGM 地图像素数据长度不正确")
    return PgmOccupancyMap(
        width=width,
        height=height,
        maximum_value=maximum_value,
        pixels=pixels,
        metadata=metadata,
    )


def load_occupancy_map(map_yaml_path: str) -> PgmOccupancyMap:
    """读取地图 YAML 及其引用的 PGM 图像。"""

    yaml_path = Path(map_yaml_path).expanduser()
    try:
        raw = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise RouteConfigError(f"无法读取地图 YAML: {yaml_path}") from exc
    if not isinstance(raw, dict):
        raise RouteConfigError("地图 YAML 根节点必须是键值映射")
    if str(raw.get("mode", "trinary")).casefold() != "trinary":
        raise RouteConfigError("路线安全校验目前仅支持 trinary 地图")

    try:
        origin = raw["origin"]
        metadata = MapMetadata(
            resolution=float(raw["resolution"]),
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=float(origin[2]),
            negate=bool(int(raw.get("negate", 0))),
            occupied_threshold=float(raw["occupied_thresh"]),
            free_threshold=float(raw["free_thresh"]),
        )
        image_path = Path(str(raw["image"]))
    except (KeyError, IndexError, TypeError, ValueError) as exc:
        raise RouteConfigError("地图 YAML 缺少合法的必要字段") from exc

    if metadata.resolution <= 0.0:
        raise RouteConfigError("地图 resolution 必须大于 0")
    if not (
        0.0 <= metadata.free_threshold
        < metadata.occupied_threshold
        <= 1.0
    ):
        raise RouteConfigError("地图占用阈值范围不正确")
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    return _load_pgm(image_path, metadata)


def validate_route_book_against_map(
    route_book: RouteBook,
    map_yaml_path: str,
    minimum_clearance: float,
) -> None:
    """拒绝位于障碍物、未知区域或离边界过近的路线配置。"""

    if minimum_clearance < 0.0 or not math.isfinite(minimum_clearance):
        raise RouteConfigError("minimum_waypoint_clearance_m 不能为负")
    if route_book.frame_id != "map":
        raise RouteConfigError("静态地图巡检路线的 frame_id 必须是 map")

    occupancy_map = load_occupancy_map(map_yaml_path)
    if route_book.initial_pose is not None:
        occupancy_map.validate_pose(
            route_book.initial_pose.x,
            route_book.initial_pose.y,
            minimum_clearance,
            "initial_pose",
        )
    for route in route_book.routes.values():
        for waypoint in route.waypoints:
            occupancy_map.validate_pose(
                waypoint.x,
                waypoint.y,
                minimum_clearance,
                f'routes.{route.name}.{waypoint.waypoint_id}',
            )
