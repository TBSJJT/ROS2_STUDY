"""巡检路线 YAML 的数据模型、读取和严格校验。"""

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional

import yaml


class RouteConfigError(ValueError):
    """表示巡检路线文件缺少字段或包含危险参数。"""


@dataclass(frozen=True)
class Pose2DConfig:
    """地图坐标系中的二维位姿配置。"""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class InitialPoseConfig(Pose2DConfig):
    """带有 AMCL 初始协方差的二维位姿。"""

    covariance_x: float
    covariance_y: float
    covariance_yaw: float


@dataclass(frozen=True)
class InspectionWaypoint(Pose2DConfig):
    """一个带业务名称和停留时间的巡检点。"""

    waypoint_id: str
    description: str
    dwell_sec: float


@dataclass(frozen=True)
class InspectionRoute:
    """一条可重复执行并带有失败策略的巡检路线。"""

    name: str
    description: str
    target_gas: str
    alarm_threshold: float
    stop_on_critical_risk: bool
    repeat_count: int
    continue_on_failure: bool
    max_retries: int
    navigation_timeout_sec: float
    waypoints: List[InspectionWaypoint]


@dataclass(frozen=True)
class RouteBook:
    """一个场地的初始化位姿和全部命名路线。"""

    frame_id: str
    site_configured: bool
    initial_pose: Optional[InitialPoseConfig]
    routes: Dict[str, InspectionRoute]

    def route(self, name: str) -> InspectionRoute:
        """按名称返回路线；名称不存在时给出清晰错误。"""

        try:
            return self.routes[name]
        except KeyError as exc:
            available = "、".join(sorted(self.routes)) or "无"
            raise RouteConfigError(
                f"未找到巡检路线“{name}”，可用路线：{available}"
            ) from exc


def _mapping(value, field_name: str) -> Mapping:
    """校验字段为 YAML 映射。"""

    if not isinstance(value, Mapping):
        raise RouteConfigError(f"{field_name} 必须是键值映射")
    return value


def _text(mapping: Mapping, key: str, context: str) -> str:
    """读取必需的非空文本字段。"""

    value = str(mapping.get(key, "")).strip()
    if not value:
        raise RouteConfigError(f"{context}.{key} 不能为空")
    return value


def _finite_number(mapping: Mapping, key: str, context: str) -> float:
    """读取必需的有限数值字段。"""

    try:
        value = float(mapping[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise RouteConfigError(f"{context}.{key} 必须是数值") from exc
    if not math.isfinite(value):
        raise RouteConfigError(f"{context}.{key} 必须是有限数值")
    return value


def _positive_number(mapping: Mapping, key: str, context: str) -> float:
    """读取严格大于零的有限数值字段。"""

    value = _finite_number(mapping, key, context)
    if value <= 0.0:
        raise RouteConfigError(f"{context}.{key} 必须大于 0")
    return value


def _nonnegative_number(mapping: Mapping, key: str, context: str) -> float:
    """读取大于或等于零的有限数值字段。"""

    value = _finite_number(mapping, key, context)
    if value < 0.0:
        raise RouteConfigError(f"{context}.{key} 不能小于 0")
    return value


def _pose(mapping: Mapping, context: str):
    """读取二维位置和以度表示的航向，并转换为弧度。"""

    return (
        _finite_number(mapping, "x", context),
        _finite_number(mapping, "y", context),
        math.radians(_finite_number(mapping, "yaw_deg", context)),
    )


def load_route_book(path: str) -> RouteBook:
    """读取巡检路线文件并返回不可变、已校验的配置。"""

    route_path = Path(path).expanduser()
    if not route_path.is_file():
        raise RouteConfigError(f"巡检路线文件不存在：{route_path}")

    try:
        raw = yaml.safe_load(route_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise RouteConfigError(f"巡检路线 YAML 语法错误：{exc}") from exc

    root = _mapping(raw, "根节点")
    version = int(root.get("version", 0))
    if version != 1:
        raise RouteConfigError("仅支持 version: 1 的巡检路线文件")

    frame_id = _text(root, "frame_id", "根节点")
    site_configured = bool(root.get("site_configured", False))

    initial_pose = None
    if root.get("initial_pose") is not None:
        value = _mapping(root["initial_pose"], "initial_pose")
        x, y, yaw = _pose(value, "initial_pose")
        covariance = _mapping(
            value.get("covariance", {}),
            "initial_pose.covariance",
        )
        initial_pose = InitialPoseConfig(
            x=x,
            y=y,
            yaw=yaw,
            covariance_x=_positive_number(
                covariance, "x", "initial_pose.covariance"
            ),
            covariance_y=_positive_number(
                covariance, "y", "initial_pose.covariance"
            ),
            covariance_yaw=_positive_number(
                covariance, "yaw", "initial_pose.covariance"
            ),
        )

    routes_value = _mapping(root.get("routes", {}), "routes")
    routes: Dict[str, InspectionRoute] = {}
    for route_name, route_raw in routes_value.items():
        name = str(route_name).strip()
        if not name:
            raise RouteConfigError("路线名称不能为空")
        value = _mapping(route_raw, f"routes.{name}")
        default_dwell = _nonnegative_number(
            value, "default_dwell_sec", f"routes.{name}"
        )
        waypoint_values = value.get("waypoints")
        if not isinstance(waypoint_values, list) or not waypoint_values:
            raise RouteConfigError(
                f"routes.{name}.waypoints 必须是非空列表"
            )

        waypoints = []
        waypoint_ids = set()
        for index, waypoint_raw in enumerate(waypoint_values):
            context = f"routes.{name}.waypoints[{index}]"
            waypoint = _mapping(waypoint_raw, context)
            waypoint_id = _text(waypoint, "id", context)
            if waypoint_id in waypoint_ids:
                raise RouteConfigError(
                    f"路线“{name}”存在重复巡检点：{waypoint_id}"
                )
            waypoint_ids.add(waypoint_id)
            x, y, yaw = _pose(waypoint, context)
            dwell_sec = float(waypoint.get("dwell_sec", default_dwell))
            if not math.isfinite(dwell_sec) or dwell_sec < 0.0:
                raise RouteConfigError(f"{context}.dwell_sec 不能小于 0")
            waypoints.append(
                InspectionWaypoint(
                    waypoint_id=waypoint_id,
                    description=str(waypoint.get("description", "")).strip(),
                    x=x,
                    y=y,
                    yaw=yaw,
                    dwell_sec=dwell_sec,
                )
            )

        repeat_count = int(value.get("repeat_count", 1))
        max_retries = int(value.get("max_retries", 1))
        if repeat_count < 1:
            raise RouteConfigError(f"routes.{name}.repeat_count 必须至少为 1")
        if max_retries < 0:
            raise RouteConfigError(f"routes.{name}.max_retries 不能小于 0")

        routes[name] = InspectionRoute(
            name=name,
            description=str(value.get("description", "")).strip(),
            target_gas=_text(value, "target_gas", f"routes.{name}"),
            alarm_threshold=_positive_number(
                value, "alarm_threshold", f"routes.{name}"
            ),
            stop_on_critical_risk=bool(
                value.get("stop_on_critical_risk", True)
            ),
            repeat_count=repeat_count,
            continue_on_failure=bool(
                value.get("continue_on_failure", False)
            ),
            max_retries=max_retries,
            navigation_timeout_sec=_positive_number(
                value, "navigation_timeout_sec", f"routes.{name}"
            ),
            waypoints=waypoints,
        )

    if not routes:
        raise RouteConfigError("routes 至少需要定义一条巡检路线")

    return RouteBook(
        frame_id=frame_id,
        site_configured=site_configured,
        initial_pose=initial_pose,
        routes=routes,
    )
