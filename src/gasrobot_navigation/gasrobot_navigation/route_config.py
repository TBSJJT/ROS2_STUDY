#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
巡检路线 YAML 配置文件的数据模型、读取和严格校验.

这个模块是巡检任务系统的基础层, 负责: 
1. 定义巡检路线的数据结构(数据类/DataClass)
   - Pose2DConfig: 二维位姿
   - InitialPoseConfig: AMCL 初始位姿(带协方差)
   - InspectionWaypoint: 单个巡检航点(带停留时间和名称)
   - InspectionRoute: 完整巡检路线(带失败策略、报警阈值等)
   - RouteBook: 一个场地的全部路线和初始位姿

2. 从 YAML 文件加载路线配置
   - YAML 是一种人类可读的配置文件格式
   - load_route_book() 函数读取 YAML → 返回 RouteBook 对象

3. 严格校验配置合法性
   - 必需字段检查(不能为空)
   - 数值范围检查(如阈值 > 0, 圈数 >= 1)
   - 航点 ID 唯一性检查(不允许重复)
   - 模板保护(site_configured 标志防止误用模板值)
"""

# math: 数学函数库, 这里用于 math.isfinite() 检查有限值和索引转换
import math
# dataclass: Python 的数据类装饰器, 自动生成 __init__、__repr__、__eq__ 等方法
from dataclasses import dataclass
# Path: 面向对象的文件路径处理(比字符串拼接更安全、跨平台)
from pathlib import Path
# typing 模块提供类型标注
# Dict、List、Mapping、Optional 帮助代码编辑器做类型检查和智能补全
from typing import Dict, List, Mapping, Optional

# PyYAML: Python 最流行的 YAML 解析库
# safe_load 是安全版本, 不会执行 YAML 中的任意 Python 代码
import yaml


# =========================================================================
# 自定义异常类
# =========================================================================
class RouteConfigError(ValueError):
    """
    表示巡检路线配置文件存在格式错误、缺少必需字段或包含危险参数.

    继承自 ValueError(值错误), 这样捕获 ValueError 的代码
    也能捕获到路线配置错误.
    错误消息应该清晰指出问题所在, 例如: 
    - "routes.standard_route.alarm_threshold 必须大于 0"
    - "路线"standard_route"存在重复巡检点: point_a"

    """


# =========================================================================
# 数据模型类(Data Models)
# =========================================================================
# @dataclass 装饰器是 Python 3.7+ 引入的功能
# 它会自动为类生成: 
#   - __init__(): 构造方法(根据下面定义的字段)
#   - __repr__(): 可读的字符串表示
#   - __eq__(): 相等性比较(两个对象字段相同时视为相等)
# frozen=True 表示对象创建后不可修改(不可变对象): 
#   - 类似于 tuple 和 str, 一旦创建就不能改
#   - 好处: 线程安全、可以作为字典的键、更易于推理

@dataclass(frozen=True)
class Pose2DConfig:
    """
    地图坐标系中的二维位姿配置.

    机器人定位中, 位姿(Pose)= 位置(Position)+ 姿态(Orientation).
    在二维平面上: 
    - 位置用 (x, y) 坐标表示
    - 姿态只需要一个 yaw(偏航角, 绕 Z 轴旋转)

    属性:
        x:   地图 X 坐标(米)
        y:   地图 Y 坐标(米)
        yaw: 偏航角(弧度), 0 表示朝向 X 轴正方向

    """

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class InitialPoseConfig(Pose2DConfig):
    """
    带有 AMCL 初始协方差的二维位姿配置.

    继承自 Pose2DConfig, 额外增加了协方差字段.
    协方差(Covariance)表示初始位姿的"不确定性": 
    - 值越小 → 越确定(如果机器人和地图完美对齐, 用很小的协方差)
    - 值越大 → 越不确定(如果只是大概知道初始位置, 用大的协方差)

    在 ROS 2 中, 定位信息以"均值+协方差"的形式表示: 
    - 均值: 最可能的位姿估计值
    - 协方差: 对这个估计的信心程度

    属性:
        covariance_x:   X 坐标的协方差(米²)
        covariance_y:   Y 坐标的协方差(米²)
        covariance_yaw: 偏航角的协方差(弧度²)

    """

    # 协方差矩阵的对角线元素(简化表示)
    covariance_x: float
    covariance_y: float
    covariance_yaw: float


@dataclass(frozen=True)
class InspectionWaypoint(Pose2DConfig):
    """
    一个带有业务名称和可选静止观察时间的巡检航点.

    巡检航点不同于普通的导航目标点: 
    - waypoint_id: 唯一标识, 用于关联气体传感器读数
    - description: 人类可读描述(如"阀门区 3 号位")
    - dwell_sec: 到达后是否额外静止观察

    气体采样由 gasrobot_gas 按固定频率连续完成，不受航点到达事件控制。
    正常覆盖巡检应使用 dwell_sec=0；传感器响应实验或重点区域复测时，
    才按实验设计配置非零停留时间。

    属性:
        waypoint_id: 航点唯一标识符(如 "point_a", "valve_03")
        description: 航点描述文本
        dwell_sec:   到达后的静止观察时间(秒), 0 表示直接前往下一点

    """

    waypoint_id: str
    description: str
    dwell_sec: float


@dataclass(frozen=True)
class InspectionRoute:
    """
    一条可重复执行并带有失败处理策略的巡检路线.

    一条路线 = 一个有序的航点列表 + 执行策略.
    机器人会从第一个航点开始依次导航；与此同时，气体传感器始终连续
    采样。dwell_sec 只是可选的静止观察策略，不是采样触发条件。

    关键策略参数: 
    - repeat_count: 路线执行圈数
      例如 repeat_count=2 表示走完所有航点后再走一遍
    - continue_on_failure: 某航点失败后是否跳过继续
      True → 跳过失败点继续下一个
      False → 整个任务立即失败
    - max_retries: 每个航点最多重试次数
    - stop_on_critical_risk: 检测到严重气体风险是否紧急停机
      True → 安全第一, 立即停止
      False → 记录风险但继续巡检

    属性:
        name:                   路线名称(如 "standard_route")
        description:            路线描述
        target_gas:            要检测的目标气体类型(如 "methane", "h2s")
        alarm_threshold:       气体浓度报警阈值
        stop_on_critical_risk: 是否在严重风险时紧急停机
        repeat_count:          路线重复执行次数(≥1)
        continue_on_failure:   某航点失败后是否继续
        max_retries:           每个航点最大重试次数(≥0)
        navigation_timeout_sec: 单次导航的超时时间(秒)
        waypoints:             巡检航点列表(有序)

    """

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
    """
    一个场地的初始化位姿和全部命名巡检路线.

    RouteBook(路线手册)是整个巡检系统的配置总入口.
    一个 YAML 文件对应一个 RouteBook, 包含: 
    - frame_id: 坐标系名称(通常是 "map")
    - site_configured: 场地是否已标定
    - initial_pose: AMCL 初始位姿(可选)
    - routes: 命名路线字典 {"route_name": InspectionRoute, ...}

    route() 方法是查找路线的主要入口: 
    根据名称返回对应的 InspectionRoute, 如果名称不存在则抛出清晰的错误.

    """

    frame_id: str
    site_configured: bool
    initial_pose: Optional[InitialPoseConfig]
    routes: Dict[str, InspectionRoute]

    def route(self, name: str) -> InspectionRoute:
        """
        按名称返回路线；名称不存在时给出清晰错误信息.

        参数:
            name: 路线名称(如 "standard_route")

        返回:
            对应的 InspectionRoute 对象

        异常:
            RouteConfigError: 当路线名称不存在时抛出, 
                              错误信息包含所有可用路线名

        """
        try:
            # 直接从字典中按键取值
            return self.routes[name]
        except KeyError as exc:
            # 键不存在 → 构建友好的错误消息
            # "、".join(sorted(self.routes)): 
            #   把所有路线名排序后用中文顿号连接
            #   例如: "night_route、standard_route"
            # 如果 routes 是空的, 显示 "无"
            available = "、".join(sorted(self.routes)) or "无"
            # raise ... from exc: 异常链, 保留原始异常的上下文
            raise RouteConfigError(
                f'未找到巡检路线"{name}", 可用路线: {available}'
            ) from exc


# =========================================================================
# YAML 字段读取辅助函数
# =========================================================================
# 这些函数封装了常见的"从映射中读取并校验"模式.
# 每个函数做两件事: 
#   1. 类型检查和转换
#   2. 值的合法性校验
# 如果校验失败, 抛出 RouteConfigError 并清晰指示问题位置.

def _mapping(value, field_name: str) -> Mapping:
    """
    校验字段是否是 YAML 映射(键值对集合).

    在 YAML 中, 映射就是缩进的键值对, 例如: 
        initial_pose:
          x: 1.0
          y: 2.0

    参数:
        value:      要校验的值
        field_name: 字段的完整路径(用于错误提示)

    返回:
        value 本身(如果是 Mapping 类型)

    异常:
        RouteConfigError: 如果 value 不是 Mapping 类型

    """
    # isinstance(obj, type): 检查 obj 是否是 type 类型的实例
    # Mapping 是 dict 类型的抽象基类, 也包括 OrderedDict 等
    if not isinstance(value, Mapping):
        raise RouteConfigError(f"{field_name} 必须是键值映射")
    return value


def _text(mapping: Mapping, key: str, context: str) -> str:
    """
    读取必需的非空文本字段.

    参数:
        mapping: YAML 映射对象
        key:     字段的键名
        context: 字段的完整路径(如 "routes.standard_route")

    返回:
        去除了首尾空白字符的文本值

    异常:
        RouteConfigError: 如果字段不存在或去除空白后为空字符串

    """
    # mapping.get(key, ""): 取 key 对应的值, 如果 key 不存在返回空字符串
    # str(): 转换为字符串(处理可能传入的数值等)
    # .strip(): 去除首尾的空白字符(空格、制表符、换行符)
    value = str(mapping.get(key, "")).strip()
    if not value:
        raise RouteConfigError(f"{context}.{key} 不能为空")
    return value


def _finite_number(mapping: Mapping, key: str, context: str) -> float:
    """
    读取必需的有限数值字段.

    "有限" 意味着不是 NaN(非数字)、Inf(无穷大)或 -Inf(负无穷大).
    这些都是 IEEE 754 浮点数标准中的特殊值, 在实际应用中没有物理意义.

    参数:
        mapping: YAML 映射对象
        key:     字段的键名
        context: 字段的完整路径

    返回:
        有限浮点数值

    异常:
        RouteConfigError: 如果字段不是有效数值或不是有限值

    """
    try:
        # float() 可以转换整数、浮点数字符串、"1e-3" 科学计数法等
        value = float(mapping[key])
    except (KeyError, TypeError, ValueError) as exc:
        # KeyError: key 不存在于 mapping 中
        # TypeError: value 是不能转为 float 的类型(如列表)
        # ValueError: value 是无法解析的字符串(如 "abc")
        raise RouteConfigError(f"{context}.{key} 必须是数值") from exc
    # math.isfinite(value): 
    #   - 对普通数字返回 True
    #   - 对 NaN、Inf、-Inf 返回 False
    if not math.isfinite(value):
        raise RouteConfigError(f"{context}.{key} 必须是有限数值")
    return value


def _positive_number(mapping: Mapping, key: str, context: str) -> float:
    """
    读取严格大于零的有限数值字段.

    用于检查像 alarm_threshold(报警阈值)、navigation_timeout_sec(导航超时)等
    必须有正值的参数.例如: 
    - 报警阈值为 0 → 任何气体读数都会触发报警, 不合理
    - 导航超时为 0 → 不等待导航完成, 不合理

    """
    value = _finite_number(mapping, key, context)
    if value <= 0.0:
        raise RouteConfigError(f"{context}.{key} 必须大于 0")
    return value


def _nonnegative_number(mapping: Mapping, key: str, context: str) -> float:
    """
    读取大于或等于零的有限数值字段.

    用于检查像 default_dwell_sec(默认静止观察时间)、max_retries(重试次数)等
    不能为负但可以为 0 的参数.

    """
    value = _finite_number(mapping, key, context)
    if value < 0.0:
        raise RouteConfigError(f"{context}.{key} 不能小于 0")
    return value


def _pose(mapping: Mapping, context: str):
    """
    读取二维位姿 (x, y, yaw_deg), 其中 yaw_deg 以度为单位, 自动转换为弧度.

    设计决策: YAML 中使用"度"(yaw_deg), 代码内部使用"弧度".
    原因: 
    - 度(degrees)更直观: 90° 比 1.5708 弧度容易理解
    - 弧度(radians)是数学计算的标准单位
    - 在输入边界做转换, 内部全部用弧度, 减少出错

    参数:
        mapping: 包含 x, y, yaw_deg 三个键的映射
        context: 字段路径(用于错误消息)

    返回:
        (x, y, yaw_radians) 元组

    """
    return (
        _finite_number(mapping, "x", context),       # X 坐标(米)
        _finite_number(mapping, "y", context),       # Y 坐标(米)
        math.radians(_finite_number(mapping, "yaw_deg", context)),  # 度→弧度
    )


# =========================================================================
# 主加载函数
# =========================================================================
def load_route_book(path: str) -> RouteBook:
    """
    读取巡检路线 YAML 文件并返回不可变的、已完成校验的 RouteBook 配置.

    这是本模块最重要的函数, 被 InspectionManager 在启动时调用.
    整个加载过程分为三个阶段: 

    第一阶段: 文件读取
    - 展开用户路径(~ 符号)
    - 检查文件是否存在
    - 用 yaml.safe_load 解析 YAML

    第二阶段: 根节点校验
    - version 字段必须是 1(当前只支持版本 1)
    - frame_id 不能为空
    - site_configured 标记场地是否已标定

    第三阶段: 逐路线逐航点校验
    - 路线名不能为空
    - 航点列表不能为空
    - 每个航点的 ID 在路线内不能重复
    - 所有数值字段范围检查
    - 角度自动从度转为弧度

    参数:
        path: YAML 配置文件的文件系统路径

    返回:
        一个 RouteBook 对象, 所有数据已校验

    异常:
        RouteConfigError: 在配置文件有任何问题时抛出

    """
    # --- 文件路径处理 ---
    # Path(path): 创建路径对象(Python 3.4+ 推荐方式)
    # .expanduser(): 展开路径中的 ~ 符号
    #   例如 ~/routes.yaml → /home/book/routes.yaml
    route_path = Path(path).expanduser()

    # 检查路径是否指向一个真实存在的文件
    if not route_path.is_file():
        raise RouteConfigError(f"巡检路线文件不存在: {route_path}")

    # --- YAML 解析 ---
    try:
        # route_path.read_text(encoding="utf-8"): 以 UTF-8 编码读取整个文件内容
        # yaml.safe_load(): 安全解析 YAML(不会执行任意 Python 代码)
        # safe_load 是 PyYAML 推荐的安全版本
        # 返回 Python 原生数据结构(dict、list、str、int、float 等)
        raw = yaml.safe_load(route_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        # YAML 语法错误 → 无法解析
        raise RouteConfigError(f"巡检路线 YAML 语法错误: {exc}") from exc

    # --- 根节点校验 ---
    # _mapping() 确保根节点是键值映射(不是列表或标量)
    root = _mapping(raw, "根节点")

    # version 字段: 当前只支持版本 1
    # .get("version", 0): 如果 version 字段不存在, 默认为 0→会被拒绝
    version = int(root.get("version", 0))
    if version != 1:
        raise RouteConfigError("仅支持 version: 1 的巡检路线文件")

    # frame_id 字段: 坐标系名称
    # 通常是 "map", 表示所有坐标都在地图坐标系中
    frame_id = _text(root, "frame_id", "根节点")

    # site_configured 字段: 场地是否已标定
    # 这是一个安全机制: 防止忘记修改模板坐标就运行巡检
    # 模板文件中的坐标是占位值(如 0, 0), 不是实际巡检位置
    site_configured = bool(root.get("site_configured", False))

    # --- 初始位姿解析 ---
    # initial_pose 是可选的(某些场景不需要 AMCL 初始位姿)
    initial_pose = None
    if root.get("initial_pose") is not None:
        # _mapping() 确保 initial_pose 部分是一个映射
        value = _mapping(root["initial_pose"], "initial_pose")
        # 解析 x, y, yaw(自动从度转为弧度)
        x, y, yaw = _pose(value, "initial_pose")
        # 解析协方差(covariance 也是可选的, 默认空字典)
        covariance = _mapping(
            value.get("covariance", {}),
            "initial_pose.covariance",
        )
        # 创建 InitialPoseConfig 不可变对象
        initial_pose = InitialPoseConfig(
            x=x,
            y=y,
            yaw=yaw,
            # 协方差必须 > 0(协方差为 0 会导致数学问题)
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

    # --- 路线解析 ---
    # routes_value 是所有路线的映射 {路线名: {路线配置...}}
    routes_value = _mapping(root.get("routes", {}), "routes")
    routes: Dict[str, InspectionRoute] = {}

    # 遍历每条路线
    for route_name, route_raw in routes_value.items():
        # 路线名不能为空
        name = str(route_name).strip()
        if not name:
            raise RouteConfigError("路线名称不能为空")

        # 路线配置必须是映射
        value = _mapping(route_raw, f"routes.{name}")

        # 默认静止观察时间(可被单个航点覆盖，正常巡检建议为 0)
        default_dwell = _nonnegative_number(
            value, "default_dwell_sec", f"routes.{name}"
        )

        # --- 航点列表解析 ---
        waypoint_values = value.get("waypoints")
        # 必须是列表, 且不能为空
        if not isinstance(waypoint_values, list) or not waypoint_values:
            raise RouteConfigError(
                f"routes.{name}.waypoints 必须是非空列表"
            )

        waypoints = []
        # waypoint_ids 集合用于检测重复 ID
        waypoint_ids = set()

        # 遍历每个航点(enumerate 返回索引和值)
        for index, waypoint_raw in enumerate(waypoint_values):
            # 航点路径: routes.路线名.waypoints[索引]
            context = f"routes.{name}.waypoints[{index}]"
            waypoint = _mapping(waypoint_raw, context)

            # 航点 ID 是必需的文本字段
            waypoint_id = _text(waypoint, "id", context)

            # 检查 ID 唯一性
            if waypoint_id in waypoint_ids:
                raise RouteConfigError(
                    f'路线 "{name}" 存在重复巡检点: {waypoint_id}'
                )
            waypoint_ids.add(waypoint_id)

            # 解析位姿(x, y, yaw_deg)
            x, y, yaw = _pose(waypoint, context)

            # 静止观察时间: 优先用航点自己的, 否则用路线默认值
            dwell_sec = float(waypoint.get("dwell_sec", default_dwell))
            # 停留时间不能为负
            if not math.isfinite(dwell_sec) or dwell_sec < 0.0:
                raise RouteConfigError(f"{context}.dwell_sec 不能小于 0")

            # 创建 InspectionWaypoint 不可变对象
            waypoints.append(
                InspectionWaypoint(
                    waypoint_id=waypoint_id,
                    # description 可以为空
                    description=str(waypoint.get("description", "")).strip(),
                    x=x,
                    y=y,
                    yaw=yaw,
                    dwell_sec=dwell_sec,
                )
            )

        # --- 路线级参数解析 ---
        # repeat_count(重复圈数): 至少为 1
        repeat_count = int(value.get("repeat_count", 1))
        # max_retries(最大重试次数): 不能为负
        max_retries = int(value.get("max_retries", 1))
        if repeat_count < 1:
            raise RouteConfigError(f"routes.{name}.repeat_count 必须至少为 1")
        if max_retries < 0:
            raise RouteConfigError(f"routes.{name}.max_retries 不能小于 0")

        # 创建 InspectionRoute 不可变对象
        routes[name] = InspectionRoute(
            name=name,
            description=str(value.get("description", "")).strip(),
            # target_gas 是必需的(需要知道检测什么气体)
            target_gas=_text(value, "target_gas", f"routes.{name}"),
            # alarm_threshold 必须是正数
            alarm_threshold=_positive_number(
                value, "alarm_threshold", f"routes.{name}"
            ),
            # stop_on_critical_risk 默认为 True(安全优先)
            stop_on_critical_risk=bool(
                value.get("stop_on_critical_risk", True)
            ),
            repeat_count=repeat_count,
            # continue_on_failure 默认为 False(一失败就停)
            continue_on_failure=bool(
                value.get("continue_on_failure", False)
            ),
            max_retries=max_retries,
            # 导航超时必须是正数
            navigation_timeout_sec=_positive_number(
                value, "navigation_timeout_sec", f"routes.{name}"
            ),
            waypoints=waypoints,
        )

    # 至少需要一条路线, 否则没有任务可执行
    if not routes:
        raise RouteConfigError("routes 至少需要定义一条巡检路线")

    # 返回最终的 RouteBook 不可变对象
    return RouteBook(
        frame_id=frame_id,
        site_configured=site_configured,
        initial_pose=initial_pose,
        routes=routes,
    )
