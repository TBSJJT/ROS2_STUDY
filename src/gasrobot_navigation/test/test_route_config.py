#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
巡检路线配置读取与校验的单元测试。

本测试文件验证 route_config 模块的核心功能：
- load_route_book() 能正确解析合法的 YAML 配置文件
- 角度从度自动转换为弧度
- 默认值（如停留时间）能正确继承
- 非法的参数值被 RouteConfigError 拒绝
- 重复的航点 ID 被拒绝
- 查询不存在的路线返回可读的错误信息

测试使用 pytest 的 tmp_path 夹具（fixture）创建临时文件，
避免测试污染实际的配置文件。
"""

import math

import pytest

# 导入被测试的函数和异常类
from gasrobot_navigation.route_config import RouteConfigError, load_route_book


# -----------------------------------------------------------------------
# 辅助函数
# -----------------------------------------------------------------------
def _write_route(tmp_path, content: str):
    """
    把测试路线内容写入独立的临时 YAML 文件。

    参数:
        tmp_path: pytest 提供的临时目录路径（每个测试独立）
        content:  YAML 文件内容字符串

    返回:
        临时文件的 Path 对象

    """
    # tmp_path / "routes.yaml"：在临时目录下创建 routes.yaml 文件
    # Python 的 / 运算符被 pathlib 重载了，用于拼接路径
    path = tmp_path / "routes.yaml"
    # write_text()：写入字符串内容，encoding="utf-8" 指定编码
    path.write_text(content, encoding="utf-8")
    return path


def _valid_route() -> str:
    """
    返回包含初始化位姿和两个巡检点的最小合法 YAML 配置。

    这是一个"最小合法配置"：包含了所有必需的字段，
    每条路线有两个航点作典型场景。
    测试通过修改这个字符串中的特定值来验证校验逻辑。

    关键字段说明：
    - version: 1：配置文件版本号（当前只支持 1）
    - frame_id: map：坐标在 map 坐标系中
    - site_configured: true：确认场地已经标定
    - initial_pose：AMCL 初始位姿 + 协方差
    - routes：路线字典
      - target_gas：目标气体类型
      - alarm_threshold：报警阈值
      - default_dwell_sec：全局默认停留时间（航点可覆盖）
      - navigation_timeout_sec：导航超时
      - waypoints：航点列表

    """
    return """
version: 1
frame_id: map
site_configured: true
initial_pose:
  x: 1.0
  y: -2.0
  yaw_deg: 90.0
  covariance: {x: 0.25, y: 0.25, yaw: 0.0685}
routes:
  standard_route:
    description: 标准巡检路线
    target_gas: methane
    alarm_threshold: 1000.0
    stop_on_critical_risk: true
    repeat_count: 2
    default_dwell_sec: 5.0
    max_retries: 2
    continue_on_failure: false
    navigation_timeout_sec: 180.0
    waypoints:
      - {id: point_a, x: 1.0, y: 2.0, yaw_deg: 180.0}
      - id: point_b
        description: 阀门区域检测点
        x: -1.0
        y: 3.0
        yaw_deg: -90.0
        dwell_sec: 8.0
"""


# -----------------------------------------------------------------------
# 测试 1：合法的配置文件能正确解析
# -----------------------------------------------------------------------
def test_load_route_book_converts_angles_and_defaults(tmp_path):
    """
    验证角度能正确从度转为弧度，以及默认值和业务字段正确读取。

    这个测试覆盖了 load_route_book() 的核心解析流程：
    - frame_id → "map"
    - site_configured → True
    - initial_pose.yaw：90° → π/2 弧度
    - repeat_count → 2
    - 航点 0 的航向：180° → π 弧度
    - 航点 0 的停留时间：应该是默认值 5.0（没有显式指定）
    - 航点 1 的停留时间：应该是显式指定的 8.0

    参数:
        tmp_path: pytest 提供的临时目录（自动创建和清理）

    """
    # 1. 写入测试文件
    # 2. 加载为 RouteBook
    book = load_route_book(str(_write_route(tmp_path, _valid_route())))
    # 3. 按名称获取路线
    route = book.route("standard_route")

    # === 基本字段 ===
    # 断言坐标系名称
    assert book.frame_id == "map"
    # 断言场地已标定
    assert book.site_configured

    # === 角度转换 ===
    # initial_pose.yaw_deg=90.0 → 90° × π/180 = π/2 弧度
    assert book.initial_pose.yaw == pytest.approx(math.pi / 2.0)

    # === 路线级字段 ===
    # 圈数为 2
    assert route.repeat_count == 2

    # === 航点 0 ===
    # yaw_deg=180.0 → π 弧度
    assert route.waypoints[0].yaw == pytest.approx(math.pi)
    # 没有指定 dwell_sec → 使用路线默认值 5.0
    assert route.waypoints[0].dwell_sec == pytest.approx(5.0)

    # === 航点 1 ===
    # 显式指定 dwell_sec=8.0 → 覆盖默认值
    assert route.waypoints[1].dwell_sec == pytest.approx(8.0)
    # description 被正确读取
    assert "阀门" in route.waypoints[1].description


# -----------------------------------------------------------------------
# 测试 2：非法参数值被拒绝（参数化测试）
# -----------------------------------------------------------------------
@pytest.mark.parametrize(
    "old, new, error_text",
    (
        # 每行：(原始字符串, 替换后字符串, 期望在错误信息中出现的文本)
        # 测试不支持的版本号
        ("version: 1", "version: 2", "version: 1"),
        # 测试 repeat_count 为 0
        ("repeat_count: 2", "repeat_count: 0", "repeat_count"),
        # 测试负的重试次数
        ("max_retries: 2", "max_retries: -1", "max_retries"),
        # 测试报警阈值为 0（必须是正数）
        ("alarm_threshold: 1000.0", "alarm_threshold: 0", "alarm_threshold"),
    ),
)
def test_invalid_route_values_are_rejected(tmp_path, old, new, error_text):
    """
    验证危险的版本号、圈数、重试次数和阈值会被 RouteConfigError 拒绝。

    使用 pytest.mark.parametrize 参数化测试：
    同一个测试函数运行 4 次，每次测试一个不同的非法参数。

    参数:
        tmp_path:   临时目录
        old:        要替换的原始字符串片段
        new:        替换后的非法值字符串
        error_text: 期望在异常信息中出现的文本

    测试方法：
    1. 用 .replace(old, new) 把合法配置中的一个值改成非法的
    2. 尝试加载这个非法配置
    3. 断言抛出 RouteConfigError 且消息包含预期文本

    """
    # 把合法 YAML 中的一个值替换为非法值
    content = _valid_route().replace(old, new)

    # pytest.raises(异常类型, match=匹配模式)：
    #   断言 with 块中的代码会抛出指定异常
    #   match 参数是正则表达式，匹配异常消息
    with pytest.raises(RouteConfigError, match=error_text):
        load_route_book(str(_write_route(tmp_path, content)))


# -----------------------------------------------------------------------
# 测试 3：重复航点 ID 被拒绝
# -----------------------------------------------------------------------
def test_duplicate_waypoint_ids_are_rejected(tmp_path):
    """
    验证重复的巡检点名称不会进入任务执行阶段。

    为什么要检查重复 ID？
    - 航点 ID 用于关联传感器数据和日志记录
    - 如果两个航点有相同的 ID，无法区分数据来自哪里
    - 在配置阶段就拒绝，避免运行时产生混乱的数据

    测试方法：
    把第二个航点的 ID "point_b" 改成 "point_a"（与第一个重复），
    然后断言加载会失败。

    """
    # 把 "id: point_b" 替换为 "id: point_a"，制造重复
    content = _valid_route().replace("id: point_b", "id: point_a")

    # 期望抛出 RouteConfigError，消息中包含"重复巡检点"
    with pytest.raises(RouteConfigError, match="重复巡检点"):
        load_route_book(str(_write_route(tmp_path, content)))


# -----------------------------------------------------------------------
# 测试 4：不存在的路线返回友好错误
# -----------------------------------------------------------------------
def test_unknown_route_has_readable_error(tmp_path):
    """
    验证查询不存在的路线名称时，返回包含可用路线列表的可读错误。

    好的错误消息示例：
    "未找到巡检路线"night_route"，可用路线：standard_route"

    而不是：
    KeyError: 'night_route'（对用户不友好）

    这个测试确保 RouteBook.route() 在路线不存在时：
    1. 抛出 RouteConfigError（而不是 KeyError）
    2. 错误消息中包含要查询的名称
    3. 错误消息中列出了所有可用的路线名

    """
    # 先正常加载配置
    book = load_route_book(str(_write_route(tmp_path, _valid_route())))

    # 尝试获取不存在的路线（"night_route" 不在配置中）
    # 期望抛出 RouteConfigError
    # match="standard_route"：错误消息应该列出可用的路线名
    with pytest.raises(RouteConfigError, match="standard_route"):
        book.route("night_route")
