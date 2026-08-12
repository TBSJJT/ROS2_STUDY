"""巡检路线配置读取与校验测试。"""

import math

import pytest

from gasrobot_navigation.route_config import RouteConfigError, load_route_book


def _write_route(tmp_path, content: str):
    """把测试路线写入独立临时文件。"""

    path = tmp_path / "routes.yaml"
    path.write_text(content, encoding="utf-8")
    return path


def _valid_route() -> str:
    """返回包含初始化位姿和两个巡检点的最小合法配置。"""

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
    description: 标准路线
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
        description: 阀门区域
        x: -1.0
        y: 3.0
        yaw_deg: -90.0
        dwell_sec: 8.0
"""


def test_load_route_book_converts_angles_and_defaults(tmp_path):
    """验证角度转换、默认停留时间和业务字段。"""

    book = load_route_book(str(_write_route(tmp_path, _valid_route())))
    route = book.route("standard_route")

    assert book.frame_id == "map"
    assert book.site_configured
    assert book.initial_pose.yaw == pytest.approx(math.pi / 2.0)
    assert route.repeat_count == 2
    assert route.waypoints[0].yaw == pytest.approx(math.pi)
    assert route.waypoints[0].dwell_sec == pytest.approx(5.0)
    assert route.waypoints[1].dwell_sec == pytest.approx(8.0)


@pytest.mark.parametrize(
    "old, new, error_text",
    (
        ("version: 1", "version: 2", "version: 1"),
        ("repeat_count: 2", "repeat_count: 0", "repeat_count"),
        ("max_retries: 2", "max_retries: -1", "max_retries"),
        ("alarm_threshold: 1000.0", "alarm_threshold: 0", "alarm_threshold"),
    ),
)
def test_invalid_route_values_are_rejected(tmp_path, old, new, error_text):
    """验证危险的版本、圈数、重试和阈值会被拒绝。"""

    content = _valid_route().replace(old, new)
    with pytest.raises(RouteConfigError, match=error_text):
        load_route_book(str(_write_route(tmp_path, content)))


def test_duplicate_waypoint_ids_are_rejected(tmp_path):
    """验证重复巡检点名称不会进入任务执行阶段。"""

    content = _valid_route().replace("id: point_b", "id: point_a")
    with pytest.raises(RouteConfigError, match="重复巡检点"):
        load_route_book(str(_write_route(tmp_path, content)))


def test_unknown_route_has_readable_error(tmp_path):
    """验证不存在的路线名称会返回可用路线列表。"""

    book = load_route_book(str(_write_route(tmp_path, _valid_route())))
    with pytest.raises(RouteConfigError, match="standard_route"):
        book.route("night_route")
