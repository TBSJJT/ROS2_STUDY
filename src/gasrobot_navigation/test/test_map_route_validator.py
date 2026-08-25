"""验证巡检航点不会进入障碍物、未知区域或贴近边界。"""

from pathlib import Path

import pytest

from gasrobot_navigation.map_route_validator import (
    load_occupancy_map,
    validate_route_book_against_map,
)
from gasrobot_navigation.route_config import (
    InspectionRoute,
    InspectionWaypoint,
    RouteBook,
    RouteConfigError,
)


def _write_test_map(tmp_path: Path) -> Path:
    """生成包含自由、未知和障碍栅格的最小 PGM 测试地图。"""

    width = 20
    height = 20
    pixels = bytearray([254] * (width * height))
    pixels[10 * width + 10] = 205
    pixels[15 * width + 15] = 0
    (tmp_path / "test_map.pgm").write_bytes(
        b"P5\n20 20\n255\n" + bytes(pixels)
    )
    yaml_path = tmp_path / "test_map.yaml"
    yaml_path.write_text(
        """
image: test_map.pgm
mode: trinary
resolution: 0.1
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""",
        encoding="utf-8",
    )
    return yaml_path


def _route_book(x: float, y: float) -> RouteBook:
    """构造只含一个航点的测试路线。"""

    waypoint = InspectionWaypoint(
        waypoint_id="test_point",
        description="测试点",
        x=x,
        y=y,
        yaw=0.0,
        dwell_sec=0.0,
    )
    route = InspectionRoute(
        name="standard_route",
        description="测试路线",
        target_gas="ethanol",
        alarm_threshold=1.0,
        stop_on_critical_risk=True,
        repeat_count=1,
        continue_on_failure=False,
        max_retries=1,
        navigation_timeout_sec=60.0,
        waypoints=[waypoint],
    )
    return RouteBook(
        frame_id="map",
        site_configured=True,
        initial_pose=None,
        routes={route.name: route},
    )


def test_map_loader_preserves_saved_unknown_cells(tmp_path):
    """205 灰度在 free_thresh=0.196 时必须保持未知状态。"""

    occupancy_map = load_occupancy_map(str(_write_test_map(tmp_path)))
    assert occupancy_map.cell_state(0, 0) == "free"
    assert occupancy_map.cell_state(10, 10) == "unknown"
    assert occupancy_map.cell_state(15, 15) == "occupied"


def test_route_in_known_free_area_is_accepted(tmp_path):
    """具有足够安全距离的自由区航点应通过校验。"""

    map_path = _write_test_map(tmp_path)
    validate_route_book_against_map(
        _route_book(0.4, 1.5),
        str(map_path),
        minimum_clearance=0.2,
    )


def test_unknown_or_insufficient_clearance_is_rejected(tmp_path):
    """未知栅格和安全距离不足的航点都应被拒绝。"""

    map_path = _write_test_map(tmp_path)
    # 图像(10,10)对应OccupancyGrid坐标(10,9)。
    with pytest.raises(RouteConfigError, match="地图状态=unknown"):
        validate_route_book_against_map(
            _route_book(1.05, 0.95),
            str(map_path),
            minimum_clearance=0.0,
        )

    with pytest.raises(RouteConfigError, match="不足"):
        validate_route_book_against_map(
            _route_book(1.3, 0.4),
            str(map_path),
            minimum_clearance=0.3,
        )
