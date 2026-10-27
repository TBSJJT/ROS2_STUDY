"""验证有向路径、失败保留和归档损坏检测。"""

import hashlib
import json
import math
import pytest
from delivery_planner.costs import (
    assemble_matrix,
    digest,
    load_distance_cache,
    path_length,
    write_json,
)


def test_path_is_polyline_not_endpoint_distance():
    assert path_length([[0, 0], [3, 0], [3, 4]]) == 7


@pytest.mark.parametrize(
    "points", [[], [[0, 0]], [[0, 0], [math.nan, 2]], [[0, 0], [2, math.inf]], [[0, 0], [True, 2]]]
)
def test_invalid_paths(points):
    with pytest.raises(ValueError):
        path_length(points)


def test_directed_costs_and_failures_are_not_mirrored():
    names = ["depot", "S1", "S2"]
    records = [
        {"from": "depot", "to": "S1", "status": "reachable", "path_xy_m": [[0, 0], [3, 4]]},
        {
            "from": "S1",
            "to": "depot",
            "status": "reachable",
            "path_xy_m": [[3, 4], [3, 0], [0, 0]],
        },
        {"from": "S1", "to": "S2", "status": "planning_failed", "path_xy_m": []},
    ]
    assert assemble_matrix(names, records) == [[0, 5, None], [7, 0, None], [None, None, 0]]
    with pytest.raises(ValueError, match="duplicate"):
        assemble_matrix(names, records + [records[0]])


def make_cache(tmp_path):
    snapshot = tmp_path / "snapshot.txt"
    snapshot.write_text("original configuration")
    hashes = {"snapshot.txt": hashlib.sha256(snapshot.read_bytes()).hexdigest()}
    write_json(tmp_path / "provenance.json", {"sha256": hashes})
    costmap = {"data": [0, 254]}
    write_json(tmp_path / "costmap.json", costmap)
    pairs = [
        {"from": "A", "to": "B", "status": "reachable", "path_xy_m": [[0, 0], [3, 4]]},
        {"from": "B", "to": "A", "status": "planning_failed", "path_xy_m": []},
    ]
    data = {
        "version": 1,
        "distance_source": "nav2_static_planned_path",
        "complete": True,
        "stations": ["A", "B"],
        "pairs": pairs,
        "distance_m": [[0, 5], [None, 0]],
        "config_sha256": digest(hashes),
        "costmap_sha256": digest(costmap),
    }
    data["content_sha256"] = digest(data)
    path = tmp_path / "matrix.json"
    write_json(path, data)
    return path


def test_snapshot_binding(tmp_path):
    path = make_cache(tmp_path)
    cache = load_distance_cache(path)
    assert cache["distance_m"][1][0] is None
    with pytest.raises(ValueError, match="requested configuration"):
        load_distance_cache(path, "another configuration")
    (tmp_path / "snapshot.txt").write_text("edited configuration")
    with pytest.raises(ValueError, match="snapshot changed"):
        load_distance_cache(path)


@pytest.mark.parametrize("target", ["matrix.json", "costmap.json"])
def test_modified_matrix_or_costmap_rejected(tmp_path, target):
    path = make_cache(tmp_path)
    corrupt = tmp_path / target
    data = json.loads(corrupt.read_text())
    data["unexpected_edit"] = 1
    write_json(corrupt, data)
    with pytest.raises(ValueError, match="hash mismatch"):
        load_distance_cache(path)


def test_incomplete_cache_rejected_even_with_valid_checksum(tmp_path):
    path = make_cache(tmp_path)
    data = json.loads(path.read_text())
    data.pop("content_sha256")
    data["complete"] = False
    data["content_sha256"] = digest(data)
    write_json(path, data)
    with pytest.raises(ValueError, match="incomplete"):
        load_distance_cache(path)


def test_plan_repeat_never_averages_and_detects_same_length_different_path():
    from delivery_planner.costs import check_plan_trials

    a = {"status": "reachable", "path_xy_m": [[0, 0], [1, 0], [1, 1]]}
    b = {"status": "reachable", "path_xy_m": [[0, 0], [0, 1], [1, 1]]}
    assert check_plan_trials([a, a, a])["passed"]
    result = check_plan_trials([a, a, b])
    assert result["distance_spread_m"] == 0 and not result["passed"]
    assert not check_plan_trials([a, a, dict(a, status="planning_failed")])["passed"]
