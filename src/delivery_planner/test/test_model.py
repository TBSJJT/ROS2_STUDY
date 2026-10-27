"""用手算用例和独立目标公式验证卸货时刻、回程、排列与 Pareto。"""

from itertools import permutations
import math
import pytest
import yaml
from delivery_planner.model import load_instance, evaluate, enumerate_routes, validate_matrix

NAMES = ["depot", "A", "B"]
DISTANCE = [[0, 1, 5], [4, 0, 2], [3, 6, 0]]
TIME = [[0, 10, 50], [40, 0, 20], [30, 60, 0]]
INSTANCE = {
    "tasks": {"A": {"due_sec": 15, "service_sec": 5}, "B": {"due_sec": 37, "service_sec": 5}}
}


def test_service_completion_and_return():
    row = evaluate(["A", "B"], INSTANCE, NAMES, "depot", DISTANCE, TIME)
    assert row["feasible"]
    assert row["distance_m"] == 6  # 1+2+3，包含回程
    assert row["duration_sec"] == 70  # 10+5+20+5+30
    assert row["tardiness_sec"] == 3  # A 完成于15恰好按时；B到达35，完成40迟交3
    assert row["visits"][1]["arrival_sec"] == 35
    assert row["visits"][1]["completion_sec"] == 40


@pytest.mark.parametrize("route", [[], ["A"], ["A", "A"], ["A", "B", "A"], ["A", "C"]])
def test_invalid_permutation(route):
    with pytest.raises(ValueError):
        evaluate(route, INSTANCE, NAMES, "depot", DISTANCE, TIME)


def test_missing_edge_including_return_is_infeasible():
    for a, b in [(0, 1), (2, 0)]:
        times = [r[:] for r in TIME]
        times[a][b] = None
        result = evaluate(["A", "B"], INSTANCE, NAMES, "depot", DISTANCE, times)
        assert not result["feasible"] and result["distance_m"] is None


@pytest.mark.parametrize("value", [-1, float("nan"), float("inf"), True])
def test_illegal_times(value):
    times = [r[:] for r in TIME]
    times[0][1] = value
    with pytest.raises(ValueError):
        validate_matrix(times, NAMES, "time")


def test_manual_config_and_duplicate_keys(tmp_path):
    raw = {"version": 1, "instance_id": "manual", "tasks": INSTANCE["tasks"]}
    path = tmp_path / "tasks.yaml"
    path.write_text(yaml.safe_dump(raw))
    assert load_instance(path, NAMES, "depot")["tasks"]["A"]["due_sec"] == 15
    raw["tasks"]["A"]["service_sec"] = -1
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError):
        load_instance(path, NAMES, "depot")
    path.write_text("version: 1\ninstance_id: a\ntasks:\n  A: {}\n  A: {}\n")
    with pytest.raises(ValueError, match="duplicate"):
        load_instance(path, NAMES, "depot")


def test_720_permutations_against_independent_formula():
    # 人工数学测试实例，不是 Gazebo 时间测量。用独立前缀和公式交叉验证。
    names = ["depot", *[f"S{i}" for i in range(1, 7)]]
    n = len(names)
    distances = [[0 if i == j else 1 + (i * 3 + j * 7) % 13 for j in range(n)] for i in range(n)]
    times = [[3 * x for x in row] for row in distances]
    tasks = {name: {"due_sec": 25 * i, "service_sec": 2 * i} for i, name in enumerate(names) if i}
    instance = {"tasks": tasks}
    results = enumerate_routes(instance, names, "depot", distances, times)
    assert results["permutation_count"] == 720 and results["feasible_count"] == 720
    objectives = {}
    for route in permutations(range(1, 7)):
        full = (0, *route, 0)
        distance = sum(distances[a][b] for a, b in zip(full, full[1:]))
        increments = [times[a][b] + tasks[names[b]]["service_sec"] for a, b in zip(full, route)]
        tardiness = sum(
            max(0, sum(increments[: k + 1]) - tasks[names[b]]["due_sec"])
            for k, b in enumerate(route)
        )
        objectives[tuple(names[i] for i in route)] = (distance, tardiness)
    for row in results["results"]:
        assert (row["distance_m"], row["tardiness_sec"]) == objectives[tuple(row["route"])]
    expected = {
        route
        for route, value in objectives.items()
        if not any(
            other[0] <= value[0] and other[1] <= value[1] and other != value
            for other in objectives.values()
        )
    }
    assert {tuple(row["route"]) for row in results["pareto"]} == expected


@pytest.mark.parametrize(
    "tasks",
    [
        {},
        {"depot": {"due_sec": 0, "service_sec": 0}},
        {"missing": {"due_sec": 0, "service_sec": 0}},
        {"A": {"due_sec": None, "service_sec": 0}},
    ],
)
def test_empty_unknown_and_missing_deadline_rejected(tmp_path, tasks):
    path = tmp_path / "instance.yaml"
    path.write_text(yaml.safe_dump({"version": 1, "instance_id": "invalid", "tasks": tasks}))
    with pytest.raises(ValueError):
        load_instance(path, NAMES, "depot")


def test_tardiness_at_mean_is_not_expected_tardiness():
    instance = {"tasks": {"A": {"due_sec": 10, "service_sec": 0}}}

    def score(travel):
        return evaluate(
            ["A"], instance, ["depot", "A"], "depot", [[0, 1], [1, 0]], [[0, travel], [1, 0]]
        )["tardiness_sec"]

    assert score(10) == 0
    assert (score(5) + score(15)) / 2 == 2.5
