"""确保重复采样覆盖有向边、不吞失败，也不以一条样本冒充稳定时间。"""

from collections import Counter
import math
import pytest
from delivery_planner.calibration import schedule, summarize


def test_euler_schedule_covers_42_pairs_and_returns():
    names = ["depot", *[f"S{i}" for i in range(1, 7)]]
    first = schedule(names, "depot", 42)
    assert first[0] == first[-1] == "depot"
    assert len(first) == 43
    assert Counter(zip(first, first[1:])) == Counter(
        {(a, b): 1 for a in names for b in names if a != b}
    )
    assert first == schedule(names, "depot", 42)
    assert first != schedule(names, "depot", 43)


def sample(a, b, k, value=10, status="succeeded"):
    return {"from": a, "to": b, "round": k, "travel_time_sec": value, "status": status}


def test_mean_std_and_directional_coverage():
    rows = [sample("A", "B", k, value) for k, value in enumerate([10, 12, 14])]
    result = summarize(["A", "B"], rows, 3)
    assert not result["complete"]
    assert result["travel_time_sec"] == [[0, 12], [None, 0]]
    assert result["pairs"][0]["stddev_sec"] == 2
    rows += [sample("B", "A", k, 20) for k in range(3)]
    assert summarize(["A", "B"], rows, 3)["complete"]


def test_failed_trial_not_deleted_or_used_as_zero():
    rows = [sample("A", "B", k) for k in range(3)] + [sample("A", "B", 3, None, "failed")]
    result = summarize(["A", "B"], rows, 3)
    assert result["pairs"][0]["successes"] == 3 and result["pairs"][0]["failures"] == 1
    assert result["travel_time_sec"][0][1] is None


def test_single_trial_is_not_complete_and_std_is_unknown():
    result = summarize(["A", "B"], [sample("A", "B", 0)])
    assert result["pairs"][0]["stddev_sec"] is None
    assert not result["complete"] and result["travel_time_sec"][0][1] is None


@pytest.mark.parametrize("value", [None, -1, 0, float("nan"), True])
def test_invalid_duration(value):
    with pytest.raises(ValueError):
        summarize(["A", "B"], [sample("A", "B", 0, value)])


def test_duplicate_round_pair_rejected():
    with pytest.raises(ValueError, match="duplicate"):
        summarize(["A", "B"], [sample("A", "B", 0), sample("A", "B", 0)])


def make_time_archive(tmp_path):
    import hashlib
    from delivery_planner.costs import write_json, digest

    names = ["A", "B"]
    samples = [sample(a, b, k, 10 + k) for k in range(3) for a, b in [("A", "B"), ("B", "A")]]
    runtime = {"controller_server": {"max_speed": 0.15}}
    write_json(tmp_path / "runtime_parameters.json", runtime)
    (tmp_path / "trajectory.csv").write_text("ros_time,x,y\n0,0,0\n")
    data = dict(
        version=1,
        time_source="independent_gazebo_navigation_ros_time_mean",
        stations=names,
        distance_content_sha256="distance-hash",
        finished=True,
        error=None,
        collisions=[],
        samples=samples,
        runtime_parameters_sha256=digest(runtime),
        trajectory_sha256=hashlib.sha256((tmp_path / "trajectory.csv").read_bytes()).hexdigest(),
    )
    data.update(summarize(names, samples, 3))
    data["content_sha256"] = digest(data)
    write_json(tmp_path / "time_matrix.json", data)
    return tmp_path / "time_matrix.json", dict(stations=names, content_sha256="distance-hash")


def test_time_archive_binding_and_raw_trajectory(tmp_path):
    from delivery_planner.calibration import load_time_cache

    path, distance = make_time_archive(tmp_path)
    assert load_time_cache(path, distance)["travel_time_sec"] == [[0, 11], [11, 0]]
    with pytest.raises(ValueError, match="not bound"):
        load_time_cache(path, dict(distance, content_sha256="other"))
    (tmp_path / "trajectory.csv").write_text("edited")
    with pytest.raises(ValueError, match="trajectory hash"):
        load_time_cache(path, distance)


def test_user_example_percentile_and_failure_denominator():
    rows = [sample("A", "B", k, v) for k, v in enumerate([13.2, 13.8, 13.4, 14.1, 13.5])]
    row = summarize(["A", "B"], rows)["pairs"][0]
    assert row["mean_sec"] == pytest.approx(13.6)
    assert row["p95_sec"] == pytest.approx(14.04)
    assert row["n"] == 5 and row["success_rate"] == 1 and row["usable"]
    rows += [
        dict(sample("A", "B", 5, None, "failed"), failure_kind="navigation_timeout"),
        sample("A", "B", 6, None, "in_progress"),
    ]
    row = summarize(["A", "B"], rows)["pairs"][0]
    assert row["failures"] == 1 and row["pending"] == 1 and row["timeouts"] == 1
    assert row["success_rate"] == pytest.approx(5 / 6)
    assert row["mean_sec"] == pytest.approx(13.6) and not row["usable"]


def test_pending_only_does_not_claim_failure_rate():
    row = summarize(["A", "B"], [sample("A", "B", 0, None, "in_progress")])["pairs"][0]
    assert row["failures"] == 0 and row["pending"] == 1 and row["success_rate"] is None
