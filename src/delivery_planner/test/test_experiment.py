"""用明确的人工夹具验证完整冻结/读取流程，不把夹具当成仿真测量。"""

import hashlib
import json
import pytest
import yaml
from delivery_planner.costs import digest, write_json, check_plan_trials
from delivery_planner.calibration import summarize
from delivery_planner.experiment import freeze, load_bundle


def fixtures(root, repeats=5):
    d, t = root / "distance", root / "time"
    d.mkdir()
    t.mkdir()
    write_json(d / "costmap.json", {"data": [0, 0]})
    (d / "config.txt").write_text("synthetic test only")
    hashes = {"config.txt": hashlib.sha256((d / "config.txt").read_bytes()).hexdigest()}
    write_json(d / "provenance.json", {"sha256": hashes})
    pairs = []
    for a, b in [("A", "B"), ("B", "A")]:
        trial = {"from": a, "to": b, "status": "reachable", "path_xy_m": [[0, 0], [1, 0]]}
        pairs.append(
            dict(trial, planning_trials=[trial] * 3, consistency=check_plan_trials([trial] * 3))
        )
    distance = dict(
        version=1,
        distance_source="nav2_static_planned_path",
        complete=True,
        all_reachable=True,
        stations=["A", "B"],
        depot="A",
        pairs=pairs,
        distance_m=[[0, 1], [1, 0]],
        config_sha256=digest(hashes),
        costmap_sha256=digest({"data": [0, 0]}),
        planning_consistency={"epsilon_m": 1e-6},
    )
    distance["content_sha256"] = digest(distance)
    write_json(d / "matrix.json", distance)
    samples = [
        {"from": a, "to": b, "round": k, "travel_time_sec": 10 + k, "status": "succeeded"}
        for k in range(repeats)
        for a, b in [("A", "B"), ("B", "A")]
    ]
    write_json(t / "runtime_parameters.json", {})
    (t / "trajectory.csv").write_text("synthetic test only")
    times = dict(
        version=1,
        time_source="independent_gazebo_navigation_ros_time_mean",
        stations=["A", "B"],
        finished=True,
        error=None,
        collisions=[],
        samples=samples,
        distance_content_sha256=distance["content_sha256"],
        runtime_parameters_sha256=digest({}),
        trajectory_sha256=hashlib.sha256((t / "trajectory.csv").read_bytes()).hexdigest(),
    )
    times.update(summarize(["A", "B"], samples, repeats))
    times["content_sha256"] = digest(times)
    write_json(t / "time_matrix.json", times)
    instance = root / "instance.yaml"
    instance.write_text(
        yaml.safe_dump(
            {
                "version": 1,
                "instance_id": "test-only",
                "tasks": {"B": {"due_sec": 15, "service_sec": 5}},
            }
        )
    )
    return d / "matrix.json", t / "time_matrix.json", instance


def test_freeze_roundtrip_and_mutation_rejection(tmp_path):
    inputs = fixtures(tmp_path)
    output = tmp_path / "frozen"
    manifest = freeze(*inputs, output)
    paths = load_bundle(output)
    assert all(p.is_relative_to(output) for p in paths)
    assert len(manifest["evaluation_input_sha256"]) == 64
    (output / "instance.yaml").write_text(
        (output / "instance.yaml").read_text().replace("15", "16")
    )
    with pytest.raises(ValueError, match="hash mismatch"):
        load_bundle(output)


def test_three_pilot_samples_cannot_be_frozen_formally(tmp_path):
    inputs = fixtures(tmp_path, 3)
    with pytest.raises(ValueError, match="five"):
        freeze(*inputs, tmp_path / "rejected")
    assert not (tmp_path / "rejected").exists()
