from pathlib import Path
import math
import pytest
import yaml
from gasrobot_delivery.config import Batch, load_batch, load_stations
from gasrobot_delivery.engine import DeliveryEngine


def engine(service=5, retries=1):
    return DeliveryEngine(Batch("test", ("S1", "S2"), service, 180, retries), "depot")


def test_delivery_requires_service_and_return():
    e = engine()
    assert e.start(0)[0]
    assert not e.start(1)[0]
    e.nav_result(e.active, True, 10)
    assert e.state == "SERVICING" and e.completed == []
    e.tick(14.9)
    assert e.completed == []
    e.tick(15)
    assert e.completed == ["S1"] and e.target == "S2"
    e.nav_result(e.active, True, 25)
    e.tick(30)
    assert e.state == "RETURNING"
    e.nav_result(e.active, True, 35)
    assert e.state == "COMPLETED"


def test_pause_navigation_waits_for_terminal_result_and_ignores_stale_callbacks():
    e = engine()
    e.start(0)
    old = e.active
    assert e.pause(1)[0]
    assert not e.resume(2)[0]
    assert e.active == old
    e.nav_result(old, True, 3)  # Success races with cancel: do not unload.
    assert e.state == "PAUSED" and e.completed == []
    assert e.resume(4)[0]
    new = e.active
    assert new != old
    e.nav_result(old, True, 5)
    assert e.active == new and e.state == "NAVIGATING"


def test_service_pause_preserves_remaining_and_elapsed_includes_pause():
    e = engine()
    e.start(0)
    e.nav_result(e.active, True, 1)
    e.pause(3)
    assert e.remaining_service == 3
    e.tick(40)
    assert e.remaining_service == 3
    e.resume(41)
    e.tick(43.9)
    assert e.state == "SERVICING"
    e.tick(44)
    event = [x for x in e.events if x["event"] == "service_completed"][0]
    assert event["elapsed"] == 44 and e.completed == ["S1"]


def test_cancel_pending_acceptance_and_restart():
    e = engine()
    e.start(0)
    old = e.active
    e.cancel(1)
    assert e.state == "CANCELLING" and not e.start(2)[0]
    e.nav_result(old, True, 3)
    assert e.state == "CANCELLED" and e.completed == []
    assert e.start(4)[0]
    token = e.active
    e.nav_result(old, True, 5)
    assert e.active == token


def test_timeout_waits_for_cancel_then_retries_only_once():
    e = engine()
    e.start(0)
    first = e.active
    e.tick(180)
    assert e.active == first and e.state == "CANCELLING"
    e.nav_result(first, False, 181)
    assert e.retry == 1 and e.active != first
    e.tick(361)
    e.nav_result(e.active, False, 362)
    assert e.state == "FAILED" and not e.completed


def test_unconfirmed_cancellation_fails_closed():
    e = engine()
    e.start(0)
    token = e.active
    e.cancel(1)
    e.tick(12)
    assert e.state == "FAILED" and not e.start(13)[0]
    e.nav_result(token, True, 14)
    assert e.state == "FAILED" and not e.completed


def test_simulation_pause_and_clock_reset():
    e = engine()
    e.start(100)
    e.nav_result(e.active, True, 101)
    e.tick(101)
    e.tick(101)
    assert e.remaining_service == 5
    e.tick(0)
    assert e.state == "FAILED"


@pytest.mark.parametrize(
    "field,value",
    [
        ("route", []),
        ("route", ["S1", "S1"]),
        ("route", ["missing"]),
        ("route", ["depot"]),
        ("service_sec", -1),
        ("service_sec", math.nan),
        ("service_sec", True),
        ("navigation_timeout_sec", 0),
        ("max_retries", 1.5),
    ],
)
def test_invalid_batch(tmp_path, field, value):
    data = dict(
        version=1,
        batch_id="test",
        route=["S1"],
        service_sec=5,
        navigation_timeout_sec=180,
        max_retries=1,
    )
    data[field] = value
    p = tmp_path / "batch.yaml"
    p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_batch(p, {"S1": {}, "depot": {}}, "depot")


def test_committed_stations_bound_to_map_and_clearance():
    root = Path(__file__).resolve().parents[2]
    book = load_stations(
        root / "gasrobot_delivery/config/stations.yaml",
        root / "gasrobot_gas_mapping/maps/workshop_delivery_v1.yaml",
    )
    assert len(book["stations"]) == 7


def test_changed_map_hash_rejected(tmp_path):
    root = Path(__file__).resolve().parents[2]
    original = root / "gasrobot_delivery/config/stations.yaml"
    data = yaml.safe_load(original.read_text())
    data["map_image_sha256"] = "incorrect"
    p = tmp_path / "stations.yaml"
    p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="hash"):
        load_stations(p, root / "gasrobot_gas_mapping/maps/workshop_delivery_v1.yaml")


def test_cancel_during_service_never_marks_delivery_complete():
    e = engine()
    e.start(0)
    e.nav_result(e.active, True, 1)
    e.tick(3)
    assert e.cancel(3)[0]
    e.tick(100)
    assert e.state == "CANCELLED" and e.completed == []


def test_cancel_overrides_pending_pause():
    e = engine()
    e.start(0)
    token = e.active
    e.pause(1)
    e.cancel(2)
    e.nav_result(token, False, 3)
    assert e.state == "CANCELLED" and not e.resume(4)[0]


def test_failure_never_marks_station_delivered():
    e = engine(retries=1)
    e.start(0)
    e.nav_result(e.active, False, 1, "unreachable")
    e.nav_result(e.active, False, 2, "unreachable")
    assert e.state == "FAILED" and e.completed == []
    assert len([event for event in e.events if event["event"] == "retry"]) == 1


def test_zero_service_completes_on_next_tick_and_returns():
    e = engine(service=0)
    e.start(0)
    e.nav_result(e.active, True, 1)
    assert e.completed == []
    e.tick(1)
    assert e.completed == ["S1"]


def test_malformed_yaml_is_reported_as_configuration_error(tmp_path):
    p = tmp_path / "batch.yaml"
    p.write_text("version: 1\nroute: [unterminated")
    with pytest.raises(ValueError, match="invalid YAML"):
        load_batch(p, {"S1": {}, "depot": {}}, "depot")


def test_new_batch_clears_cancelled_service_remaining():
    e = engine()
    e.start(0)
    e.nav_result(e.active, True, 1)
    e.cancel(2)
    assert e.state == "CANCELLED"
    assert e.start(3)[0]
    assert e.remaining_service == 0
    assert e.completed == []


def test_duplicate_yaml_station_definition_is_rejected(tmp_path):
    from gasrobot_delivery.config import mapping

    path = tmp_path / "stations.yaml"
    path.write_text("version: 1\nstations:\n  S1: {x: 1}\n  S1: {x: 2}\n")
    with pytest.raises(ValueError, match="duplicate YAML key: S1"):
        mapping(path)
