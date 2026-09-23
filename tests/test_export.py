"""Run download: the logs and metrics of a run, packed as a zip for evaluation.

The point of the feature is that the archive is *complete*. Both the EventBus
and the LiveHub are bounded ring buffers, so a run of any length holds only its
tail in either. These tests exist mostly to prove the download does not inherit
that truncation, and says so when it cannot avoid it.
"""

from __future__ import annotations

import io
import json
import time
import zipfile

import pytest
from fastapi.testclient import TestClient

from core.events import EventBus, EventType
from dashboard.api import create_app
from dashboard.export import build_archive, safe_name
from dashboard.session import SessionManager
from dashboard.websocket import LiveHub
from swarm_logging.event_logger import EventLogger
from tests.helpers import make_sim, run_until

MEMBERS = {"summary.json", "timeseries.csv", "events.csv", "events.jsonl",
           "scenario.json", "manifest.json"}

SPEC = {"uav_count": 4, "duration_s": 60.0, "speed": 20.0,
        "pois": [{"x_m": 400.0, "y_m": 200.0, "priority": 4, "survey_time_s": 20.0}]}


def open_zip(blob: bytes):
    """Return (stem, ZipFile). Every member lives under one run directory."""
    zf = zipfile.ZipFile(io.BytesIO(blob))
    assert zf.testzip() is None
    return zf.namelist()[0].split("/")[0], zf


def read_json(zf, stem: str, name: str):
    return json.loads(zf.read(f"{stem}/{name}"))


def wait_until(predicate, timeout_s: float = 10.0) -> bool:
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture()
def finished_run():
    hub = LiveHub()
    sim = make_sim(hub=hub)
    sim.start()
    run_until(sim, 40)
    sim.finish("duration reached")
    return sim, hub


# ------------------------------------------------------------------- the archive
def test_archive_holds_everything_a_results_directory_holds(finished_run):
    sim, _ = finished_run
    name, blob = build_archive(sim)
    assert name.endswith(".zip")
    stem, zf = open_zip(blob)
    assert {n.split("/")[1] for n in zf.namelist()} == MEMBERS


def test_summary_carries_the_evaluation_sections(finished_run):
    sim, _ = finished_run
    stem, zf = open_zip(build_archive(sim)[1])
    summary = read_json(zf, stem, "summary.json")
    assert {"run", "mission", "communication", "resilience", "safety", "efficiency"} <= set(summary)


def test_timeseries_has_one_row_per_sample_with_the_metric_columns(finished_run):
    sim, _ = finished_run
    stem, zf = open_zip(build_archive(sim)[1])
    lines = zf.read(f"{stem}/timeseries.csv").decode().strip().splitlines()
    assert len(lines) - 1 == len(sim.metrics.rows())
    assert {"t_s", "connectivity_ratio", "mean_route_pdr", "mean_latency_ms"} <= set(lines[0].split(","))


def test_the_two_event_files_describe_the_same_events(finished_run):
    sim, _ = finished_run
    stem, zf = open_zip(build_archive(sim)[1])
    jsonl = [json.loads(line) for line in
             zf.read(f"{stem}/events.jsonl").decode().strip().splitlines()]
    csv_rows = zf.read(f"{stem}/events.csv").decode().strip().splitlines()
    assert len(csv_rows) - 1 == len(jsonl)
    assert [e["seq"] for e in jsonl] == sorted(e["seq"] for e in jsonl)


# --------------------------------------------------------------- completeness
def test_the_archive_keeps_events_the_live_hub_has_already_dropped():
    # The hub is a ring buffer. Exporting from it would silently hand over the
    # tail of a run and call it the run.
    hub = LiveHub(max_events=10)
    sim = make_sim(hub=hub)
    sim.start()
    run_until(sim, 40)
    sim.push(force=True)
    sim.finish("duration reached")

    stem, zf = open_zip(build_archive(sim)[1])
    exported = [json.loads(line) for line in
                zf.read(f"{stem}/events.jsonl").decode().strip().splitlines()]
    assert len(hub.events_since(0)) == 10            # the hub kept only its last ten
    assert len(exported) > 10                        # the archive kept all of them
    assert exported[0]["seq"] == 1                   # including the very first
    assert read_json(zf, stem, "manifest.json")["complete"] is True


def test_a_truncated_log_announces_itself_instead_of_lying():
    bus = EventBus()
    logger = EventLogger(None, bus, max_records=5)
    for i in range(20):
        bus.publish(float(i), EventType.UAV_ARRIVED, f"event {i}")
    assert len(logger.records) == 5
    assert logger.dropped == 15
    assert [r["message"] for r in logger.records][0] == "event 15"   # the most recent


def test_a_run_with_no_recorder_says_its_log_may_be_incomplete():
    # Constructed without a hub, so nothing recorded it; the bus is all there is.
    sim = make_sim()
    sim.start()
    run_until(sim, 10)
    stem, zf = open_zip(build_archive(sim)[1])
    manifest = read_json(zf, stem, "manifest.json")
    assert manifest["complete"] is False
    assert "incomplete" in manifest["events_source"]


# -------------------------------------------------------------- the event logger
def test_a_logger_without_a_directory_writes_nothing(tmp_path):
    bus = EventBus()
    logger = EventLogger(None, bus)
    bus.publish(1.0, EventType.UAV_ARRIVED, "hello")
    logger.close()
    assert len(logger.records) == 1
    assert list(tmp_path.iterdir()) == []


def test_a_logger_with_a_directory_still_writes_both_files(tmp_path):
    bus = EventBus()
    logger = EventLogger(tmp_path / "run", bus)
    bus.publish(1.0, EventType.UAV_ARRIVED, "hello")
    logger.close()
    assert (tmp_path / "run" / "events.jsonl").read_text().strip()
    assert (tmp_path / "run" / "events.csv").read_text().strip()
    assert logger.dropped == 0


# ------------------------------------------------------- single-simulation route
def test_the_dashboard_serves_its_run_as_a_download(finished_run):
    sim, hub = finished_run
    with TestClient(create_app(hub)) as client:
        response = client.get("/api/export")
    assert response.status_code == 200
    assert response.headers["content-type"] == "application/zip"
    assert "attachment" in response.headers["content-disposition"]
    stem, zf = open_zip(response.content)
    assert {n.split("/")[1] for n in zf.namelist()} == MEMBERS


def test_exporting_with_no_simulation_attached_is_a_conflict():
    with TestClient(create_app(LiveHub())) as client:
        assert client.get("/api/export").status_code == 409


def test_a_simulation_registers_itself_with_the_hub_it_is_given():
    hub = LiveHub()
    assert hub.simulation is None
    sim = make_sim(hub=hub)
    assert hub.simulation is sim
    assert sim.logger is not None       # and starts recording for the download


# ------------------------------------------------------------------ studio route
@pytest.fixture()
def manager():
    mgr = SessionManager(max_sessions=3)
    yield mgr
    mgr.shutdown()


@pytest.fixture()
def client(manager):
    with TestClient(create_app(manager=manager)) as test_client:
        yield test_client


def test_a_session_run_can_be_downloaded(client):
    session_id = client.post("/api/sessions", json=SPEC).json()["id"]
    assert wait_until(lambda: client.get(f"/api/sessions/{session_id}").json()["state"] == "finished",
                      timeout_s=30.0)
    response = client.get(f"/api/sessions/{session_id}/export")
    assert response.status_code == 200
    stem, zf = open_zip(response.content)
    manifest = read_json(zf, stem, "manifest.json")
    assert manifest["session_id"] == session_id
    assert manifest["run_finished"] is True
    assert manifest["samples"] > 0


def test_a_studio_download_records_the_mission_that_produced_it(client):
    # The mission came from clicks on a map, so without the spec the run cannot
    # be reproduced from the archive.
    session_id = client.post("/api/sessions", json=SPEC).json()["id"]
    stem, zf = open_zip(client.get(f"/api/sessions/{session_id}/export").content)
    spec = read_json(zf, stem, "scenario.json")["mission_spec"]
    assert spec["uav_count"] == SPEC["uav_count"]
    assert spec["pois"] == SPEC["pois"]


def test_a_run_still_going_can_be_downloaded(client):
    session_id = client.post("/api/sessions", json=SPEC).json()["id"]
    assert wait_until(lambda: client.get(f"/api/sessions/{session_id}").json()["t_s"] > 2)
    stem, zf = open_zip(client.get(f"/api/sessions/{session_id}/export").content)
    manifest = read_json(zf, stem, "manifest.json")
    assert manifest["run_finished"] is False
    assert manifest["state"] in ("running", "paused", "finished")


def test_downloading_an_unknown_session_is_404(client):
    assert client.get("/api/sessions/nosuchthing/export").status_code == 404


# ---------------------------------------------------------------------- the name
@pytest.mark.parametrize("hostile", [
    'evil" filename',
    "evil\r\nX-Injected: yes",
    "../../etc/passwd",
    "nåme with spaces",
])
def test_a_scenario_name_cannot_break_out_of_the_download_header(hostile):
    cleaned = safe_name(hostile)
    assert not set(cleaned) & set('"\r\n/\\ ')
    assert cleaned


def test_an_empty_name_still_produces_a_usable_filename():
    assert safe_name("", fallback="run") == "run"
    assert safe_name("!!!", fallback="run") == "run"


def test_the_download_filename_is_built_from_the_run(finished_run):
    sim, hub = finished_run
    with TestClient(create_app(hub)) as client:
        disposition = client.get("/api/export").headers["content-disposition"]
    assert disposition.count('"') == 2               # exactly the quotes we put there
    assert sim.mode in disposition
    assert disposition.endswith('.zip"')
