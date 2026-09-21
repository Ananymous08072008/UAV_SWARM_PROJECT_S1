"""Stage 8 tests: mission builder, per-visitor sessions and the studio API.

These cover the multi-session web app - one independent world per visitor,
built and driven from the browser. The single-simulation dashboard is covered
by test_dashboard.py and must keep working unchanged.
"""

from __future__ import annotations

import time

import pytest
from fastapi.testclient import TestClient

from core.config import ConfigError
from dashboard.api import create_app
from dashboard.mission import MAX_POIS, MAX_UAVS, build_scenario, limits
from dashboard.session import SessionError, SessionManager

# A mission small and short enough that a test never waits on physics.
SPEC = {"uav_count": 4, "duration_s": 60.0, "speed": 20.0,
        "pois": [{"x_m": 400.0, "y_m": 200.0, "priority": 4, "survey_time_s": 20.0}]}


def wait_until(predicate, timeout_s: float = 10.0) -> bool:
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture()
def manager():
    mgr = SessionManager(max_sessions=3)
    yield mgr
    mgr.shutdown()


@pytest.fixture()
def client(manager):
    with TestClient(create_app(manager=manager)) as test_client:
        yield test_client


# --------------------------------------------------------------- mission builder
def test_builder_uses_the_uav_count_and_pois_from_the_browser():
    scenario = build_scenario({"uav_count": 9, "pois": [{"x_m": 500, "y_m": 300, "priority": 5},
                                                        {"x_m": 800, "y_m": 600}]})
    assert scenario.uavs.count == 9
    assert [p.id for p in scenario.pois] == ["POI-1", "POI-2"]
    assert scenario.pois[0].position_m == (500.0, 300.0)
    assert scenario.pois[0].priority == 5


def test_builder_without_a_spec_reproduces_the_template_scenario():
    assert build_scenario({}).uavs.count == build_scenario(None).uavs.count
    assert build_scenario({}).pois


def test_builder_reflows_the_launch_grid_for_a_single_uav():
    # per_row must not exceed the count, or spawn positions collapse.
    assert build_scenario({"uav_count": 1}).uavs.per_row == 1


@pytest.mark.parametrize("spec, message", [
    ({"uav_count": MAX_UAVS + 1}, "uav_count"),
    ({"uav_count": 0}, "uav_count"),
    ({"uav_count": "many"}, "whole number"),
    ({"duration_s": 99999}, "duration_s"),
    ({"pois": []}, "at least one PoI"),
    ({"pois": [{"x_m": 99999, "y_m": 0}]}, "outside the area"),
    ({"pois": [{"x_m": 400, "y_m": 200, "priority": 9}]}, "priority"),
    ({"pois": "nope"}, "must be a list"),
    ({"pois": [{"x_m": 400, "y_m": 200}] * (MAX_POIS + 1)}, "at most"),
])
def test_builder_rejects_impossible_missions(spec, message):
    with pytest.raises(ConfigError) as excinfo:
        build_scenario(spec)
    assert message in str(excinfo.value)


def test_builder_drops_timeline_triggers_that_reference_missing_pois():
    # The stock timeline completes POI-5 early; a custom mission has no POI-5,
    # so that trigger would only ever log TRIGGER_REJECTED.
    scenario = build_scenario({"pois": [{"x_m": 400, "y_m": 200}]})
    referenced = [t.params.get("poi_id") for t in scenario.timeline]
    assert all(ref is None or ref == "POI-1" for ref in referenced)


def test_faults_can_be_switched_off():
    assert build_scenario({"pois": [{"x_m": 400, "y_m": 200}], "faults": False}).timeline == ()


def test_limits_describe_the_area_the_ui_must_clamp_to():
    bounds = limits()
    assert bounds["max_uavs"] == MAX_UAVS
    assert bounds["area"]["x_min_m"] < bounds["area"]["x_max_m"]


# ---------------------------------------------------------------------- sessions
def test_two_sessions_run_independent_worlds(manager):
    a = manager.create({**SPEC, "uav_count": 3})
    b = manager.create({**SPEC, "uav_count": 7, "mode": "baseline"})
    assert a.sim.world is not b.sim.world
    assert a.hub is not b.hub
    assert wait_until(lambda: a.sim.world.t > 1 and b.sim.world.t > 1)
    assert len(a.sim.world.snapshot().uavs) == 3
    assert len(b.sim.world.snapshot().uavs) == 7
    assert a.mode == "adaptive" and b.mode == "baseline"


def test_pausing_one_session_leaves_the_other_running(manager):
    a = manager.create(SPEC)
    b = manager.create(SPEC)
    assert wait_until(lambda: a.sim.world.t > 1 and b.sim.world.t > 1)
    a.pause()
    frozen, other = a.sim.world.t, b.sim.world.t
    assert wait_until(lambda: b.sim.world.t > other + 1)
    assert a.sim.world.t == frozen
    a.resume()
    assert wait_until(lambda: a.sim.world.t > frozen)


def test_restart_rebuilds_the_world_and_bumps_the_generation(manager):
    session = manager.create(SPEC)
    assert wait_until(lambda: session.sim.world.t > 2)
    session.restart({"uav_count": 6})
    assert session.generation == 1
    assert wait_until(lambda: session.sim.world.t > 0.5)
    assert len(session.sim.world.snapshot().uavs) == 6


def test_restart_with_an_invalid_spec_is_rejected(manager):
    session = manager.create(SPEC)
    with pytest.raises(ConfigError):
        session.restart({"uav_count": MAX_UAVS + 1})


def test_capacity_is_enforced(manager):
    for _ in range(manager.max_sessions):
        manager.create(SPEC)
    with pytest.raises(SessionError):
        manager.create(SPEC)


def test_a_malformed_spec_reports_itself_even_on_a_full_server(manager):
    # Validation must run before the capacity check, or a typo looks like load.
    for _ in range(manager.max_sessions):
        manager.create(SPEC)
    with pytest.raises(ConfigError):
        manager.create({"uav_count": MAX_UAVS + 1})


def test_idle_sessions_are_reaped(manager):
    session = manager.create(SPEC)
    manager.idle_timeout_s = -1.0
    assert manager.reap() == 1
    with pytest.raises(KeyError):
        manager.get(session.id)


def test_deleting_a_session_stops_its_thread(manager):
    session = manager.create(SPEC)
    assert wait_until(lambda: session.sim.world.t > 1)
    manager.delete(session.id)
    stopped = session.sim.world.t
    time.sleep(0.4)
    assert session.sim.world.t == stopped


# --------------------------------------------------------------------- studio API
def test_create_and_inspect_a_session_over_http(client):
    created = client.post("/api/sessions", json=SPEC)
    assert created.status_code == 201
    body = created.json()
    assert body["uav_count"] == 4 and body["poi_count"] == 1

    status = client.get(f"/api/sessions/{body['id']}")
    assert status.status_code == 200 and status.json()["state"] in ("running", "finished")

    state = client.get(f"/api/sessions/{body['id']}/state").json()
    assert wait_until(lambda: client.get(f"/api/sessions/{body['id']}/state").json()["state"])
    assert "status" in state


def test_control_endpoint_drives_the_run(client):
    session_id = client.post("/api/sessions", json=SPEC).json()["id"]
    assert client.post(f"/api/sessions/{session_id}/control",
                       json={"action": "pause"}).json()["state"] == "paused"
    assert client.post(f"/api/sessions/{session_id}/control",
                       json={"action": "resume"}).json()["state"] == "running"
    assert client.post(f"/api/sessions/{session_id}/control",
                       json={"action": "speed", "speed": 12}).json()["speed"] == 12.0
    assert client.post(f"/api/sessions/{session_id}/control",
                       json={"action": "restart"}).json()["generation"] == 1


def test_control_rejects_an_unknown_action(client):
    session_id = client.post("/api/sessions", json=SPEC).json()["id"]
    assert client.post(f"/api/sessions/{session_id}/control",
                       json={"action": "explode"}).status_code == 400


def test_pausing_a_paused_session_is_a_conflict(client):
    session_id = client.post("/api/sessions", json=SPEC).json()["id"]
    client.post(f"/api/sessions/{session_id}/control", json={"action": "pause"})
    assert client.post(f"/api/sessions/{session_id}/control",
                       json={"action": "pause"}).status_code == 409


def test_injection_is_scoped_to_one_session(client):
    a = client.post("/api/sessions", json=SPEC).json()["id"]
    b = client.post("/api/sessions", json=SPEC).json()["id"]
    assert client.post(f"/api/sessions/{a}/inject",
                       json={"action": "fail_uav", "params": {"uav_id": 1}}).json()["queued"]
    assert client.post(f"/api/sessions/{b}/inject",
                       json={"action": "nonsense", "params": {}}).status_code == 400


def test_history_endpoint_feeds_the_charts(client):
    session_id = client.post("/api/sessions", json=SPEC).json()["id"]
    assert wait_until(lambda: client.get(f"/api/sessions/{session_id}/history")
                      .json()["samples"])
    samples = client.get(f"/api/sessions/{session_id}/history").json()["samples"]
    assert {"t_s", "connectivity", "pdr", "latency"} <= set(samples[0])


def test_history_thinning(client):
    session_id = client.post("/api/sessions", json=SPEC).json()["id"]
    assert wait_until(lambda: len(client.get(f"/api/sessions/{session_id}/history")
                                  .json()["samples"]) > 10)
    every_one = client.get(f"/api/sessions/{session_id}/history?every=1").json()["samples"]
    every_five = client.get(f"/api/sessions/{session_id}/history?every=5").json()["samples"]
    assert len(every_five) < len(every_one)


def test_bad_mission_returns_400_with_the_reason(client):
    response = client.post("/api/sessions", json={"uav_count": MAX_UAVS + 1})
    assert response.status_code == 400
    assert "uav_count" in response.json()["detail"]


def test_a_full_server_returns_503(client, manager):
    for _ in range(manager.max_sessions):
        client.post("/api/sessions", json=SPEC)
    assert client.post("/api/sessions", json=SPEC).status_code == 503


def test_unknown_session_is_404(client):
    assert client.get("/api/sessions/nosuchthing").status_code == 404
    assert client.post("/api/sessions/nosuchthing/inject",
                       json={"action": "fail_uav", "params": {}}).status_code == 404


def test_closing_a_session_removes_it(client):
    session_id = client.post("/api/sessions", json=SPEC).json()["id"]
    assert client.delete(f"/api/sessions/{session_id}").status_code == 200
    assert client.get(f"/api/sessions/{session_id}").status_code == 404


def test_session_listing_reports_capacity(client):
    client.post("/api/sessions", json=SPEC)
    body = client.get("/api/sessions").json()
    assert len(body["sessions"]) == 1
    assert body["capacity"] >= 1


def test_limits_endpoint_carries_the_origin_for_the_builder_map(client):
    body = client.get("/api/limits").json()
    assert {"max_uavs", "max_pois", "area", "origin"} <= set(body)
    assert "lat_deg" in body["origin"]


def test_studio_page_and_its_assets_are_served(client):
    assert "mission studio" in client.get("/").text
    for asset in ("studio.js", "studio.css", "builder.js", "charts.js"):
        assert client.get(f"/static/{asset}").status_code == 200


def test_session_websocket_streams_state_and_status(client):
    session_id = client.post("/api/sessions", json=SPEC).json()["id"]
    with client.websocket_connect(f"/ws/{session_id}") as ws:
        for _ in range(12):
            message = ws.receive_json()
            if message["type"] == "state":
                assert "world" in message["state"]
                assert message["status"]["id"] == session_id
                return
    pytest.fail("no state message arrived")
