"""Stage 7 tests: dashboard API, live hub and operator command path."""

import json

import pytest
from fastapi.testclient import TestClient

from dashboard.api import create_app
from dashboard.websocket import LiveHub
from tests.helpers import make_sim, run_until


@pytest.fixture()
def running_sim():
    hub = LiveHub()
    sim = make_sim()
    sim.hub = hub
    sim.start()
    run_until(sim, 20)
    sim.push(force=True)
    return sim, hub


def test_state_endpoint_carries_everything_the_page_needs(running_sim):
    sim, hub = running_sim
    with TestClient(create_app(hub)) as client:
        body = client.get("/api/state").json()
        assert body["version"] >= 1
        state = body["state"]
        assert {"world", "env", "swarm", "metrics", "scenario"} <= set(state)
        assert len(state["world"]["uavs"]) == len(sim.world.state.uavs)
        assert "origin" in state["scenario"] and "area_latlon" in state["scenario"]
        assert "links" in state["env"] and "node_positions" in state["env"]
        assert "relays" in state["swarm"] and "incidents" in state["swarm"]
        json.dumps(state)      # the whole payload must be JSON serialisable


def test_events_endpoint_is_incremental(running_sim):
    _, hub = running_sim
    with TestClient(create_app(hub)) as client:
        events = client.get("/api/events?since=0").json()["events"]
        assert events and events[0]["type"] == "SIM_STARTED"
        later = client.get(f"/api/events?since={events[-1]['seq']}").json()["events"]
        assert all(e["seq"] > events[-1]["seq"] for e in later)


def test_operator_injection_reaches_the_world(running_sim):
    sim, hub = running_sim
    with TestClient(create_app(hub)) as client:
        response = client.post("/api/inject", json={"action": "add_poi", "params": {
            "id": "POI-OP", "position_m": [300, 200], "priority": 5, "survey_time_s": 30}})
        assert response.status_code == 200 and response.json()["queued"] is True
        sim.tick()
        assert "POI-OP" in sim.world.state.pois

        bad = client.post("/api/inject", json={"action": "self_destruct", "params": {}})
        assert bad.status_code == 400


def test_index_and_static_files_are_served(running_sim):
    _, hub = running_sim
    with TestClient(create_app(hub)) as client:
        page = client.get("/")
        assert page.status_code == 200 and "Live map" in page.text
        for asset in ("/static/app.js", "/static/map.js", "/static/network.js", "/static/style.css"):
            assert client.get(asset).status_code == 200


def test_websocket_pushes_state(running_sim):
    sim, hub = running_sim
    with TestClient(create_app(hub, push_interval_s=0.01)) as client:
        with client.websocket_connect("/ws") as ws:
            message = ws.receive_json()
            assert message["type"] == "state"
            assert message["state"]["world"]["uavs"]
            sim.tick()
            sim.push(force=True)
            assert ws.receive_json()["type"] == "state"


def test_hub_rejects_unknown_actions():
    hub = LiveHub()
    with pytest.raises(ValueError):
        hub.submit_command("format_c", {})
    hub.submit_command("fail_uav", {"uav_id": 1})
    assert hub.pop_commands() == [("fail_uav", {"uav_id": 1})]
    assert hub.pop_commands() == []
