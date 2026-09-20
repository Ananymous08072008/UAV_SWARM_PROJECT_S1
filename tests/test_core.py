"""Stage 1 tests: config, UAV motion/battery, PoIs, events, World command API."""

import json
from dataclasses import replace

import numpy as np
import pytest

from core.config import BatteryParams, ConfigError, GeoOrigin, Parameters, ScenarioConfig, UAVParams
from core.events import EventBus, EventType, TriggerSchedule
from core.poi import PoIStatus
from core.uav import UAV, FlightMode, HealthState, UAVRole
from core.world import CommandError, World


def make_world(**overrides) -> World:
    data = {
        "scenario": {"name": "test", "duration_s": 120.0},
        "area": {"x_min_m": -100, "x_max_m": 300, "y_min_m": -100, "y_max_m": 300},
        "uavs": {"count": 3, "formation": "line", "spacing_m": 5.0, "start_m": [0, 0]},
        "pois": [
            {"id": "A", "position_m": [50, 0], "priority": 5, "survey_time_s": 10},
            {"id": "B", "position_m": [0, 80], "priority": 1, "survey_time_s": 20},
        ],
    }
    data.update(overrides)
    return World(Parameters(), ScenarioConfig.from_dict(data))


def run_until(world: World, t_s: float) -> None:
    while world.t < t_s - 1e-9:
        world.step()


# --------------------------------------------------------------------- config
def test_geo_origin_round_trip():
    origin = GeoOrigin()
    lat, lon, alt = origin.to_geodetic(250.0, -400.0, 30.0)
    assert lat < origin.lat_deg and lon > origin.lon_deg
    assert origin.to_local(lat, lon, alt) == pytest.approx((250.0, -400.0, 30.0), abs=1e-6)
    # ~111 km per degree of latitude
    assert origin.to_geodetic(0, 111_319.5, 0)[0] - origin.lat_deg == pytest.approx(1.0, abs=1e-3)


def test_unknown_parameter_key_is_rejected():
    with pytest.raises(ConfigError, match="cruise_sped"):
        Parameters.from_dict({"uav": {"cruise_sped_mps": 10}})


def test_later_stage_sections_are_kept_raw():
    params = Parameters.from_dict({"communication": {"range_m": 250}})
    assert params.section("communication") == {"range_m": 250}


def test_scenario_rejects_poi_outside_area_and_duplicates():
    with pytest.raises(ConfigError, match="outside the area"):
        make_world(pois=[{"id": "X", "position_m": [900, 0]}])
    with pytest.raises(ConfigError, match="duplicate"):
        make_world(pois=[{"id": "X", "position_m": [1, 0]}, {"id": "X", "position_m": [2, 0]}])


def test_unknown_trigger_action_is_rejected():
    with pytest.raises(ConfigError, match="unknown action"):
        make_world(timeline=[{"at_s": 1, "action": "explode"}])


def test_uav_selectors_and_injection():
    world = make_world()
    world.register_uav_selector("first", lambda w: 1)
    assert world.resolve_uav("first") == 1
    assert world.resolve_uav("role:IDLE") == 1
    assert world.resolve_uav(3) == 3
    with pytest.raises(CommandError, match="unknown UAV selector"):
        world.resolve_uav("nobody")
    world.inject("add_poi", {"id": "NEW", "position_m": [50, 50], "priority": 5, "survey_time_s": 20})
    assert "NEW" in world.state.pois


def test_real_config_files_load():
    world = World.from_files("config/parameters.yaml", "config/scenario.yaml")
    assert len(world.state.uavs) == 12 and len(world.state.pois) == 6


# ------------------------------------------------------------------------ UAV
def test_uav_flies_to_waypoint_within_limits():
    p = UAVParams()
    uav = UAV(1, (0, 0, 0))
    uav.goto((100, 0, 30))
    arrived_at = None
    max_speed = 0.0
    for i in range(400):
        if uav.step(0.1, p, BatteryParams()).arrived:
            arrived_at = i
        max_speed = max(max_speed, uav.ground_speed_mps)
    assert arrived_at is not None
    assert uav.mode is FlightMode.ON_STATION
    assert np.allclose(uav.position, (100, 0, 30), atol=0.05)
    assert max_speed <= p.cruise_speed_mps + 1e-6
    assert uav.heading_deg == pytest.approx(90.0, abs=1.0)  # flew East
    # take-off climbs to the flight level first (keeps UAVs vertically separated), then translates
    assert uav.distance_travelled_m == pytest.approx(30 + 100, rel=0.05)


def test_battery_thresholds_fire_once_and_depletion_reported():
    bat = BatteryParams(hover_drain_pct_per_min=60.0, cruise_drain_pct_per_min=0.0, low_pct=30, critical_pct=15)
    uav = UAV(1, (0, 0, 20), battery_pct=35.0)
    reports = [uav.step(0.1, UAVParams(), bat) for _ in range(400)]
    assert sum(r.battery_low for r in reports) == 1
    assert sum(r.battery_critical for r in reports) == 1
    assert sum(r.battery_depleted for r in reports) == 1


def test_grounded_uav_does_not_drain():
    uav = UAV(1, (0, 0, 0))
    for _ in range(100):
        uav.step(0.1, UAVParams(), BatteryParams())
    assert uav.battery_pct == 100.0


# ---------------------------------------------------------------------- world
def test_survey_completes_and_releases_uav():
    world = make_world()
    completed = []
    world.events.subscribe(completed.append, types=[EventType.POI_COMPLETED])
    world.assign_poi(1, "A")
    assert world.state.get_uav(1).role is UAVRole.SURVEY
    run_until(world, 60)
    poi = world.state.pois.get("A")
    uav = world.state.get_uav(1)
    assert poi.status is PoIStatus.COMPLETED and poi.completed_by == 1
    assert uav.role is UAVRole.IDLE and uav.assigned_poi is None
    assert len(completed) == 1 and completed[0].data["early"] is False


def test_failure_releases_poi_and_keeps_progress():
    world = make_world()
    world.assign_poi(2, "B")
    run_until(world, 40)  # climbs out, flies ~80 m, then needs 20 s of survey
    progress = world.state.pois.get("B").progress_s
    assert 0 < progress < 20
    world.fail_uav(2, "test")
    poi = world.state.pois.get("B")
    assert poi.status is PoIStatus.PENDING and poi.progress_s == progress
    assert world.state.get_uav(2).health is HealthState.FAILED
    with pytest.raises(CommandError):
        world.assign_poi(2, "B")
    world.assign_poi(3, "B")  # re-tasking works
    assert poi.assigned_uav == 3


def test_poi_cannot_be_double_assigned():
    world = make_world()
    world.assign_poi(1, "A")
    with pytest.raises(CommandError, match="already assigned"):
        world.assign_poi(2, "A")


def test_timeline_triggers_fire_and_bad_parameters_are_reported():
    world = make_world(timeline=[
        {"at_s": 5.0, "action": "complete_poi", "params": {"poi_id": "A"}},
        {"at_s": 6.0, "action": "degrade_link", "params": {"uav_id": 1, "quality": 0.2}},
        {"at_s": 7.0, "action": "complete_poi", "params": {"poi_id": "NOPE"}},
        {"at_s": 8.0, "action": "add_obstacle", "params": {"id": "X", "polygon_m": [[0, 0], [1, 0], [1, 1]]}},
    ])
    world.assign_poi(1, "A")
    run_until(world, 10)
    poi = world.state.pois.get("A")
    assert poi.is_completed and poi.completed_early and poi.completed_at_s == pytest.approx(5.0)
    assert world.state.get_uav(1).comm.radio_health == pytest.approx(0.2)
    rejected = world.events.history(types=[EventType.TRIGGER_REJECTED])
    # unknown PoI is rejected; add_obstacle has no handler until the simulation layer registers one
    assert [("NOPE" in r.message, "add_obstacle" in r.message) for r in rejected] == [(True, False), (False, True)]


def test_return_home_lands_and_charges():
    world = make_world()
    world.goto(1, (100.0, 0.0, 40.0), "test")
    run_until(world, 30)
    world.return_home(1, "test")
    uav = world.state.get_uav(1)
    assert uav.role is UAVRole.RETURNING
    run_until(world, 120)
    assert uav.role in (UAVRole.CHARGING, UAVRole.IDLE)
    assert not uav.is_airborne
    run_until(world, 300)
    assert uav.battery_pct >= world.params.battery.resume_pct - 1
    assert uav.role is UAVRole.IDLE


def test_same_seed_gives_identical_runs():
    def run(seed):
        world = make_world(scenario={"name": "det", "duration_s": 30.0, "seed": seed},
                           random_pois={"count": 4})
        for poi in world.state.pois.pending():
            idle = world.state.uavs_with_role(UAVRole.IDLE)
            if idle:
                world.assign_poi(idle[0].uav_id, poi.poi_id)
        world.run()
        return json.dumps(world.snapshot().to_dict(), sort_keys=True)

    assert run(7) == run(7)
    assert run(7) != run(8)


def test_snapshot_is_json_serialisable_and_has_geodetic_position():
    world = make_world()
    world.step()
    snap = world.snapshot().to_dict()
    json.dumps(snap)
    assert snap["uavs"][0]["lat_deg"] == pytest.approx(world.params.geo_origin.lat_deg, abs=1e-3)


def test_duration_override_revalidates_timeline():
    scenario = ScenarioConfig.from_dict({"scenario": {"duration_s": 100},
                                         "timeline": [{"at_s": 90, "action": "fail_uav", "params": {"uav_id": 1}}]})
    with pytest.raises(ConfigError, match="after the scenario end"):
        replace(scenario, duration_s=50)


# --------------------------------------------------------------------- events
def test_event_bus_filters_isolates_failures_and_unsubscribes():
    bus = EventBus(history_size=3)
    got = []

    def broken(_event):
        raise RuntimeError("boom")

    bus.subscribe(broken)
    unsubscribe = bus.subscribe(got.append, types=[EventType.UAV_FAILED])
    bus.publish(0.0, EventType.SIM_STARTED, "start")
    bus.publish(1.0, EventType.UAV_FAILED, "fail")
    unsubscribe()
    bus.publish(2.0, EventType.UAV_FAILED, "fail again")
    bus.publish(3.0, EventType.SIM_STOPPED, "stop")
    assert [e.message for e in got] == ["fail"]
    assert len(bus.history()) == 3 and bus.count(EventType.UAV_FAILED) == 2
    assert [e.seq for e in bus.history(since_seq=2)] == [3, 4]


def test_trigger_schedule_orders_by_time_then_file_order():
    class Spec:
        def __init__(self, at_s, action):
            self.at_s, self.action, self.params = at_s, action, {}

    sched = TriggerSchedule.from_specs([Spec(5, "fail_uav"), Spec(1, "complete_poi"), Spec(5, "add_obstacle")])
    assert sched.pop_due(0.5) == []
    assert [t.action.value for t in sched.pop_due(5.0)] == ["complete_poi", "fail_uav", "add_obstacle"]
    assert sched.remaining == 0
