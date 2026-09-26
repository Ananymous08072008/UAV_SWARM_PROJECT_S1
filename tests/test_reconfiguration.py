"""Stage 6 tests: fault detection, re-planning, recovery and priority pre-emption."""

import pytest

from core.events import EventType
from core.poi import PoIStatus
from core.uav import UAVRole
from tests.helpers import make_sim, run_until


def test_radio_degradation_is_detected_and_the_relay_is_replaced():
    # A single, long-running PoI: fault detection/recovery is orthogonal to which PoI is
    # active, and a single PoI keeps the farthest-PoI deferral (TaskAllocator._defer_farthest,
    # a no-op with only one PoI) from changing which UAV ends up in this scenario's shoes.
    sim = make_sim(scenario={"duration_s": 600.0},
                   pois=[{"id": "POI-A", "position_m": [420, 120], "priority": 4, "survey_time_s": 300}])
    run_until(sim, 70)
    relay = sim.manager.routes.critical_relay(sim.world)
    assert relay is not None, "the scenario needs a relay carrying traffic"
    sim.world.set_radio_health(relay, 0.15, "test")
    run_until(sim, 110)

    detected = sim.world.events.history(types=[EventType.FAULT_DETECTED])
    assert detected, "the swarm must notice the degradation from measurements alone"
    assert detected[0].data["detection_time_s"] <= 8.0
    assert relay in sim.manager.relays.excluded
    assert sim.world.state.get_uav(relay).role is not UAVRole.RELAY

    run_until(sim, 400)
    incidents = [i for i in sim.manager.reconfig.incidents if i.cause == "radio_degradation"]
    assert incidents and incidents[0].recovered_s is not None
    assert incidents[0].recovery_time_s >= incidents[0].detection_time_s


def test_a_baseline_swarm_does_not_detect_anything():
    sim = make_sim("baseline", scenario={"duration_s": 400.0})
    run_until(sim, 70)
    relay = next(iter(sim.world.state.uavs_with_role(UAVRole.RELAY)), None)
    if relay is None:
        pytest.skip("baseline planned no relays for this geometry")
    sim.world.set_radio_health(relay.uav_id, 0.15, "test")
    run_until(sim, 150)
    assert not sim.world.events.history(types=[EventType.FAULT_DETECTED])
    assert not sim.manager.relays.excluded


def test_uav_loss_is_declared_after_the_heartbeat_timeout_and_recovered():
    sim = make_sim(scenario={"duration_s": 600.0})
    run_until(sim, 70)
    relay = sim.manager.routes.critical_relay(sim.world)
    assert relay is not None
    dependents = sim.manager.routes.dependents(relay)
    sim.world.fail_uav(relay, "test")
    run_until(sim, 80)
    detected = [e for e in sim.world.events.history(types=[EventType.FAULT_DETECTED])
                if e.data.get("cause") == "uav_failure"]
    assert detected and detected[0].t_s >= 70 + sim.manager.reconfig.params.failure_timeout_s - 1e-6
    run_until(sim, 300)
    if dependents:
        incident = next(i for i in sim.manager.reconfig.incidents if i.cause == "uav_failure")
        assert incident.recovered_s is not None or incident.closed_reason


def test_an_obstacle_triggers_a_re_plan():
    sim = make_sim(scenario={"duration_s": 600.0})
    run_until(sim, 70)
    replans_before = sim.manager.relays.replans
    sim.world.inject("add_obstacle", {"id": "DEBRIS", "center_m": "backbone_midpoint",
                                      "size_m": 150.0, "height_m": 70.0, "attenuation_db": 40.0})
    run_until(sim, 90)
    assert sim.world.events.count(EventType.OBSTACLE_ADDED) == 1
    assert sim.manager.relays.replans > replans_before
    assert sim.env.obstacles.version >= 1


def test_a_new_high_priority_poi_pre_empts_a_low_priority_survey():
    sim = make_sim(uavs={"count": 1, "per_row": 1},
                   pois=[{"id": "LOW", "position_m": [250, 0], "priority": 1, "survey_time_s": 120}],
                   scenario={"duration_s": 600.0})
    run_until(sim, 60)
    uav = sim.world.state.get_uav(1)
    assert uav.assigned_poi == "LOW"
    sim.world.add_poi("URGENT", (300, 120), priority=5, survey_time_s=40, reason="test")
    run_until(sim, 80)
    assert uav.assigned_poi == "URGENT"
    assert sim.world.state.pois.get("LOW").status is PoIStatus.PENDING
    assert sim.world.events.count(EventType.PREEMPTION) == 1
    assert sim.manager.priority.preemptions == 1


def test_response_time_to_a_new_region_is_measured():
    sim = make_sim(scenario={"duration_s": 600.0})
    run_until(sim, 60)
    sim.world.add_poi("URGENT", (300, 200), priority=5, survey_time_s=30, reason="test")
    run_until(sim, 90)
    assert sim.metrics.response_times_s and sim.metrics.response_times_s[0] <= 10.0
