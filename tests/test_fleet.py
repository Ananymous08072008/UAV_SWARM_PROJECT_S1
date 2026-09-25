"""Fleet sizing: with uavs.count: auto the fleet follows the mission, not a fixed number."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from core.config import Parameters, ScenarioConfig
from core.events import EventType
from simulation.runner import Simulation
from tests.helpers import make_sim, parameters, run_until

PROJECT_ROOT = Path(__file__).resolve().parent.parent
AUTO = {"count": "auto", "max_count": 40}
NEAR = {"id": "NEAR", "position_m": [150, 0], "priority": 3, "survey_time_s": 30}   # in GCS radio range
FAR = {"id": "FAR", "position_m": [800, 600], "priority": 3, "survey_time_s": 30}   # needs a relay chain


def fleet(sim: Simulation) -> dict:
    return sim.world.fleet


def test_a_fixed_count_is_left_alone():
    sim = make_sim()
    assert len(sim.world.state.uavs) == 8
    assert fleet(sim) == {"sizing": "fixed"}


def test_a_single_nearby_poi_needs_one_surveyor_and_a_spare():
    sim = make_sim(uavs=AUTO, pois=[NEAR])
    f = fleet(sim)
    assert (f["surveyors"], f["relays"], f["spares"], f["fault_reserve"]) == (1, 0, 1, 0)
    assert len(sim.world.state.uavs) == f["uavs"] == 2


def test_distant_pois_bring_the_relays_that_keep_them_connected():
    near = fleet(make_sim(uavs=AUTO, pois=[NEAR]))
    far = fleet(make_sim(uavs=AUTO, pois=[FAR]))
    assert far["relays"] > near["relays"] == 0
    assert far["uavs"] > near["uavs"]


def test_the_relay_count_is_what_the_swarm_itself_plans():
    # The planner must not guess: once every PoI is being surveyed, the swarm's
    # own relay plan should need exactly the relays the fleet was sized for.
    sim = make_sim(uavs=AUTO)
    run_until(sim, 60)
    assert sim.manager.relays.plan.relay_count == fleet(sim)["relays"]


def test_scheduled_faults_get_a_reserve():
    timeline = [{"at_s": 50, "action": "fail_uav", "params": {"uav_id": 1}},
                {"at_s": 60, "action": "add_poi", "params": {"id": "URGENT", "position_m": "random"}}]
    sim = make_sim(uavs=AUTO, pois=[NEAR], random_pois={"count": 0, "region_m": [300, -100, 800, 600]},
                   timeline=timeline)
    assert fleet(sim)["fault_reserve"] == 2


def test_a_new_poi_at_a_known_place_is_planned_like_the_others():
    timeline = [{"at_s": 60, "action": "add_poi", "params": {"id": "URGENT", "position_m": [800, 600]}}]
    sim = make_sim(uavs=AUTO, pois=[NEAR], timeline=timeline)
    f = fleet(sim)
    assert f["pois"] == 2 and f["fault_reserve"] == 0


def test_sizing_can_be_tuned_in_the_parameters():
    params = parameters(swarm={"fleet": {"spares": 3, "fault_reserve": False}})
    timeline = [{"at_s": 50, "action": "fail_uav", "params": {"uav_id": 1}}]
    f = fleet(make_sim(params=params, uavs=AUTO, pois=[NEAR], timeline=timeline))
    assert (f["spares"], f["fault_reserve"], f["uavs"]) == (3, 0, 4)


def test_the_fleet_is_capped_and_says_so():
    sim = make_sim(uavs={"count": "auto", "max_count": 2}, pois=[FAR])
    f = fleet(sim)
    assert f["required"] > 2 and f["capped"] is True
    assert len(sim.world.state.uavs) == 2
    sim.start()
    [event] = sim.world.events.history(types=[EventType.FLEET_PLANNED])
    assert "capped" in event.message and event.severity.value == "WARNING"


def test_both_modes_fly_the_same_fleet():
    # Otherwise an adaptive-vs-baseline comparison would compare fleet sizes too.
    assert fleet(make_sim("adaptive", uavs=AUTO))["uavs"] == fleet(make_sim("baseline", uavs=AUTO))["uavs"]


def test_the_summary_reports_how_the_fleet_was_sized():
    sim = make_sim(uavs=AUTO, scenario={"duration_s": 30.0})
    sim.run()
    run = sim.finish()["run"]
    assert run["fleet_sizing"] == "auto"
    assert run["uavs"] == run["fleet_surveyors"] + run["fleet_relays"] + run["fleet_spares"] + run["fleet_fault_reserve"]


# ------------------------------------------------- fewer surveyors than PoIs
MANY = [{"id": f"P{i}", "position_m": [150 + 40 * i, 60 * (i % 3)], "priority": 1 + i % 5, "survey_time_s": 30}
        for i in range(8)]


def test_the_fleet_flies_fewer_surveyors_than_pois():
    f = fleet(make_sim(uavs=AUTO, pois=MANY, scenario={"duration_s": 1200.0}))
    assert f["pois"] == len(MANY)
    assert 1 <= f["surveyors"] < len(MANY)
    assert f["makespan_s"] <= f["time_budget_s"] and f["over_budget"] is False


def test_a_tighter_deadline_needs_more_surveyors():
    long = fleet(make_sim(uavs=AUTO, pois=MANY, scenario={"duration_s": 1200.0}))["surveyors"]
    short = fleet(make_sim(uavs=AUTO, pois=MANY, scenario={"duration_s": 300.0}))["surveyors"]
    assert short > long


def test_without_a_relay_chain_there_are_no_relays():
    params = parameters(swarm={"fleet": {"relay_chain": False}})
    f = fleet(make_sim(params=params, uavs=AUTO, pois=[FAR, {**FAR, "id": "FAR2", "position_m": [800, 400]}]))
    assert f["relays"] == 0 and len(f["unreachable"]) == 0


def test_a_small_fleet_still_surveys_every_poi_in_priority_order():
    sim = make_sim(uavs=AUTO, pois=MANY, scenario={"duration_s": 1200.0})
    assert fleet(sim)["surveyors"] < len(MANY)
    sim.run()
    assert sim.world.state.pois.all_completed
    started = [e.poi_id for e in sim.world.events.history(types=[EventType.POI_ASSIGNED])]
    first = {p["id"]: p["priority"] for p in MANY}[started[0]]
    assert first == max(p["priority"] for p in MANY)


@pytest.mark.parametrize("seed", [1, 2])
def test_the_demo_fleet_follows_its_random_pois(seed):
    scenario = replace(ScenarioConfig.load(PROJECT_ROOT / "config" / "scenario.yaml"), seed=seed)
    sim = Simulation(Parameters.load(PROJECT_ROOT / "config" / "parameters.yaml"), scenario, results_dir=None)
    f = fleet(sim)
    assert f["sizing"] == "auto"
    assert f["pois"] == len(sim.world.state.pois)
    assert 1 <= f["surveyors"] <= f["pois"] and f["over_budget"] is False
    assert len(sim.world.state.uavs) == f["uavs"]
