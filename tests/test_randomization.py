"""Per-run randomness: a fresh seed, random PoI count and placement, and event
times drawn from windows - all replayable from the seed."""

from __future__ import annotations

import math

import numpy as np
import pytest

from core.config import ConfigError, ScenarioConfig, TriggerSpec
from core.events import EventType, Trigger, TriggerAction, TriggerSchedule
from core.poi import PoIStatus
from core.uav import UAVRole
from core.world import CommandError, _spatial_priorities
from main import build_simulation, parse_args
from simulation.obstacles import square
from tests.helpers import make_sim, make_world, run_until, scenario, step_world

REGION = [300.0, 100.0, 800.0, 600.0]


# ------------------------------------------------------------------------- seed
def test_seed_random_draws_a_new_seed_on_every_load():
    data = {"scenario": {"name": "r", "seed": "random"}}
    seeds = {ScenarioConfig.from_dict(data).seed for _ in range(5)}
    assert len(seeds) > 1
    assert ScenarioConfig.from_dict(data).random_seed is True


def test_a_fixed_seed_is_left_alone():
    loaded = ScenarioConfig.from_dict({"scenario": {"name": "f", "seed": 9}})
    assert loaded.seed == 9 and loaded.random_seed is False


def test_an_unknown_seed_word_is_rejected():
    with pytest.raises(ConfigError, match="seed"):
        ScenarioConfig.from_dict({"scenario": {"name": "x", "seed": "sometimes"}})


def test_main_draws_a_seed_per_run_and_seed_flag_pins_it():
    def seed_of(*argv):
        sim, _, _ = build_simulation(parse_args(["--no-results", *argv]))
        return sim.world.seed, sim.world.scenario.random_seed

    drawn = {seed_of()[0] for _ in range(4)}
    assert len(drawn) > 1
    assert seed_of("--seed", "123") == (123, False)


# ----------------------------------------------------------------- random PoIs
def test_poi_count_is_drawn_from_its_range():
    def drawn(seed):
        world = make_world(scenario={"seed": seed}, random_pois={"count": [2, 6]})
        return sum(p.poi_id.startswith("POI-R") for p in world.state.pois)   # the 2 fixed ones aside

    counts = {drawn(s) for s in range(40)}
    assert counts <= {2, 3, 4, 5, 6} and len(counts) > 2


def test_random_pois_stay_in_their_region_and_apart():
    for seed in range(15):
        world = make_world(scenario={"seed": seed},
                           random_pois={"count": 6, "region_m": REGION, "min_spacing_m": 100.0})
        xy = [(float(p.position[0]), float(p.position[1])) for p in world.state.pois
              if p.poi_id.startswith("POI-R")]
        assert len(xy) == 6
        for x, y in xy:
            assert REGION[0] <= x <= REGION[2] and REGION[1] <= y <= REGION[3]
        gaps = [math.dist(a, b) for i, a in enumerate(xy) for b in xy[i + 1:]]
        assert min(gaps) >= 100.0


def test_an_overcrowded_region_still_places_every_poi():
    # 30 PoIs 500 m apart cannot fit; the run must still start, not crash.
    world = make_world(random_pois={"count": 30, "region_m": REGION, "min_spacing_m": 500.0})
    assert sum(p.poi_id.startswith("POI-R") for p in world.state.pois) == 30


def test_a_fixed_count_draws_exactly_what_it_always_did():
    # Existing scenarios with `count: N` must reproduce their old PoIs per seed.
    world = make_world(scenario={"seed": 11}, random_pois={"count": 3})
    cfg, area = world.scenario.random_pois, world.scenario.area
    rng = np.random.default_rng(11)
    positions = []
    for _ in range(1, 4):
        x = rng.uniform(area.x_min_m + cfg.margin_m, area.x_max_m - cfg.margin_m)
        y = rng.uniform(area.y_min_m + cfg.margin_m, area.y_max_m - cfg.margin_m)
        rng.uniform(*cfg.survey_time_s)
        positions.append((x, y))
    gcs = world.state.gcs_position[:2]
    priorities = _spatial_priorities(positions, gcs, cfg.cluster_radius_m)
    for i, ((x, y), priority) in enumerate(zip(positions, priorities), start=1):
        poi = world.state.pois.get(f"POI-R{i}")
        assert poi.position[:2] == pytest.approx((x, y))
        assert poi.priority == priority   # not drawn: derived from the final positions


@pytest.mark.parametrize("random_pois, message", [
    ({"count": [5, 3]}, "count"),
    ({"count": [1, 2.5]}, "count"),
    ({"count": 2, "region_m": [0, 0, 5000, 100]}, "not inside the area"),
    ({"count": 2, "region_m": [100, 100, 50, 200]}, "min < max"),
    ({"count": 2, "min_spacing_m": -1}, "min_spacing_m"),
])
def test_impossible_random_poi_configs_are_rejected(random_pois, message):
    with pytest.raises(ConfigError, match=message):
        scenario(random_pois=random_pois)


# ------------------------------------------------------------- event windows
def test_windowed_event_times_are_drawn_inside_the_window_per_seed():
    timeline = [{"at_s": [50, 150], "action": "degrade_link", "params": {"uav_id": 1}},
                {"at_s": 20, "action": "restore_link", "params": {"uav_id": 1}}]

    def times(seed):
        world = make_world(scenario={"seed": seed}, timeline=timeline)
        return [(t.action, t.at_s) for t in world.timeline]

    drawn = [times(s)[0][1] for s in range(20)]
    assert all(50 <= t <= 150 for t in drawn) and len(set(drawn)) > 10
    assert times(4) == times(4)
    assert all(t[1] == ("restore_link", 20.0) for t in (times(s) for s in range(5)))   # fixed stays fixed


def test_the_drawn_schedule_is_published_at_start():
    world = make_world(timeline=[{"at_s": [10, 20], "action": "degrade_link", "params": {"uav_id": 1}}])
    world.start()
    started = world.events.history(types=[EventType.SIM_STARTED])[0]
    assert started.data["timeline"] == [{"action": "degrade_link", "at_s": world.timeline[0].at_s}]


@pytest.mark.parametrize("trigger, message", [
    ({"at_s": [200, 100], "action": "fail_uav"}, "earliest <= latest"),
    ({"at_s": [10, 20], "at_s_max": 30, "action": "fail_uav"}, "not both"),
    ({"at_s": [100, 400], "action": "fail_uav"}, "after the scenario end"),
])
def test_bad_windows_are_rejected(trigger, message):
    with pytest.raises(ConfigError, match=message):
        scenario(timeline=[trigger])     # duration 300 s


def test_a_window_is_clipped_to_a_shorter_run():
    spec = TriggerSpec([50.0, 200.0], "fail_uav")
    assert spec.clipped(300.0) is spec
    assert (spec.clipped(120.0).at_s, spec.clipped(120.0).latest_s) == (50.0, 120.0)
    assert spec.clipped(40.0) is None


def test_deferred_triggers_keep_the_schedule_ordered():
    schedule = TriggerSchedule([Trigger(1.0, TriggerAction.FAIL_UAV, {}, 0),
                                Trigger(5.0, TriggerAction.FAIL_UAV, {}, 1)])
    first = schedule.pop_due(1.0)[0]
    schedule.defer(first, 7.0)
    assert [t.index for t in schedule.pop_due(10.0)] == [1, 0]


# ------------------------------------------- triggers that need a target to exist
def test_a_windowed_trigger_waits_for_its_target_then_fires():
    world = make_world(timeline=[{"at_s": [5, 6], "action": "degrade_link",
                                  "params": {"uav_id": "role:RELAY", "quality": 0.3}}])
    world.start()
    while world.t < 15.0:
        world.step()
    assert not world.events.history(types=[EventType.TRIGGER_FIRED])   # no relay yet: still waiting
    world.state.get_uav(2).role = UAVRole.RELAY
    while world.t < 20.0:
        world.step()
    fired = world.events.history(types=[EventType.TRIGGER_FIRED])
    assert len(fired) == 1 and fired[0].data["planned_s"] <= 6.0
    assert world.state.get_uav(2).comm.radio_health == pytest.approx(0.3)


def test_a_windowed_trigger_gives_up_after_its_grace_period():
    world = make_world(timeline=[{"at_s": [5, 6], "action": "fail_uav", "params": {"uav_id": "role:RELAY"}}])
    world.start()
    while world.t < 60.0:
        world.step()
    rejected = world.events.history(types=[EventType.TRIGGER_REJECTED])
    assert len(rejected) == 1 and 30.0 <= rejected[0].t_s <= 40.0


def test_a_fixed_time_trigger_never_waits():
    world = make_world(timeline=[{"at_s": 5, "action": "fail_uav", "params": {"uav_id": "role:RELAY"}}])
    world.start()
    while world.t < 8.0:
        world.step()
    assert world.events.history(types=[EventType.TRIGGER_REJECTED])[0].t_s == pytest.approx(5.0, abs=0.2)


# ------------------------------------------------------------------ selectors
def test_random_active_prefers_a_poi_under_survey():
    world = make_world()
    world.state.pois.get("POI-B").status = PoIStatus.IN_PROGRESS
    assert {world.resolve_poi("random_active") for _ in range(10)} == {"POI-B"}
    assert world.resolve_poi("POI-A") == "POI-A"


def test_random_active_fails_once_everything_is_done():
    world = make_world()
    for poi in world.state.pois:
        poi.status = PoIStatus.COMPLETED
    with pytest.raises(CommandError, match="matched no PoI"):
        world.resolve_poi("random_active")


def test_complete_poi_trigger_accepts_the_selector():
    world = make_world(timeline=[{"at_s": 1, "action": "complete_poi", "params": {"poi_id": "random_active"}}])
    world.start()
    while world.t < 2.0:
        world.step()
    assert len(world.state.pois.completed()) == 1


def test_a_random_urgent_poi_lands_in_the_region_and_not_in_debris():
    sim = make_sim(random_pois={"count": 0, "region_m": REGION})
    # Debris over most of the region: the new PoI must go to the free strip.
    sim.env.obstacles.add(square("DEBRIS", (500.0, 350.0), 390.0))
    sim.world.inject("add_poi", {"id": "URGENT", "position_m": "random", "priority": 5})
    poi = sim.world.state.pois.get("URGENT")
    x, y = float(poi.position[0]), float(poi.position[1])
    assert REGION[0] <= x <= REGION[2] and REGION[1] <= y <= REGION[3]
    assert sim.env.obstacles.inside((x, y, 1.0)) is None


# ------------------------------------------------------------ random spawn times
def test_pois_can_appear_at_random_times():
    world = make_world(pois=[], scenario={"duration_s": 900.0},
                       random_pois={"count": 4, "region_m": REGION, "spawn_s": [100.0, 500.0]})
    assert len(world.state.pois) == 0 and world.pending_spawns == 4      # nothing to see at launch
    times = [p.created_at_s for p in world.scheduled_pois]
    assert times == sorted(times) and all(100.0 <= t <= 500.0 for t in times)
    step_world(world, times[0] + 0.5)
    assert len(world.state.pois) == 1 and world.pending_spawns == 3
    assert world.events.count(EventType.POI_ADDED) == 1
    step_world(world, 501.0)
    assert len(world.state.pois) == 4 and world.pending_spawns == 0


def test_spawn_times_do_not_move_the_pois_of_a_seed():
    def layout(pois):
        return sorted((p.poi_id, tuple(np.round(p.position, 3)), p.priority, p.survey_time_s) for p in pois)

    common = {"pois": [], "scenario": {"seed": 11, "duration_s": 900.0}}
    at_once = make_world(**common, random_pois={"count": 5, "region_m": REGION})
    later = make_world(**common, random_pois={"count": 5, "region_m": REGION, "spawn_s": [0.0, 600.0]})
    assert layout(at_once.state.pois) == layout(later.scheduled_pois)


def test_a_spawn_window_past_the_end_is_rejected():
    with pytest.raises(ConfigError, match="spawn_s"):
        scenario(random_pois={"count": 2, "spawn_s": [0.0, 400.0]})       # the run lasts 300 s


# -------------------------------------------- bugs the random layouts exposed
def test_a_uav_lifted_over_new_debris_keeps_flying_to_its_destination():
    sim = make_sim()
    run_until(sim, 40)
    uav = next(u for u in sim.world.state.uavs.values()
               if u.is_airborne and u.target is not None and u.role is not UAVRole.RETURNING
               and u.horizontal_distance_to(u.target) > 20.0)
    destination = uav.target[:2].copy()
    x, y = float(uav.position[0]), float(uav.position[1])
    sim.env.obstacles.add(square("DEBRIS", (x, y), 60, height_m=float(uav.position[2]) + 20.0))
    sim.manager.safety.monitor()
    assert uav.target[:2] == pytest.approx(destination)      # was: its own position, forever
    assert uav.target[2] >= uav.position[2] + 20.0


def test_a_poi_added_after_the_mission_reopens_it():
    sim = make_sim(scenario={"duration_s": 900.0})   # time left to recharge and fly again
    sim.start()
    while sim.manager.mission_complete_s is None and sim.world.t < 890:
        sim.tick()
    assert sim.manager.mission_complete_s is not None
    sim.world.inject("add_poi", {"id": "LATE", "position_m": [400, 200], "priority": 5, "survey_time_s": 20})
    assert not sim.is_finished                          # at once, not only at the next decision cycle
    start = sim.world.t
    while sim.world.t < start + 1.5:
        sim.tick()
    assert sim.manager.mission_complete_s is None       # the mission is open again
    sim.run()
    assert sim.world.state.pois.get("LATE").is_completed


def test_the_mission_is_not_over_while_pois_are_still_to_appear():
    """Finishing every PoI seen so far must not recall the swarm when more are due."""
    sim = make_sim(pois=[{"id": "EARLY", "position_m": [200, 0], "priority": 3, "survey_time_s": 20}],
                   random_pois={"count": 1, "region_m": [150, -100, 300, 100], "spawn_s": [400.0, 400.0]},
                   scenario={"duration_s": 900.0})
    run_until(sim, 350)
    assert sim.world.state.pois.get("EARLY").is_completed
    assert sim.manager.mission_complete_s is None and not sim.is_finished
    recalls = [e for e in sim.world.events.history(types=[EventType.RTH_STARTED]) if "mission complete" in e.message]
    assert not recalls
    sim.run()
    assert sim.world.state.pois.get("POI-R1").is_completed
