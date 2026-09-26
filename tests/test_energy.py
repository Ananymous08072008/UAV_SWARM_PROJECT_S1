"""Stage 5 tests: energy estimates, return-to-home, recharging and relay handover."""

import numpy as np

from core.config import Parameters
from core.events import EventType
from core.poi import PoIStatus
from core.uav import UAV, UAVRole
from simulation.battery import BatteryModel
from tests.helpers import make_sim, parameters, run_until, step_world

# The mission constraints' rule: home at 20 % - earlier only if the trip needs more - and no other early return.
RTH_AT_20 = parameters(battery={"critical_pct": 20.0},
                       swarm={"energy": {"idle_recharge_below_pct": 0.0, "park_when_idle": False,
                                         "leave_unfinishable_survey": False}})


def test_estimates_grow_with_distance_and_hover_time():
    params = Parameters()
    model = BatteryModel(params.uav, params.battery)
    uav = UAV(1, (0, 0, 40))
    near = model.task_cost_pct(uav, (200, 0, 40), 30)
    far = model.task_cost_pct(uav, (800, 0, 40), 30)
    longer_survey = model.task_cost_pct(uav, (200, 0, 40), 120)
    assert near < far and near < longer_survey
    assert model.endurance_s(100.0) > model.endurance_s(50.0)
    far, near_home = UAV(2, (0, 0, 0)), UAV(3, (0, 0, 0))   # home is where a UAV starts
    far.position[:] = (600.0, 0.0, 40.0)
    near_home.position[:] = (50.0, 0.0, 40.0)
    assert model.return_cost_pct(far) > model.return_cost_pct(near_home)


def test_uav_returns_home_when_the_reserve_is_reached():
    sim = make_sim()
    run_until(sim, 60)
    uav = next(u for u in sim.world.state.uavs.values() if u.is_airborne and u.role is UAVRole.SURVEY)
    needed = sim.manager.energy.needed_pct(uav)
    poi = sim.world.state.pois.get(uav.assigned_poi)
    # Enough to finish the survey too, so only the reserve can send it home.
    finish = (sim.env.battery.task_cost_pct(uav, uav.target, poi.survey_time_s - poi.progress_s)
              + sim.manager.energy.params.reserve_pct * 0.5)
    sim.world.set_battery(uav.uav_id, max(needed, finish) + 1.0, "test")
    run_until(sim, 70)
    assert uav.role is UAVRole.SURVEY, "above the reserve it keeps working"
    sim.world.set_battery(uav.uav_id, needed - 0.5, "test")
    run_until(sim, 80)
    assert uav.role in (UAVRole.RETURNING, UAVRole.CHARGING)
    assert sim.world.events.count(EventType.RTH_STARTED) >= 1


def test_returning_uav_lands_charges_and_becomes_available_again():
    sim = make_sim(scenario={"duration_s": 900.0})
    run_until(sim, 60)
    uav = next(u for u in sim.world.state.uavs.values() if u.is_airborne)
    sim.world.return_home(uav.uav_id, "test")
    run_until(sim, 240)
    assert sim.world.events.count(EventType.UAV_LANDED) >= 1
    assert uav.battery_pct > 50
    run_until(sim, 400)
    assert uav.role is not UAVRole.RETURNING
    assert not uav.is_airborne or uav.role in (UAVRole.SURVEY, UAVRole.RELAY, UAVRole.BACKUP, UAVRole.IDLE)


def test_a_draining_relay_asks_for_a_replacement_before_leaving():
    sim = make_sim(scenario={"duration_s": 900.0})
    run_until(sim, 70)
    relay = next((u for u in sim.world.state.uavs_with_role(UAVRole.RELAY)), None)
    assert relay is not None, "the scenario needs relays for this test"
    needed = sim.manager.energy.needed_pct(relay)
    assert needed >= sim.world.params.battery.critical_pct        # never plan to land below the floor
    sim.world.set_battery(relay.uav_id, needed + 3.0, "test")     # inside the handover margin
    run_until(sim, 90)
    assert sim.world.events.count(EventType.HANDOVER_STARTED) >= 1
    assert relay.uav_id in sim.manager.relays.leaving or relay.role is UAVRole.RETURNING
    run_until(sim, 200)
    assert relay.role in (UAVRole.RETURNING, UAVRole.CHARGING, UAVRole.IDLE)


def test_critical_battery_beats_the_handover_wait():
    sim = make_sim()
    run_until(sim, 70)
    relay = next(iter(sim.world.state.uavs_with_role(UAVRole.RELAY)), None)
    assert relay is not None
    sim.world.set_battery(relay.uav_id, sim.manager.energy.needed_pct(relay) - 1.0, "test")
    run_until(sim, 85)
    assert relay.role is UAVRole.RETURNING


def test_an_idle_uav_with_nothing_to_do_parks_instead_of_hovering():
    """Above the recharge threshold too: hovering with no role only burns battery."""
    sim = make_sim()
    run_until(sim, 40)
    idle, busy = [u for u in sim.world.state.uavs.values() if u.is_airborne][:2]
    for uav in (idle, busy):
        sim.world.release_uav(uav.uav_id, "test")
    energy = sim.manager.energy
    energy.update(sim.world, lambda u: u is busy)        # starts both idle clocks
    step_world(sim.world, sim.world.t + energy.params.idle_recharge_after_s + 1)
    energy.update(sim.world, lambda u: u is busy)
    assert idle.battery_pct > energy.params.idle_recharge_below_pct
    assert idle.role is UAVRole.RETURNING
    assert busy.role is UAVRole.IDLE                      # it could still be tasked: it waits airborne


def test_the_return_level_is_the_threshold_unless_the_trip_home_needs_more():
    sim = make_sim(params=parameters(battery={"critical_pct": 20.0}, uav={"cruise_speed_mps": 5.0}))
    energy, uav = sim.manager.energy, sim.world.state.get_uav(1)
    assert energy.needed_at(uav, np.array([100.0, 0.0, 60.0])) == 20.0     # close in: exactly 20 %
    assert energy.needed_at(uav, np.array([850.0, 600.0, 60.0])) > 30.0    # 1 km out at 5 m/s: what the trip takes


def test_an_idle_uav_stays_available_until_the_threshold():
    sim = make_sim(params=RTH_AT_20)
    run_until(sim, 40)
    idle = next(u for u in sim.world.state.uavs.values() if u.is_airborne)
    sim.world.release_uav(idle.uav_id, "test")
    idle.battery_pct = 50.0                                  # below the default 60 % idle-recharge line
    energy = sim.manager.energy
    energy.update(sim.world, lambda u: False)
    step_world(sim.world, sim.world.t + energy.params.idle_recharge_after_s + 1)
    energy.update(sim.world, lambda u: False)
    assert idle.role is UAVRole.IDLE and idle.is_airborne    # no parking, no early recharge


def test_an_idle_uav_waits_airborne_over_its_own_pad():
    """With nothing to do it comes back to stand by over the operational center - still flying and
    taskable, not landed - instead of hovering out in the field."""
    params = parameters(swarm={"idle_standby_after_s": 20.0,
                               "energy": {"park_when_idle": False, "idle_recharge_below_pct": 0.0}})
    sim = make_sim(params=params, scenario={"duration_s": 900.0},
                   pois=[{"id": "ONE", "position_m": [300, 0], "priority": 3, "survey_time_s": 10}],
                   random_pois={"count": 1, "region_m": [150, -100, 300, 100], "spawn_s": [600.0, 600.0]})
    run_until(sim, 250)
    assert sim.world.state.pois.get("ONE").is_completed
    uav = sim.world.state.get_uav(sim.world.state.pois.get("ONE").completed_by)
    assert uav.role is UAVRole.IDLE and uav.is_airborne
    assert uav.horizontal_distance_to(uav.home) < 3.0
    assert uav.position[2] <= sim.manager.safety.levels[0] + 1.0          # low, under the transit levels


def test_an_idle_uav_comes_in_above_the_waiting_ones_and_down_its_own_column():
    """The others wait at the standby level over pads 30 m apart - no gap to fly through - so
    a UAV heading for its pad stays above that level until it is over its own."""
    params = parameters(swarm={"idle_standby_after_s": 20.0,
                               "energy": {"park_when_idle": False, "idle_recharge_below_pct": 0.0}})
    sim = make_sim(params=params, scenario={"duration_s": 900.0},
                   pois=[{"id": "ONE", "position_m": [300, 0], "priority": 3, "survey_time_s": 10}],
                   random_pois={"count": 1, "region_m": [150, -100, 300, 100], "spawn_s": [600.0, 600.0]})
    levels = sim.manager.safety.levels
    lowest_on_the_way: dict[int, float] = {}
    while sim.world.t < 250.0:
        sim.tick()
        for u in sim.world.state.uavs.values():
            homing = (u.role is UAVRole.IDLE and u.target is not None
                      and float(np.hypot(*(u.target[:2] - u.home[:2]))) < 1.0)
            if homing and u.horizontal_distance_to(u.home) > 3.0:
                lowest_on_the_way[u.uav_id] = min(lowest_on_the_way.get(u.uav_id, 1e9), float(u.position[2]))
    uav = sim.world.state.get_uav(sim.world.state.pois.get("ONE").completed_by)
    assert lowest_on_the_way[uav.uav_id] >= levels[1] - 1.0
    assert uav.horizontal_distance_to(uav.home) < 3.0 and uav.position[2] <= levels[0] + 1.0


def test_a_surveyor_short_of_battery_keeps_surveying_until_the_threshold():
    sim = make_sim(params=RTH_AT_20, uavs={"count": 1, "per_row": 1}, scenario={"duration_s": 1800.0},
                   pois=[{"id": "LONG", "position_m": [200, 0], "priority": 5, "survey_time_s": 300}])
    run_until(sim, 60)
    uav, poi = sim.world.state.get_uav(1), sim.world.state.pois.get("LONG")
    assert poi.status is PoIStatus.IN_PROGRESS
    sim.world.set_battery(uav.uav_id, 30.0, "test")         # nowhere near enough to finish
    run_until(sim, 90)
    assert uav.role is UAVRole.SURVEY                         # by default it would already be heading home
    while uav.role is UAVRole.SURVEY and sim.world.t < 400:
        sim.tick()
    assert uav.role is UAVRole.RETURNING
    assert 19.0 <= uav.battery_pct <= 21.0                    # it left at the threshold, not before
    assert poi.progress_s > 60 and poi.status is PoIStatus.PENDING     # and the survey so far is kept


def test_battery_depletion_ends_the_mission_for_that_uav():
    sim = make_sim()
    run_until(sim, 40)
    uav = next(u for u in sim.world.state.uavs.values() if u.is_airborne)
    uav.battery_pct = 0.05
    run_until(sim, 60)
    assert not uav.is_operational
    assert sim.world.events.count(EventType.UAV_FAILED) >= 1
