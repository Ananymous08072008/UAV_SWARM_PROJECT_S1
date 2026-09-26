"""Stage 5 tests: energy estimates, return-to-home, recharging and relay handover."""

from core.config import Parameters
from core.events import EventType
from core.uav import UAV, UAVRole
from simulation.battery import BatteryModel
from tests.helpers import make_sim, run_until, step_world


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


def test_battery_depletion_ends_the_mission_for_that_uav():
    sim = make_sim()
    run_until(sim, 40)
    uav = next(u for u in sim.world.state.uavs.values() if u.is_airborne)
    uav.battery_pct = 0.05
    run_until(sim, 60)
    assert not uav.is_operational
    assert sim.world.events.count(EventType.UAV_FAILED) >= 1
