"""Stage 7 tests: safety (flight levels, separation, geofence, obstacles, deadline) and imagery delivery."""

import pytest

from core.events import EventType
from core.poi import PoIStatus
from core.uav import UAVRole
from simulation.data_model import DataModel, DataParams
from simulation.obstacles import square
from tests.helpers import make_sim, make_world, run_until, step_world


# ------------------------------------------------------------------- safety
def test_every_uav_owns_a_different_flight_level():
    sim = make_sim(uavs={"count": 14, "per_row": 7})     # more UAVs than configured layers
    safety = sim.manager.safety
    levels = [safety.slot_altitude(uav_id) for uav_id in range(1, 15)]
    assert len(set(levels)) == 14
    assert min(levels) >= safety.params.altitude_floor_m
    assert max(levels) <= sim.world.params.uav.max_altitude_m
    assert min(b - a for a, b in zip(levels, levels[1:])) >= safety.params.min_separation_m


def test_waypoints_are_raised_over_obstacles_on_the_way():
    sim = make_sim()
    safety = sim.manager.safety
    uav = sim.world.state.get_uav(1)
    clear = safety.safe_altitude(uav, (400, 0))
    sim.env.obstacles.add(square("TOWER", (200, 0), 60, height_m=90.0))
    raised = safety.safe_altitude(uav, (400, 0))
    assert raised >= 90.0 + safety.params.obstacle_clearance_m > clear


def test_separation_violation_is_reported_once_per_encounter():
    sim = make_sim()
    world, safety = sim.world, sim.manager.safety
    a, b = world.state.get_uav(1), world.state.get_uav(2)
    a.position[:] = (100.0, 100.0, 50.0)
    b.position[:] = (102.0, 100.0, 50.0)
    safety.monitor()
    safety.monitor()                       # still too close: no second event
    assert safety.violations["separation"] == 1
    assert world.events.count(EventType.SAFETY_VIOLATION) == 1
    b.position[:] = (150.0, 100.0, 50.0)
    safety.monitor()
    b.position[:] = (101.0, 100.0, 50.0)   # a new encounter counts again
    safety.monitor()
    assert safety.violations["separation"] == 2
    assert safety.min_separation_observed_m <= 2.0


def test_geofence_violation_is_detected():
    sim = make_sim()
    uav = sim.world.state.get_uav(1)
    uav.position[:] = (5000.0, 0.0, 50.0)
    sim.manager.safety.monitor()
    assert sim.manager.safety.violations["geofence"] == 1


def test_uav_under_a_new_obstacle_climbs_clear():
    sim = make_sim()
    run_until(sim, 40)
    uav = next(u for u in sim.world.state.uavs.values() if u.is_airborne and u.role is not UAVRole.RETURNING)
    x, y = float(uav.position[0]), float(uav.position[1])
    sim.env.obstacles.add(square("DEBRIS", (x, y), 60, height_m=float(uav.position[2]) + 20.0))
    sim.manager.safety.monitor()
    assert sim.manager.safety.violations["obstacle"] == 1
    assert uav.target is not None
    assert uav.target[2] >= uav.position[2] + 20.0      # commanded above the obstacle


def test_everyone_is_recalled_and_lands_before_the_deadline():
    sim = make_sim(scenario={"duration_s": 240.0})    # POI-B cannot be finished in time
    sim.run()
    summary = sim.finish()
    assert sim.world.events.count(EventType.MISSION_RECALL) == 1
    assert summary["safety"]["uavs_airborne_at_end"] == 0


def test_tasks_that_cannot_finish_in_time_are_not_started():
    sim = make_sim(scenario={"duration_s": 60.0})   # POI-A/B are ~400-700 m away with 60 s surveys
    sim.start()
    sim.tick()
    assert not sim.world.state.uavs_with_role(UAVRole.SURVEY)


# --------------------------------------------------------------------- data
def surveying_world():
    world = make_world(pois=[{"id": "NEAR", "position_m": [30, 0], "priority": 5, "survey_time_s": 200}],
                       uavs={"count": 2, "per_row": 2})
    world.assign_poi(1, "NEAR")
    step_world(world, 30)
    assert world.state.pois.get("NEAR").status is PoIStatus.IN_PROGRESS
    return world


def test_imagery_is_buffered_while_disconnected_and_delivered_once_connected():
    world = surveying_world()
    data = DataModel(DataParams(imagery_rate_mbps=3.0, link_capacity_mbps=40.0))
    for _ in range(100):                     # 10 s with no route to the GCS
        world.step()
        data.update(world, world.dt)
    assert data.generated_mb == pytest.approx(30.0, rel=0.05)
    assert data.delivered_mb == 0 and data.buffer_mb(1) == pytest.approx(data.generated_mb)

    world.update_comm(1, connected=True, hop_count=2, pdr=0.9)
    for _ in range(50):
        world.step()
        data.update(world, world.dt)
    stats = data.stats()
    assert stats["delivered_mb"] > 30.0
    assert stats["mean_delay_s"] > 1.0         # the backlog arrived late
    assert data.buffer_mb(1) < 1.0             # 40 Mbit/s / 2 hops easily clears a 3 Mbit/s stream


def test_live_delivery_when_the_route_exists_from_the_start():
    world = surveying_world()
    world.update_comm(1, connected=True, hop_count=1, pdr=1.0)
    data = DataModel(DataParams())
    for _ in range(100):
        world.step()
        data.update(world, world.dt)
    stats = data.stats()
    assert stats["live_ratio"] > 0.95 and stats["mean_delay_s"] < 2.0


def test_landing_downloads_the_buffer_and_a_crash_loses_it():
    world = surveying_world()
    data = DataModel(DataParams())
    for _ in range(100):
        world.step()
        data.update(world, world.dt)
    buffered = data.buffer_mb(1)
    assert buffered > 0
    world.return_home(1, "test")
    for _ in range(600):
        world.step()
        data.update(world, world.dt)
    assert not world.state.get_uav(1).is_airborne
    assert data.buffer_mb(1) == 0 and data.delivered_mb >= buffered - 1e-6

    crashed = surveying_world()
    data2 = DataModel(DataParams())
    for _ in range(50):
        crashed.step()
        data2.update(crashed, crashed.dt)
    crashed.fail_uav(1, "test")
    data2.update(crashed, crashed.dt)
    assert data2.lost_mb > 0 and data2.buffer_mb(1) == 0


def test_converging_uavs_at_similar_heights_are_deconflicted():
    sim = make_sim()
    run_until(sim, 40)
    flying = [u for u in sim.world.state.uavs.values() if u.is_airborne and u.target is not None][:2]
    assert len(flying) == 2
    a, b = sorted(flying, key=lambda u: u.uav_id)
    a.position[:] = (300.0, 300.0, 60.0)
    b.position[:] = (306.0, 300.0, 61.0)       # 6 m apart, 1 m vertically: about to conflict
    sim.manager.safety.monitor()
    assert sim.manager.safety.deconflictions == 1
    assert abs(b.target[2] - a.position[2]) >= 2 * sim.manager.safety.params.min_separation_m - 1e-6
    sim.manager.safety.monitor()                # same encounter: no second command
    assert sim.manager.safety.deconflictions == 1


def test_auto_placed_debris_never_lands_on_a_uav():
    sim = make_sim()
    run_until(sim, 110)
    sim.world.inject("add_obstacle", {"id": "D", "center_m": "backbone_midpoint", "size_m": 400.0,
                                      "height_m": 70.0, "attenuation_db": 40.0})
    obstacle = next(iter(sim.env.obstacles))
    assert all(not obstacle.contains_xy(u.position[0], u.position[1])
               for u in sim.world.state.operational_uavs() if u.is_airborne)
