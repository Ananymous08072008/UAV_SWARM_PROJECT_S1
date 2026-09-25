"""Stage 7 tests: safety (flight levels, separation, geofence, obstacles, deadline) and imagery delivery."""

import pytest

from core.config import BatteryParams, ConfigError, UAVParams
from core.events import EventType
from core.poi import PoIStatus
from core.uav import UAV, UAVRole
from core.world import CommandError
from simulation.battery import BatteryModel
from simulation.data_model import DataModel, DataParams
from simulation.obstacles import square
from tests.helpers import make_sim, make_world, run_until, step_world


# ------------------------------------------------------------------- safety
def test_flight_levels_are_a_collision_distance_apart_and_under_the_ceiling():
    sim = make_sim()
    safety = sim.manager.safety
    assert safety.levels == (20.0, 40.0, 60.0, 80.0, 100.0)
    assert sim.world.params.uav.max_altitude_m == 100.0
    assert min(b - a for a, b in zip(safety.levels, safety.levels[1:])) >= safety.params.min_separation_m == 20.0


def test_nothing_may_fly_above_100_m():
    sim = make_sim()
    uav = sim.world.state.get_uav(1)
    with pytest.raises(CommandError, match="outside 0..100"):
        sim.world.goto(uav.uav_id, (100.0, 0.0, 110.0))


def test_a_full_battery_lasts_1200_s_at_most():
    params = UAVParams()
    battery = BatteryParams()
    uav = UAV(1, (0, 0, 0))
    uav.goto((0, 0, 40))                        # climb, then hover: the least draining way to fly
    steps = 0
    while uav.is_operational and uav.battery_pct > 0 and steps < 20000:
        uav.step(0.1, params, battery)
        steps += 1
    assert uav.flight_time_s == pytest.approx(1200.0, abs=15.0)
    assert BatteryModel(params, battery).endurance_s(100.0) == pytest.approx(1200.0)


def test_launch_pads_closer_than_the_separation_are_rejected():
    with pytest.raises(ConfigError, match="collide on take-off"):
        make_sim(uavs={"spacing_m": 15.0})


def test_a_station_next_to_another_uav_gets_a_different_level():
    sim = make_sim()
    world, safety = sim.world, sim.manager.safety
    a, b = world.state.get_uav(1), world.state.get_uav(2)
    a.position[:] = (300.0, 300.0, 60.0)
    world.goto(1, (300.0, 300.0, 60.0))
    level = safety.safe_altitude(b, (310.0, 300.0), preferred_m=60.0)
    assert abs(level - 60.0) >= safety.params.min_separation_m
    assert safety.safe_altitude(b, (500.0, 300.0), preferred_m=60.0) == 60.0    # far away: its own level


def test_waypoints_are_raised_over_obstacles_on_the_way():
    sim = make_sim()
    safety = sim.manager.safety
    uav = sim.world.state.get_uav(1)
    clear = safety.safe_altitude(uav, (400, 0))
    sim.env.obstacles.add(square("TOWER", (200, 0), 60, height_m=90.0))
    raised = safety.safe_altitude(uav, (400, 0))
    assert raised >= 90.0 + safety.params.obstacle_clearance_m > clear


def test_closer_than_20_m_is_a_collision_that_destroys_both_uavs():
    sim = make_sim()
    world, safety = sim.world, sim.manager.safety
    a, b, c = (world.state.get_uav(i) for i in (1, 2, 3))
    a.position[:] = (100.0, 100.0, 60.0)
    b.position[:] = (115.0, 100.0, 60.0)       # 15 m: collision
    c.position[:] = (100.0, 100.0, 80.0)       # exactly 20 m above a: not a collision
    safety.monitor()
    assert not a.is_operational and not b.is_operational and c.is_operational
    assert safety.collisions == 1 and safety.violations["separation"] == 1
    reasons = [e.data["reason"] for e in world.events.history(types=[EventType.UAV_FAILED])]
    assert reasons == ["mid-air collision with UAV-02", "mid-air collision with UAV-01"]
    assert safety.min_separation_observed_m == pytest.approx(15.0)


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
    assert sim.manager.safety.violations["obstacle"] >= 1
    assert uav.target is not None
    assert uav.target[2] >= min(uav.position[2] + 20.0, 100.0)      # commanded above the obstacle


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


def fly(sim, seconds: float) -> None:
    """Only the world and the safety layer: no other swarm module re-tasks these UAVs."""
    safety, world = sim.manager.safety, sim.world
    for _ in range(int(seconds / world.dt)):
        safety.monitor()
        safety.avoid()
        world.step()


def airborne_at(world, uav_id: int, position) -> None:
    uav = world.state.get_uav(uav_id)
    uav.position[:] = position
    uav.home[:] = (uav.home[0], uav.home[1], 0.0)


def test_head_on_uavs_on_one_level_pass_safely_and_return_to_it():
    sim = make_sim()
    world, safety = sim.world, sim.manager.safety
    airborne_at(world, 1, (0.0, 300.0, 60.0))
    airborne_at(world, 2, (300.0, 300.0, 60.0))
    world.goto(1, (300.0, 300.0, 60.0))
    world.goto(2, (0.0, 300.0, 60.0))
    fly(sim, 60)
    a, b = world.state.get_uav(1), world.state.get_uav(2)
    assert safety.collisions == 0 and a.is_operational and b.is_operational
    assert safety.deconflictions >= 1
    assert a.distance_to((300.0, 300.0, 60.0)) < 3.0 and b.distance_to((0.0, 300.0, 60.0)) < 3.0
    assert not a.braking and not b.braking


def test_a_uav_does_not_climb_into_the_one_parked_above_it():
    sim = make_sim()
    world, safety = sim.world, sim.manager.safety
    airborne_at(world, 1, (300.0, 300.0, 60.0))
    airborne_at(world, 2, (300.0, 300.0, 40.0))
    world.goto(1, (300.0, 300.0, 60.0))
    world.goto(2, (500.0, 300.0, 60.0))          # sent off - on the level of the one above
    fly(sim, 40)
    assert safety.collisions == 0
    assert world.state.get_uav(2).distance_to((500.0, 300.0, 60.0)) < 3.0


def test_crossing_traffic_never_comes_within_20_m():
    sim = make_sim()
    world, safety = sim.world, sim.manager.safety
    starts = [(0.0, 300.0), (300.0, 0.0), (300.0, 600.0), (600.0, 300.0)]
    for uav_id, (x, y) in enumerate(starts, start=1):
        airborne_at(world, uav_id, (x, y, 60.0))
        world.goto(uav_id, (600.0 - x, 600.0 - y, 60.0))   # all through the centre, all on one level
    fly(sim, 90)
    assert safety.collisions == 0
    assert safety.min_separation_observed_m >= 20.0
    for uav_id, (x, y) in enumerate(starts, start=1):
        assert world.state.get_uav(uav_id).horizontal_distance_to((600.0 - x, 600.0 - y)) < 3.0


def test_auto_placed_debris_never_lands_on_a_uav():
    sim = make_sim()
    run_until(sim, 110)
    sim.world.inject("add_obstacle", {"id": "D", "center_m": "backbone_midpoint", "size_m": 400.0,
                                      "height_m": 70.0, "attenuation_db": 40.0})
    obstacle = next(iter(sim.env.obstacles))
    assert all(not obstacle.contains_xy(u.position[0], u.position[1])
               for u in sim.world.state.operational_uavs() if u.is_airborne)
