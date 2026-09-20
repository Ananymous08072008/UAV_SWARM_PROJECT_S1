"""Stage 3 tests: communication- and energy-aware task allocation."""

from core.poi import PoIStatus
from core.uav import UAVRole
from tests.helpers import make_sim, run_until


def test_highest_priority_poi_is_served_first():
    sim = make_sim()
    sim.start()
    sim.tick()
    assigned = {p.poi_id: p.assigned_uav for p in sim.world.state.pois if p.assigned_uav}
    assert "POI-B" in assigned          # priority 4 before priority 3
    poi_b = sim.world.state.pois.get("POI-B")
    assert sim.world.state.get_uav(poi_b.assigned_uav).role is UAVRole.SURVEY


def test_nearest_capable_uav_wins_the_auction():
    sim = make_sim()
    sim.start()
    sim.tick()
    poi = sim.world.state.pois.get("POI-A")
    winner = sim.world.state.get_uav(poi.assigned_uav)
    waypoint = (poi.position[0], poi.position[1], 40.0)
    best = min(sim.world.state.uavs.values(), key=lambda u: u.distance_to(waypoint))
    assert winner.distance_to(waypoint) <= best.distance_to(waypoint) + 60.0


def test_uav_without_the_battery_for_the_round_trip_is_not_used():
    sim = make_sim(uavs={"count": 2, "per_row": 2, "initial_battery_pct": 12.0})
    sim.start()
    sim.tick()
    assert all(u.role is not UAVRole.SURVEY for u in sim.world.state.uavs.values())
    assert all(p.status is PoIStatus.PENDING for p in sim.world.state.pois)


def test_communication_budget_holds_back_surveys_a_small_fleet_cannot_support():
    """Adaptive mode keeps enough UAVs for relays; baseline sends everyone out."""
    adaptive = make_sim("adaptive", uavs={"count": 3, "per_row": 3})
    baseline = make_sim("baseline", uavs={"count": 3, "per_row": 3})
    for sim in (adaptive, baseline):
        sim.start()
        sim.tick()
    surveying = lambda sim: len(sim.world.state.uavs_with_role(UAVRole.SURVEY))  # noqa: E731
    assert surveying(adaptive) < surveying(baseline)
    assert surveying(baseline) == 2


def test_blocked_poi_is_eventually_surveyed_by_data_ferrying():
    sim = make_sim("adaptive", uavs={"count": 3, "per_row": 3},
                   scenario={"duration_s": 900.0})
    sim.manager.allocator.params = type(sim.manager.allocator.params)(
        reserve_pct=10.0, battery_weight_s=2.0, ferry_wait_s=30.0)
    run_until(sim, 200)
    ferried = [e for e in sim.world.events.history() if "data ferry" in e.message]
    assert ferried, "a long-blocked PoI should eventually be surveyed with store-and-forward"
    assert sim.manager.allocator.ferrying or sim.world.state.pois.completed()


def test_allocation_is_fast_enough_for_real_time():
    sim = make_sim()
    run_until(sim, 60)
    runtimes = sim.manager.allocator.runtime_ms
    assert runtimes and max(runtimes) < 50.0     # well inside the 1 s decision interval


def test_completed_poi_releases_its_uav_for_the_next_task():
    sim = make_sim(pois=[{"id": "ONE", "position_m": [200, 0], "priority": 5, "survey_time_s": 10},
                         {"id": "TWO", "position_m": [260, 60], "priority": 1, "survey_time_s": 10}],
                   uavs={"count": 1, "per_row": 1})
    run_until(sim, 200)
    assert sim.world.state.pois.get("ONE").is_completed
    assert sim.world.state.pois.get("TWO").is_completed     # the same UAV did both, in priority order
