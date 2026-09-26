"""Stage 3 tests: communication- and energy-aware task allocation."""

from core.events import EventType
from core.poi import PoIStatus
from core.uav import UAVRole
from tests.helpers import make_sim, run_until


def test_highest_priority_poi_is_served_first():
    """POI-B is the nearer of the two here, so the farthest-PoI deferral (POI-A) does not
    interfere with the priority check this test is actually about."""
    sim = make_sim(pois=[{"id": "POI-A", "position_m": [640, 380], "priority": 3, "survey_time_s": 60},
                         {"id": "POI-B", "position_m": [420, 120], "priority": 4, "survey_time_s": 60}])
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
    """Adaptive mode keeps enough UAVs for relays; baseline sends everyone out.
    (Deadline far off, so the last-resort data ferry is not in play.)"""
    adaptive = make_sim("adaptive", uavs={"count": 3, "per_row": 3}, scenario={"duration_s": 1800.0})
    baseline = make_sim("baseline", uavs={"count": 3, "per_row": 3}, scenario={"duration_s": 1800.0})
    for sim in (adaptive, baseline):
        sim.start()
        sim.tick()
    surveying = lambda sim: len(sim.world.state.uavs_with_role(UAVRole.SURVEY))  # noqa: E731
    assert surveying(adaptive) < surveying(baseline)
    assert surveying(baseline) == 2


def test_a_poi_the_fleet_can_never_connect_does_not_hold_the_others_back():
    """3 UAVs: POI-A fits with its relays, POI-B needs more relays than the whole fleet has.
    Waiting for POI-B's chain is futile, so POI-A must be flown (connected) first - not held
    behind it - and POI-B ferried once UAVs are free, long before the deadline gate would open."""
    sim = make_sim("adaptive", uavs={"count": 3, "per_row": 3}, scenario={"duration_s": 1800.0})
    run_until(sim, 30)
    assert sim.world.state.pois.get("POI-A").status is not PoIStatus.PENDING
    assert sim.world.state.pois.get("POI-A").assigned_uav not in sim.manager.allocator.ferrying
    assert sim.manager.allocator.ferry_assignments == 0, "no UAV is free yet: POI-B just waits"

    run_until(sim, 400)
    ferried = sim.world.events.history(types=[EventType.DATA_FERRY_ASSIGNED])
    assert [e.poi_id for e in ferried] == ["POI-B"]
    assert all(e.severity.value == "WARNING" for e in ferried)
    assert ferried[0].t_s < sim.world.duration_s - sim.manager.allocator.params.ferry_deadline_margin_s


def test_a_poi_that_only_waits_for_relays_is_flown_connected_not_ferried():
    """BUSY (nearer) is tasked immediately. LATER is the farthest PoI in the mission, so it is
    held back (TaskAllocator._defer_farthest) until BUSY is done, then flown with its own relay
    chain once the fleet is free - not ferried."""
    sim = make_sim("adaptive", uavs={"count": 6, "per_row": 3}, scenario={"duration_s": 1800.0},
                   pois=[{"id": "BUSY", "position_m": [420, -150], "priority": 4, "survey_time_s": 400},
                         {"id": "LATER", "position_m": [640, 380], "priority": 2, "survey_time_s": 30}])
    run_until(sim, 300)
    later = sim.world.state.pois.get("LATER")
    assert later.status is PoIStatus.PENDING
    run_until(sim, 600)
    assert later.status is not PoIStatus.PENDING
    assert sim.manager.allocator.ferry_assignments == 0
    assert not sim.world.events.history(types=[EventType.DATA_FERRY_ASSIGNED])


def test_farthest_poi_is_deferred_and_flown_with_a_multi_hop_relay_chain():
    """FAR is the highest-priority PoI here but also the farthest, so it waits for the two
    near PoIs to finish. Once tasked, it gets a relay chain of several UAVs - a multi-hop
    escort - instead of the lone, disconnected data-ferry run it would otherwise need."""
    sim = make_sim("adaptive", uavs={"count": 8, "per_row": 4}, scenario={"duration_s": 1800.0},
                   pois=[{"id": "NEAR-1", "position_m": [150, 0], "priority": 3, "survey_time_s": 30},
                         {"id": "NEAR-2", "position_m": [0, 150], "priority": 3, "survey_time_s": 30},
                         {"id": "FAR", "position_m": [640, 380], "priority": 5, "survey_time_s": 30}])
    sim.start()
    sim.tick()
    assert sim.manager.allocator.farthest_poi_id == "FAR"
    far = sim.world.state.pois.get("FAR")
    assert far.status is PoIStatus.PENDING            # held back despite the highest priority

    run_until(sim, 100)
    assert sim.world.state.pois.get("NEAR-1").is_completed
    assert sim.world.state.pois.get("NEAR-2").is_completed
    assert far.status is not PoIStatus.PENDING
    assert far.assigned_uav not in sim.manager.allocator.ferrying
    assert len(sim.world.state.uavs_with_role(UAVRole.RELAY)) >= 3

    run_until(sim, 400)
    assert far.is_completed
    assert sim.manager.allocator.ferry_assignments == 0
    assert not sim.world.events.history(types=[EventType.DATA_FERRY_ASSIGNED])


def test_the_farthest_poi_stops_waiting_once_the_deadline_draws_near():
    """FAR waits for SLOW - but not past the point where there would be too little time left
    to fly it (``defer_deadline_margin_s``): a nearer survey that drags on must not cost the
    farthest PoI its own."""
    sim = make_sim("adaptive", uavs={"count": 8, "per_row": 4}, scenario={"duration_s": 1000.0},
                   pois=[{"id": "SLOW", "position_m": [150, 0], "priority": 3, "survey_time_s": 600},
                         {"id": "FAR", "position_m": [640, 380], "priority": 3, "survey_time_s": 30}])
    slow, far = (sim.world.state.pois.get(p) for p in ("SLOW", "FAR"))
    run_until(sim, 300)
    assert slow.status is PoIStatus.IN_PROGRESS and far.status is PoIStatus.PENDING   # waiting its turn
    run_until(sim, 560)
    assert not slow.is_completed and far.status is not PoIStatus.PENDING             # the deadline decides
    run_until(sim, 1000)
    assert slow.is_completed and far.is_completed


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
