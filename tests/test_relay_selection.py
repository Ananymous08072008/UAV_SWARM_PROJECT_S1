"""Stage 4 tests: relay placement and relay UAV selection."""

import numpy as np

from core.uav import UAVRole
from simulation.obstacles import square
from swarm.relay_selector import Terminal
from tests.helpers import make_sim, run_until


def terminal(x: float, y: float, priority: int = 3, key: str = "T") -> Terminal:
    return Terminal(key, np.array([x, y, 50.0]), priority)


def test_a_far_terminal_needs_a_chain_of_relays():
    sim = make_sim()
    relays = sim.manager.relays
    assert relays.count_relays([terminal(200, 0)]) == 0        # inside the GCS radio horizon
    one = relays.count_relays([terminal(450, 0)])
    two = relays.count_relays([terminal(700, 0)])
    assert 1 <= one < two


def test_every_planned_hop_meets_the_planned_quality():
    sim = make_sim()
    relays, comm = sim.manager.relays, sim.env.comm
    gcs = comm.gcs_antenna_position(sim.world)
    plan = relays.make_plan([terminal(800, 200, key="FAR")])
    chain = [gcs, *plan.chains[0].points, np.array([800, 200, 50.0])]
    for i, (a, b) in enumerate(zip(chain, chain[1:])):
        assert comm.predict_pdr(a, b, involves_gcs=(i == 0)) >= relays.params.min_planned_pdr


def test_chains_share_a_backbone_instead_of_duplicating_it():
    sim = make_sim()
    relays = sim.manager.relays
    far_alone = relays.count_relays([terminal(800, 200, key="A")])
    both = relays.count_relays([terminal(800, 200, key="A"), terminal(760, 260, key="B")])
    assert both <= far_alone + 1       # the second terminal hangs off the same chain


def test_the_plan_detours_around_an_obstacle():
    sim = make_sim()
    relays = sim.manager.relays
    straight = relays.make_plan([terminal(700, 0, key="T")]).chains[0].points
    sim.env.obstacles.add(square("WALL", (350, 0), 300, height_m=90.0, attenuation_db=45.0))
    detour = relays.make_plan([terminal(700, 0, key="T")]).chains[0].points
    assert detour, "the terminal must still be reachable"
    assert max(abs(float(p[1])) for p in detour) > max(abs(float(p[1])) for p in straight) + 50
    assert all(sim.env.obstacles.inside(p) is None for p in detour)


def test_relays_are_assigned_and_reused():
    sim = make_sim()
    run_until(sim, 60)
    relays = sim.manager.relays
    assert relays.assignment, "survey UAVs beyond the horizon need relays"
    for uav_id in relays.assignment:
        assert sim.world.state.get_uav(uav_id).role is UAVRole.RELAY
    before = dict(relays.assignment)
    run_until(sim, 90)
    kept = set(before) & set(relays.assignment)
    assert kept, "stickiness should keep relays in place while the plan is unchanged"


def test_an_excluded_uav_is_replaced_as_relay():
    sim = make_sim()
    run_until(sim, 70)
    relays = sim.manager.relays
    victim = sorted(relays.assignment)[0]
    relays.exclude(victim, "test")
    run_until(sim, 110)
    assert victim not in relays.assignment
    assert sim.world.state.get_uav(victim).role is not UAVRole.RELAY
    assert relays.assignment, "another UAV takes the relay point over"


def test_unreachable_terminal_is_reported():
    sim = make_sim()
    relays = sim.manager.relays
    # far outside the operating area: no relay point can be placed legally
    plan = relays.make_plan([terminal(5000, 5000, key="MOON")])
    assert plan.unreachable == ["MOON"]
    assert relays.count_relays([terminal(5000, 5000, key="MOON")]) is None
