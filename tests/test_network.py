"""Stage 2 tests: network graph and multi-hop routing to the GCS."""

from core.events import EventType
from core.uav import GCS_NODE_ID
from simulation.communication import Link
from swarm.network_manager import NetworkView
from swarm.route_manager import RouteManager, RouteParams, shortest_etx
from tests.helpers import make_sim, run_until


def view_from(edges: dict[tuple[int, int], float]) -> NetworkView:
    view = NetworkView(nodes={GCS_NODE_ID})
    for (a, b), pdr in edges.items():
        link = Link(min(a, b), max(a, b), 100.0, 12.0, pdr, 5.0, False, True)
        view.nodes.update({a, b})
        view.adjacency.setdefault(a, {})[b] = link
        view.adjacency.setdefault(b, {})[a] = link
    return view


def test_reachability_components_and_critical_nodes():
    view = view_from({(0, 1): 0.9, (1, 2): 0.9, (3, 4): 0.9})
    assert view.reachable_from(GCS_NODE_ID) == {0, 1, 2}
    assert sorted(len(c) for c in view.components()) == [2, 3]
    assert view.articulation_points() == {1}        # UAV-1 carries UAV-2's only route


def test_dijkstra_prefers_the_better_quality_path():
    view = view_from({(0, 1): 0.95, (1, 3): 0.95, (0, 2): 0.35, (2, 3): 0.35})
    dist, parent = shortest_etx(view)
    assert parent[3] == 1
    assert dist[3] < 1.0 / 0.35 * 2


def test_a_weak_shortcut_loses_to_a_strong_detour_but_is_kept_as_last_resort():
    # 3 can reach the GCS via one weak hop to 1, or via strong hops 3-2-1
    view = view_from({(0, 1): 0.95, (1, 3): 0.58, (1, 2): 0.93, (2, 3): 1.0})
    _, parent = shortest_etx(view)
    assert parent[3] == 2
    # without the detour the weak link is still used - connectivity beats quality
    _, parent = shortest_etx(view_from({(0, 1): 0.95, (1, 3): 0.58}))
    assert parent[3] == 1


def test_routes_are_written_to_uavs_and_have_no_loops():
    sim = make_sim()
    run_until(sim, 60)
    for uav in sim.world.state.uavs.values():
        route = uav.comm.route
        if uav.comm.connected:
            assert route[-1] == GCS_NODE_ID
            assert len(set(route)) == len(route)              # no loops
            assert uav.comm.hop_count == len(route) - 1
            assert 0 < uav.comm.pdr <= 1
        else:
            assert route == () and uav.comm.next_hop is None


def test_disconnection_and_reconnection_are_announced_once():
    sim = make_sim()
    run_until(sim, 120)
    down = [e for e in sim.world.events.history(types=[EventType.UAV_DISCONNECTED])]
    up = [e for e in sim.world.events.history(types=[EventType.UAV_RECONNECTED])]
    assert len(down) >= len(up)     # every reconnection follows a disconnection
    relay_ids = {u.uav_id for u in sim.world.state.uavs.values()}
    assert all(e.uav_id in relay_ids for e in down + up)


def test_route_hysteresis_keeps_a_good_enough_next_hop():
    manager = RouteManager(RouteParams(hysteresis=0.5))
    sim = make_sim()
    world = sim.world
    first = view_from({(0, 1): 0.9, (1, 2): 0.9})
    manager.update(world, first)
    assert manager.routes[2].path == (2, 1, 0)
    # a slightly better direct route appears, but the current hop stays within hysteresis
    second = view_from({(0, 1): 0.9, (1, 2): 0.9, (0, 2): 0.55})
    manager.update(world, second)
    assert manager.routes[2].path == (2, 1, 0)


def test_dependents_and_critical_relay():
    manager = RouteManager(RouteParams())
    sim = make_sim()
    manager.update(sim.world, view_from({(0, 1): 0.9, (1, 2): 0.9, (2, 3): 0.9}))
    assert manager.dependents(1) == {2, 3}
    assert manager.dependents(2) == {3}
    assert manager.dependents(3) == set()
