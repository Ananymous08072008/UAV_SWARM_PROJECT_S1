"""
swarm/route_manager.py
Multi-hop routes from every UAV to the GCS.

Metric: ETX (expected transmission count) = sum over hops of 1 / PDR, with a
penalty on weak links (PDR < ``min_link_pdr``). Plain ETX favours fewer hops
even over a marginal link; the penalty makes the router take a slightly longer
path of good links instead, and use a weak link only when nothing else reaches
the GCS. Dijkstra from the GCS gives a shortest-path tree; each UAV's next hop
is its parent in that tree. Route hysteresis keeps the current next hop while it is
within ``hysteresis`` of the best route, which stops flapping between
near-equal routes. A kept next hop must still be strictly closer to the GCS,
so routing loops cannot form.

Writes the result into every UAV's CommState (connected, next_hop, route,
PDR, latency) and publishes ROUTE_CHANGED / UAV_DISCONNECTED / UAV_RECONNECTED.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Optional

from core.events import EventType, Severity
from core.uav import GCS_NODE_ID, UAVRole
from swarm.network_manager import NetworkView

if TYPE_CHECKING:
    from core.world import World

_TRACKED_ROLES = (UAVRole.SURVEY, UAVRole.RELAY, UAVRole.BACKUP, UAVRole.IDLE)


@dataclass(frozen=True)
class RouteParams:
    hysteresis: float = 0.15          # keep the current next hop while within 15 % of the best
    min_link_pdr: float = 0.7         # links below this are a last resort
    weak_link_penalty: float = 10.0   # extra cost (in transmissions) of using a weak link

    def __post_init__(self) -> None:
        if self.hysteresis < 0 or self.weak_link_penalty < 0 or not 0 <= self.min_link_pdr < 1:
            raise ValueError("invalid routing parameters")


def link_cost(pdr: float, params: RouteParams) -> float:
    """ETX of one hop, plus a penalty when the link is weak."""
    cost = 1.0 / max(pdr, 1e-3)
    return cost + params.weak_link_penalty if pdr < params.min_link_pdr else cost


@dataclass(frozen=True)
class Route:
    uav_id: int
    path: tuple[int, ...]       # (uav, ..., 0)
    pdr: float                  # end-to-end delivery ratio
    bottleneck_pdr: float
    latency_ms: float
    etx: float

    @property
    def hops(self) -> int:
        return len(self.path) - 1

    def to_dict(self) -> dict[str, Any]:
        return {"uav_id": self.uav_id, "path": list(self.path), "pdr": round(self.pdr, 3),
                "bottleneck_pdr": round(self.bottleneck_pdr, 3), "latency_ms": round(self.latency_ms, 1),
                "hops": self.hops}


def shortest_etx(view: NetworkView, params: RouteParams = RouteParams()) -> tuple[dict[int, float], dict[int, int]]:
    """Dijkstra from the GCS over penalised ETX edge costs. Returns (cost, parent)."""
    dist = {GCS_NODE_ID: 0.0}
    parent: dict[int, int] = {}
    heap = [(0.0, GCS_NODE_ID)]
    while heap:
        d, n = heapq.heappop(heap)
        if d > dist.get(n, math.inf):
            continue
        for m, lk in sorted(view.adjacency.get(n, {}).items()):
            nd = d + link_cost(lk.pdr, params)
            if nd < dist.get(m, math.inf) - 1e-12:
                dist[m] = nd
                parent[m] = n
                heapq.heappush(heap, (nd, m))
    return dist, parent


class RouteManager:
    def __init__(self, params: RouteParams) -> None:
        self.params = params
        self.routes: dict[int, Route] = {}
        self._next_hop: dict[int, int] = {}
        self._announced_down: set[int] = set()
        self.route_changes = 0

    def update(self, world: "World", view: NetworkView) -> dict[int, Route]:
        dist, parent = shortest_etx(view, self.params)
        next_hop: dict[int, int] = {}
        for node in sorted(dist):
            if node == GCS_NODE_ID:
                continue
            best = parent[node]
            prev = self._next_hop.get(node)
            if prev is not None and prev != best and prev in dist and dist[prev] < dist[node]:
                lk = view.link(node, prev)
                if lk is not None and link_cost(lk.pdr, self.params) + dist[prev] <= dist[node] * (1 + self.params.hysteresis):
                    best = prev
            next_hop[node] = best

        routes: dict[int, Route] = {}
        for node in next_hop:
            path, pdr, bottleneck, latency, etx = [node], 1.0, 1.0, 0.0, 0.0
            cur = node
            while cur != GCS_NODE_ID and len(path) <= len(next_hop) + 1:
                nxt = next_hop[cur]
                lk = view.link(cur, nxt)
                pdr *= lk.pdr
                bottleneck = min(bottleneck, lk.pdr)
                latency += lk.latency_ms
                etx += 1.0 / max(lk.pdr, 1e-3)
                path.append(nxt)
                cur = nxt
            routes[node] = Route(node, tuple(path), pdr, bottleneck, latency, etx)

        self._publish_changes(world, next_hop)
        self._write_comm_state(world, view, routes)
        self.routes = routes
        self._next_hop = next_hop
        return routes

    def _publish_changes(self, world: "World", next_hop: dict[int, int]) -> None:
        for uav in world.state.uavs.values():
            uid = uav.uav_id
            tracked = uav.is_operational and uav.is_airborne and uav.role in _TRACKED_ROLES
            old, new = self._next_hop.get(uid), next_hop.get(uid)
            if new is None:
                if old is not None and tracked and uid not in self._announced_down:
                    self._announced_down.add(uid)
                    world.publish(EventType.UAV_DISCONNECTED, f"{uav.name} lost its route to the GCS ({uav.role.value})",
                                  severity=Severity.WARNING, uav_id=uid, data={"role": uav.role.value})
                continue
            if uid in self._announced_down:
                self._announced_down.discard(uid)
                world.publish(EventType.UAV_RECONNECTED, f"{uav.name} reconnected via node {new}",
                              uav_id=uid, data={"next_hop": new})
            elif old is not None and old != new:
                self.route_changes += 1
                world.publish(EventType.ROUTE_CHANGED, f"{uav.name} next hop {old} -> {new}",
                              uav_id=uid, data={"old": old, "new": new})
        for uid in list(self._announced_down):
            uav = world.state.uavs[uid]
            if not (uav.is_operational and uav.is_airborne):
                self._announced_down.discard(uid)

    @staticmethod
    def _write_comm_state(world: "World", view: NetworkView, routes: dict[int, Route]) -> None:
        for uav in world.state.uavs.values():
            route = routes.get(uav.uav_id)
            if route is None:
                world.update_comm(uav.uav_id, neighbours=view.neighbours(uav.uav_id), connected=False,
                                  next_hop=None, route=(), hop_count=None, gcs_link_quality=0.0, pdr=0.0,
                                  latency_ms=None)
            else:
                world.update_comm(uav.uav_id, neighbours=view.neighbours(uav.uav_id), connected=True,
                                  next_hop=route.path[1], route=route.path, hop_count=route.hops,
                                  gcs_link_quality=route.bottleneck_pdr, pdr=route.pdr,
                                  latency_ms=route.latency_ms)

    # ---------------------------------------------------------------- queries
    def dependents(self, uav_id: int) -> set[int]:
        """UAVs whose current route to the GCS passes through ``uav_id``."""
        return {r.uav_id for r in self.routes.values() if uav_id in r.path[1:-1]}

    def backbone_midpoint(self, positions: dict[int, Any]) -> Optional[Any]:
        """Midpoint of the longest UAV-to-UAV hop in use - where debris in the disaster area hurts most.
        Hops touching the GCS are only used when no UAV-to-UAV hop exists."""
        best = None
        for route in sorted(self.routes.values(), key=lambda r: r.uav_id):
            dependents = len(self.dependents(route.uav_id))
            for a, b in zip(route.path, route.path[1:]):
                if a not in positions or b not in positions:
                    continue
                length = float(((positions[a][0] - positions[b][0]) ** 2
                                + (positions[a][1] - positions[b][1]) ** 2) ** 0.5)
                key = (GCS_NODE_ID not in (a, b), length, dependents)
                if best is None or key > best[0]:
                    best = (key, (positions[a] + positions[b]) / 2.0)
        return None if best is None else best[1]

    def critical_relay(self, world: "World") -> Optional[int]:
        """The RELAY carrying the most traffic (most dependents); ties -> lowest id."""
        relays = [u.uav_id for u in world.state.uavs_with_role(UAVRole.RELAY) if u.uav_id in self.routes]
        if not relays:
            return None
        return max(sorted(relays), key=lambda uid: len(self.dependents(uid)))
