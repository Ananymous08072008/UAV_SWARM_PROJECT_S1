"""
swarm/network_manager.py
The swarm's view of its communication graph, rebuilt from the latest link
measurements: nodes = GCS + airborne operational UAVs, edges = links that are up.

Graph queries used by the rest of the swarm layer:
  * which nodes can reach the GCS (connectivity)
  * connected components (network partitions)
  * articulation points = single points of failure (critical relays)
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Optional

from core.uav import GCS_NODE_ID
from simulation.communication import CommunicationModel, Link, link_key

if TYPE_CHECKING:
    from core.world import World


@dataclass
class NetworkView:
    t_s: float = 0.0
    nodes: set[int] = field(default_factory=lambda: {GCS_NODE_ID})
    adjacency: dict[int, dict[int, Link]] = field(default_factory=dict)

    def neighbours(self, node: int) -> set[int]:
        return set(self.adjacency.get(node, {}))

    def link(self, a: int, b: int) -> Optional[Link]:
        return self.adjacency.get(a, {}).get(b)

    def edges(self) -> list[Link]:
        seen, out = set(), []
        for a, nbrs in self.adjacency.items():
            for b, lk in nbrs.items():
                key = link_key(a, b)
                if key not in seen:
                    seen.add(key)
                    out.append(lk)
        return out

    def reachable_from(self, start: int = GCS_NODE_ID) -> set[int]:
        if start not in self.nodes:
            return set()
        seen = {start}
        queue = deque([start])
        while queue:
            n = queue.popleft()
            for m in self.adjacency.get(n, {}):
                if m not in seen:
                    seen.add(m)
                    queue.append(m)
        return seen

    def components(self) -> list[set[int]]:
        remaining, parts = set(self.nodes), []
        while remaining:
            comp = self.reachable_from(min(remaining))
            parts.append(comp)
            remaining -= comp
        return parts

    def articulation_points(self) -> set[int]:
        """Nodes whose loss would split the network (Tarjan, iterative)."""
        index: dict[int, int] = {}
        low: dict[int, int] = {}
        points: set[int] = set()
        counter = 0
        for root in sorted(self.nodes):
            if root in index:
                continue
            index[root] = low[root] = counter
            counter += 1
            children = 0
            stack = [(root, None, iter(sorted(self.adjacency.get(root, {}))))]
            while stack:
                node, parent, it = stack[-1]
                advanced = False
                for nxt in it:
                    if nxt == parent:
                        continue
                    if nxt in index:
                        low[node] = min(low[node], index[nxt])
                    else:
                        index[nxt] = low[nxt] = counter
                        counter += 1
                        if node == root:
                            children += 1
                        stack.append((nxt, node, iter(sorted(self.adjacency.get(nxt, {})))))
                        advanced = True
                        break
                if not advanced:
                    stack.pop()
                    if parent is not None:
                        low[parent] = min(low[parent], low[node])
                        if parent != root and low[node] >= index[parent]:
                            points.add(parent)
            if children > 1:
                points.add(root)
        points.discard(GCS_NODE_ID)
        return points

    def to_dict(self) -> dict[str, Any]:
        return {"nodes": sorted(self.nodes), "links": [lk.to_dict() for lk in self.edges()],
                "components": len(self.components()), "critical_nodes": sorted(self.articulation_points())}


class NetworkManager:
    def __init__(self, comm: CommunicationModel) -> None:
        self.comm = comm
        self.view = NetworkView()

    def rebuild(self, world: "World") -> NetworkView:
        view = NetworkView(t_s=world.t, nodes=set(self.comm.positions) | {GCS_NODE_ID})
        for lk in self.comm.links.values():
            if lk.up and lk.a in view.nodes and lk.b in view.nodes:
                view.adjacency.setdefault(lk.a, {})[lk.b] = lk
                view.adjacency.setdefault(lk.b, {})[lk.a] = lk
        self.view = view
        return view
