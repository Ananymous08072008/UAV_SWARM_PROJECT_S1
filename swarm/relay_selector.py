"""
swarm/relay_selector.py
Keeps every surveying UAV connected to the GCS by planning relay positions and
choosing which UAVs fly them.

1. Plan (communication-aware geometry)
   Terminals = survey waypoints of SURVEY UAVs, highest priority first.
   The connected set starts with the GCS. Each terminal is attached to the
   anchor (GCS, planned relay or already-attached terminal) that needs the
   fewest relays. Relays are spaced evenly along the segment so every hop has
   predicted PDR >= ``min_planned_pdr`` (obstacles included). If a straight
   chain is blocked, dog-leg detours around the obstruction are tried.

2. Assign (who flies each relay point)
   Chains are filled in priority order and only if the whole chain can be
   filled. A UAV already relaying near a point keeps it (stickiness), others
   are chosen by travel time, battery and exclusion lists (UAVs that are
   leaving for recharge or whose radio was diagnosed as degraded).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Optional, Sequence

import numpy as np

from core.uav import UAV, UAVRole

if TYPE_CHECKING:
    from core.world import World
    from simulation.environment import Environment
    from swarm.network_manager import NetworkView
    from swarm.role_manager import RoleManager

GCS_KEY = "GCS"


@dataclass(frozen=True)
class RelayParams:
    relay_altitude_m: float = 50.0
    min_planned_pdr: float = 0.85         # every planned hop must predict at least this PDR
    spacing_safety: float = 0.9           # hop length = range_for_pdr(min_planned_pdr) * spacing_safety
    reposition_threshold_m: float = 25.0
    min_battery_pct: float = 35.0
    replan_interval_s: float = 10.0
    stickiness_s: float = 30.0

    def __post_init__(self) -> None:
        if not 0.5 <= self.min_planned_pdr < 1 or not 0 < self.spacing_safety <= 1:
            raise ValueError("invalid relay planning parameters")


@dataclass(frozen=True)
class Terminal:
    key: str                  # PoI id (or any label)
    point: np.ndarray         # x, y, z
    priority: int


@dataclass
class RelayChain:
    terminal: str
    priority: int
    anchor: str                     # anchor key: "GCS", a terminal key, or "<terminal>/r<i>"
    depends_on: Optional[str]       # terminal whose chain this one hangs off (None = straight from the GCS)
    points: list[np.ndarray]


@dataclass
class RelayPlan:
    chains: list[RelayChain] = field(default_factory=list)
    unreachable: list[str] = field(default_factory=list)

    @property
    def relay_count(self) -> int:
        return sum(len(c.points) for c in self.chains)

    def to_dict(self) -> dict[str, Any]:
        return {"chains": [{"terminal": c.terminal, "priority": c.priority, "anchor": c.anchor,
                            "points": [[round(float(v), 1) for v in p] for p in c.points]} for c in self.chains],
                "unreachable": list(self.unreachable), "relay_count": self.relay_count}


class RelaySelector:
    def __init__(self, world: "World", env: "Environment", roles: "RoleManager", params: RelayParams,
                 static: bool = False) -> None:
        self.world = world
        self.env = env
        self.roles = roles
        self.params = params
        self.static = static                 # baseline: plan once, never re-plan
        self.plan = RelayPlan()
        self.assignment: dict[int, tuple[np.ndarray, str]] = {}   # uav_id -> (point, terminal)
        self.excluded: dict[int, str] = {}   # uav_id -> reason (not eligible as relay)
        self.leaving: set[int] = set()       # relays waiting for a replacement before RTH
        self._signature: Optional[tuple] = None
        self._last_plan_t = -math.inf
        self.replans = 0
        comm = env.comm
        self.hop_m = comm.range_for_pdr(params.min_planned_pdr) * params.spacing_safety
        self.gcs_hop_m = comm.range_for_pdr(params.min_planned_pdr, involves_gcs=True) * params.spacing_safety

    # ---------------------------------------------------------------- planning
    def terminals(self, world: "World") -> list[Terminal]:
        out = []
        for uav in world.state.uavs_with_role(UAVRole.SURVEY):
            if uav.assigned_poi is None or uav.target is None:
                continue
            poi = world.state.pois.get(uav.assigned_poi)
            out.append(Terminal(poi.poi_id, uav.target.copy(), poi.priority))
        return out

    def make_plan(self, terminals: Sequence[Terminal]) -> RelayPlan:
        gcs = self.env.comm.gcs_antenna_position(self.world)
        anchors: list[tuple[str, np.ndarray, bool]] = [(GCS_KEY, gcs, True)]  # (key, point, is_gcs)
        plan = RelayPlan()
        remaining = list(terminals)
        while remaining:
            # Grow the tree by the terminal that needs the fewest relays (Prim-style):
            # attaching near terminals first keeps shared backbones short.
            best: Optional[tuple] = None
            for term in remaining:
                for key, point, is_gcs in anchors:
                    path = self._hop_path(point, term.point, is_gcs)
                    if path is None:
                        continue
                    length = float(np.linalg.norm(term.point[:2] - point[:2]))
                    candidate = (len(path), -term.priority, length, term.key, key, path, term)
                    if best is None or candidate[:4] < best[:4]:
                        best = candidate
            if best is None:
                plan.unreachable.extend(sorted(t.key for t in remaining))
                break
            _, _, _, _, anchor_key, path, term = best
            depends_on = None if anchor_key == GCS_KEY else anchor_key.split("/")[0]
            plan.chains.append(RelayChain(term.key, term.priority, anchor_key, depends_on, path))
            for i, p in enumerate(path):
                anchors.append((f"{term.key}/r{i + 1}", p, False))
            anchors.append((term.key, term.point, False))
            remaining.remove(term)
        return plan

    def count_relays(self, terminals: Sequence[Terminal]) -> Optional[int]:
        plan = self.make_plan(terminals)
        return None if plan.unreachable else plan.relay_count

    def _hop_ok(self, a: np.ndarray, b: np.ndarray, a_is_gcs: bool) -> bool:
        return self.env.comm.predict_pdr(a, b, involves_gcs=a_is_gcs) >= self.params.min_planned_pdr

    def _point_ok(self, p: np.ndarray) -> bool:
        return self.world.state.area.contains(p[0], p[1]) and self.env.obstacles.inside(p) is None

    def _straight(self, a: np.ndarray, b: np.ndarray, a_is_gcs: bool) -> list[np.ndarray]:
        """Relay points spaced evenly on a->b so that no hop exceeds the planning spacing."""
        d = float(np.hypot(*(b[:2] - a[:2])))
        first = self.gcs_hop_m if a_is_gcs else self.hop_m
        if d <= first:
            return []
        n = math.ceil((d - first) / self.hop_m - 1e-9)
        scale = d / (first + n * self.hop_m)
        direction = (b[:2] - a[:2]) / d
        alt = self.params.relay_altitude_m
        return [np.array([*(a[:2] + direction * scale * (first + i * self.hop_m)), alt]) for i in range(n)]

    def _validate(self, start: np.ndarray, points: list[np.ndarray], end: np.ndarray, start_is_gcs: bool) -> bool:
        chain = [start, *points, end]
        if not all(self._point_ok(p) for p in points):
            return False
        return all(self._hop_ok(chain[i], chain[i + 1], start_is_gcs and i == 0) for i in range(len(chain) - 1))

    def _hop_path(self, a: np.ndarray, b: np.ndarray, a_is_gcs: bool) -> Optional[list[np.ndarray]]:
        points = self._straight(a, b, a_is_gcs)
        if self._validate(a, points, b, a_is_gcs):
            return points
        best: Optional[list[np.ndarray]] = None
        d = float(np.hypot(*(b[:2] - a[:2])))
        if d < 1.0:
            return None
        base = math.atan2(b[1] - a[1], b[0] - a[0])
        for angle_deg in (25, 45, 65):
            for sign in (1, -1):
                ang = math.radians(angle_deg)
                leg = (d / 2) / math.cos(ang)
                w = np.array([a[0] + leg * math.cos(base + sign * ang), a[1] + leg * math.sin(base + sign * ang),
                              self.params.relay_altitude_m])
                path = self._straight(a, w, a_is_gcs) + [w] + self._straight(w, b, False)
                if self._validate(a, path, b, a_is_gcs) and (best is None or len(path) < len(best)):
                    best = path
        return best

    # -------------------------------------------------------------- assignment
    def exclude(self, uav_id: int, reason: str) -> None:
        self.excluded[uav_id] = reason
        self._signature = None  # force a re-plan

    def include(self, uav_id: int) -> None:
        if self.excluded.pop(uav_id, None) is not None:
            self._signature = None

    def request_replan(self) -> None:
        self._signature = None

    def update(self, world: "World", view: "NetworkView", force: bool = False) -> bool:
        """Re-plan and re-assign relays when something relevant changed. Returns True if it ran.

        "Repair only what is broken": a new plan is made when the survey targets or the
        eligible UAVs change, when a fault forces it, or periodically - but only while a
        surveyor on station is actually cut off. Routing already works around many
        changes (e.g. a new obstacle) on its own; moving relays that still carry traffic
        would break links while they fly.
        """
        terminals = self.terminals(world)
        signature = (tuple(sorted((t.key, round(float(t.point[0])), round(float(t.point[1]))) for t in terminals)),
                     tuple(sorted(self.excluded)), tuple(sorted(self.leaving)))
        if self.static:
            if self._last_plan_t > -math.inf or not terminals:
                return False
        elif not force and signature == self._signature:
            if world.t - self._last_plan_t < self.params.replan_interval_s or self._surveyors_connected(world):
                return False
        self._signature = signature
        self._last_plan_t = world.t
        self.plan = self.make_plan(terminals)
        self._assign(world)
        self.replans += 1
        return True

    @staticmethod
    def _surveyors_connected(world: "World") -> bool:
        """True when every surveyor that has reached its PoI has a route to the GCS."""
        radius = world.params.uav.arrival_radius_m * 2
        for uav in world.state.uavs_with_role(UAVRole.SURVEY):
            on_station = uav.target is not None and uav.distance_to(uav.target) <= radius
            if on_station and not uav.comm.connected:
                return False
        return True

    def _candidates(self, world: "World") -> list[UAV]:
        out = []
        for uav in world.state.operational_uavs():
            if uav.uav_id in self.excluded or uav.uav_id in self.leaving:
                continue
            if uav.role is UAVRole.RELAY or (uav.role in (UAVRole.IDLE, UAVRole.BACKUP)
                                             and uav.battery_pct >= self.params.min_battery_pct):
                out.append(uav)
        return out

    def _cost(self, uav: UAV, point: np.ndarray) -> float:
        cost = self.env.battery.travel_time_s(uav.position, point) + (100.0 - uav.battery_pct) * 0.5
        if uav.role is UAVRole.RELAY and uav.target is not None and \
                np.hypot(*(uav.target[:2] - point[:2])) <= self.params.reposition_threshold_m:
            cost -= self.params.stickiness_s
        return cost

    def _assign(self, world: "World") -> None:
        free = {u.uav_id: u for u in self._candidates(world)}
        assignment: dict[int, tuple[np.ndarray, str]] = {}
        filled: set[str] = set()
        for chain in self.plan.chains:  # plan order = dependency order
            if chain.depends_on is not None and chain.depends_on not in filled:
                continue  # its anchor chain could not be filled, so this one would connect nothing
            if len(chain.points) > len(free):
                continue  # a partial chain does not connect anything
            filled.add(chain.terminal)
            for point in chain.points:
                uav = min(free.values(), key=lambda u: (self._cost(u, point), u.uav_id))
                assignment[uav.uav_id] = (point, chain.terminal)
                del free[uav.uav_id]

        for uav_id, (point, terminal) in assignment.items():
            uav = world.state.uavs[uav_id]
            moved = uav.target is None or np.hypot(*(uav.target[:2] - point[:2])) > self.params.reposition_threshold_m
            if uav.role is not UAVRole.RELAY or moved:
                self.roles.assign_relay(uav, point, "relay plan", serves=(terminal,))

        for uav in world.state.uavs_with_role(UAVRole.RELAY):
            if uav.uav_id not in assignment and uav.uav_id not in self.leaving:
                self.roles.release(uav, "relay no longer needed")
        self.assignment = assignment

    def replacement_ready(self, world: "World", leaving_uav: UAV) -> bool:
        """True when the relay spot a leaving UAV was covering is taken over (or no longer needed)."""
        if leaving_uav.target is None:
            return True
        radius = max(world.params.uav.arrival_radius_m * 3, 10.0)
        for uav_id, (point, _) in self.assignment.items():
            if uav_id == leaving_uav.uav_id:
                continue
            if np.hypot(*(point[:2] - leaving_uav.target[:2])) <= self.params.reposition_threshold_m * 2:
                other = world.state.uavs[uav_id]
                return other.target is not None and other.distance_to(other.target) <= radius
        return True

    def to_dict(self) -> dict[str, Any]:
        return {**self.plan.to_dict(), "assignment": {str(k): [round(float(v), 1) for v in p]
                                                     for k, (p, _) in self.assignment.items()},
                "excluded": {str(k): v for k, v in self.excluded.items()}, "leaving": sorted(self.leaving),
                "hop_spacing_m": round(self.hop_m, 1), "gcs_hop_spacing_m": round(self.gcs_hop_m, 1)}
