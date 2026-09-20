"""
swarm/task_allocator.py
Communication- and energy-aware assignment of survey tasks (sequential auction).

For each pending PoI, in effective-priority order:
  1. Feasible UAVs: IDLE or BACKUP, enough battery to fly there, survey the
     remaining time and still return home with a reserve, and able to finish
     before the mission deadline.
  2. Communication budget (adaptive mode): the swarm must still have enough
     UAVs for the surveyor *plus* the relays that keep every surveyor
     connected (computed by the relay planner). If not, the PoI waits - unless
     it has waited longer than ``ferry_wait_s``, in which case it is surveyed
     disconnected and its data is ferried back (store-and-forward).
     A lower-priority PoI may not jump the queue if it needs extra relays.
  3. Winner = lowest bid: travel time + battery cost.

Baseline mode skips step 2 (nearest feasible UAV, communication-unaware).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

import numpy as np

from core.poi import PoI
from core.uav import TASKABLE_ROLES, UAV, UAVRole
from swarm.relay_selector import Terminal

if TYPE_CHECKING:
    from core.world import World
    from simulation.environment import Environment
    from swarm.priority_manager import PriorityManager
    from swarm.relay_selector import RelaySelector
    from swarm.role_manager import RoleManager
    from swarm.safety_manager import SafetyManager


@dataclass(frozen=True)
class AllocationParams:
    reserve_pct: float = 10.0
    battery_weight_s: float = 2.0     # seconds of travel one % of battery is worth
    ferry_wait_s: float = 150.0


class TaskAllocator:
    def __init__(self, world: "World", env: "Environment", roles: "RoleManager", safety: "SafetyManager",
                 relays: "RelaySelector", priority: "PriorityManager", params: AllocationParams,
                 comm_aware: bool = True) -> None:
        self.world = world
        self.env = env
        self.roles = roles
        self.safety = safety
        self.relays = relays
        self.priority = priority
        self.params = params
        self.comm_aware = comm_aware
        self.assignments = 0
        self.runtime_ms: list[float] = []
        self._blocked_since: dict[str, float] = {}   # PoI -> when the relay budget first blocked it
        self._ferry_pois: set[str] = set()           # PoIs deliberately surveyed without a relay chain
        self.ferrying: set[int] = set()              # UAVs on those PoIs (expected to be disconnected)

    # ------------------------------------------------------------- feasibility
    def _waypoint(self, poi: PoI):
        alt = poi.altitude_m if poi.altitude_m is not None else self.world.params.uav.default_altitude_m
        return (float(poi.position[0]), float(poi.position[1]), alt)

    def can_do(self, uav: UAV, poi: PoI) -> bool:
        waypoint = self._waypoint(poi)
        remaining = max(0.0, poi.survey_time_s - poi.progress_s)
        needed = self.env.battery.task_cost_pct(uav, waypoint, remaining) + self.params.reserve_pct
        return uav.battery_pct >= needed and self.safety.fits_deadline(uav, waypoint, remaining)

    def _bid(self, uav: UAV, poi: PoI) -> float:
        waypoint = self._waypoint(poi)
        remaining = max(0.0, poi.survey_time_s - poi.progress_s)
        return (self.env.battery.travel_time_s(uav.position, waypoint)
                + self.params.battery_weight_s * self.env.battery.task_cost_pct(uav, waypoint, remaining))

    # -------------------------------------------------------------- allocation
    def allocate(self, world: "World") -> int:
        started = time.perf_counter()
        pending = self.priority.ordered_pending(world)
        available = [u for u in world.state.operational_uavs() if u.role in TASKABLE_ROLES]
        made = 0
        if pending and available:
            made = self._allocate(world, pending, available)
        self.ferrying.clear()
        self.ferrying.update(u.uav_id for u in world.state.uavs_with_role(UAVRole.SURVEY)
                             if u.assigned_poi in self._ferry_pois)
        self.runtime_ms.append((time.perf_counter() - started) * 1000.0)
        return made

    def _allocate(self, world: "World", pending: list[PoI], available: list[UAV]) -> int:
        terminals = self.relays.terminals(world)
        fleet = [u for u in world.state.operational_uavs() if u.role not in (UAVRole.RETURNING, UAVRole.CHARGING)]
        current_relays = self.relays.count_relays(terminals) or 0
        blocked_priority: Optional[int] = None
        made = 0
        for poi in pending:
            if not available:
                break
            bids = sorted((self._bid(u, poi), u.uav_id, u) for u in available if self.can_do(u, poi))
            if not bids:
                continue
            reason = "auction"
            if self.comm_aware:
                candidate_terms = terminals + [Terminal(poi.poi_id, np.array(self._waypoint(poi)), poi.priority)]
                relays_needed = self.relays.count_relays(candidate_terms)
                extra_relays = None if relays_needed is None else relays_needed - current_relays
                fits = relays_needed is not None and len(candidate_terms) + relays_needed <= len(fleet)
                if blocked_priority is not None and poi.priority < blocked_priority and (extra_relays or 0) > 0:
                    continue  # do not let a lower-priority PoI take the relays a waiting one needs
                if not fits:
                    blocked_at = self._blocked_since.setdefault(poi.poi_id, world.t)
                    if world.t - blocked_at < self.params.ferry_wait_s:
                        blocked_priority = poi.priority if blocked_priority is None else blocked_priority
                        continue
                    reason = "data ferry (not enough UAVs to stay connected)"
                    self._ferry_pois.add(poi.poi_id)
                else:
                    self._blocked_since.pop(poi.poi_id, None)
                    terminals = candidate_terms
                    current_relays = relays_needed
            _, _, winner = bids[0]
            self._blocked_since.pop(poi.poi_id, None)
            self.roles.assign_survey(winner, poi, reason)
            available.remove(winner)
            self.assignments += 1
            made += 1
        return made
