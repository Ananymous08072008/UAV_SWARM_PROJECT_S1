"""
swarm/task_allocator.py
Communication- and energy-aware assignment of survey tasks (sequential auction).

For each pending PoI, in effective-priority order:
  1. Feasible UAVs: IDLE or BACKUP, enough battery to fly there, survey the
     remaining time and still return home with a reserve - finishing the
     survey above the return-to-home level (``battery.critical_pct``), so the
     energy manager never has to pull it off halfway - and able to finish
     before the mission deadline. If no UAV clears the normal reserve, a UAV
     that clears a slimmer (but never unsafe) margin is used instead, rather
     than leaving the PoI - and the UAV - idle when nothing else can take it.
  2. Communication budget (adaptive mode): the swarm must still have enough
     UAVs for the surveyor *plus* the relays that keep every surveyor
     connected (computed by the relay planner). If not, the PoI waits for a
     relay path to free up. A lower-priority PoI may not jump the queue if it
     needs extra relays a waiting higher-priority one would need too.
  3. Winner = lowest bid: travel time + battery cost.

Once every PoI that fits the communication budget has been handed out, any
still blocked for lack of a relay path gets one last look (``_ferry_last_resort``):
it is surveyed disconnected, its data ferried back (store-and-forward), only if
UAVs are left with nothing else to do and either it is high priority and has
stayed blocked for a little while, or the mission deadline itself is close (so
a low-priority PoI a tight relay budget never reaches is not abandoned for good).
This is deliberately rare - see the module docstring in swarm/mission_manager.py
for where it sits in the decision order.

Before any of that, one PoI is singled out: of the mission's planned PoIs that
have appeared and are not done yet, the one farthest from the GCS
(``_farthest_planned``). While any other PoI is still open it is held out of the
auction entirely (``_defer_farthest``), so by the time it is finally tasked the
whole fleet is free for it. Being the farthest point, its relay chain is
naturally the longest in the mission - built for it alone rather than shared
with, or crowded out by, whatever else is in flight. That is what turns it into
a multi-hop escort instead of a lone data-ferry run. PoIs appear over time
(``random_pois.spawn_s``), so the choice is re-made every tick: a farther one
that appears later takes over; one already being flown is never pulled back. A
region that emerges outside the plan (an ``add_poi`` report) is never held back.
The hold ends early only when waiting any longer could leave too little time to
fly it before the deadline (``_last_call``).

Baseline mode skips step 2 (nearest feasible UAV, communication-unaware) and
never ferries; it does not defer the farthest PoI either.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

import numpy as np

from core.events import EventType, Severity
from core.poi import PoI, PoIStatus
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
    relaxed_reserve_pct: float = 4.0  # fallback reserve when no UAV clears the normal one
    ferry_min_priority: int = 4       # a PoI at/above this priority may ferry before the deadline crunch ...
    ferry_min_blocked_s: float = 60.0     # ... once it has been continuously blocked at least this long
    ferry_deadline_margin_s: float = 600.0  # any blocked PoI may ferry once this little mission time is left
    defer_deadline_margin_s: float = 300.0  # the farthest PoI stops waiting its turn with this little slack left


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
        self.ferry_assignments = 0
        self.runtime_ms: list[float] = []
        self._blocked_since: dict[str, float] = {}   # PoI -> when it was first seen with no relay path
        self._hopeless: set[str] = set()             # blocked PoIs the whole fleet could never connect
        self.ferry_pois: set[str] = set()           # PoIs deliberately surveyed without a relay chain
        self.ferrying: set[int] = set()              # UAVs on those PoIs (expected to be disconnected)
        self.pending: list[PoI] = []                 # still-pending after the last allocate(), priority order
        self.waiting: set[str] = set()               # pending PoIs held back last tick for want of relays
        self.farthest_poi_id: Optional[str] = None   # the PoI the multi-hop chain is ultimately built for
        self.deferred: set[str] = set()              # held back last tick as that farthest PoI

    # ------------------------------------------------------------- feasibility
    def _waypoint(self, poi: PoI):
        alt = poi.altitude_m if poi.altitude_m is not None else self.world.params.uav.default_altitude_m
        return (float(poi.position[0]), float(poi.position[1]), alt)

    def can_do(self, uav: UAV, poi: PoI, reserve_pct: Optional[float] = None) -> bool:
        waypoint = self._waypoint(poi)
        remaining = max(0.0, poi.survey_time_s - poi.progress_s)
        reserve = self.params.reserve_pct if reserve_pct is None else reserve_pct
        bat = self.env.battery
        outbound = bat.travel_cost_pct(uav.position, waypoint) + bat.hover_cost_pct(remaining)
        home_leg = bat.task_cost_pct(uav, waypoint, remaining) - outbound
        needed = outbound + max(home_leg + reserve, self.world.params.battery.critical_pct)
        return uav.battery_pct >= needed and self.safety.fits_deadline(uav, waypoint, remaining)

    def _bid(self, uav: UAV, poi: PoI) -> float:
        waypoint = self._waypoint(poi)
        remaining = max(0.0, poi.survey_time_s - poi.progress_s)
        return (self.env.battery.travel_time_s(uav.position, waypoint)
                + self.params.battery_weight_s * self.env.battery.task_cost_pct(uav, waypoint, remaining))

    def _feasible(self, available: list[UAV], poi: PoI) -> tuple[list[tuple[float, int, UAV]], bool]:
        """Bids under the normal reserve; if none, bids under a slimmer (never unsafe) one -
        so a UAV a little short of the usual margin still flies rather than sitting idle when
        it is the only one that can take this PoI at all."""
        strict = sorted((self._bid(u, poi), u.uav_id, u) for u in available if self.can_do(u, poi))
        if strict:
            return strict, False
        # can_do keeps the return-to-home floor under every reserve, so a slimmer one stays safe.
        relaxed = sorted((self._bid(u, poi), u.uav_id, u) for u in available
                         if self.can_do(u, poi, reserve_pct=self.params.relaxed_reserve_pct))
        return relaxed, bool(relaxed)

    # -------------------------------------------------------------- allocation
    @staticmethod
    def _farthest_planned(world: "World") -> Optional[PoI]:
        """Of the mission's planned PoIs that have appeared and are not done, the one farthest
        from the GCS. An emerging region reported outside the plan never counts."""
        gcs = np.asarray(world.state.gcs_position[:2], dtype=float)
        open_pois = [p for p in world.state.pois if not p.is_completed and p.poi_id in world.planned_poi_ids]
        return max(open_pois, key=lambda p: (float(np.hypot(*(p.position[:2] - gcs))), p.poi_id), default=None)

    def _defer_farthest(self, world: "World", pending: list[PoI]) -> list[PoI]:
        """Hold the farthest PoI out of the auction while any other PoI is still open. By the
        time its turn comes the whole fleet is idle, so its relay chain - the longest in the
        mission, being the farthest point - is planned for it alone instead of competing with
        everything else still in flight: a multi-hop escort the swarm can commit to fully,
        rather than a lone data-ferry run."""
        farthest = self._farthest_planned(world)
        self.farthest_poi_id = None if farthest is None else farthest.poi_id
        self.deferred.clear()
        if farthest is None or farthest.status is not PoIStatus.PENDING:
            return pending
        if all(p.is_completed for p in world.state.pois if p is not farthest) or self._last_call(world, farthest):
            return pending
        self.deferred.add(farthest.poi_id)
        return [p for p in pending if p is not farthest]

    def _last_call(self, world: "World", poi: PoI) -> bool:
        """True once holding ``poi`` back any longer could leave too little time to fly it: a UAV
        launched from its pad needs to fly out, survey and come home before the recall, with
        ``defer_deadline_margin_s`` to spare. A nearer PoI that is taking long must not cost the
        farthest one its survey."""
        fleet = world.state.operational_uavs()
        if not fleet:
            return False
        waypoint = self._waypoint(poi)
        remaining = max(0.0, poi.survey_time_s - poi.progress_s)
        needed = min(self.env.battery.task_time_s(u, waypoint, remaining, start=u.home) for u in fleet)
        return world.time_left_s <= needed + self.safety.params.recall_margin_s + self.params.defer_deadline_margin_s

    def allocate(self, world: "World") -> int:
        started = time.perf_counter()
        pending = self.priority.ordered_pending(world)
        if self.comm_aware:
            pending = self._defer_farthest(world, pending)
        available = [u for u in world.state.operational_uavs() if u.role in TASKABLE_ROLES]
        made = 0
        self.waiting = set()
        if pending and available:
            made = self._allocate(world, pending, available)
        self.pending = [p for p in pending if p.status is PoIStatus.PENDING]
        self.ferrying.clear()
        self.ferrying.update(u.uav_id for u in world.state.uavs_with_role(UAVRole.SURVEY)
                             if u.assigned_poi in self.ferry_pois)
        self.runtime_ms.append((time.perf_counter() - started) * 1000.0)
        return made

    def _allocate(self, world: "World", pending: list[PoI], available: list[UAV]) -> int:
        operational = len(world.state.operational_uavs())
        # Surveys flown disconnected (ferried, or pre-empted onto a PoI no chain can reach) use no
        # relays. Counting them would make every new PoI look unaffordable and ground the spares.
        terminals = [t for t in self.relays.terminals(world)
                     if t.key not in self.ferry_pois and not self._never_fits(t, operational)]
        fleet = [u for u in world.state.operational_uavs() if u.role not in (UAVRole.RETURNING, UAVRole.CHARGING)]
        current_relays = self.relays.count_relays(terminals) or 0
        blocked_priority: Optional[int] = None
        blocked: list[PoI] = []
        held: list[PoI] = []
        hopeless: set[str] = set()
        made = 0
        for poi in pending:
            if not available:
                break
            bids, relaxed = self._feasible(available, poi)
            if not bids:
                continue
            reason = "auction (reduced battery margin: no fully-reserved UAV available)" if relaxed else "auction"
            if self.comm_aware:
                candidate_terms = terminals + [Terminal(poi.poi_id, np.array(self._waypoint(poi)), poi.priority)]
                relays_needed = self.relays.count_relays(candidate_terms)
                extra_relays = None if relays_needed is None else relays_needed - current_relays
                fits = relays_needed is not None and len(candidate_terms) + relays_needed <= len(fleet)
                if blocked_priority is not None and poi.priority < blocked_priority and (extra_relays or 0) > 0:
                    held.append(poi)
                    continue  # do not let a lower-priority PoI take the relays a waiting one needs
                if not fits:
                    blocked.append(poi)
                    if self._never_fits(candidate_terms[-1], operational):
                        # Waiting cannot help, so it must not hold lower-priority PoIs back either.
                        hopeless.add(poi.poi_id)
                    elif blocked_priority is None:
                        blocked_priority = poi.priority
                    continue
                terminals, current_relays = candidate_terms, relays_needed
            _, _, winner = bids[0]
            self.ferry_pois.discard(poi.poi_id)   # once ferried, now flown with a relay chain
            self.roles.assign_survey(winner, poi, reason)
            available.remove(winner)
            self.assignments += 1
            made += 1
        # Only PoIs still blocked at the end of this pass keep their clock running; one that
        # got a relay path (or was completed/re-tasked elsewhere) starts fresh if it ever blocks again.
        self._blocked_since = {p.poi_id: self._blocked_since.get(p.poi_id, world.t) for p in blocked}
        self._hopeless = hopeless
        self.waiting = {p.poi_id for p in blocked + held}
        if self.comm_aware and available and blocked:
            made += self._ferry_last_resort(world, blocked, available)
        return made

    def _never_fits(self, terminal: Terminal, operational: int) -> bool:
        """True when even the whole operational fleet, with nothing else flying, could not
        both survey this point and relay it to the GCS - no amount of waiting frees enough UAVs."""
        alone = self.relays.count_relays([terminal])
        return alone is None or alone + 1 > operational

    def _ferry_last_resort(self, world: "World", blocked: list[PoI], available: list[UAV]) -> int:
        """Survey a comms-blocked PoI without a relay chain - only when no relay path fits the
        fleet (why it is in ``blocked``), there are UAVs left with nothing else to do, and it has
        stayed blocked for ``ferry_min_blocked_s`` (not just one contended tick) while either
        - it is high priority,
        - the whole fleet could never connect it (waiting is futile - the swarm would only idle), or
        - the deadline is close. This is what keeps a low-priority PoI a tight relay budget never
          gets around to from being abandoned: once time is short, whatever is left is the last
          resort regardless of priority, and it does not wait out ``ferry_min_blocked_s`` first."""
        made = 0
        for poi in sorted(blocked, key=lambda p: (-p.priority, p.created_at_s, p.poi_id)):
            if not available:
                break
            deadline_close = world.time_left_s <= self.params.ferry_deadline_margin_s
            persisted = world.t - self._blocked_since[poi.poi_id] >= self.params.ferry_min_blocked_s
            urgent = poi.priority >= self.params.ferry_min_priority
            if not (deadline_close or (persisted and (urgent or poi.poi_id in self._hopeless))):
                continue
            bids, _ = self._feasible(available, poi)
            if not bids:
                continue
            _, _, winner = bids[0]
            self.ferry_pois.add(poi.poi_id)
            self.ferry_assignments += 1
            self.world.publish(EventType.DATA_FERRY_ASSIGNED,
                               f"{poi.poi_id} (p{poi.priority}) surveyed by {winner.name} without a relay chain "
                               f"- last resort, {world.time_left_s:.0f}s left, no other PoI to task",
                               severity=Severity.WARNING, uav_id=winner.uav_id, poi_id=poi.poi_id,
                               data={"priority": poi.priority, "time_left_s": round(world.time_left_s, 1)})
            self.roles.assign_survey(winner, poi, "data ferry (last resort: no relay path fits the fleet)")
            available.remove(winner)
            self.assignments += 1
            made += 1
        return made
