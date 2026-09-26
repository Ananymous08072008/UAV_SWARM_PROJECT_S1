"""
swarm/priority_manager.py
Handles newly emerging high-priority regions.

* Effective priority = base priority + ageing bonus, so low-priority PoIs are
  not starved forever.
* When a new PoI with priority >= ``preempt_min_priority`` appears and no idle
  UAV can take it, the lowest-priority surveyor (at least ``preempt_priority_gap``
  below) that can reach it is re-tasked immediately. The interrupted PoI keeps
  its survey progress and goes back to the queue.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable

from core.events import EventType, Severity
from core.poi import PoI, PoIStatus
from core.uav import UAV, UAVRole

if TYPE_CHECKING:
    from core.world import World
    from swarm.role_manager import RoleManager


@dataclass(frozen=True)
class PriorityParams:
    ageing_per_min: float = 0.25
    max_ageing_bonus: float = 1.5
    preempt: bool = True
    preempt_min_priority: int = 4
    preempt_priority_gap: int = 2


class PriorityManager:
    def __init__(self, world: "World", roles: "RoleManager", params: PriorityParams) -> None:
        self.world = world
        self.roles = roles
        self.params = params
        self._watch: list[str] = []
        self.held: set[str] = set()   # PoIs the allocator is holding back on purpose: never pre-empt for them
        self.preemptions = 0
        world.events.subscribe(self._on_poi_added, types=[EventType.POI_ADDED])

    def _on_poi_added(self, event) -> None:
        self._watch.append(event.poi_id)

    def effective_priority(self, poi: PoI, t_s: float) -> float:
        bonus = min(self.params.max_ageing_bonus, self.params.ageing_per_min * max(0.0, t_s - poi.created_at_s) / 60.0)
        return poi.priority + bonus

    def ordered_pending(self, world: "World") -> list[PoI]:
        return sorted(world.state.pois.pending(),
                      key=lambda p: (-self.effective_priority(p, world.t), p.created_at_s, p.poi_id))

    def update(self, world: "World", can_do: Callable[[UAV, PoI], bool]) -> None:
        """Pre-empt surveyors for new high-priority PoIs that are still waiting."""
        still_waiting = []
        for poi_id in self._watch:
            poi = world.state.pois.get(poi_id)
            if poi.status is not PoIStatus.PENDING:
                continue
            if not self.params.preempt or poi.priority < self.params.preempt_min_priority:
                continue
            if poi_id in self.held:
                still_waiting.append(poi_id)
                continue
            victim = self._pick_victim(world, poi, can_do)
            if victim is None:
                still_waiting.append(poi_id)
                continue
            old_poi = world.state.pois.get(victim.assigned_poi)
            self.roles.assign_survey(victim, poi, f"pre-empted from {old_poi.poi_id}")
            self.preemptions += 1
            world.publish(EventType.PREEMPTION,
                          f"{victim.name} re-tasked {old_poi.poi_id} (p{old_poi.priority}) -> "
                          f"{poi.poi_id} (p{poi.priority})",
                          severity=Severity.WARNING, uav_id=victim.uav_id, poi_id=poi.poi_id,
                          data={"from": old_poi.poi_id, "to": poi.poi_id})
        self._watch = still_waiting

    def _pick_victim(self, world: "World", poi: PoI, can_do: Callable[[UAV, PoI], bool]) -> UAV | None:
        max_victim_priority = poi.priority - self.params.preempt_priority_gap
        candidates = []
        for uav in world.state.uavs_with_role(UAVRole.SURVEY):
            current = world.state.pois.get(uav.assigned_poi) if uav.assigned_poi else None
            if current is None or current.priority > max_victim_priority or not can_do(uav, poi):
                continue
            candidates.append((current.priority, uav.horizontal_distance_to(poi.position), uav.uav_id, uav))
        return min(candidates)[3] if candidates else None
