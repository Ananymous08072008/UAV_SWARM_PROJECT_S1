"""
swarm/energy_manager.py
Return-to-home decisions for UAVs with limited flight time.

For every airborne UAV the manager estimates the battery needed to fly home
and land (+ reserve). That level never drops below ``battery.critical_pct``,
the return-to-home threshold: a UAV near the pad goes home exactly there, one
so far out that the trip needs more leaves as soon as its battery only just
covers it.
  * battery <= needed                    -> return home now
  * RELAY within ``handover_margin_pct``   -> handover: excluded from the relay
    plan so a replacement is sent; it keeps relaying until the replacement is
    on station (or ``max_handover_wait_s`` passes), then returns home
  * SURVEY that cannot finish its PoI      -> releases the PoI (progress kept)
    and returns home; the allocator re-tasks the PoI. With
    ``leave_unfinishable_survey`` off it keeps surveying instead - every second
    on station is progress the next UAV does not have to fly - and only leaves
    once it reaches the return-to-home level like everyone else
  * IDLE / BACKUP with low battery and *nothing it could still fly* -> recharge
    proactively. One that could still take a pending PoI keeps flying instead
    of pulling itself out of an active mission early.
  * IDLE (no role at all) with nothing it could be given, at any battery level
    (``park_when_idle``) -> land and top up on the pad rather than hover in place
    burning battery while the next PoI waits for relays. It stays assignable
    there. The one BACKUP parked near the network is left in the air.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Optional

import numpy as np

from core.events import EventType, Severity
from core.uav import UAV, UAVRole

if TYPE_CHECKING:
    from core.world import World
    from simulation.environment import Environment
    from swarm.relay_selector import RelaySelector
    from swarm.role_manager import RoleManager


@dataclass(frozen=True)
class EnergyParams:
    reserve_pct: float = 8.0
    handover: bool = True
    handover_margin_pct: float = 6.0
    max_handover_wait_s: float = 60.0
    idle_recharge_below_pct: float = 60.0
    idle_recharge_after_s: float = 20.0
    park_when_idle: bool = True           # an IDLE UAV with nothing to do waits on the pad, not hovering
    leave_unfinishable_survey: bool = True  # False = keep surveying until the return-to-home level


class EnergyManager:
    def __init__(self, world: "World", env: "Environment", roles: "RoleManager", relays: "RelaySelector",
                 params: EnergyParams) -> None:
        self.world = world
        self.env = env
        self.roles = roles
        self.relays = relays
        self.params = params
        self._handover_start: dict[int, float] = {}
        self._idle_since: dict[int, float] = {}
        self.rth_count = 0
        self.handovers = 0

    def needed_pct(self, uav: UAV) -> float:
        """Battery at which this UAV must start home: the trip home plus the reserve,
        and never below the critical floor (so the handover window is always usable)."""
        return self.needed_at(uav)

    def needed_at(self, uav: UAV, point: Optional[np.ndarray] = None) -> float:
        """``needed_pct`` as if the UAV were at ``point`` (default: where it is now)."""
        return max(self.env.battery.return_cost_pct(uav, point) + self.params.reserve_pct,
                   self.world.params.battery.critical_pct)

    def update(self, world: "World", has_pending_work: Optional[Callable[[UAV], bool]] = None) -> None:
        p, bat = self.params, self.env.battery
        for uav in world.state.operational_uavs():
            if uav.role in (UAVRole.RETURNING, UAVRole.CHARGING):
                self._forget(uav.uav_id)
                continue
            if not uav.is_airborne:
                self._idle_since.pop(uav.uav_id, None)
                continue
            needed = self.needed_pct(uav)

            if uav.uav_id in self._handover_start:
                waited = world.t - self._handover_start[uav.uav_id]
                if uav.battery_pct <= needed or waited >= p.max_handover_wait_s \
                        or self.relays.replacement_ready(world, uav):
                    self._go_home(uav, "relay handover complete" if waited < p.max_handover_wait_s
                                  else "handover timeout")
                continue

            if uav.battery_pct <= needed:
                self._go_home(uav, "battery reserve reached")
            elif uav.role is UAVRole.RELAY and p.handover and uav.battery_pct <= needed + p.handover_margin_pct:
                self._start_handover(uav)
            elif uav.role is UAVRole.SURVEY and uav.assigned_poi is not None and p.leave_unfinishable_survey:
                poi = world.state.pois.get(uav.assigned_poi)
                remaining = max(0.0, poi.survey_time_s - poi.progress_s)
                if uav.battery_pct < bat.task_cost_pct(uav, uav.target if uav.target is not None else uav.position,
                                                       remaining) + p.reserve_pct * 0.5:
                    self._go_home(uav, f"cannot finish {poi.poi_id} on this battery")
            elif uav.role in (UAVRole.IDLE, UAVRole.BACKUP):
                since = self._idle_since.setdefault(uav.uav_id, world.t)
                idle_long = world.t - since >= p.idle_recharge_after_s
                low = uav.battery_pct < p.idle_recharge_below_pct
                park = p.park_when_idle and uav.role is UAVRole.IDLE
                if idle_long and (low or park) and (has_pending_work is None or not has_pending_work(uav)):
                    self._go_home(uav, "recharge while idle" if low else "nothing to do: parking on the pad")
            if uav.role not in (UAVRole.IDLE, UAVRole.BACKUP):
                self._idle_since.pop(uav.uav_id, None)

    def _start_handover(self, uav: UAV) -> None:
        self._handover_start[uav.uav_id] = self.world.t
        self.relays.leaving.add(uav.uav_id)
        self.relays.request_replan()
        self.handovers += 1
        self.world.publish(EventType.HANDOVER_STARTED,
                           f"{uav.name} battery {uav.battery_pct:.0f}%: requesting a replacement relay",
                           severity=Severity.WARNING, uav_id=uav.uav_id, data={"battery_pct": round(uav.battery_pct, 1)})

    def _go_home(self, uav: UAV, reason: str) -> None:
        self._forget(uav.uav_id)
        self.roles.return_home(uav, reason)
        self.relays.request_replan()
        self.rth_count += 1

    def _forget(self, uav_id: int) -> None:
        if self._handover_start.pop(uav_id, None) is not None:
            self.relays.request_replan()
        self.relays.leaving.discard(uav_id)
        self._idle_since.pop(uav_id, None)
