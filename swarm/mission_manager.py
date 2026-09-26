"""
swarm/mission_manager.py
The swarm brain: runs every decision module in a fixed order each decision tick.

    reconfiguration -> deadline/safety -> energy -> task allocation ->
    priority pre-emption -> relay plan -> spares (backup / data ferry / standby)

Modes
    adaptive  full communication-aware autonomy (the proposed system)
    baseline  comparison system: nearest-UAV allocation, relays planned once,
              no fault detection, no pre-emption, no relay handover
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Any, Mapping, Optional

import numpy as np

from core.config import build
from core.events import EventType
from core.uav import GCS_NODE_ID, UAV, UAVRole
from swarm.energy_manager import EnergyManager, EnergyParams
from swarm.fleet_planner import FleetParams
from swarm.network_manager import NetworkManager
from swarm.priority_manager import PriorityManager, PriorityParams
from swarm.reconfiguration import ReconfigParams, ReconfigurationEngine
from swarm.relay_selector import RelayParams, RelaySelector, Terminal
from swarm.role_manager import RoleManager
from swarm.route_manager import RouteManager, RouteParams
from swarm.safety_manager import SafetyManager, SafetyParams
from swarm.task_allocator import AllocationParams, TaskAllocator

if TYPE_CHECKING:
    from core.world import World
    from simulation.environment import Environment

MODES = ("adaptive", "baseline")


@dataclass(frozen=True)
class SwarmParams:
    decision_interval_s: float = 1.0
    backup_enabled: bool = True
    backup_min_battery_pct: float = 60.0
    data_ferry: bool = True
    idle_standby_after_s: Optional[float] = None   # idle this long -> wait airborne over its own pad (None = stay put)
    routing: RouteParams = field(default_factory=RouteParams)
    allocation: AllocationParams = field(default_factory=AllocationParams)
    relay: RelayParams = field(default_factory=RelayParams)
    energy: EnergyParams = field(default_factory=EnergyParams)
    priority: PriorityParams = field(default_factory=PriorityParams)
    safety: SafetyParams = field(default_factory=SafetyParams)
    reconfiguration: ReconfigParams = field(default_factory=ReconfigParams)
    fleet: FleetParams = field(default_factory=FleetParams)

    _SUB = {"routing": RouteParams, "allocation": AllocationParams, "relay": RelayParams, "energy": EnergyParams,
            "priority": PriorityParams, "safety": SafetyParams, "reconfiguration": ReconfigParams,
            "fleet": FleetParams}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "SwarmParams":
        data = dict(data or {})
        subs = {name: build(kind, data.pop(name, None), f"swarm.{name}") for name, kind in cls._SUB.items()}
        return build(cls, {**data, **subs}, "swarm")


class MissionManager:
    def __init__(self, world: "World", env: "Environment", mode: str = "adaptive") -> None:
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}")
        self.world = world
        self.env = env
        self.mode = mode
        self.params = SwarmParams.from_dict(world.params.section("swarm"))
        adaptive = mode == "adaptive"

        self.network = NetworkManager(env.comm)
        self.routes = RouteManager(self.params.routing)
        self.safety = SafetyManager(world, env, self.params.safety)
        self.roles = RoleManager(world, self.safety)
        self.priority = PriorityManager(world, self.roles, replace(self.params.priority, preempt=adaptive))
        self.relays = RelaySelector(world, env, self.roles, self.params.relay, static=not adaptive)
        self.allocator = TaskAllocator(world, env, self.roles, self.safety, self.relays, self.priority,
                                       self.params.allocation, comm_aware=adaptive)
        self.energy = EnergyManager(world, env, self.roles, self.relays,
                                    replace(self.params.energy, handover=adaptive))
        self.reconfig = ReconfigurationEngine(world, env, self.routes, self.relays,
                                              replace(self.params.reconfiguration, enabled=adaptive))
        self.reconfig.ferrying = self.allocator.ferrying   # intentionally disconnected UAVs
        self.relays.skip = self.allocator.ferry_pois       # ... whose PoIs need no relay chain
        self.priority.held = self.allocator.deferred       # the farthest PoI waits its turn: no pre-emption
        self.relays.needed_at = self.energy.needed_at      # relay points only for UAVs that can hold them
        world.landing_clearance = self.safety.landing_clear   # no descent into traffic over the pads
        world.register_uav_selector("critical_relay", lambda w: self.routes.critical_relay(w))
        world.register_point_selector("backbone_midpoint", lambda w: self.routes.backbone_midpoint(
            self.env.comm.positions, {u.uav_id for u in w.state.uavs_with_role(UAVRole.SURVEY)}))
        self._next_decision_s = 0.0
        self.mission_complete_s: Optional[float] = None
        self._recalled = False
        self._idle_since: dict[int, float] = {}   # airborne IDLE UAVs with nothing to do -> since when
        self._standing_by: set[int] = set()        # ... already sent down to the standby level over their pad

    # ------------------------------------------------------------------ update
    def update(self, world: "World", network_updated: bool) -> None:
        if network_updated:
            view = self.network.rebuild(world)
            self.routes.update(world, view)
        self.safety.monitor()
        if world.t + 1e-9 >= self._next_decision_s:
            self._next_decision_s = world.t + self.params.decision_interval_s
            self._decide(world)
        # Last, so every plan - including commands issued just now - is checked before anything moves.
        self.safety.avoid()

    def _decide(self, world: "World") -> None:
        view = self.network.view
        reasons = self.reconfig.evaluate(world, view)
        self.safety.enforce_deadline(self.roles)
        self.energy.update(world, self._has_pending_work)
        self.allocator.allocate(world)
        self.priority.update(world, self.allocator.can_do)
        lookahead = self._lookahead_terminals()
        if self.relays.update(world, view, lookahead=lookahead, force=bool(reasons)) and reasons:
            world.publish(EventType.RECONFIGURATION, f"Re-planned after: {'; '.join(reasons[:3])}",
                          data={"reasons": reasons, "relays": self.relays.plan.relay_count})
        self._manage_spares(world, view)
        self.reconfig.check_recovery(world)
        self._check_mission_complete(world)

    def _lookahead_terminals(self) -> list[Terminal]:
        """The next few PoIs still in the queue (swarm/task_allocator.py fills ``allocator.pending``
        every tick) and the farthest PoI held back for its turn, so the relay planner can reach
        toward them ahead of a surveyor being sent."""
        alt = self.params.relay.relay_altitude_m
        queued = self.allocator.pending[:self.params.relay.lookahead_pois]
        held = [self.world.state.pois.get(poi_id) for poi_id in sorted(self.allocator.deferred)]
        return [Terminal(p.poi_id, np.array([*p.position[:2], alt]), p.priority) for p in (*queued, *held)]

    def _has_pending_work(self, uav: UAV) -> bool:
        """True if this UAV could still fly a pending PoI the allocator is able to hand out -
        used to hold off a proactive recharge that would pull a usable UAV out of the mission.
        A PoI waiting for a relay path does not count: holding a UAV in the air for one would
        only leave it hovering with no role until its reserve forces it home anyway."""
        waiting = self.allocator.waiting
        return any(self.allocator.can_do(uav, p) for p in self.allocator.pending if p.poi_id not in waiting)

    # ------------------------------------------------------------------ spares
    def _manage_spares(self, world: "World", view) -> None:
        spares = [u for u in world.state.uavs_with_role(UAVRole.IDLE, UAVRole.BACKUP)
                  if u.is_airborne and u.uav_id not in self.relays.assignment]
        if self.params.data_ferry:
            spares = [u for u in spares if not self._ferry(world, u)]
        self._manage_backup(world, spares)
        self._standby(world, [u for u in spares if u.role is UAVRole.IDLE])

    def _standby(self, world: "World", idle: list[UAV]) -> None:
        """An idle UAV with nothing to do waits low over its own launch pad, still airborne and
        taskable: connected to the GCS, next to where every relay chain starts, and only a descent
        from home when it reaches the return-to-home level - instead of hovering out in the field
        until the trip home forces it back early. Each UAV has its own pad, so no two wait in the
        same spot.

        It flies there above the standby level and only comes down once over its own pad: crossing
        the pads at the level where the others wait (30 m apart, so no gap to pass through) boxed a
        UAV in beside someone else's pad - and in the way of whoever had to land there."""
        ids = {u.uav_id for u in idle}
        self._idle_since = {uid: t for uid, t in self._idle_since.items() if uid in ids}
        self._standing_by &= ids
        after = self.params.idle_standby_after_s
        if after is None:
            return
        low, transit = self.safety.levels[0], self.safety.levels[min(1, len(self.safety.levels) - 1)]
        for uav in idle:
            if world.t - self._idle_since.setdefault(uav.uav_id, world.t) < after:
                continue                          # a new role usually comes sooner
            home = (float(uav.home[0]), float(uav.home[1]))
            if uav.horizontal_distance_to(uav.home) <= world.params.uav.arrival_radius_m:
                if uav.uav_id not in self._standing_by:     # arrived: down its own column, once clear
                    alt = self.safety.safe_altitude(uav, uav.home, low)
                    if alt < uav.position[2] and not self.safety.column_clear(uav, alt):
                        continue                  # someone is passing the pad: wait up here
                    self._standing_by.add(uav.uav_id)
                    world.goto(uav.uav_id, (*home, alt), "standby over the pad")
                continue
            if uav.target is not None and float(np.hypot(*(uav.target[:2] - uav.home[:2]))) < 1.0:
                continue                          # on the way
            self._standing_by.discard(uav.uav_id)
            alt = self.safety.safe_altitude(uav, uav.home, max(float(uav.position[2]), transit))
            world.goto(uav.uav_id, (*home, alt), "to its pad for standby")

    def _manage_backup(self, world: "World", spares: list[UAV]) -> None:
        if not self.params.backup_enabled or self.mode != "adaptive":
            return
        staging = self._staging_point(world)
        if staging is None:
            return
        backups = [u for u in spares if u.role is UAVRole.BACKUP]
        if not backups:
            ready = [u for u in spares if u.battery_pct >= self.params.backup_min_battery_pct]
            if not ready:
                return
            uav = min(ready, key=lambda u: (u.distance_to(staging), u.uav_id))
            self.roles.make_backup(uav, staging, "spare parked near the network")
        else:
            uav = backups[0]
            if uav.target is None or float(np.hypot(*(uav.target[:2] - staging[:2]))) > 60.0:
                self.roles.make_backup(uav, staging, "staging point moved")

    def _staging_point(self, world: "World") -> Optional[np.ndarray]:
        """Toward the next queued PoI, no farther than the relay backbone already reaches (or
        the GCS, with no backbone yet): connected, and a head start on whichever PoI a backup
        is likely to fly next, instead of just parking near the network."""
        points = [p for p, _ in self.relays.assignment.values()]
        gcs = np.asarray(world.state.gcs_position[:2], dtype=float)
        anchor = gcs if not points else np.mean([p[:2] for p in points], axis=0)
        pending = self.allocator.pending
        if pending:
            mid = (anchor + np.asarray(pending[0].position[:2], dtype=float)) / 2.0
        elif points:
            mid = (gcs + anchor) / 2.0
        else:
            return None
        return np.array([mid[0], mid[1], self.params.relay.relay_altitude_m])

    def _ferry(self, world: "World", uav: UAV) -> bool:
        """Disconnected UAV holding data: fly toward the nearest connected node to deliver it."""
        if uav.comm.connected or self.env.data.buffer_mb(uav.uav_id) <= 0.1:
            return False
        positions = self.env.comm.positions
        connected = [(nid, pos) for nid, pos in positions.items()
                     if nid == GCS_NODE_ID or world.state.uavs[nid].comm.connected]
        if not connected:
            target = self.env.comm.gcs_antenna_position(world)
        else:
            target = min(connected, key=lambda item: float(np.linalg.norm(item[1][:2] - uav.position[:2])))[1]
        waypoint = np.array([target[0], target[1], uav.position[2]])
        if uav.target is not None and float(np.hypot(*(uav.target[:2] - waypoint[:2]))) < 30.0:
            return True
        try:
            self.world.goto(uav.uav_id, waypoint, "data ferry")
        except Exception:  # outside the area or otherwise rejected - keep hovering
            return False
        return True

    def _check_mission_complete(self, world: "World") -> None:
        # PoIs still due to appear (random_pois.spawn_s) are part of the mission: finishing the
        # ones seen so far is not the end, and must not send the whole swarm home early.
        done = world.state.pois.all_completed and world.pending_spawns == 0
        if self.mission_complete_s is None and done:
            self.mission_complete_s = world.t
        if done and not self._recalled:
            self._recalled = True
            for uav in world.state.operational_uavs():
                if uav.is_airborne and uav.role not in (UAVRole.RETURNING, UAVRole.CHARGING):
                    self.roles.return_home(uav, "mission complete")
        elif not done:
            # A PoI appeared after everything was done: the mission is open again.
            # Leaving the old completion time would let the run stop as soon as
            # the first wave has landed, with the new PoI never flown.
            self._recalled = False
            self.mission_complete_s = None

    @property
    def all_landed(self) -> bool:
        return all(not u.is_airborne for u in self.world.state.operational_uavs())

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "relays": self.relays.to_dict(),
            "routes": [r.to_dict() for r in sorted(self.routes.routes.values(), key=lambda r: r.uav_id)],
            "incidents": self.reconfig.to_dict(),
            "network": self.network.view.to_dict(),
            "allocation": {"assignments": self.allocator.assignments,
                           "mean_runtime_ms": round(float(np.mean(self.allocator.runtime_ms)), 3)
                           if self.allocator.runtime_ms else None},
            "mission_complete_s": self.mission_complete_s,
        }
