"""
swarm/mission_manager.py
The swarm brain: runs every decision module in a fixed order each decision tick.

    reconfiguration -> deadline/safety -> energy -> task allocation ->
    priority pre-emption -> relay plan -> spares (backup / data ferry)

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
from swarm.network_manager import NetworkManager
from swarm.priority_manager import PriorityManager, PriorityParams
from swarm.reconfiguration import ReconfigParams, ReconfigurationEngine
from swarm.relay_selector import RelayParams, RelaySelector
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
    routing: RouteParams = field(default_factory=RouteParams)
    allocation: AllocationParams = field(default_factory=AllocationParams)
    relay: RelayParams = field(default_factory=RelayParams)
    energy: EnergyParams = field(default_factory=EnergyParams)
    priority: PriorityParams = field(default_factory=PriorityParams)
    safety: SafetyParams = field(default_factory=SafetyParams)
    reconfiguration: ReconfigParams = field(default_factory=ReconfigParams)

    _SUB = {"routing": RouteParams, "allocation": AllocationParams, "relay": RelayParams, "energy": EnergyParams,
            "priority": PriorityParams, "safety": SafetyParams, "reconfiguration": ReconfigParams}

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
        world.register_uav_selector("critical_relay", lambda w: self.routes.critical_relay(w))
        world.register_point_selector("backbone_midpoint",
                                      lambda w: self.routes.backbone_midpoint(self.env.comm.positions))
        self._next_decision_s = 0.0
        self.mission_complete_s: Optional[float] = None
        self._recalled = False

    # ------------------------------------------------------------------ update
    def update(self, world: "World", network_updated: bool) -> None:
        if network_updated:
            view = self.network.rebuild(world)
            self.routes.update(world, view)
        self.safety.monitor()
        if world.t + 1e-9 < self._next_decision_s:
            return
        self._next_decision_s = world.t + self.params.decision_interval_s
        view = self.network.view

        reasons = self.reconfig.evaluate(world, view)
        self.safety.enforce_deadline(self.roles)
        self.energy.update(world)
        self.allocator.allocate(world)
        self.priority.update(world, self.allocator.can_do)
        if self.relays.update(world, view, force=bool(reasons)) and reasons:
            world.publish(EventType.RECONFIGURATION, f"Re-planned after: {'; '.join(reasons[:3])}",
                          data={"reasons": reasons, "relays": self.relays.plan.relay_count})
        self._manage_spares(world, view)
        self.reconfig.check_recovery(world)
        self._check_mission_complete(world)

    # ------------------------------------------------------------------ spares
    def _manage_spares(self, world: "World", view) -> None:
        spares = [u for u in world.state.uavs_with_role(UAVRole.IDLE, UAVRole.BACKUP)
                  if u.is_airborne and u.uav_id not in self.relays.assignment]
        if self.params.data_ferry:
            spares = [u for u in spares if not self._ferry(world, u)]
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
        """Midway between the GCS and the relay backbone: connected and close to where help is needed."""
        points = [p for p, _ in self.relays.assignment.values()]
        gcs = world.state.gcs_position
        if not points:
            return None
        centre = np.mean([p[:2] for p in points], axis=0)
        mid = (np.asarray(gcs[:2], dtype=float) + centre) / 2.0
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
        pois = world.state.pois
        if self.mission_complete_s is None and pois.all_completed:
            self.mission_complete_s = world.t
        if pois.all_completed and not self._recalled:
            self._recalled = True
            for uav in world.state.operational_uavs():
                if uav.is_airborne and uav.role not in (UAVRole.RETURNING, UAVRole.CHARGING):
                    self.roles.return_home(uav, "mission complete")
        elif not pois.all_completed:
            self._recalled = False

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
