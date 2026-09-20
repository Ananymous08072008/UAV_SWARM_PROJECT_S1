"""
swarm/safety_manager.py
Keeps the mission safe and on time.

Prevention
  * altitude layers: each UAV flies at base altitude + a per-UAV offset, so
    crossing paths are vertically separated
  * obstacle clearance: a waypoint's altitude is raised above any obstacle the
    straight path crosses
  * mission deadline: every UAV is recalled early enough to land before the
    allotted mission time ends; tasks that cannot finish in time are not given

Reaction
  * reactive deconfliction: when two UAVs come within 2x the minimum separation
    and are vertically too close (e.g. one was raised over an obstacle into
    another's level), the higher-numbered one changes altitude

Monitoring (SAFETY_VIOLATION events + counters)
  * minimum separation between airborne UAVs
  * geofence (operating area)
  * flying inside an obstacle
"""

from __future__ import annotations

import itertools
import logging
import math
from collections import Counter
from dataclasses import dataclass
from typing import TYPE_CHECKING, Sequence

from core.events import EventType, Severity
from core.uav import UAV, UAVRole

log = logging.getLogger(__name__)

if TYPE_CHECKING:
    from core.world import World
    from simulation.environment import Environment
    from swarm.role_manager import RoleManager


@dataclass(frozen=True)
class SafetyParams:
    min_separation_m: float = 5.0
    altitude_floor_m: float = 40.0   # lowest flight level
    altitude_layers: int = 12        # minimum number of levels; grows to the fleet size
    layer_spacing_m: float = 6.0     # shrunk automatically if the levels would pass the ceiling
    obstacle_clearance_m: float = 10.0
    recall_at_deadline: bool = True
    recall_margin_s: float = 30.0

    def __post_init__(self) -> None:
        if self.min_separation_m <= 0 or self.altitude_layers < 1 or self.layer_spacing_m < 0:
            raise ValueError("invalid safety parameters")


class SafetyManager:
    def __init__(self, world: "World", env: "Environment", params: SafetyParams) -> None:
        self.world = world
        self.env = env
        self.params = params
        self.violations: Counter[str] = Counter()
        self.min_separation_observed_m = math.inf
        self._active: set[tuple] = set()
        self.recall_started = False
        self.deconflictions = 0
        self._deconflicted: set[tuple[int, int]] = set()
        self.layers, self.layer_spacing_m = self._fit_levels(len(world.state.uavs))

    def _fit_levels(self, fleet: int) -> tuple[int, float]:
        """One flight level per UAV: at least one layer per UAV, squeezed under the ceiling if needed."""
        p = self.params
        layers = max(p.altitude_layers, fleet)
        spacing = p.layer_spacing_m
        ceiling = self.world.params.uav.max_altitude_m
        if layers > 1 and p.altitude_floor_m + (layers - 1) * spacing > ceiling:
            spacing = (ceiling - p.altitude_floor_m) / (layers - 1)
        if layers > 1 and spacing < p.min_separation_m:
            log.warning("%d UAVs do not fit between %.0f m and %.0f m with %.1f m separation; "
                        "levels are %.1f m apart", fleet, p.altitude_floor_m, ceiling, p.min_separation_m, spacing)
        return layers, spacing

    # -------------------------------------------------------------- altitudes
    def slot_altitude(self, uav_id: int, base_m: float = 0.0) -> float:
        """This UAV's own flight level (``base_m`` only raises it, e.g. a high survey altitude)."""
        offset = ((uav_id - 1) % self.layers) * self.layer_spacing_m
        return min(max(self.params.altitude_floor_m, base_m) + offset, self.world.params.uav.max_altitude_m)

    def safe_altitude(self, uav: UAV, target_xy: Sequence[float], base_m: float = 0.0) -> float:
        alt = self.slot_altitude(uav.uav_id, base_m)
        tallest = self.env.obstacles.max_height_crossed(uav.position, (target_xy[0], target_xy[1], alt))
        if tallest > 0:
            alt = max(alt, tallest + self.params.obstacle_clearance_m)
        return min(alt, self.world.params.uav.max_altitude_m)

    # ---------------------------------------------------------------- deadline
    def fits_deadline(self, uav: UAV, waypoint: Sequence[float], hover_s: float) -> bool:
        needed = self.env.battery.task_time_s(uav, waypoint, hover_s) + self.params.recall_margin_s
        return needed <= self.world.time_left_s

    def enforce_deadline(self, roles: "RoleManager") -> None:
        if not self.params.recall_at_deadline:
            return
        world = self.world
        for uav in world.state.operational_uavs():
            if not uav.is_airborne or uav.role in (UAVRole.RETURNING, UAVRole.CHARGING):
                continue
            if world.time_left_s <= self.env.battery.return_time_s(uav) + self.params.recall_margin_s:
                if not self.recall_started:
                    self.recall_started = True
                    world.publish(EventType.MISSION_RECALL,
                                  f"Mission time almost over ({world.time_left_s:.0f}s left): recalling the swarm",
                                  severity=Severity.WARNING)
                roles.return_home(uav, "mission deadline")

    # -------------------------------------------------------------- monitoring
    def monitor(self) -> None:
        world = self.world
        airborne = [u for u in world.state.operational_uavs() if u.is_airborne]
        active: set[tuple] = set()
        near: set[tuple[int, int]] = set()
        for a, b in itertools.combinations(airborne, 2):
            d = a.distance_to(b.position)
            self.min_separation_observed_m = min(self.min_separation_observed_m, d)
            if d < 2.0 * self.params.min_separation_m:
                near.add((a.uav_id, b.uav_id))
                self._deconflict(a, b)
            if d < self.params.min_separation_m:
                key = ("separation", a.uav_id, b.uav_id)
                active.add(key)
                if key not in self._active:
                    self._violation(key, f"{a.name} and {b.name} only {d:.1f} m apart", a.uav_id)
        area = world.state.area
        for uav in airborne:
            if not area.contains(uav.position[0], uav.position[1]):
                key = ("geofence", uav.uav_id)
                active.add(key)
                if key not in self._active:
                    self._violation(key, f"{uav.name} left the operating area", uav.uav_id)
            obstacle = self.env.obstacles.inside(uav.position)
            if obstacle is not None:
                key = ("obstacle", uav.uav_id, obstacle.obstacle_id)
                active.add(key)
                if key not in self._active:
                    self._violation(key, f"{uav.name} inside obstacle {obstacle.obstacle_id}", uav.uav_id)
                self._climb_clear(uav, obstacle)
        self._active = active
        self._deconflicted &= near     # a pair that separated may be deconflicted again later

    def _deconflict(self, a: UAV, b: UAV) -> None:
        """Move the higher-numbered UAV to a clear altitude when two converge at similar heights."""
        pair = (a.uav_id, b.uav_id)
        sep = self.params.min_separation_m
        if pair in self._deconflicted or abs(a.position[2] - b.position[2]) >= sep:
            return
        mover, other = (b, a) if b.uav_id > a.uav_id else (a, b)
        ceiling = self.world.params.uav.max_altitude_m
        up = other.position[2] + 2.0 * sep
        new_alt = up if up <= ceiling else max(other.position[2] - 2.0 * sep, 5.0)
        try:
            self.world.set_altitude(mover.uav_id, new_alt, f"deconflict with {other.name}")
        except Exception:   # landing / no waypoint: nothing to adjust, the monitor still records it
            return
        self._deconflicted.add(pair)
        self.deconflictions += 1

    def _climb_clear(self, uav: UAV, obstacle) -> None:
        """Immediate avoidance: climb above an obstacle that appeared under this UAV."""
        safe_alt = min(obstacle.height_m + self.params.obstacle_clearance_m, self.world.params.uav.max_altitude_m)
        if uav.target is not None and uav.target[2] >= safe_alt - 0.5:
            return
        try:
            self.world.goto(uav.uav_id, (uav.position[0], uav.position[1], safe_alt), "obstacle avoidance")
        except Exception:  # FAILED / RETURNING UAVs cannot be re-tasked; the metric still records it
            pass

    def _violation(self, key: tuple, message: str, uav_id: int) -> None:
        self.violations[key[0]] += 1
        self.world.publish(EventType.SAFETY_VIOLATION, message, severity=Severity.CRITICAL, uav_id=uav_id,
                           data={"kind": key[0]})
