"""
swarm/safety_manager.py
Keeps the mission safe and on time.

Collisions
  * two airborne UAVs closer than ``min_separation_m`` collide: both are lost

Prevention
  * flight levels: every ``layer_spacing_m`` from the floor up to the ceiling
    (20, 40, 60, 80, 100 m). Levels are at least the minimum separation apart,
    so UAVs on different levels can never collide. Each waypoint gets the level
    its role prefers (survey 40, relay 60, return home 80), moved to another
    level if a different UAV is stationed within ``alert_distance_m`` of it
  * obstacle clearance: a waypoint's level is raised above any obstacle the
    straight path crosses
  * mission deadline: every UAV is recalled early enough to land before the
    allotted mission time ends; tasks that cannot finish in time are not given

Avoidance (every tick, after the swarm's decisions and before anything moves)
  Every airborne UAV's plan is projected ``conflict_horizon_s`` ahead,
  including a returning UAV's landing descent. A pair predicted to come within
  the alert distance horizontally (the minimum separation now, growing to
  ``alert_distance_m`` further ahead) while less than the minimum separation
  apart vertically is a conflict. One UAV gives way - the moving one before a
  stationary one, otherwise the higher id; a UAV touching down never does. It
  takes the least disruptive plan that stays clear of everyone's projected
  paths: carry on, change level, hold position, both, or stop dead. Conflicts
  are handled most urgent first and each decision is seen by the next. A UAV
  that gave way returns to its planned level once that is clear.

Monitoring (SAFETY_VIOLATION events + counters)
  * collisions, geofence (operating area), flying inside an obstacle
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional, Sequence

import numpy as np

from core.config import ConfigError
from core.events import EventType, Severity
from core.uav import UAV, UAVRole
from core.world import CommandError

if TYPE_CHECKING:
    from core.world import World
    from simulation.environment import Environment
    from swarm.role_manager import RoleManager

_SAMPLE_S = 0.5      # time step of the path projection
_EPS_M = 1e-3        # two UAVs exactly one level apart are separated, not in conflict
_RESUME = "back to planned level"
_UNSTICK_S = 5.0     # held this long without being able to carry on: try another level


@dataclass(frozen=True)
class SafetyParams:
    min_separation_m: float = 20.0
    altitude_floor_m: float = 20.0
    layer_spacing_m: float = 20.0
    alert_distance_m: float = 30.0
    conflict_horizon_s: float = 12.0
    obstacle_clearance_m: float = 10.0
    recall_at_deadline: bool = True
    recall_margin_s: float = 30.0

    def __post_init__(self) -> None:
        if self.min_separation_m <= 0 or self.altitude_floor_m < 0 or self.conflict_horizon_s <= 0:
            raise ValueError("invalid safety parameters")
        if self.layer_spacing_m < self.min_separation_m:
            raise ValueError("layer_spacing_m must be >= min_separation_m, "
                             "or UAVs on neighbouring levels would collide")
        if self.alert_distance_m < self.min_separation_m:
            raise ValueError("alert_distance_m must be >= min_separation_m")


class SafetyManager:
    def __init__(self, world: "World", env: "Environment", params: SafetyParams) -> None:
        self.world = world
        self.env = env
        self.params = params
        self.violations: Counter[str] = Counter()
        self.min_separation_observed_m = math.inf
        self._active: set[tuple] = set()
        self.recall_started = False
        self.collisions = 0
        self.deconflictions = 0
        # UAVs moved off their level: uav_id -> (planned level, waypoint x/y, level we commanded instead)
        self._planned_level: dict[int, tuple[float, np.ndarray, float]] = {}
        self._held_since: dict[int, float] = {}           # uav_id -> when it started holding position

        ceiling = world.params.uav.max_altitude_m
        if params.altitude_floor_m > ceiling:
            raise ConfigError(f"swarm.safety.altitude_floor_m ({params.altitude_floor_m:g} m) "
                              f"is above uav.max_altitude_m ({ceiling:g} m)")
        count = int((ceiling - params.altitude_floor_m) / params.layer_spacing_m + 1e-9) + 1
        self.levels = tuple(params.altitude_floor_m + k * params.layer_spacing_m for k in range(count))

        spawn = world.scenario.uavs
        if len(world.state.uavs) > 1 and spawn.spacing_m < params.min_separation_m:
            raise ConfigError(f"uavs.spacing_m ({spawn.spacing_m:g} m) is below swarm.safety.min_separation_m "
                              f"({params.min_separation_m:g} m): neighbouring UAVs would collide on take-off")

    # -------------------------------------------------------------- altitudes
    def _levels_above(self, lowest_m: float) -> list[float]:
        return [lv for lv in self.levels if lv >= lowest_m - 1e-6] or [self.world.params.uav.max_altitude_m]

    def _obstacle_floor(self, start: Sequence[float], end: Sequence[float]) -> float:
        tallest = self.env.obstacles.max_height_crossed(start, end)
        return tallest + self.params.obstacle_clearance_m if tallest > 0 else 0.0

    def safe_altitude(self, uav: UAV, target_xy: Sequence[float], preferred_m: Optional[float] = None) -> float:
        """The flight level for a waypoint: closest to ``preferred_m``, above obstacles on the way,
        and not on the level of another UAV stationed within the alert distance."""
        preferred = self.world.params.uav.default_altitude_m if preferred_m is None else preferred_m
        target = (float(target_xy[0]), float(target_xy[1]), 0.0)
        levels = sorted(self._levels_above(self._obstacle_floor(uav.position, target)),
                        key=lambda lv: (abs(lv - preferred), lv))
        return next((lv for lv in levels if self._station_free(uav, target, lv)), levels[0])

    def _station_free(self, uav: UAV, xy: Sequence[float], level: float) -> bool:
        p = self.params
        for other in self.world.state.operational_uavs():
            if other is uav:
                continue
            station = other.target if other.target is not None else (other.position if other.is_airborne else None)
            if station is None:
                continue
            if math.hypot(station[0] - xy[0], station[1] - xy[1]) < p.alert_distance_m \
                    and abs(station[2] - level) < p.min_separation_m - _EPS_M:
                return False
        return True

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
    def avoid(self) -> None:
        """Check every airborne UAV's plan for the next seconds and resolve predicted collisions."""
        self._avoid([u for u in self.world.state.operational_uavs() if u.is_airborne])

    def monitor(self) -> None:
        """Collisions, geofence and obstacle checks on where the UAVs are now."""
        world = self.world
        self._check_collisions([u for u in world.state.operational_uavs() if u.is_airborne])

        active: set[tuple] = set()
        area = world.state.area
        for uav in world.state.operational_uavs():
            if not uav.is_airborne:
                continue
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

    def _check_collisions(self, airborne: list[UAV]) -> None:
        if len(airborne) < 2:
            return
        pos = np.array([u.position for u in airborne])
        dist = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=-1)
        upper = np.triu_indices(len(airborne), 1)
        self.min_separation_observed_m = min(self.min_separation_observed_m, float(dist[upper].min()))
        sep = self.params.min_separation_m
        lost: dict[int, str] = {}
        for i, j in np.argwhere(np.triu(dist < sep - 1e-6, 1)):
            a, b = airborne[i], airborne[j]
            self.collisions += 1
            self._violation(("separation", a.uav_id, b.uav_id),
                            f"{a.name} and {b.name} collided: {dist[i, j]:.1f} m apart (minimum {sep:g} m)",
                            a.uav_id)
            lost.setdefault(a.uav_id, f"mid-air collision with {b.name}")
            lost.setdefault(b.uav_id, f"mid-air collision with {a.name}")
        for uav_id, reason in lost.items():
            self.world.fail_uav(uav_id, reason)
            self._planned_level.pop(uav_id, None)

    # --------------------------------------------------------------- avoidance
    def _project(self, uav: UAV, level: float, brake: bool) -> tuple[np.ndarray, np.ndarray]:
        """Where ``uav`` will be over the horizon if it flies to ``level`` and either carries on to its
        waypoint (accelerating to cruise, x/y only after a take-off climb) or holds position (``brake``)."""
        p, times = self.world.params.uav, self._times
        accel, pos, vel = p.max_accel_mps2, uav.position[:2], uav.velocity[:2]
        z0 = float(uav.position[2])
        dz = level - z0
        z = z0 + np.sign(dz) * np.minimum(abs(dz), p.climb_rate_mps * times)
        delta = None if uav.target is None else uav.target[:2] - pos
        dist = 0.0 if delta is None else float(np.hypot(*delta))
        if brake or delta is None:
            speed = float(np.hypot(*vel))
            if speed < 1e-6:
                return np.repeat(pos[None, :], len(times), axis=0), z
            t_stop = speed / accel
            s = np.where(times < t_stop, speed * times - 0.5 * accel * times ** 2, speed * t_stop / 2.0)
            return pos + (vel / speed)[None, :] * s[:, None], z
        if dist < 1e-6:
            xy = np.repeat(pos[None, :], len(times), axis=0)
            arrive_s = 0.0
        else:
            direction = delta / dist
            v0 = min(max(0.0, float(np.dot(vel, direction))), p.cruise_speed_mps)
            t = np.maximum(0.0, times - (abs(dz) / p.climb_rate_mps if uav.climbing_out else 0.0))
            t_acc = (p.cruise_speed_mps - v0) / accel
            s = np.where(t < t_acc, v0 * t + 0.5 * accel * t ** 2,
                         v0 * t_acc + 0.5 * accel * t_acc ** 2 + p.cruise_speed_mps * (t - t_acc))
            xy = pos + direction[None, :] * np.minimum(s, dist)[:, None]
            # Momentum not pointing at the waypoint (e.g. just re-tasked) carries on until braked off.
            drift = vel - v0 * direction
            drift_speed = float(np.hypot(*drift))
            if drift_speed > 1e-6:
                t_stop = drift_speed / accel
                d = np.where(times < t_stop, drift_speed * times - 0.5 * accel * times ** 2, drift_speed * t_stop / 2.0)
                xy = xy + (drift / drift_speed)[None, :] * d[:, None]
            reached = s >= dist - p.arrival_radius_m
            arrive_s = float(times[np.argmax(reached)]) if reached.any() else math.inf
        if uav.role is UAVRole.RETURNING and float(np.hypot(*(uav.target[:2] - uav.home[:2]))) < 1.0:
            # Over the pad it lands: that descent crosses every level below.
            down_s = max(arrive_s, abs(dz) / p.climb_rate_mps)
            z = np.where(times > down_s, np.maximum(0.0, level - p.climb_rate_mps * (times - down_s)), z)
        return xy, z

    def _avoid(self, uavs: list[UAV]) -> None:
        for uav in self.world.state.operational_uavs():
            if uav.braking and not uav.is_airborne:
                self.world.set_brake(uav.uav_id, False)
        self._held_since = {i: t for i, t in self._held_since.items() if self.world.state.uavs[i].braking}
        for uav_id, (_, xy, commanded) in list(self._planned_level.items()):
            uav = self.world.state.uavs.get(uav_id)
            if (uav is None or not self._can_manoeuvre(uav) or float(np.hypot(*(uav.target[:2] - xy))) > 1.0
                    or abs(float(uav.target[2]) - commanded) > 1e-6):
                self._planned_level.pop(uav_id)            # re-tasked, landing or landed: that plan is gone
        if not uavs:
            return

        p = self.params
        sep = p.min_separation_m
        self._uavs = uavs
        self._times = np.arange(0.0, p.conflict_horizon_s + 1e-9, _SAMPLE_S)
        # The alert buffer covers prediction error, which grows with look-ahead time.
        self._h_limit = sep + (p.alert_distance_m - sep) * np.minimum(1.0, self._times / 4.0)
        plans = [self._project(u, self._level(u), u.braking) for u in uavs]
        self._xy = np.stack([pl[0] for pl in plans])                # (n, S, 2)
        self._z = np.stack([pl[1] for pl in plans])                 # (n, S)
        self._still = np.array([self._is_still(xy) for xy in self._xy])

        # Back on track: release holds and return to the planned level once that is safe.
        for k, uav in enumerate(uavs):
            planned = self._planned_level.get(uav.uav_id)
            if (uav.braking or planned) and self._can_manoeuvre(uav):
                level = planned[0] if planned else float(uav.target[2])
                if self._margin(k, level, False) >= 0.0:
                    self._apply(k, level, False, _RESUME)
                elif uav.braking and self.world.t - self._held_since.get(uav.uav_id, self.world.t) >= _UNSTICK_S:
                    # Two UAVs can hold for each other indefinitely; a level change breaks the tie.
                    self._replan(k, other_level_only=True)

        # Conflicts, most urgent first. Each decision updates that UAV's projection,
        # so the decisions after it plan around it.
        n = len(uavs)
        if n < 2:
            return
        hd = np.linalg.norm(self._xy[:, None] - self._xy[None, :], axis=-1)     # (n, n, S)
        vd = np.abs(self._z[:, None] - self._z[None, :])
        both_still = (self._still[:, None] & self._still[None, :])[..., None]
        hit = (hd < np.where(both_still, sep, self._h_limit)) & (vd < sep - _EPS_M)
        hit &= np.triu(np.ones((n, n), dtype=bool), 1)[..., None]
        pairs = sorted((int(np.argmax(hit[i, j])), int(i), int(j)) for i, j in np.argwhere(hit.any(axis=-1)))
        for _, i, j in pairs:
            if self._pair_margin(i, j) >= 0.0:
                continue                                   # an earlier decision already solved it
            order = self._give_way_order(i, j)
            if not any(self._replan(k) for k in order):
                self._least_risky(order)

    @staticmethod
    def _can_manoeuvre(uav: UAV) -> bool:
        """Airborne with a waypoint in the air. Touching down is committed: others go round it."""
        return (uav.is_operational and uav.is_airborne and uav.target is not None
                and uav.target[2] >= 0.5 and uav.role is not UAVRole.CHARGING)

    @staticmethod
    def _level(uav: UAV) -> float:
        return float(uav.target[2]) if uav.target is not None else float(uav.position[2])

    @staticmethod
    def _is_still(xy: np.ndarray) -> bool:
        return float(np.hypot(*(xy[-1] - xy[0]))) < 1.0

    def _clearance(self, xy: np.ndarray, z: np.ndarray, still: bool, others: np.ndarray) -> np.ndarray:
        """Per other UAV: how far the closest predicted moment is from a conflict (< 0 = conflict)."""
        sep = self.params.min_separation_m
        hd = np.linalg.norm(self._xy[others] - xy[None], axis=-1)              # (m, S)
        vd = np.abs(self._z[others] - z[None])
        h_limit = np.where((still & self._still[others])[:, None], sep, self._h_limit[None, :])
        return np.maximum(hd - h_limit, vd - (sep - _EPS_M)).min(axis=1)

    def _pair_margin(self, i: int, j: int) -> float:
        return float(self._clearance(self._xy[i], self._z[i], bool(self._still[i]), np.array([j]))[0])

    def _margin(self, k: int, level: float, brake: bool) -> float:
        """How clear UAV k stays of everyone else's current plans if it adopts (level, brake)."""
        others = np.array([m for m in range(len(self._uavs)) if m != k], dtype=int)
        if len(others) == 0:
            return math.inf
        xy, z = self._project(self._uavs[k], level, brake)
        return float(self._clearance(xy, z, self._is_still(xy), others).min())

    def _closest_approach(self, k: int, level: float, brake: bool) -> float:
        """The nearest UAV k would come to anyone (3-D, metres) with plan (level, brake)."""
        others = [m for m in range(len(self._uavs)) if m != k]
        if not others:
            return math.inf
        xy, z = self._project(self._uavs[k], level, brake)
        hd = np.linalg.norm(self._xy[others] - xy[None], axis=-1)
        return float(np.hypot(hd, self._z[others] - z[None]).min())

    def _give_way_order(self, i: int, j: int) -> list[int]:
        """The moving UAV gives way to a stationary one, otherwise the higher id does."""
        if self._still[i] != self._still[j]:
            first = j if self._still[i] else i
        else:
            first = i if self._uavs[i].uav_id > self._uavs[j].uav_id else j
        return [first, j if first == i else i]

    def _options(self, k: int, other_level_only: bool = False) -> list[tuple[float, bool]]:
        """Plans for UAV k, least disruptive first: carry on, change level, hold position,
        change level while holding, stop dead."""
        uav = self._uavs[k]
        if not self._can_manoeuvre(uav):
            return []
        current = float(uav.target[2])
        levels = sorted((lv for lv in self._levels_above(self._obstacle_floor(uav.position, uav.target))
                         if abs(lv - current) > 1e-6), key=lambda lv: (abs(lv - current), lv))
        if other_level_only:
            return [(lv, False) for lv in levels] + [(lv, True) for lv in levels]
        return ([(current, False)] + [(lv, False) for lv in levels] + [(current, True)]
                + [(lv, True) for lv in levels] + [(float(uav.position[2]), True)])

    def _replan(self, k: int, other_level_only: bool = False) -> bool:
        """Give UAV k the least disruptive plan that keeps it clear of everyone's current plans."""
        for level, brake in self._options(k, other_level_only):
            if self._margin(k, level, brake) >= 0.0:
                return self._apply(k, level, brake, "collision avoidance")
        return False

    def _least_risky(self, order: list[int]) -> None:
        """Nothing is fully clear: of both UAVs' plans (keeping the current one included), take the
        one that keeps everyone furthest apart."""
        best = None
        for k in order:
            current = (self._level(self._uavs[k]), self._uavs[k].braking)
            for rank, option in enumerate([current] + self._options(k)):
                score = (round(self._closest_approach(k, *option), 2), -rank)
                if best is None or score > best[0]:
                    best = (score, k, option, rank == 0)
        if best is not None and not best[3]:             # rank 0 = its current plan: nothing to change
            self._apply(best[1], *best[2], "no fully clear option")

    def _apply(self, k: int, level: float, brake: bool, reason: str) -> bool:
        uav = self._uavs[k]
        before = float(uav.target[2])
        changed = abs(level - before) > 1e-6
        if changed:
            try:
                self.world.set_altitude(uav.uav_id, level, reason)
            except CommandError:
                return False
            planned = self._planned_level.get(uav.uav_id, (before, uav.target[:2].copy(), level))[0]
            if abs(planned - level) < 1e-6:
                self._planned_level.pop(uav.uav_id, None)
            else:
                self._planned_level[uav.uav_id] = (planned, uav.target[:2].copy(), level)
        if reason != _RESUME and (changed or (brake and not uav.braking)):
            self.deconflictions += 1
        if brake and not uav.braking:
            self._held_since[uav.uav_id] = self.world.t
        self.world.set_brake(uav.uav_id, brake, reason)
        self._xy[k], self._z[k] = self._project(uav, level, brake)
        self._still[k] = self._is_still(self._xy[k])
        return True

    def _climb_clear(self, uav: UAV, obstacle) -> None:
        """Immediate avoidance: climb to the first level above an obstacle that appeared under this UAV."""
        safe_alt = self._levels_above(obstacle.height_m + self.params.obstacle_clearance_m)[0]
        if uav.target is not None and uav.target[2] >= safe_alt - 0.5:
            return
        # Keep the destination and only raise the altitude. Retargeting to the
        # current position stranded a UAV in transit: it hovered above the debris
        # still holding its PoI, which only counts survey time directly overhead.
        x, y = (uav.target[:2] if uav.target is not None else uav.position[:2])
        try:
            self.world.goto(uav.uav_id, (float(x), float(y), safe_alt), "obstacle avoidance")
        except Exception:  # FAILED / RETURNING UAVs cannot be re-tasked; the metric still records it
            pass

    def _violation(self, key: tuple, message: str, uav_id: int) -> None:
        self.violations[key[0]] += 1
        self.world.publish(EventType.SAFETY_VIOLATION, message, severity=Severity.CRITICAL, uav_id=uav_id,
                           data={"kind": key[0]})
