"""
core/uav.py
Virtual UAV model: state plus low-level *execution* (fly to a waypoint,
hold, drain battery).

A UAV never decides what to do. Decisions come from the swarm layer and reach
the UAV through the World command API (core/world.py), which validates them
and publishes events. Keeping this split is a core project rule.

Frame: local ENU in metres (x = East, y = North, z = Up), origin at the GCS.
Heading: degrees clockwise from North (0 = N, 90 = E), as MAVLink expects.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np

from core.config import MAX_UAVS, BatteryParams, UAVParams

GCS_NODE_ID = 0     # node id of the ground station in the swarm communication graph
BRAKE_MARGIN = 0.8   # plan braking with 80 % of max acceleration so discrete steps never overshoot
TAKEOFF_ALT_TOL_M = 1.0  # a take-off climbs to its flight level before translating


class UAVRole(str, Enum):
    IDLE = "IDLE"            # available, no task
    SURVEY = "SURVEY"        # flying to / surveying a PoI
    RELAY = "RELAY"          # positioned to bridge the swarm network to the GCS
    BACKUP = "BACKUP"        # airborne spare parked near the network for fast replacement
    RETURNING = "RETURNING"  # return-to-home in progress
    CHARGING = "CHARGING"    # landed on its home pad, recharging


TASKABLE_ROLES = frozenset({UAVRole.IDLE, UAVRole.BACKUP})


class FlightMode(str, Enum):
    HOLD = "HOLD"              # no waypoint: hover in place (or stay on the ground)
    TRANSIT = "TRANSIT"        # flying toward the commanded waypoint
    ON_STATION = "ON_STATION"  # waypoint reached, station-keeping


class HealthState(str, Enum):
    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"  # still flying but impaired (set by later fault models)
    FAILED = "FAILED"      # out of the mission: battery depleted or injected loss


@dataclass
class CommState:
    """This UAV's view of the *swarm research network*.

    Filled in by simulation/communication.py (Stage 2). This is NOT the MAVLink
    telemetry link to Mission Planner - the two networks are kept separate.
    """

    neighbours: set[int] = field(default_factory=set)
    connected: bool = False             # has a multi-hop route to the GCS
    next_hop: Optional[int] = None      # next node toward the GCS (GCS_NODE_ID = direct)
    route: tuple[int, ...] = ()         # full path to the GCS, e.g. (5, 3, 0)
    hop_count: Optional[int] = None
    gcs_link_quality: float = 0.0       # 0..1, quality of the weakest link on the route
    pdr: float = 0.0                    # end-to-end packet delivery ratio to the GCS, 0..1
    latency_ms: Optional[float] = None  # end-to-end latency to the GCS
    radio_health: float = 1.0           # ground-truth radio condition (fault injection), 0..1


@dataclass(frozen=True)
class StepReport:
    """Edge-triggered things that happened during one ``UAV.step``."""

    arrived: bool = False
    battery_low: bool = False
    battery_critical: bool = False
    battery_depleted: bool = False


def drain_rate_pct_per_min(ground_speed_mps: float, uav_p: UAVParams, battery_p: BatteryParams) -> float:
    """Battery drain of an airborne UAV. Shared by the UAV and simulation/battery.py estimates."""
    speed_ratio = min(ground_speed_mps / uav_p.cruise_speed_mps, 1.5)
    return battery_p.hover_drain_pct_per_min + battery_p.cruise_drain_pct_per_min * speed_ratio


def _as_vec3(value) -> np.ndarray:
    vec = np.asarray(value, dtype=float).reshape(-1)
    if vec.shape == (2,):
        vec = np.append(vec, 0.0)
    if vec.shape != (3,) or not np.all(np.isfinite(vec)):
        raise ValueError(f"expected a finite (x, y[, z]) position, got {value!r}")
    return vec


@dataclass(eq=False)  # identity equality: numpy fields make value equality ambiguous
class UAV:
    uav_id: int
    position: np.ndarray
    name: str = ""
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    heading_deg: float = 0.0
    battery_pct: float = 100.0
    role: UAVRole = UAVRole.IDLE
    mode: FlightMode = FlightMode.HOLD
    health: HealthState = HealthState.HEALTHY
    target: Optional[np.ndarray] = None
    assigned_poi: Optional[str] = None
    comm: CommState = field(default_factory=CommState)
    distance_travelled_m: float = 0.0
    flight_time_s: float = 0.0
    climbing_out: bool = False   # taking off: climb to the flight level before flying on
    home: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        if not 1 <= self.uav_id <= MAX_UAVS:
            raise ValueError(f"uav_id must be 1..{MAX_UAVS} (it doubles as the MAVLink system id)")
        if not 0.0 <= self.battery_pct <= 100.0:
            raise ValueError("battery_pct must be within 0..100")
        self.position = _as_vec3(self.position)
        self.velocity = _as_vec3(self.velocity)
        self.home = self.position.copy()
        if self.target is not None:
            self.target = _as_vec3(self.target)
        if not self.name:
            self.name = f"UAV-{self.uav_id:02d}"

    # ------------------------------------------------------------------ status
    @property
    def is_operational(self) -> bool:
        return self.health is not HealthState.FAILED

    @property
    def is_airborne(self) -> bool:
        return bool(self.position[2] > 0.1)

    @property
    def ground_speed_mps(self) -> float:
        return float(math.hypot(self.velocity[0], self.velocity[1]))

    def distance_to(self, point) -> float:
        return float(np.linalg.norm(_as_vec3(point) - self.position))

    def horizontal_distance_to(self, point) -> float:
        p = _as_vec3(point)
        return float(math.hypot(p[0] - self.position[0], p[1] - self.position[1]))

    # ---------------------------------------------------------------- commands
    def goto(self, target) -> None:
        """Fly to ``target`` (x, y, z) and station-keep there."""
        if not self.is_operational:
            raise RuntimeError(f"{self.name} is FAILED and cannot accept commands")
        waypoint = _as_vec3(target)
        if waypoint[2] < 0:
            raise ValueError(f"waypoint altitude must be >= 0, got {waypoint[2]}")
        self.climbing_out = not self.is_airborne and waypoint[2] > self.position[2] + TAKEOFF_ALT_TOL_M
        self.target = waypoint
        self.mode = FlightMode.TRANSIT

    def hold(self) -> None:
        """Drop the waypoint and hover where the UAV is."""
        self.target = None
        self.mode = FlightMode.HOLD

    def mark_failed(self) -> None:
        """Out of the mission: forced landing / crash at the current x, y."""
        self.health = HealthState.FAILED
        self.target = None
        self.mode = FlightMode.HOLD
        self.velocity[:] = 0.0
        self.position[2] = 0.0

    # --------------------------------------------------------------- execution
    def step(self, dt: float, uav_p: UAVParams, battery_p: BatteryParams) -> StepReport:
        """Advance this UAV by ``dt`` seconds."""
        if not self.is_operational:
            return StepReport()
        arrived = self._integrate_motion(dt, uav_p)
        low, critical, depleted = self._drain_battery(dt, uav_p, battery_p)
        return StepReport(arrived, low, critical, depleted)

    def _desired_velocity(self, p: UAVParams) -> np.ndarray:
        desired = np.zeros(3)
        if self.target is None:
            return desired  # hover / stay put
        delta = self.target - self.position
        dist_h = math.hypot(delta[0], delta[1])
        brake = BRAKE_MARGIN * p.max_accel_mps2
        if self.climbing_out and delta[2] <= TAKEOFF_ALT_TOL_M:
            self.climbing_out = False
        if dist_h > 1e-6 and not self.climbing_out:
            # Cruise, then brake along v = sqrt(2*a*d) so the UAV can stop on the waypoint.
            speed = min(p.cruise_speed_mps, math.sqrt(2.0 * brake * dist_h))
            desired[:2] = delta[:2] / dist_h * speed
        dz = delta[2]
        if abs(dz) > 1e-6:
            desired[2] = math.copysign(min(p.climb_rate_mps, math.sqrt(2.0 * brake * abs(dz))), dz)
        return desired

    def _integrate_motion(self, dt: float, p: UAVParams) -> bool:
        # Acceleration-limited velocity change (horizontal and vertical budgets are
        # independent) -> smooth, plausible trajectories.
        desired = self._desired_velocity(p)
        max_dv = p.max_accel_mps2 * dt
        dv_h = desired[:2] - self.velocity[:2]
        dv_h_norm = math.hypot(dv_h[0], dv_h[1])
        if dv_h_norm > max_dv:
            dv_h *= max_dv / dv_h_norm
        self.velocity[:2] += dv_h
        self.velocity[2] += float(np.clip(desired[2] - self.velocity[2], -max_dv, max_dv))

        displacement = self.velocity * dt
        if self.target is not None:
            to_target = self.target - self.position
            # Snap onto the waypoint instead of stepping past it (per axis group).
            if (np.dot(displacement[:2], to_target[:2]) > 0
                    and math.hypot(*displacement[:2]) >= math.hypot(*to_target[:2])):
                displacement[:2] = to_target[:2]
                self.velocity[:2] = 0.0
            if displacement[2] * to_target[2] > 0 and abs(displacement[2]) >= abs(to_target[2]):
                displacement[2] = to_target[2]
                self.velocity[2] = 0.0

        self.position += displacement
        if self.position[2] < 0.0:  # ground
            self.position[2] = 0.0
            self.velocity[2] = max(0.0, self.velocity[2])
        self.distance_travelled_m += float(np.linalg.norm(displacement))
        if self.is_airborne:
            self.flight_time_s += dt
        if self.ground_speed_mps >= max(p.heading_min_speed_mps, 1e-6):
            self.heading_deg = math.degrees(math.atan2(self.velocity[0], self.velocity[1])) % 360.0

        if (self.mode is FlightMode.TRANSIT and self.target is not None
                and np.linalg.norm(self.target - self.position) <= p.arrival_radius_m):
            self.mode = FlightMode.ON_STATION
            return True
        return False

    def _drain_battery(self, dt: float, uav_p: UAVParams, p: BatteryParams) -> tuple[bool, bool, bool]:
        if not self.is_airborne:
            return False, False, False
        before = self.battery_pct
        self.battery_pct = max(0.0, before - drain_rate_pct_per_min(self.ground_speed_mps, uav_p, p) * dt / 60.0)
        after = self.battery_pct
        return (
            before > p.low_pct >= after,
            before > p.critical_pct >= after,
            before > 0.0 >= after,
        )
