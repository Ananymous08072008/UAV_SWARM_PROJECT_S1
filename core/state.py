"""
core/state.py
The single authoritative WorldState plus immutable snapshots of it.

Project rule: ONE WorldState is shared by simulation, swarm logic, dashboard
and telemetry. Only the World (core/world.py) mutates it. Code that runs
outside the simulation loop - the dashboard server, the MAVLink gateway -
reads WorldSnapshot objects, never the live UAV/PoI objects.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Optional

import numpy as np

from core.config import Area, GeoOrigin
from core.poi import PoI, PoIManager
from core.uav import UAV, UAVRole


def _r(value: float, ndigits: int = 3) -> float:
    return round(float(value), ndigits)


@dataclass(frozen=True)
class UAVSnapshot:
    uav_id: int
    name: str
    role: str
    mode: str
    health: str
    # local ENU (simulation) and geodetic (GCS display) position
    x_m: float
    y_m: float
    z_m: float
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    home_m: tuple[float, float, float]
    # motion
    vx_mps: float
    vy_mps: float
    vz_mps: float
    ground_speed_mps: float
    heading_deg: float
    airborne: bool
    # energy / tasking
    battery_pct: float
    assigned_poi: Optional[str]
    target_m: Optional[tuple[float, float, float]]
    # swarm research network
    connected: bool
    neighbours: tuple[int, ...]
    next_hop: Optional[int]
    route: tuple[int, ...]
    hop_count: Optional[int]
    gcs_link_quality: float
    pdr: float
    latency_ms: Optional[float]
    radio_health: float
    # metrics
    distance_travelled_m: float
    flight_time_s: float

    @classmethod
    def from_uav(cls, uav: UAV, origin: GeoOrigin) -> "UAVSnapshot":
        x, y, z = (float(v) for v in uav.position)
        lat, lon, alt = origin.to_geodetic(x, y, z)
        vx, vy, vz = (float(v) for v in uav.velocity)
        c = uav.comm
        return cls(
            uav_id=uav.uav_id, name=uav.name,
            role=uav.role.value, mode=uav.mode.value, health=uav.health.value,
            x_m=_r(x), y_m=_r(y), z_m=_r(z),
            lat_deg=round(lat, 7), lon_deg=round(lon, 7), alt_msl_m=_r(alt),
            home_m=tuple(_r(v) for v in uav.home),
            vx_mps=_r(vx), vy_mps=_r(vy), vz_mps=_r(vz),
            ground_speed_mps=_r(uav.ground_speed_mps), heading_deg=_r(uav.heading_deg, 1),
            airborne=uav.is_airborne,
            battery_pct=_r(uav.battery_pct, 2),
            assigned_poi=uav.assigned_poi,
            target_m=None if uav.target is None else tuple(_r(v) for v in uav.target),
            connected=c.connected,
            neighbours=tuple(sorted(c.neighbours)),
            next_hop=c.next_hop,
            route=tuple(c.route),
            hop_count=c.hop_count,
            gcs_link_quality=_r(c.gcs_link_quality),
            pdr=_r(c.pdr),
            latency_ms=None if c.latency_ms is None else _r(c.latency_ms, 1),
            radio_health=_r(c.radio_health),
            distance_travelled_m=_r(uav.distance_travelled_m, 1),
            flight_time_s=_r(uav.flight_time_s, 1),
        )


@dataclass(frozen=True)
class PoISnapshot:
    poi_id: str
    x_m: float
    y_m: float
    lat_deg: float
    lon_deg: float
    priority: int
    status: str
    assigned_uav: Optional[int]
    progress_ratio: float
    survey_time_s: float
    created_at_s: float
    completed_at_s: Optional[float]
    completed_by: Optional[int]
    completed_early: bool

    @classmethod
    def from_poi(cls, poi: PoI, origin: GeoOrigin) -> "PoISnapshot":
        x, y = float(poi.position[0]), float(poi.position[1])
        lat, lon, _ = origin.to_geodetic(x, y, 0.0)
        return cls(
            poi_id=poi.poi_id, x_m=_r(x), y_m=_r(y), lat_deg=round(lat, 7), lon_deg=round(lon, 7),
            priority=poi.priority, status=poi.status.value, assigned_uav=poi.assigned_uav,
            progress_ratio=_r(poi.progress_ratio), survey_time_s=poi.survey_time_s,
            created_at_s=_r(poi.created_at_s), completed_at_s=poi.completed_at_s,
            completed_by=poi.completed_by, completed_early=poi.completed_early,
        )


@dataclass(frozen=True)
class WorldSnapshot:
    """Immutable, JSON-ready picture of the world at one instant."""

    t_s: float
    tick: int
    duration_s: float
    gcs_m: tuple[float, float, float]
    gcs_lat_deg: float
    gcs_lon_deg: float
    uavs: tuple[UAVSnapshot, ...]
    pois: tuple[PoISnapshot, ...]
    last_event_seq: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class WorldState:
    """Authoritative, mutable world state. Mutated only by core.world.World."""

    origin: GeoOrigin
    area: Area
    gcs_position: np.ndarray
    time_s: float = 0.0
    tick: int = 0
    uavs: dict[int, UAV] = field(default_factory=dict)
    pois: PoIManager = field(default_factory=PoIManager)

    def add_uav(self, uav: UAV) -> UAV:
        if uav.uav_id in self.uavs:
            raise ValueError(f"duplicate UAV id {uav.uav_id}")
        self.uavs[uav.uav_id] = uav
        return uav

    def get_uav(self, uav_id: int) -> UAV:
        try:
            return self.uavs[uav_id]
        except KeyError:
            raise KeyError(f"unknown UAV id {uav_id}") from None

    def operational_uavs(self) -> list[UAV]:
        return [u for u in self.uavs.values() if u.is_operational]

    def uavs_with_role(self, *roles: UAVRole) -> list[UAV]:
        return [u for u in self.uavs.values() if u.role in roles and u.is_operational]

    def snapshot(self, duration_s: float, last_event_seq: int = 0) -> WorldSnapshot:
        gx, gy, gz = (float(v) for v in self.gcs_position)
        glat, glon, _ = self.origin.to_geodetic(gx, gy, gz)
        return WorldSnapshot(
            t_s=_r(self.time_s),
            tick=self.tick,
            duration_s=duration_s,
            gcs_m=(gx, gy, gz),
            gcs_lat_deg=round(glat, 7),
            gcs_lon_deg=round(glon, 7),
            uavs=tuple(UAVSnapshot.from_uav(u, self.origin) for u in sorted(self.uavs.values(), key=lambda u: u.uav_id)),
            pois=tuple(PoISnapshot.from_poi(p, self.origin) for p in sorted(self.pois, key=lambda p: p.poi_id)),
            last_event_seq=last_event_seq,
        )
