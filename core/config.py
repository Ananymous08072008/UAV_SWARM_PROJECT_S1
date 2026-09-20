"""
core/config.py
Typed, validated loading of config/parameters.yaml and config/scenario.yaml.

This module imports nothing else from the project, so every layer (core,
swarm, simulation, telemetry, dashboard) can depend on it without import
cycles. Sections that later stages add (e.g. ``communication``) are kept in
``extra_sections`` and parsed by the module that owns them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any, Mapping, Optional

import yaml

EARTH_RADIUS_M = 6_378_137.0  # WGS-84 equatorial radius
MAX_UAVS = 250                # MAVLink system IDs 1..250 (255 is the GCS)


class ConfigError(ValueError):
    """A configuration file is missing, malformed or has invalid values."""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def load_yaml(path: str | Path) -> dict[str, Any]:
    """Read a YAML file whose top level must be a mapping."""
    p = Path(path)
    if not p.is_file():
        raise ConfigError(f"Config file not found: {p}")
    with p.open(encoding="utf-8") as fh:
        try:
            data = yaml.safe_load(fh)
        except yaml.YAMLError as exc:
            raise ConfigError(f"{p}: invalid YAML: {exc}") from exc
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ConfigError(f"{p}: top level must be a mapping")
    return data


def build(cls: type, data: Any, section: str) -> Any:
    """Instantiate dataclass ``cls`` from a mapping, rejecting unknown keys."""
    if data is None:
        data = {}
    if not isinstance(data, Mapping):
        raise ConfigError(f"[{section}] must be a mapping, got {type(data).__name__}")
    allowed = {f.name for f in fields(cls) if f.init}
    unknown = set(data) - allowed
    if unknown:
        raise ConfigError(f"[{section}] unknown key(s) {sorted(unknown)}; allowed: {sorted(allowed)}")
    try:
        return cls(**data)
    except ConfigError as exc:
        raise ConfigError(f"[{section}] {exc}") from exc
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"[{section}] invalid value: {exc}") from exc


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ConfigError(message)


def _float_tuple(value: Any, n: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != n:
        raise ConfigError(f"{name} must be a list of {n} numbers, got {value!r}")
    try:
        return tuple(float(v) for v in value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{name} must contain only numbers, got {value!r}") from exc


# ---------------------------------------------------------------------------
# parameters.yaml
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SimulationParams:
    dt_s: float = 0.1
    seed: int = 42
    status_interval_s: float = 10.0
    event_history: int = 2000

    def __post_init__(self) -> None:
        _require(self.dt_s > 0, "dt_s must be > 0")
        _require(isinstance(self.seed, int), "seed must be an integer")
        _require(self.status_interval_s > 0, "status_interval_s must be > 0")
        _require(self.event_history > 0, "event_history must be > 0")


@dataclass(frozen=True)
class GeoOrigin:
    """Geographic anchor of the local ENU frame.

    Uses a flat-earth (equirectangular) approximation, which stays well under
    1 m of error within a few km of the origin - plenty for a swarm area.
    """

    lat_deg: float = -35.363261
    lon_deg: float = 149.165230
    alt_msl_m: float = 584.0

    def __post_init__(self) -> None:
        _require(-89.0 < self.lat_deg < 89.0, "lat_deg must be within (-89, 89)")
        _require(-180.0 <= self.lon_deg <= 180.0, "lon_deg must be within [-180, 180]")

    def to_geodetic(self, x_east_m: float, y_north_m: float, z_up_m: float) -> tuple[float, float, float]:
        """Local ENU metres -> (lat_deg, lon_deg, alt_msl_m)."""
        lat = self.lat_deg + math.degrees(y_north_m / EARTH_RADIUS_M)
        lon = self.lon_deg + math.degrees(x_east_m / (EARTH_RADIUS_M * math.cos(math.radians(self.lat_deg))))
        return lat, lon, self.alt_msl_m + z_up_m

    def to_local(self, lat_deg: float, lon_deg: float, alt_msl_m: float) -> tuple[float, float, float]:
        """(lat_deg, lon_deg, alt_msl_m) -> local ENU metres."""
        y = math.radians(lat_deg - self.lat_deg) * EARTH_RADIUS_M
        x = math.radians(lon_deg - self.lon_deg) * EARTH_RADIUS_M * math.cos(math.radians(self.lat_deg))
        return x, y, alt_msl_m - self.alt_msl_m


@dataclass(frozen=True)
class UAVParams:
    cruise_speed_mps: float = 10.0
    max_accel_mps2: float = 3.0
    climb_rate_mps: float = 3.0
    arrival_radius_m: float = 2.0
    default_altitude_m: float = 30.0
    rth_altitude_m: float = 40.0
    max_altitude_m: float = 120.0
    heading_min_speed_mps: float = 0.5

    def __post_init__(self) -> None:
        for name in ("cruise_speed_mps", "max_accel_mps2", "climb_rate_mps",
                     "arrival_radius_m", "default_altitude_m", "rth_altitude_m"):
            _require(getattr(self, name) > 0, f"{name} must be > 0")
        _require(self.default_altitude_m <= self.max_altitude_m and self.rth_altitude_m <= self.max_altitude_m,
                 "default_altitude_m and rth_altitude_m must be <= max_altitude_m")
        _require(self.heading_min_speed_mps >= 0, "heading_min_speed_mps must be >= 0")


@dataclass(frozen=True)
class BatteryParams:
    hover_drain_pct_per_min: float = 1.2
    cruise_drain_pct_per_min: float = 0.8
    low_pct: float = 30.0
    critical_pct: float = 15.0
    charge_rate_pct_per_min: float = 25.0  # on the home pad (fast charge / battery swap)
    resume_pct: float = 95.0               # charging UAV becomes available again at this level

    def __post_init__(self) -> None:
        _require(self.hover_drain_pct_per_min >= 0, "hover_drain_pct_per_min must be >= 0")
        _require(self.cruise_drain_pct_per_min >= 0, "cruise_drain_pct_per_min must be >= 0")
        _require(0 < self.critical_pct < self.low_pct < 100,
                 "require 0 < critical_pct < low_pct < 100")
        _require(self.charge_rate_pct_per_min > 0, "charge_rate_pct_per_min must be > 0")
        _require(self.low_pct < self.resume_pct <= 100, "resume_pct must be in (low_pct, 100]")


@dataclass(frozen=True)
class Parameters:
    simulation: SimulationParams = field(default_factory=SimulationParams)
    geo_origin: GeoOrigin = field(default_factory=GeoOrigin)
    uav: UAVParams = field(default_factory=UAVParams)
    battery: BatteryParams = field(default_factory=BatteryParams)
    extra_sections: Mapping[str, Any] = field(default_factory=dict)

    _SECTIONS = {
        "simulation": SimulationParams,
        "geo_origin": GeoOrigin,
        "uav": UAVParams,
        "battery": BatteryParams,
    }

    def section(self, name: str) -> dict[str, Any]:
        """Raw dict of a section owned by a later-stage module (empty if absent)."""
        return dict(self.extra_sections.get(name) or {})

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "Parameters":
        known = {name: build(kind, data.get(name), name) for name, kind in cls._SECTIONS.items()}
        extra = {k: v for k, v in data.items() if k not in cls._SECTIONS}
        return cls(**known, extra_sections=extra)

    @classmethod
    def load(cls, path: str | Path) -> "Parameters":
        try:
            return cls.from_dict(load_yaml(path))
        except ConfigError as exc:
            raise ConfigError(f"{Path(path).name}: {exc}") from exc


# ---------------------------------------------------------------------------
# scenario.yaml
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ScenarioMeta:
    name: str = "unnamed"
    description: str = ""
    duration_s: float = 300.0
    seed: Optional[int] = None

    def __post_init__(self) -> None:
        _require(isinstance(self.name, str) and self.name != "", "name must be a non-empty string")
        _require(isinstance(self.description, str), "description must be a string")
        _require(self.duration_s > 0, "duration_s must be > 0")
        _require(self.seed is None or isinstance(self.seed, int), "seed must be an integer or null")


@dataclass(frozen=True)
class Area:
    x_min_m: float = -500.0
    x_max_m: float = 500.0
    y_min_m: float = -500.0
    y_max_m: float = 500.0

    def __post_init__(self) -> None:
        _require(self.x_min_m < self.x_max_m, "x_min_m must be < x_max_m")
        _require(self.y_min_m < self.y_max_m, "y_min_m must be < y_max_m")

    def contains(self, x: float, y: float) -> bool:
        return self.x_min_m <= x <= self.x_max_m and self.y_min_m <= y <= self.y_max_m


@dataclass(frozen=True)
class GCSConfig:
    position_m: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        object.__setattr__(self, "position_m", _float_tuple(self.position_m, 3, "position_m"))


@dataclass(frozen=True)
class SpawnConfig:
    count: int = 6
    formation: str = "grid"
    per_row: int = 3
    spacing_m: float = 8.0
    start_m: tuple[float, float] = (0.0, 0.0)
    initial_battery_pct: float = 100.0
    battery_overrides: Mapping[int, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        _require(isinstance(self.count, int) and 1 <= self.count <= MAX_UAVS,
                 f"count must be an integer in 1..{MAX_UAVS}")
        _require(self.formation in ("grid", "line"), "formation must be 'grid' or 'line'")
        _require(isinstance(self.per_row, int) and self.per_row >= 1, "per_row must be an integer >= 1")
        _require(self.spacing_m > 0, "spacing_m must be > 0")
        _require(0 < self.initial_battery_pct <= 100, "initial_battery_pct must be in (0, 100]")
        object.__setattr__(self, "start_m", _float_tuple(self.start_m, 2, "start_m"))
        overrides = dict(self.battery_overrides or {})
        for uav_id, pct in overrides.items():
            _require(isinstance(uav_id, int) and 1 <= uav_id <= self.count,
                     f"battery_overrides key {uav_id!r} is not a UAV id in 1..{self.count}")
            _require(0 < float(pct) <= 100, f"battery_overrides[{uav_id}] must be in (0, 100]")
        object.__setattr__(self, "battery_overrides", {k: float(v) for k, v in overrides.items()})

    def spawn_positions(self) -> list[tuple[float, float, float]]:
        """Ground positions for UAV ids 1..count, in id order."""
        per_row = self.count if self.formation == "line" else self.per_row
        x0, y0 = self.start_m
        return [
            (x0 + (i % per_row) * self.spacing_m, y0 + (i // per_row) * self.spacing_m, 0.0)
            for i in range(self.count)
        ]

    def battery_for(self, uav_id: int) -> float:
        return self.battery_overrides.get(uav_id, self.initial_battery_pct)


@dataclass(frozen=True)
class PoISpec:
    id: str
    position_m: tuple[float, float]
    priority: int = 1
    survey_time_s: float = 30.0
    altitude_m: Optional[float] = None

    def __post_init__(self) -> None:
        _require(isinstance(self.id, str) and self.id.strip() != "", "id must be a non-empty string")
        object.__setattr__(self, "position_m", _float_tuple(self.position_m, 2, "position_m"))
        _require(isinstance(self.priority, int) and 1 <= self.priority <= 5, "priority must be an integer 1..5")
        _require(self.survey_time_s > 0, "survey_time_s must be > 0")
        _require(self.altitude_m is None or self.altitude_m > 0, "altitude_m must be > 0 or null")


@dataclass(frozen=True)
class RandomPoIConfig:
    count: int = 0
    priority: tuple[int, int] = (1, 5)
    survey_time_s: tuple[float, float] = (30.0, 90.0)
    margin_m: float = 20.0

    def __post_init__(self) -> None:
        _require(isinstance(self.count, int) and self.count >= 0, "count must be an integer >= 0")
        lo, hi = _float_tuple(self.priority, 2, "priority")
        _require(1 <= lo <= hi <= 5 and lo.is_integer() and hi.is_integer(),
                 "priority must be [low, high] integers within 1..5")
        object.__setattr__(self, "priority", (int(lo), int(hi)))
        t_lo, t_hi = _float_tuple(self.survey_time_s, 2, "survey_time_s")
        _require(0 < t_lo <= t_hi, "survey_time_s must be [low, high] with 0 < low <= high")
        object.__setattr__(self, "survey_time_s", (t_lo, t_hi))
        _require(self.margin_m >= 0, "margin_m must be >= 0")


@dataclass(frozen=True)
class TriggerSpec:
    """A timed scenario input as written in YAML; validated by core.events."""

    at_s: float
    action: str
    params: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.params is None:  # "params:" left empty in YAML
            object.__setattr__(self, "params", {})
        _require(isinstance(self.at_s, (int, float)) and self.at_s >= 0, "at_s must be a number >= 0")
        _require(isinstance(self.action, str) and self.action != "", "action must be a non-empty string")
        _require(isinstance(self.params, Mapping), "params must be a mapping")


@dataclass(frozen=True)
class ScenarioConfig:
    name: str = "unnamed"
    description: str = ""
    duration_s: float = 300.0
    seed: Optional[int] = None
    area: Area = field(default_factory=Area)
    gcs: GCSConfig = field(default_factory=GCSConfig)
    uavs: SpawnConfig = field(default_factory=SpawnConfig)
    pois: tuple[PoISpec, ...] = ()
    random_pois: RandomPoIConfig = field(default_factory=RandomPoIConfig)
    timeline: tuple[TriggerSpec, ...] = ()
    extra_sections: Mapping[str, Any] = field(default_factory=dict)

    _KNOWN = ("scenario", "area", "gcs", "uavs", "pois", "random_pois", "timeline")

    def __post_init__(self) -> None:
        _require(self.area.contains(*self.gcs.position_m[:2]), "gcs.position_m is outside the area")
        for i, pos in enumerate(self.uavs.spawn_positions(), start=1):
            _require(self.area.contains(pos[0], pos[1]), f"UAV {i} spawn position {pos[:2]} is outside the area")
        seen: set[str] = set()
        for poi in self.pois:
            _require(poi.id not in seen, f"duplicate PoI id '{poi.id}'")
            seen.add(poi.id)
            _require(self.area.contains(*poi.position_m), f"PoI '{poi.id}' at {poi.position_m} is outside the area")
        margin = self.random_pois.margin_m
        _require(self.random_pois.count == 0
                 or (2 * margin < self.area.x_max_m - self.area.x_min_m
                     and 2 * margin < self.area.y_max_m - self.area.y_min_m),
                 "random_pois.margin_m leaves no room inside the area")
        for trig in self.timeline:
            _require(trig.at_s <= self.duration_s,
                     f"trigger '{trig.action}' at {trig.at_s}s is after the scenario end ({self.duration_s}s)")

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ScenarioConfig":
        meta = build(ScenarioMeta, data.get("scenario"), "scenario")
        pois_raw = data.get("pois") or []
        timeline_raw = data.get("timeline") or []
        _require(isinstance(pois_raw, list), "[pois] must be a list")
        _require(isinstance(timeline_raw, list), "[timeline] must be a list")
        return cls(
            name=meta.name,
            description=meta.description.strip(),
            duration_s=float(meta.duration_s),
            seed=meta.seed,
            area=build(Area, data.get("area"), "area"),
            gcs=build(GCSConfig, data.get("gcs"), "gcs"),
            uavs=build(SpawnConfig, data.get("uavs"), "uavs"),
            pois=tuple(build(PoISpec, p, f"pois[{i}]") for i, p in enumerate(pois_raw)),
            random_pois=build(RandomPoIConfig, data.get("random_pois"), "random_pois"),
            timeline=tuple(build(TriggerSpec, t, f"timeline[{i}]") for i, t in enumerate(timeline_raw)),
            extra_sections={k: v for k, v in data.items() if k not in cls._KNOWN},
        )

    @classmethod
    def load(cls, path: str | Path) -> "ScenarioConfig":
        try:
            return cls.from_dict(load_yaml(path))
        except ConfigError as exc:
            raise ConfigError(f"{Path(path).name}: {exc}") from exc
