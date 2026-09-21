"""
dashboard/mission.py
Turn a mission description sent by the browser into a validated ScenarioConfig.

The mission builder in the web UI posts something like::

    {"name": "my mission", "uav_count": 12, "duration_s": 600, "seed": 42,
     "faults": true,
     "pois": [{"x_m": 500, "y_m": 300, "priority": 4, "survey_time_s": 30}]}

Everything is optional. Missing fields fall back to the template scenario
(config/scenario.yaml), so an empty spec reproduces the standard demo.

Anything invalid raises ConfigError, which the API turns into a 400 with the
message shown to the operator - ScenarioConfig already checks that PoIs and UAV
spawns sit inside the area, that PoI ids are unique and that triggers fire
within the duration, so this module only adds the limits that protect a shared
server from a hostile or careless spec.
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any, Mapping, Optional

from core.config import ConfigError, PoISpec, ScenarioConfig, SpawnConfig, _require

PROJECT_ROOT = Path(__file__).resolve().parent.parent
TEMPLATE_SCENARIO = PROJECT_ROOT / "config" / "scenario.yaml"

# Ceilings for a server that strangers can post to. A 60-UAV 2-hour mission is a
# denial of service, not a research question.
MAX_UAVS = 40
MIN_UAVS = 1
MAX_POIS = 25
MAX_DURATION_S = 3600.0
MIN_DURATION_S = 10.0

_template: Optional[ScenarioConfig] = None


def template() -> ScenarioConfig:
    """The stock scenario, loaded once, used as the base for custom missions."""
    global _template
    if _template is None:
        _template = ScenarioConfig.load(TEMPLATE_SCENARIO)
    return _template


def _int(spec: Mapping[str, Any], key: str, default: int) -> int:
    value = spec.get(key, default)
    try:
        return int(value)
    except (TypeError, ValueError):
        raise ConfigError(f"'{key}' must be a whole number, got {value!r}") from None


def _float(spec: Mapping[str, Any], key: str, default: float) -> float:
    value = spec.get(key, default)
    try:
        return float(value)
    except (TypeError, ValueError):
        raise ConfigError(f"'{key}' must be a number, got {value!r}") from None


def _spawn(base: SpawnConfig, count: int) -> SpawnConfig:
    """Re-flow the launch grid so any UAV count keeps a sensible footprint."""
    per_row = min(count, max(1, base.per_row))
    return replace(base, count=count, per_row=per_row)


def _pois(raw: Any, base: ScenarioConfig) -> tuple[PoISpec, ...]:
    """Build PoIs from the map clicks. An empty list keeps the template's PoIs."""
    if raw is None:
        return base.pois
    if not isinstance(raw, (list, tuple)):
        raise ConfigError("'pois' must be a list")
    if not raw:
        return ()
    _require(len(raw) <= MAX_POIS, f"at most {MAX_POIS} PoIs (got {len(raw)})")

    out: list[PoISpec] = []
    for i, item in enumerate(raw, start=1):
        if not isinstance(item, Mapping):
            raise ConfigError(f"PoI #{i} must be an object")
        try:
            x = float(item.get("x_m"))
            y = float(item.get("y_m"))
        except (TypeError, ValueError):
            raise ConfigError(f"PoI #{i} needs numeric 'x_m' and 'y_m'") from None
        priority = _int(item, "priority", 3)
        _require(1 <= priority <= 5, f"PoI #{i} priority must be 1..5 (got {priority})")
        survey = _float(item, "survey_time_s", 45.0)
        _require(0 < survey <= 600, f"PoI #{i} survey_time_s must be within 0..600 (got {survey:g})")
        out.append(PoISpec(id=str(item.get("id") or f"POI-{i}"),
                           position_m=(x, y), priority=priority, survey_time_s=survey))
    return tuple(out)


def _timeline(base: ScenarioConfig, poi_ids: set[str], duration_s: float, faults: bool):
    """
    Keep the demonstration faults, drop the ones that cannot apply.

    The stock timeline references specific PoIs (``complete_poi POI-5``). Once the
    operator supplies their own PoIs those triggers would just log
    TRIGGER_REJECTED, so they are filtered out rather than left to fail.
    """
    if not faults:
        return ()
    kept = []
    for trig in base.timeline:
        if trig.at_s > duration_s:
            continue
        referenced = trig.params.get("poi_id")
        if referenced is not None and referenced not in poi_ids:
            continue
        kept.append(trig)
    return tuple(kept)


def build_scenario(spec: Optional[Mapping[str, Any]] = None) -> ScenarioConfig:
    """Validated ScenarioConfig from a browser mission spec. Raises ConfigError."""
    spec = dict(spec or {})
    base = template()

    uav_count = _int(spec, "uav_count", base.uavs.count)
    _require(MIN_UAVS <= uav_count <= MAX_UAVS,
             f"uav_count must be {MIN_UAVS}..{MAX_UAVS} (got {uav_count})")

    duration_s = _float(spec, "duration_s", base.duration_s)
    _require(MIN_DURATION_S <= duration_s <= MAX_DURATION_S,
             f"duration_s must be {MIN_DURATION_S:g}..{MAX_DURATION_S:g} (got {duration_s:g})")

    pois = _pois(spec.get("pois"), base)
    _require(pois or base.random_pois.count > 0, "a mission needs at least one PoI")

    name = str(spec.get("name") or "custom").strip()[:40] or "custom"
    faults = bool(spec.get("faults", True))

    return replace(
        base,
        name=name,
        description=str(spec.get("description") or "Built in the mission builder")[:300],
        duration_s=duration_s,
        seed=_int(spec, "seed", base.seed if base.seed is not None else 42),
        uavs=_spawn(base.uavs, uav_count),
        pois=pois,
        timeline=_timeline(base, {p.id for p in pois}, duration_s, faults),
    )


def area_bounds() -> dict[str, float]:
    """The map extent the UI must keep PoI clicks inside."""
    a = template().area
    return {"x_min_m": a.x_min_m, "x_max_m": a.x_max_m,
            "y_min_m": a.y_min_m, "y_max_m": a.y_max_m}


def limits() -> dict[str, Any]:
    """Published to the UI so the form can constrain input before posting."""
    return {"max_uavs": MAX_UAVS, "min_uavs": MIN_UAVS, "max_pois": MAX_POIS,
            "min_duration_s": MIN_DURATION_S, "max_duration_s": MAX_DURATION_S,
            "area": area_bounds()}
