"""
dashboard/mission.py
Turn a mission description sent by the browser into a validated ScenarioConfig.

The mission builder in the web UI posts something like::

    {"name": "my mission", "uav_count": "auto", "duration_s": 600, "seed": 42,
     "faults": true,
     "pois": [{"x_m": 500, "y_m": 300, "priority": 4, "survey_time_s": 30}]}

Everything is optional. Missing fields fall back to the template scenario
(config/scenario.yaml), so an empty spec reproduces the standard demo.
``uav_count: "auto"`` (the template's default) sizes the fleet to the PoIs once
they are known; a number fixes it.

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

from core.config import AUTO_COUNT, ConfigError, PoISpec, ScenarioConfig, SpawnConfig, _require, fresh_seed
from core.world import POI_SELECTORS

PROJECT_ROOT = Path(__file__).resolve().parent.parent
TEMPLATE_SCENARIO = PROJECT_ROOT / "config" / "scenario.yaml"

# Ceilings for a server that strangers can post to. A 60-UAV 2-hour mission is a
# denial of service, not a research question. Below 9 UAVs a relay-chained mission
# stalls: the relay budget cannot cover the later PoIs and the fleet idles.
MAX_UAVS = 17
MIN_UAVS = 9
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


def _spawn(base: SpawnConfig, count: int | str) -> SpawnConfig:
    """Re-flow the launch grid so any UAV count keeps a sensible footprint."""
    if count == AUTO_COUNT:
        return replace(base, count=AUTO_COUNT, min_count=max(base.min_count, MIN_UAVS),
                       max_count=min(base.max_count, MAX_UAVS))
    per_row = min(count, max(1, base.per_row))
    return replace(base, count=count, per_row=per_row)


def _uav_count(spec: Mapping[str, Any], base: ScenarioConfig) -> int | str:
    raw = spec.get("uav_count")
    if raw is None:
        raw = base.uavs.count
    if str(raw).strip().lower() == AUTO_COUNT:
        return AUTO_COUNT
    count = _int({"uav_count": raw}, "uav_count", 0)
    _require(MIN_UAVS <= count <= MAX_UAVS,
             f"uav_count must be {MIN_UAVS}..{MAX_UAVS} or '{AUTO_COUNT}' (got {count})")
    return count


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

    A trigger naming a specific PoI (``complete_poi POI-5``) would only log
    TRIGGER_REJECTED once the operator's PoIs replace the template's, so it is
    filtered out. Selectors such as ``random_active`` pick a PoI at fire time and
    always apply. Time windows are clipped to a shorter mission.
    """
    if not faults:
        return ()
    kept = []
    for trig in base.timeline:
        trig = trig.clipped(duration_s)
        if trig is None:
            continue
        referenced = trig.params.get("poi_id")
        if referenced is not None and referenced not in poi_ids and referenced not in POI_SELECTORS:
            continue
        kept.append(trig)
    return tuple(kept)


def build_scenario(spec: Optional[Mapping[str, Any]] = None) -> ScenarioConfig:
    """Validated ScenarioConfig from a browser mission spec. Raises ConfigError."""
    spec = dict(spec or {})
    base = template()

    uav_count = _uav_count(spec, base)

    duration_s = _float(spec, "duration_s", base.duration_s)
    _require(MIN_DURATION_S <= duration_s <= MAX_DURATION_S,
             f"duration_s must be {MIN_DURATION_S:g}..{MAX_DURATION_S:g} (got {duration_s:g})")

    placed = _pois(spec.get("pois"), base)
    # PoIs clicked on the map are the mission: drawing the template's random PoIs
    # on top would add ones nobody placed. With none clicked, the template's
    # (random) PoIs are used. The region is kept either way for a random urgent PoI.
    random_pois = replace(base.random_pois, count=0) if placed else base.random_pois
    pois = placed or base.pois
    _require(pois or random_pois.max_count > 0, "a mission needs at least one PoI")

    name = str(spec.get("name") or "custom").strip()[:40] or "custom"
    faults = bool(spec.get("faults", True))

    # The template is loaded once per server, so its drawn seed would repeat for
    # every mission; draw a fresh one per mission unless the spec pins it.
    if spec.get("seed") is not None:
        seed = _int(spec, "seed", 0)
    elif base.random_seed:
        seed = fresh_seed()
    else:
        seed = base.seed if base.seed is not None else 42

    return replace(
        base,
        name=name,
        description=str(spec.get("description") or "Built in the mission builder")[:300],
        duration_s=duration_s,
        seed=seed,
        uavs=_spawn(base.uavs, uav_count),
        pois=pois,
        random_pois=random_pois,
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
