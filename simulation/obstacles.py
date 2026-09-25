"""
simulation/obstacles.py
Obstacles (collapsed buildings, landslide debris, terrain ridges) that block or
attenuate radio links (Scenario B) and must not be flown through.

Geometry: a vertical prism = 2D polygon footprint (local ENU metres) + height.
A link is obstructed when the straight 3D segment between the two antennas
passes through the prism.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Iterator, Mapping, Optional, Sequence

from core.config import ConfigError, GeoOrigin, ScenarioConfig
from core.events import EventType, Severity, Trigger, TriggerAction

if TYPE_CHECKING:
    from core.world import World


def _seg_intersection_t(p0, p1, q0, q1) -> Optional[float]:
    """Parameter t on p0->p1 where it crosses segment q0->q1 (None if parallel / no hit)."""
    r = (p1[0] - p0[0], p1[1] - p0[1])
    s = (q1[0] - q0[0], q1[1] - q0[1])
    denom = r[0] * s[1] - r[1] * s[0]
    if abs(denom) < 1e-12:
        return None
    qp = (q0[0] - p0[0], q0[1] - p0[1])
    t = (qp[0] * s[1] - qp[1] * s[0]) / denom
    u = (qp[0] * r[1] - qp[1] * r[0]) / denom
    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return t
    return None


@dataclass(frozen=True)
class Obstacle:
    obstacle_id: str
    polygon: tuple[tuple[float, float], ...]
    height_m: float = 50.0
    attenuation_db: float = 25.0

    def __post_init__(self) -> None:
        poly = tuple((float(x), float(y)) for x, y in self.polygon)
        if len(poly) < 3:
            raise ValueError(f"obstacle '{self.obstacle_id}': polygon needs at least 3 vertices")
        if self.height_m <= 0 or self.attenuation_db < 0:
            raise ValueError(f"obstacle '{self.obstacle_id}': height_m must be > 0 and attenuation_db >= 0")
        object.__setattr__(self, "polygon", poly)
        xs, ys = zip(*poly)
        object.__setattr__(self, "_bbox", (min(xs), min(ys), max(xs), max(ys)))

    @classmethod
    def from_dict(cls, data: Mapping[str, Any], where: str = "obstacle") -> "Obstacle":
        try:
            return cls(str(data["id"]), tuple(tuple(v) for v in data["polygon_m"]),
                       float(data.get("height_m", 50.0)), float(data.get("attenuation_db", 25.0)))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{where}: needs id, polygon_m [[x, y], ...], height_m, attenuation_db ({exc})") from None

    @property
    def centroid(self) -> tuple[float, float]:
        xs, ys = zip(*self.polygon)
        return sum(xs) / len(xs), sum(ys) / len(ys)

    def contains_xy(self, x: float, y: float) -> bool:
        x0, y0, x1, y1 = self._bbox
        if not (x0 <= x <= x1 and y0 <= y <= y1):
            return False
        inside = False
        poly = self.polygon
        j = len(poly) - 1
        for i in range(len(poly)):
            xi, yi = poly[i]
            xj, yj = poly[j]
            if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
                inside = not inside
            j = i
        return inside

    def contains(self, point: Sequence[float]) -> bool:
        return point[2] < self.height_m and self.contains_xy(point[0], point[1])

    def _inside_intervals(self, a: Sequence[float], b: Sequence[float]) -> list[tuple[float, float]]:
        """Parameter intervals of segment a->b (2D) that lie inside the footprint."""
        x0, y0, x1, y1 = self._bbox
        if (max(a[0], b[0]) < x0 or min(a[0], b[0]) > x1 or max(a[1], b[1]) < y0 or min(a[1], b[1]) > y1):
            return []
        ts = {0.0, 1.0}
        poly = self.polygon
        for i in range(len(poly)):
            t = _seg_intersection_t(a, b, poly[i], poly[(i + 1) % len(poly)])
            if t is not None:
                ts.add(t)
        cuts = sorted(ts)
        intervals = []
        for t0, t1 in zip(cuts, cuts[1:]):
            tm = (t0 + t1) / 2
            if self.contains_xy(a[0] + (b[0] - a[0]) * tm, a[1] + (b[1] - a[1]) * tm):
                intervals.append((t0, t1))
        return intervals

    def crosses_xy(self, a: Sequence[float], b: Sequence[float]) -> bool:
        return bool(self._inside_intervals(a, b))

    def blocks(self, a: Sequence[float], b: Sequence[float]) -> bool:
        """True when the 3D segment a->b passes through the prism (below its height)."""
        for t0, t1 in self._inside_intervals(a, b):
            z0 = a[2] + (b[2] - a[2]) * t0
            z1 = a[2] + (b[2] - a[2]) * t1
            if min(z0, z1) < self.height_m:
                return True
        return False

    def to_dict(self, origin: Optional[GeoOrigin] = None) -> dict[str, Any]:
        out = {"id": self.obstacle_id, "polygon_m": [list(p) for p in self.polygon],
               "height_m": self.height_m, "attenuation_db": self.attenuation_db}
        if origin is not None:
            out["polygon_latlon"] = [list(origin.to_geodetic(x, y, 0.0)[:2]) for x, y in self.polygon]
        return out


class ObstacleField:
    """All known obstacles. ``version`` increases on every change so planners can re-plan."""

    def __init__(self, obstacles: Sequence[Obstacle] = ()) -> None:
        self._obstacles: dict[str, Obstacle] = {}
        self.version = 0
        for obs in obstacles:
            self.add(obs)

    @classmethod
    def from_scenario(cls, scenario: ScenarioConfig) -> "ObstacleField":
        raw = scenario.extra_sections.get("obstacles") or []
        if not isinstance(raw, list):
            raise ConfigError("[obstacles] must be a list")
        try:
            return cls([Obstacle.from_dict(o, f"obstacles[{i}]") for i, o in enumerate(raw)])
        except ValueError as exc:
            raise ConfigError(str(exc)) from exc

    def add(self, obstacle: Obstacle) -> None:
        if obstacle.obstacle_id in self._obstacles:
            raise ValueError(f"obstacle '{obstacle.obstacle_id}' already exists")
        self._obstacles[obstacle.obstacle_id] = obstacle
        self.version += 1

    def remove(self, obstacle_id: str) -> Obstacle:
        try:
            obs = self._obstacles.pop(obstacle_id)
        except KeyError:
            raise KeyError(f"unknown obstacle '{obstacle_id}'") from None
        self.version += 1
        return obs

    def __iter__(self) -> Iterator[Obstacle]:
        return iter(self._obstacles.values())

    def __len__(self) -> int:
        return len(self._obstacles)

    def attenuation_db(self, a: Sequence[float], b: Sequence[float]) -> tuple[float, bool]:
        """Total extra path loss on the a->b link and whether any obstacle blocks it."""
        total = 0.0
        for obs in self._obstacles.values():
            if obs.blocks(a, b):
                total += obs.attenuation_db
        return total, total > 0.0

    def inside(self, point: Sequence[float]) -> Optional[Obstacle]:
        return next((o for o in self._obstacles.values() if o.contains(point)), None)

    def max_height_crossed(self, a: Sequence[float], b: Sequence[float]) -> float:
        """Tallest obstacle whose footprint the horizontal path a->b crosses (0 if none)."""
        return max((o.height_m for o in self._obstacles.values()
                    if o.crosses_xy(a, b) or o.contains_xy(b[0], b[1])), default=0.0)

    def to_list(self, origin: Optional[GeoOrigin] = None) -> list[dict[str, Any]]:
        return [o.to_dict(origin) for o in self._obstacles.values()]

    # ----------------------------------------------------------- scenario hooks
    def register_triggers(self, world: "World") -> None:
        from core.world import CommandError

        # A PoI drawn inside debris would be surveyed from inside the obstacle.
        world.register_placement_filter(
            lambda x, y: not any(o.contains_xy(x, y) for o in self._obstacles.values()))

        def add(trig: Trigger) -> None:
            params = dict(trig.params)
            if "polygon_m" not in params:
                # center_m + size_m: a square, where center_m may be a named position
                # selector such as "backbone_midpoint" (registered by the swarm layer).
                raw_centre = params.pop("center_m", None)
                centre = world.resolve_point(raw_centre)
                size = float(params.pop("size_m", 120.0))
                if isinstance(raw_centre, str):
                    # placed by a selector (e.g. on the backbone): shrink so no flying UAV is
                    # inside the footprint - debris blocks the link, it does not land on the UAVs
                    clearance = min((max(abs(u.position[0] - centre[0]), abs(u.position[1] - centre[1]))
                                     for u in world.state.operational_uavs() if u.is_airborne), default=None)
                    if clearance is not None:
                        size = max(20.0, min(size, 2.0 * clearance - 10.0))
                params["polygon_m"] = [list(p) for p in square("tmp", centre, size).polygon]
            try:
                obs = Obstacle.from_dict(params, "add_obstacle")
                if not all(world.state.area.contains(x, y) for x, y in obs.polygon):
                    raise ValueError("polygon is outside the operating area")
                self.add(obs)
            except ValueError as exc:
                raise CommandError(str(exc)) from None
            cx, cy = obs.centroid
            world.publish(EventType.OBSTACLE_ADDED,
                          f"Obstacle {obs.obstacle_id} appeared at ({cx:.0f}, {cy:.0f}), "
                          f"height {obs.height_m:.0f} m, -{obs.attenuation_db:.0f} dB",
                          severity=Severity.WARNING, data=obs.to_dict())

        def remove(trig: Trigger) -> None:
            obstacle_id = str(trig.params.get("id", ""))
            try:
                obs = self.remove(obstacle_id)
            except KeyError as exc:
                raise CommandError(exc.args[0]) from None
            world.publish(EventType.OBSTACLE_REMOVED, f"Obstacle {obs.obstacle_id} removed",
                          data={"id": obs.obstacle_id})

        world.register_trigger_handler(TriggerAction.ADD_OBSTACLE, add)
        world.register_trigger_handler(TriggerAction.REMOVE_OBSTACLE, remove)


def square(obstacle_id: str, center: Sequence[float], size_m: float, height_m: float = 60.0,
           attenuation_db: float = 30.0) -> Obstacle:
    """Convenience: axis-aligned square obstacle."""
    h = size_m / 2
    cx, cy = center[0], center[1]
    return Obstacle(obstacle_id, ((cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)),
                    height_m, attenuation_db)
