"""
simulation/environment.py
Everything physical around the core world model, in one object:

    obstacles   what blocks radio links and airspace
    comm        the radio channel (link PDR, latency, range)
    battery     energy estimates used for RTH / feasibility decisions
    data        imagery captured at PoIs and delivered to the GCS

The Environment is measured *before* the swarm decides (so decisions use fresh
link measurements) and updated *after* the the world step executes (data capture and
delivery follow the new positions and routes).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Optional

from core.config import ConfigError, GeoOrigin
from simulation.battery import BatteryModel
from simulation.communication import CommunicationModel
from simulation.data_model import DataModel
from simulation.obstacles import ObstacleField

if TYPE_CHECKING:
    from core.world import World


class Environment:
    def __init__(self, world: "World") -> None:
        self.obstacles = ObstacleField.from_scenario(world.scenario)
        self.comm = CommunicationModel.from_world(world, self.obstacles)
        self.battery = BatteryModel(world.params.uav, world.params.battery)
        self.data = DataModel.from_world(world)
        self.disaster_zone = self._parse_zone(world)
        self.obstacles.register_triggers(world)

    @staticmethod
    def _parse_zone(world: "World") -> list[tuple[float, float]]:
        raw = world.scenario.extra_sections.get("disaster_zone") or []
        if not isinstance(raw, list):
            raise ConfigError("[disaster_zone] must be a list of [x, y] points")
        try:
            return [(float(x), float(y)) for x, y in raw]
        except (TypeError, ValueError) as exc:
            raise ConfigError(f"[disaster_zone] must be a list of [x, y] points ({exc})") from None

    def before_decisions(self, world: "World") -> bool:
        """Re-measure the radio channel. True when new measurements are available."""
        return self.comm.update(world)

    def after_step(self, world: "World") -> None:
        self.data.update(world, world.dt)

    def to_dict(self, origin: Optional[GeoOrigin] = None) -> dict[str, Any]:
        return {
            "obstacles": self.obstacles.to_list(origin),
            "disaster_zone": [list(p) for p in self.disaster_zone],
            "links": [lk.to_dict() for lk in self.comm.links.values()],
            "node_positions": {str(nid): [round(float(v), 1) for v in pos] for nid, pos in self.comm.positions.items()},
            "data": self.data.stats(),
        }
