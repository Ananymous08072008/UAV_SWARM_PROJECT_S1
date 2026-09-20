"""Shared test fixtures: small worlds and simulations that run in a fraction of a second."""

from __future__ import annotations

from typing import Any

from core.config import Parameters, ScenarioConfig
from core.world import World
from simulation.runner import Simulation

SMALL_SCENARIO: dict[str, Any] = {
    "scenario": {"name": "test", "duration_s": 300.0, "seed": 7},
    "area": {"x_min_m": -200, "x_max_m": 900, "y_min_m": -200, "y_max_m": 700},
    "uavs": {"count": 6, "formation": "grid", "per_row": 3, "spacing_m": 15.0, "start_m": [-15, -30]},
    "pois": [
        {"id": "POI-A", "position_m": [420, 120], "priority": 3, "survey_time_s": 60},
        {"id": "POI-B", "position_m": [640, 380], "priority": 4, "survey_time_s": 60},
    ],
}


def scenario(**overrides: Any) -> ScenarioConfig:
    data = {key: value for key, value in SMALL_SCENARIO.items()}
    for key, value in overrides.items():
        if key in ("scenario", "uavs") and isinstance(value, dict):
            data[key] = {**data.get(key, {}), **value}
        else:
            data[key] = value
    return ScenarioConfig.from_dict(data)


def parameters(**sections: Any) -> Parameters:
    data: dict[str, Any] = {}
    for name, values in sections.items():
        data[name] = values
    return Parameters.from_dict(data)


def make_world(params: Parameters | None = None, **overrides: Any) -> World:
    return World(params or Parameters(), scenario(**overrides))


def make_sim(mode: str = "adaptive", params: Parameters | None = None, **overrides: Any) -> Simulation:
    return Simulation(params or Parameters(), scenario(**overrides), mode=mode, results_dir=None)


def run_until(sim: Simulation, t_s: float) -> None:
    while sim.world.t < t_s - 1e-9 and not sim.world.is_finished:
        sim.tick()


def step_world(world: World, t_s: float) -> None:
    while world.t < t_s - 1e-9 and not world.is_finished:
        world.step()
