"""End-to-end tests: every shipped scenario runs, and the adaptive swarm beats the baseline."""

from dataclasses import replace
from pathlib import Path

import pytest

from core.config import Parameters, ScenarioConfig
from core.events import EventType
from simulation.runner import Simulation

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SCENARIOS = sorted((PROJECT_ROOT / "scenarios").glob("*.yaml"))
PARAMS = PROJECT_ROOT / "config" / "parameters.yaml"


def run(scenario_path: Path, mode: str = "adaptive", duration_s: float | None = None,
        seed: int | None = None) -> Simulation:
    scenario = ScenarioConfig.load(scenario_path)
    if duration_s is not None:
        clipped = (t.clipped(duration_s) for t in scenario.timeline)
        scenario = replace(scenario, duration_s=duration_s, timeline=tuple(t for t in clipped if t))
    if seed is not None:
        scenario = replace(scenario, seed=seed)
    sim = Simulation(Parameters.load(PARAMS), scenario, mode=mode, results_dir=None)
    sim.run()
    sim.finish()
    return sim


@pytest.mark.parametrize("scenario_path", SCENARIOS, ids=lambda p: p.stem)
def test_scenario_runs_and_completes_every_poi(scenario_path: Path):
    sim = run(scenario_path)
    summary = sim.summary
    assert summary["mission"]["completion_rate"] == 1.0
    assert summary["safety"]["separation_violations"] == 0
    assert summary["safety"]["geofence_violations"] == 0
    assert summary["communication"]["data_delivery_ratio"] == pytest.approx(1.0, abs=0.05)
    assert not sim.world.events.history(types=[EventType.TRIGGER_REJECTED])
    assert summary["safety"]["uavs_airborne_at_end"] == 0     # everyone landed safely


# The demo's PoIs and event times are drawn per run. Any seed must still give a
# complete demonstration - checked here on several. A sweep of 100 seeds met
# these too; separation and obstacle violations are real swarm outcomes that some
# random layouts produce, so they are measured, not asserted away.
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_config_scenario_runs_the_full_demonstration(seed: int):
    sim = run(PROJECT_ROOT / "config" / "scenario.yaml", seed=seed)
    events = sim.world.events
    assert not events.history(types=[EventType.TRIGGER_REJECTED])   # every fault was injected
    assert events.count(EventType.TRIGGER_FIRED) == len(sim.world.timeline)
    assert events.count(EventType.LINK_DEGRADED) >= 1        # scenario A
    assert events.count(EventType.OBSTACLE_ADDED) >= 1       # scenario B
    assert events.count(EventType.POI_ADDED) >= 1            # emerging high-priority region
    assert events.count(EventType.UAV_FAILED) >= 1           # UAV loss
    assert events.count(EventType.FAULT_DETECTED) >= 1
    assert events.count(EventType.RECOVERY_COMPLETE) >= 1
    assert events.count(EventType.POI_COMPLETED) == len(sim.world.state.pois)
    assert sim.summary["mission"]["completion_rate"] == 1.0
    assert sim.summary["safety"]["uavs_airborne_at_end"] == 0


def test_the_demo_differs_from_run_to_run_and_replays_by_seed():
    def layout(seed):
        sim = Simulation(Parameters.load(PARAMS),
                         replace(ScenarioConfig.load(PROJECT_ROOT / "config" / "scenario.yaml"), seed=seed),
                         results_dir=None)
        drawn = (*sim.world.state.pois, *sim.world.scheduled_pois)      # most appear only later
        pois = tuple((p.poi_id, round(float(p.position[0])), round(float(p.position[1])), p.created_at_s)
                     for p in drawn)
        return pois, tuple((t.action, t.at_s) for t in sim.world.timeline)

    assert layout(5) == layout(5)
    layouts = {layout(seed) for seed in range(10)}
    assert len({pois for pois, _ in layouts}) == 10          # PoIs move, and appear at other times
    assert {len(pois) for pois, _ in layouts} == {10}        # always the 10 of the mission constraints
    assert len({times for _, times in layouts}) == 10        # events move


def test_adaptive_keeps_the_swarm_connected_better_than_the_baseline():
    scenario = PROJECT_ROOT / "scenarios" / "network_degradation.yaml"
    adaptive = run(scenario, "adaptive").summary
    baseline = run(scenario, "baseline").summary
    assert adaptive["communication"]["network_availability"] > baseline["communication"]["network_availability"]
    assert adaptive["communication"]["data_live_ratio"] > baseline["communication"]["data_live_ratio"]
    assert adaptive["resilience"]["mean_detection_time_s"] is not None
    assert baseline["resilience"]["mean_detection_time_s"] is None   # the baseline never diagnoses


def _without_wall_clock(summary: dict) -> dict:
    """Allocation runtime is measured in real (wall-clock) time, so it varies between runs."""
    return {group: {k: v for k, v in values.items() if not k.endswith("_runtime_ms")}
            for group, values in summary.items()}


def test_runs_are_reproducible_and_seed_dependent():
    scenario = PROJECT_ROOT / "scenarios" / "obstacle.yaml"
    first = _without_wall_clock(run(scenario).summary)
    second = _without_wall_clock(run(scenario).summary)
    assert first == second
    other = ScenarioConfig.load(scenario)
    changed = Simulation(Parameters.load(PARAMS), replace(other, seed=99), results_dir=None)
    changed.run()
    assert _without_wall_clock(changed.finish()) != first


def test_results_are_written_when_a_results_directory_is_given(tmp_path):
    scenario = replace(ScenarioConfig.load(PROJECT_ROOT / "scenarios" / "normal.yaml"), duration_s=60.0)
    sim = Simulation(Parameters.load(PARAMS), scenario, results_dir=tmp_path)
    sim.run()
    sim.finish()
    assert (sim.run_dir / "summary.json").is_file()
    assert (sim.run_dir / "events.jsonl").is_file()
    assert (sim.run_dir / "events.csv").is_file()
    assert (sim.run_dir / "timeseries.csv").is_file()
    assert (tmp_path / "runs.sqlite").is_file()

    import sqlite3
    with sqlite3.connect(tmp_path / "runs.sqlite") as conn:
        assert conn.execute("SELECT COUNT(*) FROM runs").fetchone()[0] == 1
        assert conn.execute("SELECT COUNT(*) FROM events").fetchone()[0] > 0
        assert conn.execute("SELECT COUNT(*) FROM samples").fetchone()[0] > 0
