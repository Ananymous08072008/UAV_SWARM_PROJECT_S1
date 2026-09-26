"""
simulation/runner.py
Wires the layers together and drives the simulation loop.

Every tick:
    1. dashboard commands injected by the operator
    2. environment measurement   (radio channel)
    3. swarm decisions           (mission manager)
    4. world step                (UAV execution, triggers, charging, surveys)
    5. data capture / delivery   (imagery to the GCS)
    6. metrics sample, MAVLink telemetry, dashboard push

The same class is used by main.py (interactive / real time) and by
experiments/run_experiments.py (headless batches).
"""

from __future__ import annotations

import csv
import json
import time
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Optional

from core.config import Parameters, ScenarioConfig
from core.events import EventType
from core.world import CommandError, World
from simulation.environment import Environment
from swarm.fleet_planner import plan_fleet
from swarm.mission_manager import MissionManager, SwarmParams
from swarm_logging.database import RunDatabase
from swarm_logging.event_logger import EventLogger
from swarm_logging.metrics import MetricsCollector

if TYPE_CHECKING:
    from dashboard.websocket import LiveHub
    from telemetry.mavlink_gateway import MavlinkGateway

# Ceiling on the in-memory event log kept for dashboard downloads. A full-length
# mission produces a few tens of thousands of events, so this only ever catches a
# runaway session on a shared server - and the export reports what it dropped.
MAX_EXPORT_EVENTS = 200_000


class Simulation:
    def __init__(self, params: Parameters, scenario: ScenarioConfig, mode: str = "adaptive",
                 results_dir: Optional[str | Path] = None, hub: Optional["LiveHub"] = None,
                 gateway: Optional["MavlinkGateway"] = None, stop_when_complete: bool = True) -> None:
        self.world = World(params, scenario)
        self.env = Environment(self.world)
        if scenario.uavs.auto:
            # Before the MissionManager: its safety layer allots one flight level per UAV.
            # Sized the same way in both modes, so adaptive and baseline fly equal fleets.
            swarm = SwarmParams.from_dict(params.section("swarm"))
            fleet = plan_fleet(self.world, self.env, swarm.relay, swarm.fleet, swarm.allocation.reserve_pct)
            self.world.spawn_fleet(fleet.total, fleet.to_dict())
        self.manager = MissionManager(self.world, self.env, mode)
        self.metrics = MetricsCollector(self.world, self.env, self.manager)
        self.hub = hub
        self.gateway = gateway
        self.stop_when_complete = stop_when_complete
        self.mode = mode
        self.run_dir: Optional[Path] = None
        self.logger: Optional[EventLogger] = None
        self.db: Optional[RunDatabase] = None
        self.run_id: Optional[int] = None
        self.summary: Optional[dict[str, Any]] = None
        self._last_push_s = 0.0
        self._pushed_seq = 0
        self._finished = False
        if results_dir is not None:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.run_dir = Path(results_dir) / f"{stamp}_{scenario.name}_{mode}"
            self.logger = EventLogger(self.run_dir, self.world.events)
            self.db = RunDatabase(Path(results_dir) / "runs.sqlite")
            self.run_id = self.db.start_run(scenario.name, mode, self.world.seed)
        elif hub is not None:
            # Nothing is going to disk, but a dashboard run still has to be
            # downloadable afterwards. Record in memory only - the EventBus keeps
            # its last 2000 events, which is the tail of a run, not a run.
            self.logger = EventLogger(None, self.world.events, max_records=MAX_EXPORT_EVENTS)
        if hub is not None:
            # Lets the web thread build the download without main.py having to
            # hand the Simulation to the server it created before this existed.
            hub.simulation = self

    @classmethod
    def from_files(cls, parameters_path: str | Path, scenario_path: str | Path, **kwargs) -> "Simulation":
        return cls(Parameters.load(parameters_path), ScenarioConfig.load(scenario_path), **kwargs)

    # ------------------------------------------------------------------ status
    @property
    def is_finished(self) -> bool:
        if self.world.is_finished:
            return True
        # all_completed is checked directly as well: mission_complete_s is only
        # re-evaluated once per decision cycle, and a PoI that appeared since then
        # must keep the run going even if every UAV has already landed. The last UAV
        # down also gets the seconds it needs to offload its imagery on the pad.
        return bool(self.stop_when_complete and self.manager.mission_complete_s is not None
                    and self.world.state.pois.all_completed and self.world.pending_spawns == 0
                    and self.manager.all_landed and self.world.pending_triggers == 0
                    and self.env.data.buffered_mb <= 1e-6)

    def start(self) -> None:
        self.world.start()
        self.push()

    # -------------------------------------------------------------------- loop
    def tick(self) -> None:
        world = self.world
        self._apply_operator_commands()
        network_updated = self.env.before_decisions(world)
        self.manager.update(world, network_updated)
        world.step()
        self.env.after_step(world)
        self.metrics.update(world)
        if self.gateway is not None:
            self.gateway.update(world.snapshot())
        if self.hub is not None:
            self.push(min_interval_s=0.2)

    def run(self, realtime: bool = False, speed: float = 1.0,
            on_status: Optional[Callable[["Simulation"], None]] = None, status_interval_s: float = 0.0) -> None:
        self.start()
        wall_start = time.perf_counter()
        next_status = status_interval_s
        while not self.is_finished:
            self.tick()
            if status_interval_s and on_status is not None and self.world.t >= next_status - 1e-9:
                next_status += status_interval_s
                on_status(self)
            if realtime:
                delay = wall_start + self.world.t / speed - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)

    def finish(self, reason: str = "duration reached") -> dict[str, Any]:
        if self._finished:
            return self.summary or {}
        self._finished = True
        self.world.stop(reason)
        self.metrics.update(self.world)
        self.summary = self.metrics.summary()
        self.summary["run"]["stop_reason"] = reason
        self.push(force=True)
        if self.run_dir is not None:
            self._write_results()
        if self.logger is not None:
            self.logger.close()
        if self.db is not None and self.run_id is not None:
            self.db.add_events(self.run_id, self.logger.records if self.logger else [])
            self.db.add_samples(self.run_id, self.metrics.rows())
            self.db.finish_run(self.run_id, self.world.t, self.summary)
            self.db.close()
        if self.gateway is not None:
            self.gateway.close()
        return self.summary

    def _write_results(self) -> None:
        (self.run_dir / "summary.json").write_text(json.dumps(self.summary, indent=2), encoding="utf-8")
        rows = self.metrics.rows()
        if rows:
            with (self.run_dir / "timeseries.csv").open("w", encoding="utf-8", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        (self.run_dir / "scenario.json").write_text(
            json.dumps({"scenario": self.world.scenario.name, "mode": self.mode, "seed": self.world.seed,
                        "duration_s": self.world.duration_s}, indent=2), encoding="utf-8")

    # -------------------------------------------------------------- dashboard
    def _apply_operator_commands(self) -> None:
        if self.hub is None:
            return
        for action, params in self.hub.pop_commands():
            try:
                self.world.inject(action, params)
            except CommandError as exc:
                self.world.publish(EventType.TRIGGER_REJECTED, f"Operator command '{action}' rejected: {exc}")

    def payload(self) -> dict[str, Any]:
        area = self.world.state.area
        origin = self.world.params.geo_origin
        return {
            "world": self.world.snapshot().to_dict(),
            "env": self.env.to_dict(origin),
            "swarm": self.manager.to_dict(),
            "metrics": self.metrics.live(),
            "scenario": {"name": self.world.scenario.name, "description": self.world.scenario.description,
                         "mode": self.mode, "seed": self.world.seed,
                         "origin": {"lat_deg": origin.lat_deg, "lon_deg": origin.lon_deg,
                                    "alt_msl_m": origin.alt_msl_m},
                         "area_m": [area.x_min_m, area.y_min_m, area.x_max_m, area.y_max_m],
                         "area_latlon": [list(origin.to_geodetic(area.x_min_m, area.y_min_m, 0)[:2]),
                                         list(origin.to_geodetic(area.x_max_m, area.y_max_m, 0)[:2])],
                         "finished": self._finished},
        }

    def push(self, min_interval_s: float = 0.0, force: bool = False) -> None:
        if self.hub is None:
            return
        now = time.perf_counter()
        if not force and now - self._last_push_s < min_interval_s:
            return
        self._last_push_s = now
        events = [e.to_dict() for e in self.world.events.history(since_seq=self._pushed_seq)]
        if events:
            self._pushed_seq = events[-1]["seq"]
        self.hub.publish(self.payload(), events)
