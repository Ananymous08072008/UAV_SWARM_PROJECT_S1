"""
swarm/fleet_planner.py
Sizes the fleet to the mission when a scenario says ``uavs.count: auto``.

    fleet = surveyors + relays + spares + fault reserve

surveyors      the fewest that still finish every PoI before the mission
               deadline, flying them one after another in priority order
               (highest first), with trips home to recharge when the battery
               demands it. The schedule uses the swarm's own battery model, so
               it changes with every seed's PoI count, placement and survey times.
relays         what keeps the first wave (the highest-priority PoIs) connected
               to the GCS, from the same plan the swarm flies later
               (swarm/relay_selector.py), obstacles included. Later waves reuse
               them or fall back to the data ferry, as the task allocator
               decides. With ``relay_chain: false`` there are none: the
               smallest fleet, but imagery only reaches the GCS when a
               surveyor flies back into range.
spares         standby UAVs: the parked backup and a replacement for a relay
               that has to hand over and fly home to recharge
fault reserve  one per scheduled UAV loss, and one per scheduled new PoI whose
               position is not known in advance. A new PoI at a fixed position
               is planned exactly, like the PoIs present at launch.

The result is capped at ``uavs.max_count``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np

from core.events import TriggerAction
from swarm.relay_selector import RelayParams, RelaySelector, Terminal

if TYPE_CHECKING:
    from core.world import World
    from simulation.battery import BatteryModel
    from simulation.environment import Environment


@dataclass(frozen=True)
class FleetParams:
    spares: int = 1
    fault_reserve: bool = True
    time_budget_fraction: float = 0.75   # share of the mission time the survey schedule may fill
    relay_chain: bool = True             # False = no dedicated relays, surveyors ferry data home

    def __post_init__(self) -> None:
        if not isinstance(self.spares, int) or self.spares < 0:
            raise ValueError("spares must be an integer >= 0")
        if not 0.0 < self.time_budget_fraction <= 1.0:
            raise ValueError("time_budget_fraction must be in (0, 1]")


@dataclass(frozen=True)
class FleetPlan:
    surveyors: int
    relays: int
    spares: int
    fault_reserve: int
    max_count: int
    makespan_s: float                        # estimated time to survey every PoI and land
    time_budget_s: float
    pois: int
    unreachable: tuple[str, ...] = ()

    @property
    def required(self) -> int:
        return max(1, self.surveyors + self.relays + self.spares + self.fault_reserve)

    @property
    def total(self) -> int:
        return min(self.required, self.max_count)

    def to_dict(self) -> dict[str, Any]:
        return {"pois": self.pois, "surveyors": self.surveyors, "relays": self.relays,
                "spares": self.spares, "fault_reserve": self.fault_reserve,
                "required": self.required, "capped": self.required > self.max_count,
                "unreachable": list(self.unreachable), "makespan_s": round(self.makespan_s, 1),
                "time_budget_s": round(self.time_budget_s, 1),
                "over_budget": self.makespan_s > self.time_budget_s}


@dataclass(frozen=True)
class _Job:
    key: str
    waypoint: np.ndarray
    survey_s: float
    priority: int
    release_s: float = 0.0


def _survey_altitude(world: "World", altitude_m) -> float:
    return float(altitude_m) if altitude_m is not None else world.params.uav.default_altitude_m


def plan_fleet(world: "World", env: "Environment", relay: RelayParams, fleet: FleetParams,
               reserve_pct: float = 10.0) -> FleetPlan:
    jobs = [_Job(p.poi_id, np.array([*p.position[:2], _survey_altitude(world, p.altitude_m)]),
                 max(0.0, p.survey_time_s - p.progress_s), p.priority)
            for p in world.state.pois if not p.is_completed]

    reserve = 0
    for trig in world.timeline:
        if trig.action == TriggerAction.FAIL_UAV.value:
            reserve += 1
        elif trig.action == TriggerAction.ADD_POI.value:
            pos = trig.params.get("position_m")
            if isinstance(pos, (list, tuple)) and len(pos) >= 2:     # not "random": plan it exactly
                jobs.append(_Job(str(trig.params.get("id", f"t{len(jobs)}")),
                                 np.array([float(pos[0]), float(pos[1]),
                                           _survey_altitude(world, trig.params.get("altitude_m"))]),
                                 float(trig.params.get("survey_time_s", 30.0)),
                                 int(trig.params.get("priority", 5)), float(trig.at_s)))
            else:
                reserve += 1
    reserve = reserve if fleet.fault_reserve else 0

    budget = fleet.time_budget_fraction * world.duration_s
    surveyors, makespan = _fewest_surveyors(world, env.battery, _priority_order(world, jobs), budget, reserve_pct)
    relays, unreachable = 0, ()
    if fleet.relay_chain:
        first_wave = _priority_order(world, [j for j in jobs if j.release_s <= 0.0])[:surveyors]
        # The planner only reads the world's geometry and the radio model; it never
        # assigns a role, so it needs no RoleManager.
        plan = RelaySelector(world, env, roles=None, params=relay).make_plan(
            [Terminal(j.key, j.waypoint, j.priority) for j in first_wave])
        relays, unreachable = plan.relay_count, tuple(plan.unreachable)
    return FleetPlan(surveyors=surveyors, relays=relays, spares=fleet.spares, fault_reserve=reserve,
                     max_count=world.scenario.uavs.max_count, makespan_s=makespan, time_budget_s=budget,
                     pois=len(jobs), unreachable=unreachable)


def _priority_order(world: "World", jobs: list[_Job]) -> list[_Job]:
    """Highest priority first; among equals, the closest to the GCS (cheapest to reach) first."""
    gcs = np.asarray(world.state.gcs_position[:2], dtype=float)
    return sorted(jobs, key=lambda j: (j.release_s, -j.priority, float(np.hypot(*(j.waypoint[:2] - gcs))), j.key))


def _fewest_surveyors(world: "World", battery: "BatteryModel", jobs: list[_Job], budget_s: float,
                      reserve_pct: float) -> tuple[int, float]:
    """Smallest surveyor count whose schedule fits the budget; else the fastest one possible."""
    if not jobs:
        return 0, 0.0
    best = (len(jobs), _makespan(world, battery, jobs, len(jobs), reserve_pct))
    for k in range(1, len(jobs)):
        makespan = _makespan(world, battery, jobs, k, reserve_pct)
        if makespan <= budget_s:
            return k, makespan
    return best


def _makespan(world: "World", battery: "BatteryModel", jobs: list[_Job], k: int, reserve_pct: float) -> float:
    """List-schedule the jobs, in order, on k surveyors launched from the pad.

    Each job goes to the surveyor that would finish it first. A surveyor that
    could not fly the job and still get home above ``reserve_pct`` lands,
    recharges to ``resume_pct`` and flies it from the pad. Returns when the
    last surveyor is back on the ground.
    """
    uav_p, bat_p = world.params.uav, world.params.battery
    spawn = world.scenario.uavs
    pad = np.array([*spawn.start_m, 0.0])
    home_above = np.array([*spawn.start_m, uav_p.rth_altitude_m])
    descent_s = uav_p.rth_altitude_m / uav_p.climb_rate_mps
    full = spawn.initial_battery_pct

    def home_time(pos) -> float:
        return 0.0 if pos[2] <= 0.0 else battery.travel_time_s(pos, home_above) + descent_s

    def home_cost(pos) -> float:
        return 0.0 if pos[2] <= 0.0 else battery.travel_cost_pct(pos, home_above) + battery.hover_cost_pct(descent_s)

    def job_cost(pos, job: _Job) -> float:
        return (battery.travel_cost_pct(pos, job.waypoint) + battery.hover_cost_pct(job.survey_s)
                + battery.travel_cost_pct(job.waypoint, home_above) + battery.hover_cost_pct(descent_s))

    fleet = [(0.0, pad, full) for _ in range(k)]            # (time, position, battery %) per surveyor
    for job in jobs:
        if job_cost(pad, job) + reserve_pct > bat_p.resume_pct:
            continue                                        # out of range even on a fresh battery
        options = []
        for i, (t, pos, pct) in enumerate(fleet):
            if job_cost(pos, job) + reserve_pct > pct:      # recharge first
                landed_pct = max(0.0, pct - home_cost(pos))
                charge_s = max(0.0, bat_p.resume_pct - landed_pct) / bat_p.charge_rate_pct_per_min * 60.0
                t, pos, pct = t + home_time(pos) + charge_s, pad, max(landed_pct, bat_p.resume_pct)
            start = max(t, job.release_s)
            done = start + battery.travel_time_s(pos, job.waypoint) + job.survey_s
            spent = battery.travel_cost_pct(pos, job.waypoint) + battery.hover_cost_pct(job.survey_s)
            options.append((done, i, job.waypoint, pct - spent))
        done, i, pos, pct = min(options, key=lambda o: (o[0], o[1]))
        fleet[i] = (done, pos, pct)
    return max(t + home_time(pos) for t, pos, _ in fleet)
