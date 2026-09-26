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
               (swarm/relay_selector.py), obstacles included - and at least
               the chain of the deepest single PoI, so every PoI can be flown
               connected by itself. Later waves reuse them; the data ferry is
               only the task allocator's last resort. With ``relay_chain: false`` there are none: the
               smallest fleet, but imagery only reaches the GCS when a
               surveyor flies back into range.
spares         standby UAVs: the parked backup and a replacement for a relay
               that has to hand over and fly home to recharge - raised so the
               fleet is never smaller than ``uavs.min_count``
fault reserve  one per scheduled UAV loss, and one per scheduled new PoI whose
               position is not known in advance. A new PoI at a fixed position
               is planned exactly, like the PoIs present at launch.

The result is kept within ``uavs.min_count`` .. ``uavs.max_count``.
"""

from __future__ import annotations

import math
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
        selector = RelaySelector(world, env, roles=None, params=relay)
        plan = selector.make_plan([Terminal(j.key, j.waypoint, j.priority) for j in first_wave])
        relays, unreachable = plan.relay_count, tuple(plan.unreachable)
        # The deepest PoI sets a floor too: a fleet that cannot staff even one PoI's chain on
        # its own can only ever ferry that PoI - it would never be surveyed connected.
        deepest = max((selector.count_relays([Terminal(j.key, j.waypoint, j.priority)]) or 0 for j in jobs),
                      default=0)
        relays = max(relays, deepest)
    # A fleet below uavs.min_count flies the shortfall as extra spares: later waves and
    # relay handovers get UAVs to spare instead of waiting on a relay budget that is too tight.
    # A long chain also needs rotation margin of its own: the farthest PoI is deferred until
    # every other one is done (swarm/task_allocator.py, _defer_farthest), so the UAVs that crew
    # its chain have typically already done other duty and are not all fresh off the pad - a
    # chain sized with zero spare slack can lose several relays to recharge at once and collapse
    # entirely (chains are filled all-or-nothing), right when the demo should show it best.
    handover_margin = math.ceil(deepest / 3) if fleet.relay_chain else 0
    spares = max(fleet.spares, world.scenario.uavs.min_count - (surveyors + relays + reserve), handover_margin)
    return FleetPlan(surveyors=surveyors, relays=relays, spares=spares, fault_reserve=reserve,
                     max_count=world.scenario.uavs.max_count, makespan_s=makespan, time_budget_s=budget,
                     pois=len(jobs), unreachable=unreachable)


def _priority_order(world: "World", jobs: list[_Job]) -> list[_Job]:
    """Highest priority first; within a release/priority band, a greedy nearest-neighbour
    route (spatial clustering) instead of ranking each job by GCS distance on its own, so a
    surveyor sweeps a cluster of nearby PoIs together instead of zig-zagging between them.

    The one job farthest from the GCS among those known at launch is moved to the very end,
    mirroring the task allocator's own deferral of the farthest PoI (swarm/task_allocator.py,
    ``_defer_farthest``) - so the makespan and first-wave relay estimates this feeds match
    what the swarm actually flies, instead of assuming it goes out with the first wave."""
    gcs = np.asarray(world.state.gcs_position[:2], dtype=float)
    bands: dict[tuple[float, int], list[_Job]] = {}
    for j in jobs:
        bands.setdefault((j.release_s, j.priority), []).append(j)
    ordered: list[_Job] = []
    for band in sorted(bands, key=lambda b: (b[0], -b[1])):
        ordered.extend(_nearest_neighbour_route(bands[band], gcs))
    at_launch = [j for j in ordered if j.release_s <= 0.0]
    if len(at_launch) > 1:
        farthest = max(at_launch, key=lambda j: float(np.hypot(*(j.waypoint[:2] - gcs))))
        ordered = [j for j in ordered if j.key != farthest.key] + [farthest]
    return ordered


def _nearest_neighbour_route(group: list[_Job], start_xy: np.ndarray) -> list[_Job]:
    """Greedy nearest-neighbour chain from ``start_xy``; ties broken by key for determinism."""
    remaining = sorted(group, key=lambda j: j.key)
    route: list[_Job] = []
    at = start_xy
    while remaining:
        nxt = min(remaining, key=lambda j: (float(np.hypot(*(j.waypoint[:2] - at))), j.key))
        route.append(nxt)
        remaining.remove(nxt)
        at = nxt.waypoint[:2]
    return route


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
    recharges to ``resume_pct`` and flies it from the pad. The last job -
    ``_priority_order`` puts the farthest PoI there - does not start until every
    surveyor has finished all its other jobs, mirroring the task allocator's own
    deferral of it (swarm/task_allocator.py, ``_defer_farthest``), so this estimate
    matches what the swarm actually flies. Returns when the last surveyor is back
    on the ground.
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

    def schedule(fleet: list, job: _Job, earliest_s: float) -> None:
        options = []
        for i, (t, pos, pct) in enumerate(fleet):
            if job_cost(pos, job) + reserve_pct > pct:      # recharge first
                landed_pct = max(0.0, pct - home_cost(pos))
                charge_s = max(0.0, bat_p.resume_pct - landed_pct) / bat_p.charge_rate_pct_per_min * 60.0
                t, pos, pct = t + home_time(pos) + charge_s, pad, max(landed_pct, bat_p.resume_pct)
            start = max(t, job.release_s, earliest_s)
            done = start + battery.travel_time_s(pos, job.waypoint) + job.survey_s
            spent = battery.travel_cost_pct(pos, job.waypoint) + battery.hover_cost_pct(job.survey_s)
            options.append((done, i, job.waypoint, pct - spent))
        done, i, pos, pct = min(options, key=lambda o: (o[0], o[1]))
        fleet[i] = (done, pos, pct)

    fleet = [(0.0, pad, full) for _ in range(k)]            # (time, position, battery %) per surveyor
    regular, deferred = (jobs[:-1], jobs[-1]) if len(jobs) > 1 else (jobs, None)
    for job in regular:
        if job_cost(pad, job) + reserve_pct > bat_p.resume_pct:
            continue                                        # out of range even on a fresh battery
        schedule(fleet, job, 0.0)
    if deferred is not None and job_cost(pad, deferred) + reserve_pct <= bat_p.resume_pct:
        barrier = max(t for t, _, _ in fleet)   # every surveyor free before the farthest PoI starts
        schedule(fleet, deferred, barrier)
    return max(t + home_time(pos) for t, pos, _ in fleet)
