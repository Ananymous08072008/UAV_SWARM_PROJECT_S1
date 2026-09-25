"""
core/world.py
The World owns the authoritative WorldState and advances it in fixed steps.

    swarm layer (decisions) --command API--> World --step()--> UAV execution
                                               |
                                               +--> EventBus --> dashboard / logger / metrics

Each step:
  1. fire scenario triggers that are due     (timeline or dashboard injections)
  2. advance every UAV by dt                 (motion + battery)
  3. return-to-home phases and charging      (land on the home pad, recharge)
  4. accumulate survey time on PoIs          (and release UAVs whose PoI is done)

The World validates and logs every change but never decides which UAV does
what - that belongs to swarm/* (task allocator, relay selector, ...).
"""

from __future__ import annotations

import math
from dataclasses import fields as dc_fields
from pathlib import Path
from typing import Any, Callable, Mapping, Optional

import numpy as np

from core.config import ConfigError, Parameters, ScenarioConfig
from core.events import (EventBus, EventType, Severity, Trigger, TriggerAction,
                         TriggerHandler, TriggerSchedule, parse_action)
from core.poi import PoI, PoIStatus
from core.state import WorldSnapshot, WorldState
from core.uav import UAV, CommState, HealthState, UAVRole

Controller = Callable[["World"], None]
UAVSelector = Callable[["World"], Optional[int]]
PointSelector = Callable[["World"], Optional[Any]]
PlacementFilter = Callable[[float, float], bool]

_BUSY_HOME_ROLES = (UAVRole.RETURNING, UAVRole.CHARGING)

# Trigger values resolved at fire time rather than naming a fixed PoI or place,
# so a timeline still works when the PoIs themselves are drawn per run.
RANDOM_ACTIVE_POI = "random_active"      # poi_id: a PoI not yet done, preferring one under survey
RANDOM_POSITION = "random"               # position_m: a free spot in the random_pois region
POI_SELECTORS = (RANDOM_ACTIVE_POI,)

_PLACEMENT_TRIES = 200
_RETRY_S = 2.0      # a windowed trigger whose target does not exist yet tries again this often
_RETRY_GRACE_S = 30.0   # ... for up to this long after its window closes (e.g. a relay handover gap)

# Independent random streams per purpose, all derived from the run seed. Keeping
# them apart means editing the timeline cannot move the PoIs, and vice versa.
_STREAM_TIMELINE = 202
_STREAM_TRIGGER_CHOICES = 203


class CommandError(ValueError):
    """A command was rejected because it is invalid in the current world state."""


def _degraded_radio(world: "World") -> Optional[int]:
    """Built-in selector: the operational UAV with the worst injected radio fault."""
    faulty = [u for u in world.state.operational_uavs() if u.comm.radio_health < 1.0]
    return min(faulty, key=lambda u: (u.comm.radio_health, u.uav_id)).uav_id if faulty else None


class World:
    def __init__(self, params: Parameters, scenario: ScenarioConfig) -> None:
        self.params = params
        self.scenario = scenario
        self.seed = scenario.seed if scenario.seed is not None else params.simulation.seed
        self.rng = np.random.default_rng(self.seed)
        self._choice_rng = np.random.default_rng([self.seed, _STREAM_TRIGGER_CHOICES])
        self.events = EventBus(params.simulation.event_history)
        self.state = WorldState(
            origin=params.geo_origin,
            area=scenario.area,
            gcs_position=np.array(scenario.gcs.position_m, dtype=float),
        )
        # The schedule actually used by this run: every [earliest, latest] window
        # drawn to one time. Published with SIM_STARTED so the log shows it.
        timeline_rng = np.random.default_rng([self.seed, _STREAM_TIMELINE])
        self.timeline = tuple(spec.resolve(timeline_rng) for spec in scenario.timeline)
        # A windowed trigger that finds no target (e.g. no relay while the chain is
        # being rebuilt) keeps looking until shortly after its window closes. A
        # fixed-time trigger fires once, exactly as written, and never waits.
        self._retry_until = {i: spec.latest_s + _RETRY_GRACE_S
                             for i, spec in enumerate(scenario.timeline) if spec.at_s_max is not None}
        self._planned_s = {i: spec.at_s for i, spec in enumerate(self.timeline)}
        try:
            self._triggers = TriggerSchedule.from_specs(self.timeline)
        except ValueError as exc:
            raise ConfigError(f"scenario '{scenario.name}': {exc}") from exc
        self._trigger_handlers: dict[TriggerAction, TriggerHandler] = {
            TriggerAction.COMPLETE_POI: self._on_complete_poi,
            TriggerAction.FAIL_UAV: self._on_fail_uav,
            TriggerAction.SET_BATTERY: self._on_set_battery,
            TriggerAction.ADD_POI: self._on_add_poi,
            TriggerAction.DEGRADE_LINK: self._on_degrade_link,
            TriggerAction.RESTORE_LINK: self._on_restore_link,
        }
        self._uav_selectors: dict[str, UAVSelector] = {"degraded_radio": _degraded_radio}
        self._point_selectors: dict[str, PointSelector] = {}
        self._placement_filters: list[PlacementFilter] = []
        self._landing: set[int] = set()  # RETURNING UAVs that reached home altitude and are descending
        self._started = False
        self._stopped = False
        # How the fleet size was decided, published with SIM_STARTED. With
        # ``uavs.count: auto`` the World waits for spawn_fleet(): sizing needs the
        # radio model and relay planner, which sit above the core layer.
        self.fleet: dict[str, Any] = {"sizing": "fixed"}
        self._create_pois()
        if not scenario.uavs.auto:
            self._spawn_uavs(scenario.uavs.count)

    @classmethod
    def from_files(cls, parameters_path: str | Path, scenario_path: str | Path) -> "World":
        return cls(Parameters.load(parameters_path), ScenarioConfig.load(scenario_path))

    # ------------------------------------------------------------- properties
    @property
    def t(self) -> float:
        return self.state.time_s

    @property
    def dt(self) -> float:
        return self.params.simulation.dt_s

    @property
    def duration_s(self) -> float:
        return self.scenario.duration_s

    @property
    def time_left_s(self) -> float:
        return max(0.0, self.duration_s - self.t)

    @property
    def pending_triggers(self) -> int:
        return self._triggers.remaining

    @property
    def is_finished(self) -> bool:
        return self._stopped or self.t >= self.duration_s - 1e-9

    def snapshot(self) -> WorldSnapshot:
        return self.state.snapshot(self.duration_s, self.events.last_seq)

    # --------------------------------------------------------------- building
    def _spawn_uavs(self, count: int) -> None:
        spawn = self.scenario.uavs
        for uav_id, position in enumerate(spawn.spawn_positions(count), start=1):
            self.state.add_uav(UAV(uav_id=uav_id, position=position, battery_pct=spawn.battery_for(uav_id)))

    def spawn_fleet(self, count: int, plan: Optional[Mapping[str, Any]] = None) -> None:
        """Launch-pad a fleet sized for this mission. Only for ``uavs.count: auto``, before start()."""
        spawn = self.scenario.uavs
        if not spawn.auto:
            raise CommandError("the scenario fixes the UAV count; spawn_fleet() is for uavs.count: auto")
        if self._started or self.state.uavs:
            raise CommandError("the fleet has already been spawned")
        if not 1 <= count <= spawn.max_count:
            raise CommandError(f"fleet size {count} outside 1..{spawn.max_count}")
        self._spawn_uavs(count)
        self.fleet = {"sizing": "auto", **dict(plan or {}), "uavs": count}

    def _create_pois(self) -> None:
        for spec in self.scenario.pois:
            self.state.pois.add(PoI(spec.id, spec.position_m, spec.priority, spec.survey_time_s, spec.altitude_m))
        cfg = self.scenario.random_pois
        low, high = cfg.count
        # Only draw the count when it is a range, so a fixed count consumes exactly
        # the random numbers it always did and existing seeds reproduce unchanged.
        count = low if low == high else int(self.rng.integers(low, high + 1))
        for i in range(1, count + 1):
            x, y = self._sample_poi_xy(self.rng)
            priority = int(self.rng.integers(cfg.priority[0], cfg.priority[1] + 1))
            survey = float(self.rng.uniform(*cfg.survey_time_s))
            self.state.pois.add(PoI(f"POI-R{i}", (x, y), priority, round(survey, 1)))

    def _sample_poi_xy(self, rng: np.random.Generator) -> tuple[float, float]:
        """A spot in the random_pois region, min_spacing_m from every PoI and clear of obstacles.

        Rejection sampling. When the region is too crowded for the spacing, the
        most spread-out candidate wins rather than failing the whole run; only a
        region with no free ground at all (every candidate blocked) is an error.
        """
        cfg = self.scenario.random_pois
        x0, y0, x1, y1 = cfg.bounds(self.state.area)
        placed = [(float(p.position[0]), float(p.position[1])) for p in self.state.pois]
        best, best_gap = None, -1.0
        for _ in range(_PLACEMENT_TRIES):
            x, y = float(rng.uniform(x0, x1)), float(rng.uniform(y0, y1))
            if not all(ok(x, y) for ok in self._placement_filters):
                continue
            gap = min((math.hypot(x - px, y - py) for px, py in placed), default=math.inf)
            if gap >= cfg.min_spacing_m:
                return x, y
            if gap > best_gap:
                best, best_gap = (x, y), gap
        if best is None:
            raise CommandError("no free position left for a PoI in the random_pois region")
        return best

    # -------------------------------------------------------------- lifecycle
    def start(self) -> None:
        """Publish the initial events. Subscribe to ``world.events`` before calling this."""
        if self._started:
            return
        if not self.state.uavs:
            raise RuntimeError("uavs.count is 'auto' but no fleet was spawned - "
                               "run it through simulation.runner.Simulation, which sizes the fleet")
        self._started = True
        st = self.state
        self.publish(EventType.SIM_STARTED,
                     f"Scenario '{self.scenario.name}' started: {len(st.uavs)} UAVs, "
                     f"{len(st.pois)} PoIs, seed {self.seed}, duration {self.duration_s:.0f}s",
                     data={"scenario": self.scenario.name, "seed": self.seed, "fleet": dict(self.fleet),
                           "timeline": [{"at_s": t.at_s, "action": t.action} for t in
                                        sorted(self.timeline, key=lambda t: t.at_s)]})
        if self.fleet["sizing"] == "auto":
            f = self.fleet
            capped = f" (capped at max_count {f['uavs']}, {f['required']} needed)" if f["capped"] else ""
            self.publish(EventType.FLEET_PLANNED,
                         f"Fleet sized to the mission ({f['pois']} PoIs): {f['uavs']} UAVs = {f['surveyors']} survey + "
                         f"{f['relays']} relay + {f['spares']} spare + {f['fault_reserve']} fault reserve{capped}",
                         severity=Severity.WARNING if f["capped"] else Severity.INFO, data=dict(f))
        for uav in st.uavs.values():
            x, y, z = uav.position
            self.publish(EventType.UAV_SPAWNED, f"{uav.name} spawned at ({x:.1f}, {y:.1f}, {z:.1f})",
                         uav_id=uav.uav_id, data={"position_m": [float(x), float(y), float(z)],
                                                  "battery_pct": uav.battery_pct})
        for poi in st.pois:
            self.publish(EventType.POI_CREATED,
                         f"{poi.poi_id} at ({poi.position[0]:.0f}, {poi.position[1]:.0f}) "
                         f"priority {poi.priority}, survey {poi.survey_time_s:.0f}s",
                         poi_id=poi.poi_id, data={"priority": poi.priority})

    def stop(self, reason: str = "duration reached") -> None:
        if self._stopped:
            return
        self._stopped = True
        pois = self.state.pois
        self.publish(EventType.SIM_STOPPED,
                     f"Simulation stopped ({reason}): {len(pois.completed())}/{len(pois)} PoIs completed",
                     data={"reason": reason, "completion_rate": pois.completion_rate})

    def step(self) -> None:
        """Advance the world by one time step ``dt``."""
        if self._stopped:
            raise RuntimeError("World has been stopped")
        self.start()
        self._fire_due_triggers()

        st, dt = self.state, self.dt
        st.tick += 1
        st.time_s = st.tick * dt  # derived from the tick count -> no float drift

        for uav in st.uavs.values():
            report = uav.step(dt, self.params.uav, self.params.battery)
            if report.arrived:
                self._on_arrived(uav)
            if report.battery_low:
                self.publish(EventType.BATTERY_LOW, f"{uav.name} battery low ({uav.battery_pct:.1f}%)",
                             severity=Severity.WARNING, uav_id=uav.uav_id)
            if report.battery_critical:
                self.publish(EventType.BATTERY_CRITICAL, f"{uav.name} battery critical ({uav.battery_pct:.1f}%)",
                             severity=Severity.CRITICAL, uav_id=uav.uav_id)
            if report.battery_depleted:
                self.fail_uav(uav.uav_id, "battery depleted")

        self._charge(dt)
        self._update_surveys(dt)

    def run(self, controller: Optional[Controller] = None, until_s: Optional[float] = None) -> None:
        """Run headless (no real-time pacing). ``controller(world)`` runs before every step."""
        end = self.duration_s if until_s is None else min(until_s, self.duration_s)
        self.start()
        while self.t < end - 1e-9 and not self._stopped:
            if controller is not None:
                controller(self)
            self.step()
        if self.t >= self.duration_s - 1e-9:
            self.stop()

    # ------------------------------------------------------------ command API
    def assign_poi(self, uav_id: int, poi_id: str, reason: str = "",
                   altitude_m: Optional[float] = None) -> None:
        """Give ``uav_id`` the survey task ``poi_id`` (role -> SURVEY, fly to the PoI)."""
        uav = self._taskable_uav(uav_id)
        poi = self._poi(poi_id)
        if poi.is_completed:
            raise CommandError(f"{poi_id} is already completed")
        if poi.assigned_uav not in (None, uav_id):
            raise CommandError(f"{poi_id} is already assigned to UAV {poi.assigned_uav}; release it first")
        if uav.assigned_poi == poi_id:
            return
        waypoint = poi.survey_waypoint(self.params.uav.default_altitude_m)
        if altitude_m is not None:
            waypoint[2] = self._check_altitude(altitude_m)
        if uav.assigned_poi is not None:
            self._release_poi(uav, f"re-tasked to {poi_id}")
        poi.assign(uav_id, self.t)
        uav.assigned_poi = poi_id
        uav.goto(waypoint)
        distance = uav.distance_to(waypoint)
        self.publish(EventType.POI_ASSIGNED,
                     f"{poi_id} -> {uav.name} ({distance:.0f} m away)" + (f" [{reason}]" if reason else ""),
                     uav_id=uav_id, poi_id=poi_id, data={"distance_m": round(distance, 1), "reason": reason})
        self._set_role(uav, UAVRole.SURVEY, f"assigned {poi_id}")

    def release_uav(self, uav_id: int, reason: str = "") -> None:
        """Take a UAV off its task: PoI back to PENDING, role IDLE, hover in place."""
        uav = self._uav(uav_id)
        if uav.role in _BUSY_HOME_ROLES:
            raise CommandError(f"{uav.name} is {uav.role.value}")
        if uav.assigned_poi is not None:
            self._release_poi(uav, reason or "UAV released")
        if uav.is_operational:
            uav.hold()
        self._set_role(uav, UAVRole.IDLE, reason or "released")

    def set_role(self, uav_id: int, role: UAVRole | str, reason: str = "") -> None:
        """Set IDLE / RELAY / BACKUP. Use assign_poi() for SURVEY and return_home() for RETURNING."""
        uav = self._taskable_uav(uav_id)
        try:
            role = UAVRole(role)
        except ValueError:
            raise CommandError(f"unknown role '{role}'") from None
        if role not in (UAVRole.IDLE, UAVRole.RELAY, UAVRole.BACKUP):
            raise CommandError(f"set_role() cannot set {role.value}; use assign_poi() or return_home()")
        if uav.assigned_poi is not None:
            self._release_poi(uav, reason or f"role changed to {role.value}")
            uav.hold()  # stop flying to the old PoI; the swarm layer sends the next waypoint
        self._set_role(uav, role, reason)

    def goto(self, uav_id: int, position, reason: str = "") -> None:
        """Send a UAV to a waypoint (e.g. a relay position) without changing its task."""
        uav = self._taskable_uav(uav_id)
        target = self._check_waypoint(position)
        uav.goto(target)
        self.publish(EventType.UAV_COMMANDED,
                     f"{uav.name} -> ({target[0]:.0f}, {target[1]:.0f}, {target[2]:.0f})"
                     + (f" [{reason}]" if reason else ""),
                     uav_id=uav_id, data={"target_m": target.round(2).tolist(), "reason": reason})

    def set_altitude(self, uav_id: int, altitude_m: float, reason: str = "") -> None:
        """Safety override: change only the altitude of the current waypoint (not while landing)."""
        uav = self._operational_uav(uav_id)
        if uav.target is None or uav.uav_id in self._landing or not uav.is_airborne:
            raise CommandError(f"{uav.name} has no waypoint whose altitude can be changed")
        uav.target[2] = self._check_altitude(altitude_m)
        self.publish(EventType.UAV_COMMANDED, f"{uav.name} altitude -> {altitude_m:.0f} m"
                     + (f" [{reason}]" if reason else ""),
                     uav_id=uav_id, data={"target_m": uav.target.round(2).tolist(), "reason": reason})

    def set_brake(self, uav_id: int, on: bool, reason: str = "") -> None:
        """Collision avoidance: stop moving horizontally (``on``) or carry on to the waypoint."""
        uav = self._operational_uav(uav_id)
        if uav.braking == on:
            return
        uav.braking = on
        if on:
            self.publish(EventType.UAV_COMMANDED, f"{uav.name} holding position" + (f" [{reason}]" if reason else ""),
                         uav_id=uav_id, data={"brake": True, "reason": reason})

    def hold(self, uav_id: int, reason: str = "") -> None:
        uav = self._taskable_uav(uav_id)
        uav.hold()
        self.publish(EventType.UAV_COMMANDED, f"{uav.name} -> HOLD" + (f" [{reason}]" if reason else ""),
                     uav_id=uav_id, data={"target_m": None, "reason": reason})

    def return_home(self, uav_id: int, reason: str = "", altitude_m: Optional[float] = None) -> None:
        """Fly to the home pad at ``altitude_m``, land, then recharge (RETURNING -> CHARGING -> IDLE)."""
        uav = self._operational_uav(uav_id)
        if uav.role in _BUSY_HOME_ROLES:
            return
        if uav.assigned_poi is not None:
            self._release_poi(uav, reason or "returning home")
        home = uav.home
        if not uav.is_airborne and uav.horizontal_distance_to(home) <= self.params.uav.arrival_radius_m:
            uav.hold()
            self._set_role(uav, UAVRole.CHARGING, reason or "on home pad")
            return
        alt = self._check_altitude(altitude_m if altitude_m is not None else self.params.uav.rth_altitude_m)
        self._landing.discard(uav_id)
        uav.goto((home[0], home[1], alt))
        distance = uav.horizontal_distance_to(home)
        self.publish(EventType.RTH_STARTED,
                     f"{uav.name} returning home ({uav.battery_pct:.1f}% battery, {distance:.0f} m)"
                     + (f" [{reason}]" if reason else ""),
                     severity=Severity.WARNING, uav_id=uav_id,
                     data={"battery_pct": round(uav.battery_pct, 2), "distance_m": round(distance, 1),
                           "reason": reason})
        self._set_role(uav, UAVRole.RETURNING, reason)

    def fail_uav(self, uav_id: int, reason: str = "failure") -> None:
        """Remove a UAV from the mission; its PoI is released for re-tasking."""
        uav = self._uav(uav_id)
        if not uav.is_operational:
            return
        role_before = uav.role
        uav.mark_failed()
        self._landing.discard(uav_id)
        self.publish(EventType.UAV_FAILED, f"{uav.name} FAILED: {reason} (was {role_before.value})",
                     severity=Severity.CRITICAL, uav_id=uav_id,
                     data={"reason": reason, "role_before": role_before.value, "battery_pct": uav.battery_pct})
        if uav.assigned_poi is not None:
            self._release_poi(uav, f"{uav.name} failed")
        self._set_role(uav, UAVRole.IDLE, "UAV failed")

    def complete_poi(self, poi_id: str, early: bool = True, reason: str = "") -> None:
        """Mark a PoI completed now (Scenario C) and release the UAV that was surveying it."""
        poi = self._poi(poi_id)
        if poi.is_completed:
            raise CommandError(f"{poi_id} is already completed")
        self._finish_poi(poi, early=early, reason=reason)

    def add_poi(self, poi_id: str, position, priority: int, survey_time_s: float,
                altitude_m: Optional[float] = None, reason: str = "") -> PoI:
        """A new region of interest appears during the mission."""
        if poi_id in self.state.pois:
            raise CommandError(f"PoI '{poi_id}' already exists")
        xy = np.asarray(position, dtype=float).reshape(-1)
        if xy.shape not in ((2,), (3,)) or not self.state.area.contains(xy[0], xy[1]):
            raise CommandError(f"PoI position {position!r} is invalid or outside the operating area")
        try:
            poi = PoI(poi_id, xy[:2], int(priority), float(survey_time_s), altitude_m, created_at_s=self.t)
        except ValueError as exc:
            raise CommandError(str(exc)) from None
        self.state.pois.add(poi)
        self.publish(EventType.POI_ADDED,
                     f"NEW {poi_id} at ({xy[0]:.0f}, {xy[1]:.0f}) priority {poi.priority}, "
                     f"survey {poi.survey_time_s:.0f}s" + (f" [{reason}]" if reason else ""),
                     severity=Severity.WARNING if poi.priority >= 4 else Severity.INFO,
                     poi_id=poi_id, data={"priority": poi.priority, "reason": reason})
        return poi

    def set_radio_health(self, uav_id: int, quality: float, reason: str = "") -> None:
        """Inject (or clear) a radio fault: every link of this UAV is scaled by ``quality``."""
        uav = self._operational_uav(uav_id)
        quality = float(np.clip(quality, 0.0, 1.0))
        uav.comm.radio_health = quality
        restored = quality >= 1.0
        uav.health = HealthState.HEALTHY if restored else HealthState.DEGRADED
        if restored:
            self.publish(EventType.LINK_RESTORED, f"{uav.name} radio restored" + (f" [{reason}]" if reason else ""),
                         uav_id=uav_id, data={"quality": quality, "reason": reason})
        else:
            self.publish(EventType.LINK_DEGRADED,
                         f"{uav.name} radio degraded to {quality:.0%} (was {uav.role.value})"
                         + (f" [{reason}]" if reason else ""),
                         severity=Severity.WARNING, uav_id=uav_id,
                         data={"quality": quality, "role": uav.role.value, "reason": reason})

    def set_battery(self, uav_id: int, battery_pct: float, reason: str = "") -> None:
        uav = self._operational_uav(uav_id)
        if not 0.0 < battery_pct <= 100.0:
            raise CommandError("battery_pct must be in (0, 100]")
        uav.battery_pct = float(battery_pct)
        if battery_pct <= self.params.battery.low_pct:
            self.publish(EventType.BATTERY_LOW, f"{uav.name} battery set to {battery_pct:.1f}%"
                         + (f" [{reason}]" if reason else ""), severity=Severity.WARNING, uav_id=uav_id)

    def update_comm(self, uav_id: int, **values: Any) -> None:
        """Write the swarm's network view of a UAV (neighbours, route, PDR ...). No events."""
        uav = self._uav(uav_id)
        allowed = {f.name for f in dc_fields(CommState)} - {"radio_health"}
        unknown = set(values) - allowed
        if unknown:
            raise CommandError(f"update_comm: unknown field(s) {sorted(unknown)}")
        for name, value in values.items():
            setattr(uav.comm, name, value)

    def publish(self, type: EventType, message: str, **kwargs: Any):
        """Publish an event stamped with the current simulation time."""
        return self.events.publish(self.t, type, message, **kwargs)

    def inject(self, action: str, params: Optional[Mapping[str, Any]] = None) -> None:
        """Run a scenario action right now (dashboard buttons, tests)."""
        try:
            parsed = parse_action(action, "inject")
        except ValueError as exc:
            raise CommandError(str(exc)) from None
        self._dispatch(Trigger(self.t, parsed, dict(params or {}), -1))

    def register_trigger_handler(self, action: TriggerAction, handler: TriggerHandler) -> None:
        """Other layers plug scenario actions in here (e.g. add_obstacle from simulation/obstacles.py)."""
        self._trigger_handlers[TriggerAction(action)] = handler

    def register_uav_selector(self, name: str, selector: UAVSelector) -> None:
        """Named UAV selectors usable as ``uav_id`` in triggers, e.g. ``uav_id: critical_relay``."""
        self._uav_selectors[name] = selector

    def register_point_selector(self, name: str, selector: PointSelector) -> None:
        """Named positions usable in triggers, e.g. ``center_m: backbone_midpoint``."""
        self._point_selectors[name] = selector

    def register_placement_filter(self, allowed: PlacementFilter) -> None:
        """A check ``(x, y) -> bool`` every randomly placed PoI must pass, e.g. not inside an obstacle."""
        self._placement_filters.append(allowed)

    def resolve_poi(self, value: Any) -> str:
        """A PoI id, or ``random_active``: a PoI not yet completed, preferring one under survey."""
        if value != RANDOM_ACTIVE_POI:
            return str(value)
        active = self._active_pois()
        for status in (PoIStatus.IN_PROGRESS, PoIStatus.ASSIGNED, PoIStatus.PENDING):
            group = sorted((p for p in active if p.status is status), key=lambda p: p.poi_id)
            if group:
                return group[int(self._choice_rng.integers(len(group)))].poi_id
        raise CommandError(f"selector '{RANDOM_ACTIVE_POI}' matched no PoI (all completed)")

    def resolve_point(self, value: Any) -> np.ndarray:
        """A position from [x, y(, z)] or a registered selector name."""
        if isinstance(value, str):
            selector = self._point_selectors.get(value)
            if selector is None:
                raise CommandError(f"unknown position selector '{value}'")
            point = selector(self)
            if point is None:
                raise CommandError(f"selector '{value}' matched no position")
            value = point
        xy = np.asarray(value, dtype=float).reshape(-1)
        if xy.shape not in ((2,), (3,)) or not np.all(np.isfinite(xy)):
            raise CommandError(f"invalid position {value!r}")
        return xy

    def resolve_uav(self, value: Any) -> int:
        """UAV id from an int, a registered selector name, or ``role:<ROLE>`` (lowest id with that role)."""
        if isinstance(value, str) and not value.strip().isdigit():
            if value in self._uav_selectors:
                uav_id = self._uav_selectors[value](self)
            elif value.startswith("role:"):
                try:
                    role = UAVRole(value[5:].upper())
                except ValueError:
                    raise CommandError(f"unknown role in selector '{value}'") from None
                matches = sorted(u.uav_id for u in self.state.uavs_with_role(role))
                uav_id = matches[0] if matches else None
            else:
                raise CommandError(f"unknown UAV selector '{value}'")
            if uav_id is None:
                raise CommandError(f"selector '{value}' matched no UAV")
            return uav_id
        try:
            return int(value)
        except (TypeError, ValueError):
            raise CommandError(f"invalid uav_id {value!r}") from None

    # --------------------------------------------------------------- internal
    def _uav(self, uav_id: int) -> UAV:
        try:
            return self.state.get_uav(uav_id)
        except KeyError as exc:
            raise CommandError(exc.args[0]) from None

    def _operational_uav(self, uav_id: int) -> UAV:
        uav = self._uav(uav_id)
        if not uav.is_operational:
            raise CommandError(f"{uav.name} is FAILED and cannot accept commands")
        return uav

    def _taskable_uav(self, uav_id: int) -> UAV:
        uav = self._operational_uav(uav_id)
        if uav.role in _BUSY_HOME_ROLES:
            raise CommandError(f"{uav.name} is {uav.role.value} and cannot be tasked")
        return uav

    def _poi(self, poi_id: str) -> PoI:
        try:
            return self.state.pois.get(poi_id)
        except KeyError as exc:
            raise CommandError(exc.args[0]) from None

    def _check_altitude(self, altitude_m: float) -> float:
        if not 0.0 <= altitude_m <= self.params.uav.max_altitude_m:
            raise CommandError(f"altitude {altitude_m} m outside 0..{self.params.uav.max_altitude_m} m")
        return float(altitude_m)

    def _check_waypoint(self, position) -> np.ndarray:
        target = np.asarray(position, dtype=float).reshape(-1)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise CommandError(f"waypoint must be a finite (x, y, z), got {position!r}")
        if not self.state.area.contains(target[0], target[1]):
            raise CommandError(f"waypoint ({target[0]:.1f}, {target[1]:.1f}) is outside the operating area")
        self._check_altitude(target[2])
        return target

    def _set_role(self, uav: UAV, role: UAVRole, reason: str) -> None:
        if uav.role is role:
            return
        old = uav.role
        uav.role = role
        self.publish(EventType.ROLE_CHANGED,
                     f"{uav.name} {old.value} -> {role.value}" + (f" ({reason})" if reason else ""),
                     uav_id=uav.uav_id, data={"old": old.value, "new": role.value, "reason": reason})

    def _release_poi(self, uav: UAV, reason: str) -> None:
        poi = self.state.pois.get(uav.assigned_poi)
        uav.assigned_poi = None
        if poi.is_completed:
            return
        poi.release()
        self.publish(EventType.POI_RELEASED,
                     f"{poi.poi_id} released by {uav.name} at {poi.progress_ratio:.0%} ({reason})",
                     severity=Severity.WARNING, uav_id=uav.uav_id, poi_id=poi.poi_id,
                     data={"progress_ratio": round(poi.progress_ratio, 3), "reason": reason})

    def _on_arrived(self, uav: UAV) -> None:
        if uav.role is UAVRole.RETURNING:
            if uav.uav_id not in self._landing:  # above the home pad -> descend
                self._landing.add(uav.uav_id)
                uav.goto((uav.home[0], uav.home[1], 0.0))
            else:                                # touchdown
                self._landing.discard(uav.uav_id)
                self.publish(EventType.UAV_LANDED, f"{uav.name} landed at home ({uav.battery_pct:.1f}%)",
                             uav_id=uav.uav_id, data={"battery_pct": round(uav.battery_pct, 2)})
                self._set_role(uav, UAVRole.CHARGING, "landed")
            return
        where = f" ({uav.assigned_poi})" if uav.assigned_poi else ""
        self.publish(EventType.UAV_ARRIVED, f"{uav.name} reached its waypoint{where}",
                     uav_id=uav.uav_id, poi_id=uav.assigned_poi)

    def _charge(self, dt: float) -> None:
        bat = self.params.battery
        for uav in self.state.uavs_with_role(UAVRole.CHARGING):
            if uav.is_airborne:
                continue  # still touching down
            uav.battery_pct = min(100.0, uav.battery_pct + bat.charge_rate_pct_per_min * dt / 60.0)
            if uav.battery_pct >= bat.resume_pct:
                uav.hold()
                self.publish(EventType.CHARGING_COMPLETE, f"{uav.name} recharged to {uav.battery_pct:.0f}%",
                             uav_id=uav.uav_id)
                self._set_role(uav, UAVRole.IDLE, "recharged")

    def _update_surveys(self, dt: float) -> None:
        radius = self.params.uav.arrival_radius_m
        for uav in self.state.uavs_with_role(UAVRole.SURVEY):
            if uav.assigned_poi is None or uav.target is None:
                continue
            poi = self.state.pois.get(uav.assigned_poi)
            # Survey time only accrues while the UAV is on station above the PoI.
            if uav.horizontal_distance_to(poi.position) > radius or uav.distance_to(uav.target) > radius:
                continue
            if poi.status is PoIStatus.ASSIGNED:
                if poi.progress_s > 0:
                    verb = f"resumed surveying {poi.poi_id} at {poi.progress_ratio:.0%}"
                else:
                    verb = f"started surveying {poi.poi_id}"
                self.publish(EventType.POI_SURVEY_STARTED,
                             f"{uav.name} {verb} (needs {poi.survey_time_s - poi.progress_s:.0f}s more)",
                             uav_id=uav.uav_id, poi_id=poi.poi_id)
            if poi.add_survey_time(dt, self.t):
                self._on_poi_finished(poi, uav, early=False, reason="survey time reached")

    def _finish_poi(self, poi: PoI, early: bool, reason: str) -> None:
        uav_id = poi.assigned_uav
        poi.complete(self.t, early=early)
        uav = self.state.uavs.get(uav_id) if uav_id is not None else None
        self._on_poi_finished(poi, uav, early=early, reason=reason)

    def _on_poi_finished(self, poi: PoI, uav: Optional[UAV], early: bool, reason: str) -> None:
        by = uav.name if uav else "no UAV"
        took = "" if poi.first_assigned_at_s is None else f", {self.t - poi.first_assigned_at_s:.1f}s after assignment"
        self.publish(EventType.POI_COMPLETED,
                     f"{poi.poi_id} completed{' EARLY' if early else ''} by {by} "
                     f"({poi.progress_ratio:.0%} surveyed{took})" + (f" [{reason}]" if reason else ""),
                     uav_id=uav.uav_id if uav else None, poi_id=poi.poi_id,
                     data={"early": early, "progress_ratio": round(poi.progress_ratio, 3), "reason": reason,
                           "priority": poi.priority})
        if uav is not None and uav.is_operational:
            uav.assigned_poi = None
            uav.hold()
            self._set_role(uav, UAVRole.IDLE, f"{poi.poi_id} done")

    # --------------------------------------------------------------- triggers
    def _fire_due_triggers(self) -> None:
        for trig in self._triggers.pop_due(self.t):
            self._dispatch(trig)

    def _active_pois(self) -> list[PoI]:
        return [p for p in self.state.pois if not p.is_completed]

    def _missing_target(self, trig: Trigger) -> Optional[str]:
        """Why this trigger's selectors match nothing right now, or None if they all resolve."""
        params = trig.params
        try:
            uav = params.get("uav_id")
            if isinstance(uav, str) and not uav.strip().isdigit():
                self.resolve_uav(uav)
            if isinstance(params.get("center_m"), str):
                self.resolve_point(params["center_m"])
            if params.get("poi_id") == RANDOM_ACTIVE_POI and not self._active_pois():
                raise CommandError(f"selector '{RANDOM_ACTIVE_POI}' matched no PoI")
        except CommandError as exc:
            return str(exc)
        return None

    def _dispatch(self, trig: Trigger) -> None:
        action = trig.action.value
        handler = self._trigger_handlers.get(trig.action)
        if handler is None:
            self.publish(EventType.TRIGGER_REJECTED, f"No handler registered for '{action}' - ignored",
                         severity=Severity.WARNING, data={"action": action, "params": dict(trig.params)})
            return
        retry_until = self._retry_until.get(trig.index)
        if (retry_until is not None and self.t + _RETRY_S <= retry_until + 1e-9
                and self._missing_target(trig) is not None):
            self._triggers.defer(trig, self.t + _RETRY_S)
            return
        data: dict[str, Any] = {"action": action, "params": dict(trig.params)}
        note = ""
        planned = self._planned_s.get(trig.index)
        if planned is not None and self.t - planned > 1e-6:
            data["planned_s"] = planned
            note = f" (planned for {planned:.1f}s, waited {self.t - planned:.1f}s for a target)"
        self.publish(EventType.TRIGGER_FIRED, f"Scenario trigger '{action}' {dict(trig.params)}{note}", data=data)
        try:
            handler(trig)
        except CommandError as exc:
            self.publish(EventType.TRIGGER_REJECTED, f"Trigger '{action}' rejected: {exc}",
                         severity=Severity.WARNING, data={"action": action, "error": str(exc)})

    @staticmethod
    def _param(trig: Trigger, name: str, kind: type, default: Any = ...) -> Any:
        if name not in trig.params:
            if default is not ...:
                return default
            raise CommandError(f"missing parameter '{name}'")
        try:
            return kind(trig.params[name])
        except (TypeError, ValueError):
            raise CommandError(f"parameter '{name}' must be {kind.__name__}") from None

    def _trigger_uav(self, trig: Trigger) -> int:
        if "uav_id" not in trig.params:
            raise CommandError("missing parameter 'uav_id'")
        return self.resolve_uav(trig.params["uav_id"])

    def _on_complete_poi(self, trig: Trigger) -> None:
        poi_id = self.resolve_poi(self._param(trig, "poi_id", str))
        self.complete_poi(poi_id, early=True, reason="scenario trigger")

    def _on_fail_uav(self, trig: Trigger) -> None:
        self.fail_uav(self._trigger_uav(trig), self._param(trig, "reason", str, "scenario trigger"))

    def _on_set_battery(self, trig: Trigger) -> None:
        self.set_battery(self._trigger_uav(trig), self._param(trig, "battery_pct", float), "scenario trigger")

    def _on_add_poi(self, trig: Trigger) -> None:
        pos = trig.params.get("position_m")
        if pos == RANDOM_POSITION:
            pos = self._sample_poi_xy(self._choice_rng)
        if not isinstance(pos, (list, tuple)):
            raise CommandError(f"parameter 'position_m' must be [x, y] or '{RANDOM_POSITION}'")
        altitude = trig.params.get("altitude_m")
        self.add_poi(self._param(trig, "id", str), pos, self._param(trig, "priority", int, 5),
                     self._param(trig, "survey_time_s", float, 60.0),
                     None if altitude is None else float(altitude), reason="emerging region")

    def _on_degrade_link(self, trig: Trigger) -> None:
        self.set_radio_health(self._trigger_uav(trig), self._param(trig, "quality", float, 0.2), "scenario trigger")

    def _on_restore_link(self, trig: Trigger) -> None:
        self.set_radio_health(self._trigger_uav(trig), 1.0, "scenario trigger")
