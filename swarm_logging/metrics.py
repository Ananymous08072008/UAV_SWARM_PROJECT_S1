"""
swarm_logging/metrics.py
Performance metrics for the evaluation, grouped exactly like the project plan:

  Mission        PoI completion rate and time, allocation time, response time
                 to newly emerging high-priority regions
  Communication  route PDR, latency, network availability, communication
                 downtime, data delivery ratio and delay
  Resilience     fault detection time, recovery time, relay changes, task
                 reallocations, pre-emptions
  Safety         minimum separation, violations, UAVs lost, UAVs landed safely
  Efficiency     distance flown, energy consumed, battery left, relay utilisation

The collector samples the world every ``sample_interval_s`` and listens to the
event bus; ``summary()`` returns the numbers used in the report and by
experiments/plot_results.py.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from statistics import fmean
from typing import TYPE_CHECKING, Any, Optional

from core.events import Event, EventType
from core.uav import UAVRole

if TYPE_CHECKING:
    from core.world import World
    from simulation.environment import Environment
    from swarm.mission_manager import MissionManager

MISSION_ROLES = (UAVRole.SURVEY, UAVRole.RELAY, UAVRole.BACKUP)
HIGH_PRIORITY = 4


def _mean(values) -> Optional[float]:
    values = [v for v in values if v is not None]
    return round(fmean(values), 3) if values else None


class MetricsCollector:
    def __init__(self, world: "World", env: "Environment", manager: "MissionManager",
                 sample_interval_s: float = 1.0) -> None:
        self.world = world
        self.env = env
        self.manager = manager
        self.sample_interval_s = sample_interval_s
        self.samples: list[dict[str, Any]] = []
        self.counts: Counter[str] = Counter()
        self.downtime_s: dict[int, float] = defaultdict(float)
        self.role_seconds: dict[str, float] = defaultdict(float)
        self.airborne_seconds = 0.0
        self.energy_consumed_pct = 0.0
        self.availability_ok = 0
        self.availability_total = 0
        self.response_times_s: list[float] = []
        self._added_at: dict[str, float] = {}
        self._battery: dict[int, float] = {}
        self._next_sample_s = 0.0
        self._last_sample_s = 0.0
        world.events.subscribe(self._on_event)

    # ------------------------------------------------------------------ events
    def _on_event(self, event: Event) -> None:
        self.counts[event.type.value] += 1
        if event.type is EventType.POI_ADDED:
            self._added_at[event.poi_id] = event.t_s
        elif event.type is EventType.POI_ASSIGNED and event.poi_id in self._added_at:
            self.response_times_s.append(event.t_s - self._added_at.pop(event.poi_id))

    # ----------------------------------------------------------------- samples
    def update(self, world: "World") -> None:
        if world.t + 1e-9 < self._next_sample_s:
            return
        dt = max(world.t - self._last_sample_s, world.dt)
        self._next_sample_s = world.t + self.sample_interval_s
        self._last_sample_s = world.t

        operational = world.state.operational_uavs()
        airborne = [u for u in operational if u.is_airborne]
        ferrying = self.manager.allocator.ferrying
        mission = [u for u in airborne if u.role in MISSION_ROLES and u.uav_id not in ferrying]
        connected = [u for u in mission if u.comm.connected]

        for uav in airborne:
            self.airborne_seconds += dt
            self.role_seconds[uav.role.value] += dt
        for uav in mission:
            if not uav.comm.connected:
                self.downtime_s[uav.uav_id] += dt
        if mission:
            self.availability_total += 1
            self.availability_ok += int(len(connected) == len(mission))
        for uav in world.state.uavs.values():
            before = self._battery.get(uav.uav_id, uav.battery_pct)
            if uav.battery_pct < before:
                self.energy_consumed_pct += before - uav.battery_pct
            self._battery[uav.uav_id] = uav.battery_pct

        pois = world.state.pois
        self.samples.append({
            "t_s": round(world.t, 2),
            "airborne": len(airborne),
            "mission_uavs": len(mission),
            "ferrying_uavs": len(ferrying),
            "connected_uavs": len(connected),
            "connectivity_ratio": round(len(connected) / len(mission), 3) if mission else None,
            "mean_route_pdr": _mean([u.comm.pdr for u in connected]),
            "mean_latency_ms": _mean([u.comm.latency_ms for u in connected]),
            "relays": len(world.state.uavs_with_role(UAVRole.RELAY)),
            "pois_completed": len(pois.completed()),
            "pois_pending": len(pois.pending()),
            "mean_battery_pct": _mean([u.battery_pct for u in operational]),
            "data_delivered_mb": round(self.env.data.delivered_mb, 1),
            "open_incidents": len(self.manager.reconfig.open_incidents()),
        })

    # ----------------------------------------------------------------- results
    def summary(self) -> dict[str, Any]:
        world, pois = self.world, self.world.state.pois
        completed = pois.completed()
        incidents = self.manager.reconfig.incidents
        recovered = [i for i in incidents if i.recovery_time_s is not None]
        detected = [i for i in incidents if i.detection_time_s is not None]
        allocation_runtimes = self.manager.allocator.runtime_ms
        operational = world.state.operational_uavs()
        data = self.env.data.stats()
        safety = self.manager.safety

        return {
            "run": {
                "scenario": world.scenario.name,
                "mode": self.manager.mode,
                "seed": world.seed,
                "duration_s": round(world.t, 1),
                "uavs": len(world.state.uavs),
                "fleet_sizing": world.fleet["sizing"],
                **{f"fleet_{k}": world.fleet[k] for k in ("surveyors", "relays", "spares", "fault_reserve")
                   if k in world.fleet},
            },
            "mission": {
                "pois_total": len(pois),
                "pois_completed": len(completed),
                "completion_rate": round(pois.completion_rate, 3),
                "mission_complete_s": self.manager.mission_complete_s,
                "mean_completion_time_s": _mean([p.completed_at_s - p.created_at_s for p in completed]),
                "max_completion_time_s": round(max((p.completed_at_s - p.created_at_s for p in completed),
                                                   default=0.0), 1),
                "high_priority_response_s": _mean(self.response_times_s),
                "mean_allocation_runtime_ms": _mean(allocation_runtimes),
                "task_reallocations": self.counts.get(EventType.POI_RELEASED.value, 0),
                "preemptions": self.counts.get(EventType.PREEMPTION.value, 0),
            },
            "communication": {
                "mean_route_pdr": _mean([s["mean_route_pdr"] for s in self.samples]),
                "mean_latency_ms": _mean([s["mean_latency_ms"] for s in self.samples]),
                "mean_connectivity_ratio": _mean([s["connectivity_ratio"] for s in self.samples]),
                "network_availability": _mean([s["connectivity_ratio"] for s in self.samples]),
                "full_connectivity_fraction": round(self.availability_ok / self.availability_total, 3)
                if self.availability_total else None,
                "comm_downtime_s": round(sum(self.downtime_s.values()), 1),
                "disconnections": self.counts.get(EventType.UAV_DISCONNECTED.value, 0),
                "route_changes": self.counts.get(EventType.ROUTE_CHANGED.value, 0),
                "data_generated_mb": data["generated_mb"],
                "data_delivered_mb": data["delivered_mb"],
                "data_delivery_ratio": data["delivery_ratio"],
                "data_live_ratio": data["live_ratio"],
                "data_mean_delay_s": data["mean_delay_s"],
            },
            "resilience": {
                "incidents": len(incidents),
                "incidents_recovered": len(recovered),
                "incidents_unrecovered": len(incidents) - len(recovered),
                "mean_detection_time_s": _mean([i.detection_time_s for i in detected]),
                "mean_recovery_time_s": _mean([i.recovery_time_s for i in recovered]),
                "max_recovery_time_s": round(max((i.recovery_time_s for i in recovered), default=0.0), 2),
                # per affected UAV: how many got their route back while still on task, and how fast
                "affected_uavs": sum(len(i.affected) for i in incidents),
                "reconnected_share": round(sum(len(i.reconnected) for i in incidents)
                                           / sum(len(i.affected) for i in incidents), 3)
                if any(i.affected for i in incidents) else None,
                "mean_reconnect_time_s": _mean([s for i in incidents for s in i.reconnect_s.values()]),
                "relay_changes": self.counts.get(EventType.RELAY_ASSIGNED.value, 0),
                "relay_handovers": self.manager.energy.handovers,
                "replans": self.manager.relays.replans,
            },
            "safety": {
                "min_separation_m": round(safety.min_separation_observed_m, 2)
                if safety.min_separation_observed_m != float("inf") else None,
                "separation_violations": safety.violations.get("separation", 0),
                "collisions": safety.collisions,
                "avoidance_manoeuvres": safety.deconflictions,
                "geofence_violations": safety.violations.get("geofence", 0),
                "obstacle_violations": safety.violations.get("obstacle", 0),
                "uavs_lost": len(world.state.uavs) - len(operational),
                "uavs_airborne_at_end": sum(1 for u in operational if u.is_airborne),
            },
            "efficiency": {
                "distance_total_m": round(sum(u.distance_travelled_m for u in world.state.uavs.values()), 1),
                "flight_time_total_s": round(sum(u.flight_time_s for u in world.state.uavs.values()), 1),
                "energy_consumed_pct": round(self.energy_consumed_pct, 1),
                "mean_battery_left_pct": _mean([u.battery_pct for u in operational]),
                "relay_utilisation": round(self.role_seconds.get(UAVRole.RELAY.value, 0.0) / self.airborne_seconds, 3)
                if self.airborne_seconds else None,
                "rth_count": self.counts.get(EventType.RTH_STARTED.value, 0),
            },
        }

    def live(self) -> dict[str, Any]:
        """Compact view for the dashboard."""
        latest = self.samples[-1] if self.samples else {}
        incidents = self.manager.reconfig.incidents
        recovered = [i for i in incidents if i.recovery_time_s is not None]
        pois = self.world.state.pois
        return {
            "t_s": round(self.world.t, 1),
            "pois_completed": len(pois.completed()),
            "pois_total": len(pois),
            "connectivity_ratio": latest.get("connectivity_ratio"),
            "mean_route_pdr": latest.get("mean_route_pdr"),
            "mean_latency_ms": latest.get("mean_latency_ms"),
            "network_availability": _mean([s["connectivity_ratio"] for s in self.samples[-30:]]),
            "data": self.env.data.stats(),
            "incidents": len(incidents),
            "incidents_open": len(self.manager.reconfig.open_incidents()),
            "mean_recovery_time_s": _mean([i.recovery_time_s for i in recovered]),
            "relay_changes": self.counts.get(EventType.RELAY_ASSIGNED.value, 0),
            "safety_violations": sum(self.manager.safety.violations.values()),
            "uavs_lost": len(self.world.state.uavs) - len(self.world.state.operational_uavs()),
        }

    def rows(self) -> list[dict[str, Any]]:
        return self.samples
