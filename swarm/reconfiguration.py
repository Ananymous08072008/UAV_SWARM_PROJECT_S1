"""
swarm/reconfiguration.py
Detects faults, triggers re-planning and measures recovery.

Faults handled
  * radio degradation - detected from measurements only: a UAV whose links are
    all far below their predicted quality (measured / predicted < anomaly_ratio
    on >= 2 links) for ``detection_window_s``. The UAV is excluded from the
    relay role and the relay plan is rebuilt.
  * UAV failure       - declared after ``failure_timeout_s`` without contact
    (heartbeat loss); its task and relay spot are re-planned.
  * obstacle          - mapped immediately; relays are only moved if routing alone
    cannot keep the surveyors connected (then the planner detours around it).
  * disconnection     - a surveyor on station without a route for
    ``detection_window_s`` forces a re-plan (at most once per
    ``disconnect_backoff_s`` while it stays cut off, so a relay that is still
    flying out is not re-planned away every few seconds).

Every fault opens an Incident with the UAVs that depended on the faulty
element. The incident is recovered when all of them (still flying a mission
role) have a route to the GCS with PDR >= ``recovery_pdr`` again.
Recovery time = recovered - onset; detection time = detected - onset.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Optional

from core.events import Event, EventType, Severity
from core.uav import GCS_NODE_ID, UAVRole

if TYPE_CHECKING:
    from core.world import World
    from simulation.environment import Environment
    from swarm.network_manager import NetworkView
    from swarm.relay_selector import RelaySelector
    from swarm.route_manager import RouteManager

_MISSION_ROLES = (UAVRole.SURVEY, UAVRole.RELAY, UAVRole.BACKUP, UAVRole.IDLE)


@dataclass(frozen=True)
class ReconfigParams:
    enabled: bool = True
    anomaly_ratio: float = 0.6
    min_predicted_pdr: float = 0.5
    detection_window_s: float = 2.0
    failure_timeout_s: float = 2.0
    recovery_pdr: float = 0.7
    incident_timeout_s: float = 300.0
    disconnect_backoff_s: float = 15.0


@dataclass
class Incident:
    incident_id: int
    cause: str
    uav_id: Optional[int]
    onset_s: float
    affected: set[int]
    pending: set[int] = field(default_factory=set)     # affected UAVs that have not reconnected yet
    reconnected: set[int] = field(default_factory=set)  # affected UAVs that got a route back
    reconnect_s: dict[int, float] = field(default_factory=dict)  # uav -> seconds from onset to reconnection
    left: set[int] = field(default_factory=set)         # affected UAVs that left the mission first
    detected_s: Optional[float] = None
    recovered_s: Optional[float] = None
    actions: list[str] = field(default_factory=list)
    closed_reason: str = ""

    @property
    def detection_time_s(self) -> Optional[float]:
        return None if self.detected_s is None else self.detected_s - self.onset_s

    @property
    def recovery_time_s(self) -> Optional[float]:
        return None if self.recovered_s is None else self.recovered_s - self.onset_s

    def to_dict(self) -> dict[str, Any]:
        r = lambda v: None if v is None else round(v, 2)  # noqa: E731
        return {"id": self.incident_id, "cause": self.cause, "uav_id": self.uav_id, "onset_s": r(self.onset_s),
                "detected_s": r(self.detected_s), "recovered_s": r(self.recovered_s),
                "detection_time_s": r(self.detection_time_s), "recovery_time_s": r(self.recovery_time_s),
                "affected": sorted(self.affected), "pending": sorted(self.pending),
                "reconnected": sorted(self.reconnected),
                "reconnect_s": {str(k): round(v, 2) for k, v in sorted(self.reconnect_s.items())},
                "actions": self.actions[-8:], "closed_reason": self.closed_reason}


class ReconfigurationEngine:
    def __init__(self, world: "World", env: "Environment", routes: "RouteManager", relays: "RelaySelector",
                 params: ReconfigParams) -> None:
        self.world = world
        self.env = env
        self.routes = routes
        self.relays = relays
        self.params = params
        self.incidents: list[Incident] = []
        self._anomaly_since: dict[int, float] = {}
        self._diagnosed: set[int] = set()
        self._pending_failures: list[tuple[int, float]] = []
        self._degraded_at: dict[int, float] = {}
        self._disconnected_since: dict[int, float] = {}
        self._disconnect_replanned_at: dict[int, float] = {}
        self._replan_reasons: list[str] = []
        self.ferrying: set[int] = set()   # shared with the allocator: UAVs disconnected by design
        world.events.subscribe(self._on_event, types=[
            EventType.LINK_DEGRADED, EventType.LINK_RESTORED, EventType.UAV_FAILED, EventType.OBSTACLE_ADDED,
            EventType.RELAY_ASSIGNED, EventType.POI_ASSIGNED, EventType.PREEMPTION])

    # ------------------------------------------------------------------ events
    def _open(self, cause: str, uav_id: Optional[int], affected: set[int]) -> Incident:
        inc = Incident(len(self.incidents) + 1, cause, uav_id, self.world.t, set(affected), set(affected))
        self.incidents.append(inc)
        return inc

    def _open_incident_for(self, cause: str, uav_id: Optional[int]) -> Optional[Incident]:
        for inc in self.incidents:
            if inc.recovered_s is None and not inc.closed_reason and inc.cause == cause and inc.uav_id == uav_id:
                return inc
        return None

    def _on_event(self, event: Event) -> None:
        # Incidents are only opened for faults that actually cut UAVs off (affected set not empty).
        t = event.type
        if t is EventType.LINK_DEGRADED:
            self._degraded_at[event.uav_id] = self.world.t
            affected = self.routes.dependents(event.uav_id)
            if affected:
                self._open("radio_degradation", event.uav_id, affected)
        elif t is EventType.LINK_RESTORED:
            self._degraded_at.pop(event.uav_id, None)
            self._diagnosed.discard(event.uav_id)
            self.relays.include(event.uav_id)
        elif t is EventType.UAV_FAILED:
            affected = self.routes.dependents(event.uav_id)
            if affected:
                self._open("uav_failure", event.uav_id, affected)
            if event.data.get("role_before") in ("RELAY", "SURVEY", "BACKUP"):
                self._pending_failures.append((event.uav_id, self.world.t))
        elif t is EventType.OBSTACLE_ADDED:
            affected = self._routes_through_obstacle(event.data)
            if affected:
                self._open("obstacle", None, affected)
            if self.params.enabled:
                # mapped, but no forced re-plan: routing may already route around it; the
                # disconnection detector re-plans if a surveyor actually loses its route
                self._detect_open("obstacle", None, f"obstacle {event.data.get('id')} mapped", self.world.t,
                                  request_replan=False)
        elif t in (EventType.RELAY_ASSIGNED, EventType.POI_ASSIGNED, EventType.PREEMPTION):
            for inc in self.incidents:
                if inc.recovered_s is None and inc.detected_s is not None:
                    inc.actions.append(f"t={self.world.t:.1f}s {event.message}")

    def _routes_through_obstacle(self, data: dict) -> set[int]:
        from simulation.obstacles import Obstacle
        try:
            obs = Obstacle.from_dict(data)
        except ValueError:
            return set()
        positions = self.env.comm.positions
        affected = set()
        for route in self.routes.routes.values():
            for a, b in zip(route.path, route.path[1:]):
                if a in positions and b in positions and obs.blocks(positions[a], positions[b]):
                    affected.add(route.uav_id)
                    break
        return affected

    def _detect_open(self, cause: str, uav_id: Optional[int], detail: str, onset_s: float,
                     request_replan: bool = True) -> None:
        inc = self._open_incident_for(cause, uav_id)
        if inc is not None:
            if inc.detected_s is not None:
                return
            inc.detected_s = self.world.t
            onset_s = inc.onset_s
        detection = self.world.t - onset_s
        who = f" on {self.world.state.uavs[uav_id].name}" if uav_id is not None else ""
        self.world.publish(EventType.FAULT_DETECTED,
                           f"Fault detected{who}: {cause.replace('_', ' ')} ({detail}) after {detection:.1f}s",
                           severity=Severity.WARNING, uav_id=uav_id,
                           data={"incident": inc.incident_id if inc else None, "cause": cause,
                                 "detection_time_s": round(detection, 2)})
        if request_replan:
            self._replan_reasons.append(f"{cause}: {detail}")

    # ---------------------------------------------------------------- per tick
    def evaluate(self, world: "World", view: "NetworkView") -> list[str]:
        """Run detectors. Returns the reasons a re-plan is needed (empty if none)."""
        if self.params.enabled:
            self._detect_radio_anomalies(world)
            self._detect_failures(world)
            self._detect_disconnections(world)
        reasons, self._replan_reasons = self._replan_reasons, []
        return reasons

    def _detect_radio_anomalies(self, world: "World") -> None:
        comm, p = self.env.comm, self.params
        for uav in world.state.operational_uavs():
            uid = uav.uav_id
            if uid not in comm.positions or uid in self._diagnosed:
                self._anomaly_since.pop(uid, None)
                continue
            usable = anomalous = 0
            for lk in comm.links_of(uid):
                other = lk.other(uid)
                predicted = comm.predict_pdr(comm.positions[uid], comm.positions[other], other == GCS_NODE_ID)
                if predicted < p.min_predicted_pdr:
                    continue
                usable += 1
                if lk.pdr / predicted < p.anomaly_ratio:
                    anomalous += 1
            if usable >= 2 and anomalous >= 2 and anomalous >= 0.67 * usable:
                since = self._anomaly_since.setdefault(uid, world.t)
                if world.t - since >= p.detection_window_s:
                    self._diagnosed.add(uid)
                    self.relays.exclude(uid, "radio degraded")
                    self._detect_open("radio_degradation", uid, f"{anomalous}/{usable} links below prediction",
                                      self._degraded_at.get(uid, since))
            else:
                self._anomaly_since.pop(uid, None)

    def _detect_failures(self, world: "World") -> None:
        still = []
        for uid, onset in self._pending_failures:
            if world.t - onset >= self.params.failure_timeout_s:
                self._detect_open("uav_failure", uid, "no heartbeat", onset)
            else:
                still.append((uid, onset))
        self._pending_failures = still

    def _detect_disconnections(self, world: "World") -> None:
        for uav in world.state.uavs_with_role(UAVRole.SURVEY):
            on_station = uav.target is not None and uav.distance_to(uav.target) <= world.params.uav.arrival_radius_m * 2
            if not on_station or uav.comm.connected:
                self._disconnected_since.pop(uav.uav_id, None)
                self._disconnect_replanned_at.pop(uav.uav_id, None)
                continue
            since = self._disconnected_since.setdefault(uav.uav_id, world.t)
            if world.t - since < self.params.detection_window_s:
                continue
            last = self._disconnect_replanned_at.get(uav.uav_id)
            if last is None or world.t - last >= self.params.disconnect_backoff_s:
                self._disconnect_replanned_at[uav.uav_id] = world.t
                self._replan_reasons.append(f"{uav.name} disconnected on station")

    def check_recovery(self, world: "World") -> None:
        for inc in self.incidents:
            if inc.recovered_s is not None or inc.closed_reason:
                continue
            if world.t - inc.onset_s > self.params.incident_timeout_s:
                inc.closed_reason = "not recovered"
                continue
            for uid in list(inc.pending):
                uav = world.state.uavs[uid]
                left_mission = (uid == inc.uav_id or not uav.is_operational or not uav.is_airborne
                                or uav.role not in _MISSION_ROLES or uid in self.ferrying)
                reconnected = uav.comm.connected and uav.comm.pdr >= self.params.recovery_pdr
                if left_mission:      # checked first: a UAV flying home past the GCS is not a recovery
                    inc.left.add(uid)
                    inc.pending.discard(uid)
                elif reconnected:
                    inc.reconnected.add(uid)
                    inc.reconnect_s[uid] = world.t - inc.onset_s
                    inc.pending.discard(uid)
            if inc.pending or (inc.detected_s is None and self.params.enabled):
                continue
            if not inc.reconnected:
                # nobody regained a route: the affected UAVs simply finished or went home
                inc.closed_reason = "not recovered (affected UAVs left the mission)"
                continue
            inc.recovered_s = world.t
            inc.closed_reason = "recovered"
            if inc.detected_s is None and not self.params.enabled:
                inc.closed_reason = "recovered without reconfiguration"
            world.publish(EventType.RECOVERY_COMPLETE,
                          f"Recovered from {inc.cause.replace('_', ' ')} in {inc.recovery_time_s:.1f}s "
                          f"({len(inc.reconnected)}/{len(inc.affected)} affected UAV(s) reconnected)",
                          data={"incident": inc.incident_id, "recovery_time_s": inc.recovery_time_s,
                                "detection_time_s": inc.detection_time_s})

    def open_incidents(self) -> list[Incident]:
        return [i for i in self.incidents if i.recovered_s is None and not i.closed_reason]

    def to_dict(self) -> list[dict[str, Any]]:
        return [i.to_dict() for i in self.incidents[-20:]]
