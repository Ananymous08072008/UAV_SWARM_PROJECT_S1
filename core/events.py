"""
core/events.py
Two kinds of events flow through the platform:

1. Event records  (EventBus)
   Immutable facts about what already happened: UAV arrived, role changed,
   relay replaced, PoI completed ... The dashboard event log, metrics and the
   database logger all consume these.

2. Scenario triggers  (TriggerSchedule)
   Timed *inputs* from a scenario file or the dashboard: complete a PoI early,
   fail a UAV, degrade a radio, add an obstacle, add a new high-priority PoI ...
   When a trigger is due the World runs the handler registered for its action;
   the handler's effects show up as event records.
"""

from __future__ import annotations

import bisect
import logging
from collections import Counter, deque
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Any, Callable, Iterable, Mapping, Optional

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Event records
# ---------------------------------------------------------------------------
class Severity(str, Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"


class EventType(str, Enum):
    # simulation lifecycle
    SIM_STARTED = "SIM_STARTED"
    SIM_STOPPED = "SIM_STOPPED"
    FLEET_PLANNED = "FLEET_PLANNED"          # uavs.count: auto - how many UAVs and why
    # UAV
    UAV_SPAWNED = "UAV_SPAWNED"
    UAV_COMMANDED = "UAV_COMMANDED"
    UAV_ARRIVED = "UAV_ARRIVED"
    ROLE_CHANGED = "ROLE_CHANGED"
    BATTERY_LOW = "BATTERY_LOW"
    BATTERY_CRITICAL = "BATTERY_CRITICAL"
    UAV_FAILED = "UAV_FAILED"
    RTH_STARTED = "RTH_STARTED"
    UAV_LANDED = "UAV_LANDED"
    CHARGING_COMPLETE = "CHARGING_COMPLETE"
    # Points of Interest
    POI_CREATED = "POI_CREATED"
    POI_ADDED = "POI_ADDED"                  # new PoI appeared during the mission
    POI_ASSIGNED = "POI_ASSIGNED"
    POI_SURVEY_STARTED = "POI_SURVEY_STARTED"
    POI_RELEASED = "POI_RELEASED"
    POI_COMPLETED = "POI_COMPLETED"
    PREEMPTION = "PREEMPTION"                # UAV pulled off a PoI for a higher-priority one
    # communication network
    LINK_DEGRADED = "LINK_DEGRADED"          # ground-truth fault injected (radio)
    LINK_RESTORED = "LINK_RESTORED"
    OBSTACLE_ADDED = "OBSTACLE_ADDED"
    OBSTACLE_REMOVED = "OBSTACLE_REMOVED"
    UAV_DISCONNECTED = "UAV_DISCONNECTED"    # no route to the GCS
    UAV_RECONNECTED = "UAV_RECONNECTED"
    ROUTE_CHANGED = "ROUTE_CHANGED"
    RELAY_ASSIGNED = "RELAY_ASSIGNED"
    RELAY_RELEASED = "RELAY_RELEASED"
    HANDOVER_STARTED = "HANDOVER_STARTED"    # relay leaving, replacement requested
    # swarm resilience
    FAULT_DETECTED = "FAULT_DETECTED"
    RECONFIGURATION = "RECONFIGURATION"
    RECOVERY_COMPLETE = "RECOVERY_COMPLETE"
    # safety / mission
    SAFETY_VIOLATION = "SAFETY_VIOLATION"
    MISSION_RECALL = "MISSION_RECALL"
    # scenario triggers
    TRIGGER_FIRED = "TRIGGER_FIRED"
    TRIGGER_REJECTED = "TRIGGER_REJECTED"


@dataclass(frozen=True)
class Event:
    seq: int
    t_s: float
    type: EventType
    message: str
    severity: Severity = Severity.INFO
    uav_id: Optional[int] = None
    poi_id: Optional[str] = None
    data: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """JSON-ready representation (dashboard / database)."""
        return {
            "seq": self.seq,
            "t_s": round(self.t_s, 3),
            "type": self.type.value,
            "severity": self.severity.value,
            "message": self.message,
            "uav_id": self.uav_id,
            "poi_id": self.poi_id,
            "data": dict(self.data),
        }

    def __str__(self) -> str:
        return f"[t={self.t_s:7.1f}s] {self.severity.value:<8} {self.type.value:<18} {self.message}"


Subscriber = Callable[[Event], None]


class EventBus:
    """Synchronous publish/subscribe bus with a bounded in-memory history.

    Subscribers run inside ``publish``; an exception in one subscriber is
    logged and does not stop the simulation or the other subscribers.
    """

    def __init__(self, history_size: int = 2000) -> None:
        self._history: deque[Event] = deque(maxlen=history_size)
        self._subscribers: list[tuple[Subscriber, Optional[frozenset[EventType]]]] = []
        self._counts: Counter[EventType] = Counter()
        self._seq = 0

    def subscribe(self, callback: Subscriber,
                  types: Optional[Iterable[EventType]] = None) -> Callable[[], None]:
        """Register ``callback`` (optionally only for ``types``). Returns an unsubscribe function."""
        entry = (callback, frozenset(types) if types is not None else None)
        self._subscribers.append(entry)

        def unsubscribe() -> None:
            if entry in self._subscribers:
                self._subscribers.remove(entry)

        return unsubscribe

    def publish(self, t_s: float, type: EventType, message: str, *,
                severity: Severity = Severity.INFO, uav_id: Optional[int] = None,
                poi_id: Optional[str] = None, data: Optional[Mapping[str, Any]] = None) -> Event:
        self._seq += 1
        event = Event(self._seq, t_s, type, message, severity, uav_id, poi_id, dict(data or {}))
        self._history.append(event)
        self._counts[type] += 1
        # Iterate over a copy so subscribers may (un)subscribe while handling.
        for callback, types in tuple(self._subscribers):
            if types is not None and type not in types:
                continue
            try:
                callback(event)
            except Exception:
                log.exception("Event subscriber %r failed while handling %s", callback, type.value)
        return event

    def history(self, since_seq: int = 0, types: Optional[Iterable[EventType]] = None) -> list[Event]:
        """Events with ``seq > since_seq`` still held in memory, oldest first."""
        wanted = frozenset(types) if types is not None else None
        return [e for e in self._history
                if e.seq > since_seq and (wanted is None or e.type in wanted)]

    def count(self, type: EventType) -> int:
        """Total events of ``type`` ever published (not limited by history size)."""
        return self._counts[type]

    def counts(self) -> dict[str, int]:
        return {t.value: n for t, n in sorted(self._counts.items(), key=lambda kv: kv[0].value)}

    @property
    def last_seq(self) -> int:
        return self._seq


# ---------------------------------------------------------------------------
# Scenario triggers
# ---------------------------------------------------------------------------
class TriggerAction(str, Enum):
    COMPLETE_POI = "complete_poi"        # Scenario C - early PoI completion       {poi_id}
    FAIL_UAV = "fail_uav"                # UAV lost                                 {uav_id, reason}
    SET_BATTERY = "set_battery"          # force a battery level (energy scenario)  {uav_id, battery_pct}
    ADD_POI = "add_poi"                  # new high-priority region                 {id, position_m, priority, survey_time_s}
    DEGRADE_LINK = "degrade_link"        # Scenario A - radio degradation           {uav_id, quality}
    RESTORE_LINK = "restore_link"        #                                          {uav_id}
    ADD_OBSTACLE = "add_obstacle"        # Scenario B - handler in simulation/      {id, polygon_m, height_m, attenuation_db}
    REMOVE_OBSTACLE = "remove_obstacle"  #                                          {id}


@dataclass(frozen=True)
class Trigger:
    at_s: float
    action: TriggerAction
    params: Mapping[str, Any]
    index: int  # position in the timeline; keeps equal-time triggers in file order


TriggerHandler = Callable[[Trigger], None]


class TriggerSchedule:
    """Time-ordered queue of scenario triggers."""

    _EPS = 1e-9

    def __init__(self, triggers: Iterable[Trigger] = ()) -> None:
        self._queue = sorted(triggers, key=lambda tr: (tr.at_s, tr.index))
        self._next = 0

    @classmethod
    def from_specs(cls, specs: Iterable[Any]) -> "TriggerSchedule":
        """Build from objects with ``at_s``, ``action`` (str) and ``params`` (e.g. config.TriggerSpec)."""
        triggers = []
        for i, spec in enumerate(specs):
            triggers.append(Trigger(float(spec.at_s), parse_action(spec.action, f"timeline[{i}]"), dict(spec.params), i))
        return cls(triggers)

    def pop_due(self, t_s: float) -> list[Trigger]:
        """Remove and return every trigger with ``at_s <= t_s``."""
        due = []
        while self._next < len(self._queue) and self._queue[self._next].at_s <= t_s + self._EPS:
            due.append(self._queue[self._next])
            self._next += 1
        return due

    def defer(self, trigger: Trigger, at_s: float) -> None:
        """Put a trigger back to fire again at ``at_s`` (its target did not exist yet)."""
        pending = self._queue[self._next:]
        bisect.insort(pending, replace(trigger, at_s=at_s), key=lambda tr: (tr.at_s, tr.index))
        self._queue[self._next:] = pending

    @property
    def remaining(self) -> int:
        return len(self._queue) - self._next


def parse_action(action: str, where: str = "trigger") -> TriggerAction:
    try:
        return TriggerAction(action)
    except ValueError:
        valid = ", ".join(a.value for a in TriggerAction)
        raise ValueError(f"{where}: unknown action '{action}' (valid: {valid})") from None
