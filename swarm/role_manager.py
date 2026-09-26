"""
swarm/role_manager.py
The single place where swarm modules change UAV roles.

It checks role transitions against an allowed table, applies safe altitudes
(flight levels + obstacle clearance) to every waypoint, and forwards the change to
the World command API, which validates it again and publishes events.

    IDLE <-> SURVEY <-> RELAY <-> BACKUP  --(energy / deadline)-->  RETURNING -> CHARGING -> IDLE
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Sequence

import numpy as np

from core.events import EventType
from core.uav import UAV, UAVRole

if TYPE_CHECKING:
    from core.poi import PoI
    from core.world import World
    from swarm.safety_manager import SafetyManager

R = UAVRole
ALLOWED_TRANSITIONS: dict[UAVRole, frozenset[UAVRole]] = {
    R.IDLE: frozenset({R.SURVEY, R.RELAY, R.BACKUP, R.RETURNING}),
    R.SURVEY: frozenset({R.IDLE, R.SURVEY, R.RELAY, R.RETURNING}),
    R.RELAY: frozenset({R.IDLE, R.SURVEY, R.RELAY, R.BACKUP, R.RETURNING}),
    R.BACKUP: frozenset({R.IDLE, R.SURVEY, R.RELAY, R.BACKUP, R.RETURNING}),
    R.RETURNING: frozenset({R.CHARGING}),
    R.CHARGING: frozenset({R.IDLE}),
}


class RoleManager:
    def __init__(self, world: "World", safety: "SafetyManager") -> None:
        self.world = world
        self.safety = safety

    @staticmethod
    def can_transition(uav: UAV, new_role: UAVRole) -> bool:
        return uav.is_operational and new_role in ALLOWED_TRANSITIONS[uav.role]

    def _require(self, uav: UAV, new_role: UAVRole) -> None:
        if not self.can_transition(uav, new_role):
            raise ValueError(f"{uav.name}: transition {uav.role.value} -> {new_role.value} not allowed")

    def assign_survey(self, uav: UAV, poi: "PoI", reason: str) -> None:
        self._require(uav, R.SURVEY)
        alt = self.safety.safe_altitude(uav, poi.position, poi.altitude_m)   # a PoI may ask for its own altitude
        self.world.assign_poi(uav.uav_id, poi.poi_id, reason, altitude_m=alt)

    def assign_relay(self, uav: UAV, point: Sequence[float], reason: str, serves: Sequence[str] = ()) -> None:
        self._require(uav, R.RELAY)
        alt = self.safety.safe_altitude(uav, point, point[2] if len(point) > 2 else None)
        target = np.array([point[0], point[1], alt])
        new_relay = uav.role is not R.RELAY
        if new_relay:
            self.world.set_role(uav.uav_id, R.RELAY, reason)
        self.world.goto(uav.uav_id, target, reason)
        if new_relay:
            self.world.publish(EventType.RELAY_ASSIGNED,
                               f"{uav.name} is now a RELAY at ({point[0]:.0f}, {point[1]:.0f})"
                               + (f" serving {', '.join(serves)}" if serves else "") + f" [{reason}]",
                               uav_id=uav.uav_id, data={"point_m": [round(float(v), 1) for v in target],
                                                        "serves": list(serves), "reason": reason})

    def make_backup(self, uav: UAV, point: Sequence[float], reason: str) -> None:
        self._require(uav, R.BACKUP)
        alt = self.safety.safe_altitude(uav, point, point[2] if len(point) > 2 else None)
        if uav.role is not R.BACKUP:
            self.world.set_role(uav.uav_id, R.BACKUP, reason)
        self.world.goto(uav.uav_id, (point[0], point[1], alt), reason)

    def release(self, uav: UAV, reason: str) -> None:
        was_relay = uav.role is R.RELAY
        self._require(uav, R.IDLE)
        self.world.release_uav(uav.uav_id, reason)
        if was_relay:
            self.world.publish(EventType.RELAY_RELEASED, f"{uav.name} released from RELAY [{reason}]",
                               uav_id=uav.uav_id, data={"reason": reason})

    def return_home(self, uav: UAV, reason: str) -> None:
        if uav.role in (R.RETURNING, R.CHARGING):
            return
        was_relay = uav.role is R.RELAY
        self._require(uav, R.RETURNING)
        # Already over its pad (waiting there on standby): land from where it is. Climbing to the
        # transit altitude first would only cross every level over the pads twice.
        p = self.world.params.uav
        over_pad = uav.horizontal_distance_to(uav.home) <= p.arrival_radius_m
        alt = self.safety.safe_altitude(uav, uav.home, float(uav.position[2]) if over_pad else p.rth_altitude_m)
        self.world.return_home(uav.uav_id, reason, altitude_m=alt)
        if was_relay:
            self.world.publish(EventType.RELAY_RELEASED, f"{uav.name} left the RELAY role [{reason}]",
                               uav_id=uav.uav_id, data={"reason": reason})
