"""
telemetry/mavlink_gateway.py
ONE gateway publishes ALL virtual UAVs to Mission Planner over UDP.

Each UAV is a MAVLink system id (1..N) sharing a single socket, exactly like a
real multi-vehicle link, so Mission Planner lists them in its vehicle selector.
The gateway also answers the few requests Mission Planner makes on connect
(parameter list, mission list, commands) so the connection settles quickly.

Mission Planner: Connect -> UDP, port 14550 (see README).
"""

from __future__ import annotations

import logging
import socket
import time
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

from pymavlink.dialects.v20 import common as mavlink

from core.config import GeoOrigin, build as build_params
from core.events import Event, EventBus, EventType
from core.state import WorldSnapshot
from telemetry import mavlink_messages as msgs
from telemetry.coordinate_converter import CoordinateConverter
from telemetry.telemetry_scheduler import TelemetryScheduler

log = logging.getLogger(__name__)

STATUSTEXT_EVENTS = (EventType.ROLE_CHANGED, EventType.RELAY_ASSIGNED, EventType.UAV_FAILED,
                     EventType.RTH_STARTED, EventType.POI_COMPLETED, EventType.FAULT_DETECTED,
                     EventType.RECOVERY_COMPLETE, EventType.HANDOVER_STARTED, EventType.SAFETY_VIOLATION)


@dataclass(frozen=True)
class TelemetryParams:
    target_host: str = "127.0.0.1"
    target_port: int = 14550
    bind_port: int = 0
    statustext_events: bool = True
    rates_hz: Mapping[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not 0 <= self.bind_port <= 65535 or not 0 < self.target_port <= 65535:
            raise ValueError("ports must be within 0..65535")


class _UDPWriter:
    """File-like sink: everything pymavlink writes goes out as a UDP datagram."""

    def __init__(self, sock: socket.socket, address: tuple[str, int]) -> None:
        self.sock = sock
        self.address = address
        self.packets = 0
        self.bytes = 0

    def write(self, data: bytes) -> None:
        try:
            self.sock.sendto(data, self.address)
            self.packets += 1
            self.bytes += len(data)
        except OSError as exc:  # the GCS may not be listening yet - keep simulating
            log.debug("MAVLink send failed: %s", exc)


class MavlinkGateway:
    def __init__(self, origin: GeoOrigin, params: TelemetryParams, cruise_speed_mps: float = 10.0,
                 critical_battery_pct: float = 15.0, bus: Optional[EventBus] = None) -> None:
        self.params = params
        self.converter = CoordinateConverter(origin)
        self.scheduler = TelemetryScheduler(params.rates_hz)
        self.cruise_speed_mps = cruise_speed_mps
        self.critical_battery_pct = critical_battery_pct
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", params.bind_port))
        self.sock.setblocking(False)
        if hasattr(socket, "SIO_UDP_CONNRESET"):
            # Windows: every datagram sent before Mission Planner opens its port would
            # otherwise queue a ConnectionResetError in front of MP's first requests.
            self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        self.address = (params.target_host, params.target_port)
        self.writer = _UDPWriter(self.sock, self.address)
        self._links: dict[int, mavlink.MAVLink] = {}
        self._rx = mavlink.MAVLink(None, 255, 0)
        self._rx.robust_parsing = True
        self._statustext: list[tuple[int, str, str]] = []
        self._clock_start = time.perf_counter()
        self._unsubscribe = None
        if bus is not None and params.statustext_events:
            self._unsubscribe = bus.subscribe(self._on_event, types=STATUSTEXT_EVENTS)

    @classmethod
    def from_world(cls, world, bus: Optional[EventBus] = None) -> "MavlinkGateway":
        params = build_params(TelemetryParams, world.params.section("telemetry"), "telemetry")
        return cls(world.params.geo_origin, params, world.params.uav.cruise_speed_mps,
                   world.params.battery.critical_pct, bus)

    @property
    def local_port(self) -> int:
        return self.sock.getsockname()[1]

    # ---------------------------------------------------------------- sending
    def _link(self, system_id: int) -> mavlink.MAVLink:
        link = self._links.get(system_id)
        if link is None:
            link = mavlink.MAVLink(self.writer, srcSystem=system_id, srcComponent=mavlink.MAV_COMP_ID_AUTOPILOT1)
            self._links[system_id] = link
        return link

    def _on_event(self, event: Event) -> None:
        if event.uav_id is not None:
            self._statustext.append((event.uav_id, event.message, event.severity.value))

    def update(self, snapshot: WorldSnapshot, now_s: Optional[float] = None) -> int:
        """Send everything that is due. Returns the number of messages sent.

        Rates are kept in wall-clock time (``now_s`` defaults to seconds since the
        gateway started), so Mission Planner sees 5 Hz positions whatever the
        simulation speed; message contents and time stamps use simulation time.
        """
        self.receive()
        boot_ms = int(snapshot.t_s * 1000)
        clock = time.perf_counter() - self._clock_start if now_s is None else now_s
        due = self.scheduler.due(clock)
        extra = {"cruise_speed_mps": self.cruise_speed_mps, "critical_battery_pct": self.critical_battery_pct}
        sent = 0
        for uav in snapshot.uavs:
            link = self._link(uav.uav_id)
            for name in due:
                msgs.build(name, link, uav, self.converter, boot_ms, extra)
                sent += 1
        for uav_id, text, severity in self._statustext:
            msgs.statustext(self._link(uav_id), text, severity)
            sent += 1
        self._statustext.clear()
        return sent

    # -------------------------------------------------------------- receiving
    def receive(self, max_packets: int = 20) -> int:
        """Answer the handful of requests Mission Planner sends after connecting."""
        handled = 0
        for _ in range(max_packets):
            try:
                data, sender = self.sock.recvfrom(2048)
            except ConnectionResetError:   # stale "port unreachable" from before the GCS listened
                continue
            except OSError:                # BlockingIOError: nothing waiting
                break
            try:
                messages = self._rx.parse_buffer(data) or []
            except mavlink.MAVError:
                continue
            for message in messages:
                self._handle(message, sender)
                handled += 1
        return handled

    def _handle(self, message: Any, sender: tuple[str, int]) -> None:
        kind = message.get_type()
        if kind not in ("PARAM_REQUEST_LIST", "PARAM_REQUEST_READ", "MISSION_REQUEST_LIST",
                        "COMMAND_LONG", "COMMAND_INT"):
            return                         # HEARTBEAT / REQUEST_DATA_STREAM and anything else
        target = getattr(message, "target_system", 0) or 1
        # Reply to the sender but continue the vehicle's own sequence numbers, so
        # Mission Planner does not count the reply as a burst of lost packets.
        vehicle = self._links.get(target)
        link = mavlink.MAVLink(_UDPWriter(self.sock, sender), srcSystem=target,
                               srcComponent=mavlink.MAV_COMP_ID_AUTOPILOT1)
        if vehicle is not None:
            link.seq = vehicle.seq
        if kind in ("PARAM_REQUEST_LIST", "PARAM_REQUEST_READ"):
            link.param_value_send(b"SYSID_THISMAV", float(target), mavlink.MAV_PARAM_TYPE_REAL32, 1, 0)
        elif kind == "MISSION_REQUEST_LIST":       # mission, fence or rally: answer the type asked for
            link.mission_count_send(message.get_srcSystem(), message.get_srcComponent(), 0,
                                    getattr(message, "mission_type", mavlink.MAV_MISSION_TYPE_MISSION))
        else:                                      # COMMAND_LONG / COMMAND_INT: display only
            link.command_ack_send(message.command, mavlink.MAV_RESULT_UNSUPPORTED)
        if vehicle is not None:
            vehicle.seq = link.seq

    # ------------------------------------------------------------------ stats
    def stats(self) -> dict[str, Any]:
        return {"target": f"{self.address[0]}:{self.address[1]}", "local_port": self.local_port,
                "packets_sent": self.writer.packets, "bytes_sent": self.writer.bytes,
                "systems": sorted(self._links), "rates_hz": self.scheduler.rates_hz}

    def close(self) -> None:
        if self._unsubscribe is not None:
            self._unsubscribe()
        self.sock.close()
