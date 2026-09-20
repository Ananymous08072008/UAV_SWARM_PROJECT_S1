"""
telemetry/mavlink_monitor.py
A tiny stand-in for Mission Planner: listens on the telemetry UDP port and
prints what every vehicle is reporting. Use it to confirm the gateway works
before opening Mission Planner, or to debug a connection.

    terminal 1:  python main.py --mavlink --realtime
    terminal 2:  python -m telemetry.mavlink_monitor                 (port 14550, 30 s)
                 python -m telemetry.mavlink_monitor --port 14551 --seconds 60

Mission Planner and this monitor cannot both listen on the same port - close
Mission Planner first, or point the simulation at another port with
``main.py --mavlink-target 127.0.0.1:14551``.
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pymavlink.dialects.v20 import common as mavlink   # noqa: E402

from telemetry.mavlink_messages import COPTER_MODE, ROLE_CODE   # noqa: E402

MODE_NAME = {v: k for k, v in COPTER_MODE.items()}
ROLE_NAME = {v: k for k, v in ROLE_CODE.items()}


@dataclass
class VehicleView:
    system_id: int
    first_seen: float
    last_seen: float = 0.0
    lat: Optional[float] = None
    lon: Optional[float] = None
    rel_alt_m: Optional[float] = None
    heading_deg: Optional[float] = None
    ground_speed: Optional[float] = None
    mode: str = "?"
    armed: bool = False
    battery_pct: Optional[int] = None
    role: str = "?"
    link_quality: Optional[float] = None
    hops: Optional[int] = None
    home: Optional[tuple[float, float]] = None
    counts: Counter = field(default_factory=Counter)
    texts: deque = field(default_factory=lambda: deque(maxlen=3))

    def rate_hz(self, message: str, now: float) -> float:
        span = max(now - self.first_seen, 1e-6)
        return self.counts[message] / span


class MavlinkMonitor:
    def __init__(self, port: int = 14550, host: str = "0.0.0.0") -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))       # fails if Mission Planner already owns the port
        self.sock.setblocking(False)
        self.parser = mavlink.MAVLink(None, 255, 0)
        self.parser.robust_parsing = True
        self.vehicles: dict[int, VehicleView] = {}
        self.packets = 0

    @property
    def port(self) -> int:
        return self.sock.getsockname()[1]

    def poll(self) -> int:
        """Read everything waiting on the socket. Returns the number of messages decoded."""
        decoded = 0
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except (BlockingIOError, OSError):
                return decoded
            self.packets += 1
            for message in self.parser.parse_buffer(data) or []:
                self._handle(message)
                decoded += 1

    def _handle(self, message) -> None:
        now = time.perf_counter()
        sid = message.get_srcSystem()
        view = self.vehicles.setdefault(sid, VehicleView(sid, first_seen=now))
        view.last_seen = now
        kind = message.get_type()
        view.counts[kind] += 1
        if kind == "HEARTBEAT":
            view.mode = MODE_NAME.get(message.custom_mode, str(message.custom_mode))
            view.armed = bool(message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif kind == "GLOBAL_POSITION_INT":
            view.lat, view.lon = message.lat / 1e7, message.lon / 1e7
            view.rel_alt_m = message.relative_alt / 1000.0
            view.heading_deg = message.hdg / 100.0
        elif kind == "VFR_HUD":
            view.ground_speed = message.groundspeed
        elif kind == "SYS_STATUS":
            view.battery_pct = message.battery_remaining
        elif kind == "HOME_POSITION":
            view.home = (message.latitude / 1e7, message.longitude / 1e7)
        elif kind == "NAMED_VALUE_FLOAT":
            name = message.name.rstrip("\x00") if isinstance(message.name, str) else message.name.decode().rstrip("\x00")
            if name == "ROLE":
                view.role = ROLE_NAME.get(int(message.value), "?")
            elif name == "LINKQ":
                view.link_quality = message.value
            elif name == "HOPS":
                view.hops = int(message.value)
        elif kind == "STATUSTEXT":
            text = message.text if isinstance(message.text, str) else message.text.decode(errors="replace")
            view.texts.append(text.rstrip("\x00"))

    def table(self) -> str:
        now = time.perf_counter()
        lines = [f"{'SYS':>3} {'MODE':<9} {'ARM':<4} {'ROLE':<9} {'LAT':>11} {'LON':>11} {'ALT':>6} "
                 f"{'SPD':>5} {'BAT':>4} {'LINKQ':>5} {'HOPS':>4} {'POS Hz':>6} {'AGE':>5}  LAST MESSAGE"]
        for sid in sorted(self.vehicles):
            v = self.vehicles[sid]
            fmt = lambda value, spec: "-" if value is None else format(value, spec)  # noqa: E731
            lines.append(
                f"{sid:>3} {v.mode:<9} {'yes' if v.armed else 'no':<4} {v.role:<9} {fmt(v.lat, '11.6f')} "
                f"{fmt(v.lon, '11.6f')} {fmt(v.rel_alt_m, '6.1f')} {fmt(v.ground_speed, '5.1f')} "
                f"{fmt(v.battery_pct, '4d')} {fmt(v.link_quality, '5.2f')} {fmt(v.hops, '4d')} "
                f"{v.rate_hz('GLOBAL_POSITION_INT', now):6.1f} {now - v.last_seen:5.1f}  "
                f"{v.texts[-1] if v.texts else ''}")
        if not self.vehicles:
            lines.append("  (nothing received yet - is `python main.py --mavlink` running and pointed at this port?)")
        return "\n".join(lines)

    def close(self) -> None:
        self.sock.close()


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Check the swarm's MAVLink telemetry without Mission Planner")
    parser.add_argument("--port", type=int, default=14550)
    parser.add_argument("--seconds", type=float, default=30.0, help="how long to listen")
    parser.add_argument("--interval", type=float, default=2.0, help="table refresh period")
    args = parser.parse_args(argv)
    try:
        monitor = MavlinkMonitor(args.port)
    except OSError as exc:
        print(f"Cannot listen on UDP {args.port}: {exc}\n"
              f"Mission Planner (or SITL) is probably using it. Close it, or run the simulation with\n"
              f"  python main.py --mavlink --mavlink-target 127.0.0.1:14551\n"
              f"and start this monitor with --port 14551.", file=sys.stderr)
        return 2
    print(f"Listening for MAVLink on UDP {monitor.port} for {args.seconds:.0f} s ...")
    deadline = time.perf_counter() + args.seconds
    next_print = time.perf_counter() + args.interval
    try:
        while time.perf_counter() < deadline:
            monitor.poll()
            if time.perf_counter() >= next_print:
                next_print += args.interval
                print("\n" + monitor.table())
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    monitor.poll()
    print("\n" + monitor.table())
    print(f"\n{len(monitor.vehicles)} vehicle(s), {monitor.packets} packet(s) received.")
    monitor.close()
    return 0 if monitor.vehicles else 1


if __name__ == "__main__":
    sys.exit(main())
