"""
telemetry/telemetry_scheduler.py
Decides which MAVLink message types are due, so each type is sent at its own
rate (heartbeat 1 Hz, position 5 Hz, ...) instead of flooding the link.
"""

from __future__ import annotations

from typing import Mapping

DEFAULT_RATES_HZ: dict[str, float] = {
    "HEARTBEAT": 1.0,
    "GLOBAL_POSITION_INT": 5.0,
    "ATTITUDE": 5.0,
    "VFR_HUD": 2.0,
    "SYS_STATUS": 1.0,
    "GPS_RAW_INT": 1.0,
    "SWARM_STATUS": 1.0,   # NAMED_VALUE_FLOAT: role, link quality, hop count
    "HOME_POSITION": 0.2,  # home pad of each vehicle
}


class TelemetryScheduler:
    def __init__(self, rates_hz: Mapping[str, float] | None = None) -> None:
        self.rates_hz = dict(DEFAULT_RATES_HZ)
        for name, rate in (rates_hz or {}).items():
            if name not in DEFAULT_RATES_HZ:
                raise ValueError(f"unknown telemetry message '{name}'; known: {sorted(DEFAULT_RATES_HZ)}")
            if float(rate) < 0:
                raise ValueError(f"rate for '{name}' must be >= 0")
            self.rates_hz[name] = float(rate)
        self._next_s: dict[str, float] = {name: 0.0 for name in self.rates_hz}
        self.sent_counts: dict[str, int] = {name: 0 for name in self.rates_hz}

    def due(self, t_s: float) -> list[str]:
        """Message types whose period elapsed; schedules the next one."""
        out = []
        for name, rate in self.rates_hz.items():
            if rate <= 0:
                continue
            if t_s + 1e-9 >= self._next_s[name]:
                self._next_s[name] = t_s + 1.0 / rate
                self.sent_counts[name] += 1
                out.append(name)
        return out

    def reset(self) -> None:
        self._next_s = {name: 0.0 for name in self.rates_hz}
