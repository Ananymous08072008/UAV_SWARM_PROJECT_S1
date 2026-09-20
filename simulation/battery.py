"""
simulation/battery.py
Energy estimates used for decisions: how much battery a flight, a hover or a
return-to-home will cost, and how long a UAV can keep flying.

The per-step drain itself happens in core/uav.py (execution) using the same
``drain_rate_pct_per_min`` formula, so estimates and reality stay consistent.
"""

from __future__ import annotations

import math
from typing import Sequence

from core.config import BatteryParams, UAVParams
from core.uav import UAV, drain_rate_pct_per_min

SAFETY_FACTOR = 1.15  # estimates are padded: acceleration, detours and wind are not modelled


class BatteryModel:
    def __init__(self, uav_p: UAVParams, battery_p: BatteryParams) -> None:
        self.uav_p = uav_p
        self.battery_p = battery_p

    # ------------------------------------------------------------------ rates
    def drain_pct_per_s(self, ground_speed_mps: float) -> float:
        return drain_rate_pct_per_min(ground_speed_mps, self.uav_p, self.battery_p) / 60.0

    def endurance_s(self, battery_pct: float) -> float:
        """Hover time until the battery is empty."""
        return battery_pct / self.drain_pct_per_s(0.0)

    # ---------------------------------------------------------------- flights
    def travel_time_s(self, a: Sequence[float], b: Sequence[float]) -> float:
        p = self.uav_p
        horizontal = math.hypot(b[0] - a[0], b[1] - a[1])
        vertical = abs(b[2] - a[2])
        accel_overhead = p.cruise_speed_mps / p.max_accel_mps2 if horizontal > 1.0 else 0.0
        return max(horizontal / p.cruise_speed_mps + accel_overhead, vertical / p.climb_rate_mps)

    def travel_cost_pct(self, a: Sequence[float], b: Sequence[float]) -> float:
        return self.travel_time_s(a, b) * self.drain_pct_per_s(self.uav_p.cruise_speed_mps) * SAFETY_FACTOR

    def hover_cost_pct(self, seconds: float) -> float:
        return max(0.0, seconds) * self.drain_pct_per_s(0.0) * SAFETY_FACTOR

    def return_time_s(self, uav: UAV) -> float:
        """Fly to the home pad at RTH altitude, then descend."""
        home_above = (uav.home[0], uav.home[1], self.uav_p.rth_altitude_m)
        return self.travel_time_s(uav.position, home_above) + self.uav_p.rth_altitude_m / self.uav_p.climb_rate_mps

    def return_cost_pct(self, uav: UAV) -> float:
        home_above = (uav.home[0], uav.home[1], self.uav_p.rth_altitude_m)
        descent_s = self.uav_p.rth_altitude_m / self.uav_p.climb_rate_mps
        return self.travel_cost_pct(uav.position, home_above) + self.hover_cost_pct(descent_s)

    def task_cost_pct(self, uav: UAV, waypoint: Sequence[float], hover_s: float) -> float:
        """Battery needed to fly to ``waypoint``, hover ``hover_s`` there and still get home."""
        home_above = (uav.home[0], uav.home[1], self.uav_p.rth_altitude_m)
        descent_s = self.uav_p.rth_altitude_m / self.uav_p.climb_rate_mps
        return (self.travel_cost_pct(uav.position, waypoint) + self.hover_cost_pct(hover_s)
                + self.travel_cost_pct(waypoint, home_above) + self.hover_cost_pct(descent_s))

    def task_time_s(self, uav: UAV, waypoint: Sequence[float], hover_s: float) -> float:
        home_above = (uav.home[0], uav.home[1], self.uav_p.rth_altitude_m)
        return (self.travel_time_s(uav.position, waypoint) + hover_s + self.travel_time_s(waypoint, home_above)
                + self.uav_p.rth_altitude_m / self.uav_p.climb_rate_mps)
