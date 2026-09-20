"""
telemetry/coordinate_converter.py
Simulation units -> MAVLink units.

Local ENU metres (x East, y North, z Up) become WGS-84 degrees and the integer
units MAVLink expects: 1e7 degrees, millimetres, cm/s in NED, centi-degrees.
The geographic anchor is the same GeoOrigin the whole project uses, so the
dashboard map and Mission Planner always agree.
"""

from __future__ import annotations

import math
from typing import Sequence

from core.config import GeoOrigin


class CoordinateConverter:
    def __init__(self, origin: GeoOrigin) -> None:
        self.origin = origin

    def global_position(self, x_m: float, y_m: float, z_m: float) -> tuple[int, int, int, int]:
        """(lat_1e7, lon_1e7, alt_msl_mm, relative_alt_mm)"""
        lat, lon, alt_msl = self.origin.to_geodetic(x_m, y_m, z_m)
        return int(round(lat * 1e7)), int(round(lon * 1e7)), int(round(alt_msl * 1000)), int(round(z_m * 1000))

    @staticmethod
    def velocity_ned_cm_s(vx_east: float, vy_north: float, vz_up: float) -> tuple[int, int, int]:
        """ENU m/s -> NED cm/s (MAVLink order: north, east, down)."""
        return int(round(vy_north * 100)), int(round(vx_east * 100)), int(round(-vz_up * 100))

    @staticmethod
    def heading_cdeg(heading_deg: float) -> int:
        """Heading in centi-degrees, 0..35999 (0 = North, clockwise)."""
        return int(round(heading_deg % 360.0 * 100)) % 36000

    @staticmethod
    def yaw_rad(heading_deg: float) -> float:
        """ATTITUDE yaw in radians, wrapped to [-pi, pi]."""
        return math.radians((heading_deg + 180.0) % 360.0 - 180.0)

    @staticmethod
    def battery_voltage_mv(battery_pct: float, cells: int = 4) -> int:
        """Plausible pack voltage for the HUD: 3.5 V (empty) .. 4.2 V (full) per cell."""
        return int(round(cells * (3.5 + 0.7 * max(0.0, min(100.0, battery_pct)) / 100.0) * 1000))

    def to_local(self, lat_deg: float, lon_deg: float, alt_msl_m: float) -> tuple[float, float, float]:
        return self.origin.to_local(lat_deg, lon_deg, alt_msl_m)

    def local_from_snapshot(self, position_m: Sequence[float]) -> tuple[int, int, int, int]:
        return self.global_position(position_m[0], position_m[1], position_m[2])
