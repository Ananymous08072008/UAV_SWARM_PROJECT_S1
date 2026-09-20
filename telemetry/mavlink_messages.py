"""
telemetry/mavlink_messages.py
Builds the MAVLink messages Mission Planner expects from a UAVSnapshot.

Research-only values (role, PoI, link quality) stay in the custom dashboard;
only a small NAMED_VALUE_FLOAT set is added so the swarm decisions can also be
watched in Mission Planner if wanted.
"""

from __future__ import annotations

from typing import Any

from pymavlink.dialects.v20 import common as mavlink

from core.state import UAVSnapshot
from telemetry.coordinate_converter import CoordinateConverter

# ArduCopter custom mode numbers (what Mission Planner shows as the flight mode)
COPTER_MODE = {"STABILIZE": 0, "GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9}

ROLE_CODE = {"IDLE": 0, "SURVEY": 1, "RELAY": 2, "BACKUP": 3, "RETURNING": 4, "CHARGING": 5}

SEVERITY = {"INFO": mavlink.MAV_SEVERITY_INFO, "WARNING": mavlink.MAV_SEVERITY_WARNING,
            "CRITICAL": mavlink.MAV_SEVERITY_CRITICAL}


def flight_mode(uav: UAVSnapshot) -> int:
    if uav.role == "RETURNING":
        return COPTER_MODE["LAND"] if uav.target_m and uav.target_m[2] < 1.0 else COPTER_MODE["RTL"]
    if uav.role == "CHARGING" or not uav.airborne:
        return COPTER_MODE["STABILIZE"]
    return COPTER_MODE["GUIDED"] if uav.mode == "TRANSIT" else COPTER_MODE["LOITER"]


def system_status(uav: UAVSnapshot, critical_battery_pct: float = 15.0) -> int:
    if uav.health == "FAILED":
        return mavlink.MAV_STATE_EMERGENCY
    if uav.battery_pct <= critical_battery_pct:
        return mavlink.MAV_STATE_CRITICAL
    return mavlink.MAV_STATE_ACTIVE if uav.airborne else mavlink.MAV_STATE_STANDBY


def heartbeat(mav: mavlink.MAVLink, uav: UAVSnapshot, critical_battery_pct: float = 15.0) -> None:
    base_mode = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if uav.airborne and uav.health != "FAILED":
        base_mode |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    mav.heartbeat_send(mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                       base_mode, flight_mode(uav), system_status(uav, critical_battery_pct))


def global_position_int(mav: mavlink.MAVLink, uav: UAVSnapshot, conv: CoordinateConverter, boot_ms: int) -> None:
    lat, lon, alt_mm, rel_mm = conv.global_position(uav.x_m, uav.y_m, uav.z_m)
    vx, vy, vz = conv.velocity_ned_cm_s(uav.vx_mps, uav.vy_mps, uav.vz_mps)
    mav.global_position_int_send(boot_ms, lat, lon, alt_mm, rel_mm, vx, vy, vz, conv.heading_cdeg(uav.heading_deg))


def attitude(mav: mavlink.MAVLink, uav: UAVSnapshot, conv: CoordinateConverter, boot_ms: int) -> None:
    mav.attitude_send(boot_ms, 0.0, 0.0, conv.yaw_rad(uav.heading_deg), 0.0, 0.0, 0.0)


def vfr_hud(mav: mavlink.MAVLink, uav: UAVSnapshot, cruise_speed_mps: float) -> None:
    throttle = int(min(100, max(0, round(100 * uav.ground_speed_mps / max(cruise_speed_mps, 1e-6)))))
    mav.vfr_hud_send(uav.ground_speed_mps, uav.ground_speed_mps, int(round(uav.heading_deg)) % 360,
                     throttle, uav.z_m, uav.vz_mps)


def sys_status(mav: mavlink.MAVLink, uav: UAVSnapshot, conv: CoordinateConverter) -> None:
    sensors = (mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO | mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL
               | mavlink.MAV_SYS_STATUS_SENSOR_GPS | mavlink.MAV_SYS_STATUS_SENSOR_BATTERY)
    current_ca = 1500 if uav.airborne else 0
    mav.sys_status_send(sensors, sensors, sensors, 250, conv.battery_voltage_mv(uav.battery_pct), current_ca,
                        int(round(uav.battery_pct)), 0, 0, 0, 0, 0, 0)


def gps_raw_int(mav: mavlink.MAVLink, uav: UAVSnapshot, conv: CoordinateConverter, boot_ms: int) -> None:
    lat, lon, alt_mm, _ = conv.global_position(uav.x_m, uav.y_m, uav.z_m)
    mav.gps_raw_int_send(boot_ms * 1000, mavlink.GPS_FIX_TYPE_3D_FIX, lat, lon, alt_mm, 100, 150,
                         int(round(uav.ground_speed_mps * 100)), conv.heading_cdeg(uav.heading_deg), 12)


def home_position(mav: mavlink.MAVLink, uav: UAVSnapshot, conv: CoordinateConverter) -> None:
    """The UAV's home pad; x/y/z are its local position in NED metres."""
    hx, hy, hz = uav.home_m
    lat, lon, alt_mm, _ = conv.global_position(hx, hy, hz)
    mav.home_position_send(lat, lon, alt_mm, hy, hx, -hz, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0)


def swarm_status(mav: mavlink.MAVLink, uav: UAVSnapshot, boot_ms: int) -> None:
    """Research values as NAMED_VALUE_FLOAT: role code, end-to-end PDR, hop count."""
    mav.named_value_float_send(boot_ms, b"ROLE", float(ROLE_CODE.get(uav.role, 0)))
    mav.named_value_float_send(boot_ms, b"LINKQ", float(uav.pdr))
    mav.named_value_float_send(boot_ms, b"HOPS", float(uav.hop_count or 0))


def statustext(mav: mavlink.MAVLink, text: str, severity: str = "INFO") -> None:
    mav.statustext_send(SEVERITY.get(severity, mavlink.MAV_SEVERITY_INFO), text[:50].encode("ascii", "replace"))


def build(name: str, mav: mavlink.MAVLink, uav: UAVSnapshot, conv: CoordinateConverter, boot_ms: int,
          extra: dict[str, Any]) -> None:
    """Send the message type ``name`` for one UAV."""
    if name == "HEARTBEAT":
        heartbeat(mav, uav, extra.get("critical_battery_pct", 15.0))
    elif name == "GLOBAL_POSITION_INT":
        global_position_int(mav, uav, conv, boot_ms)
    elif name == "ATTITUDE":
        attitude(mav, uav, conv, boot_ms)
    elif name == "VFR_HUD":
        vfr_hud(mav, uav, extra.get("cruise_speed_mps", 10.0))
    elif name == "SYS_STATUS":
        sys_status(mav, uav, conv)
    elif name == "GPS_RAW_INT":
        gps_raw_int(mav, uav, conv, boot_ms)
    elif name == "SWARM_STATUS":
        swarm_status(mav, uav, boot_ms)
    elif name == "HOME_POSITION":
        home_position(mav, uav, conv)
    else:
        raise ValueError(f"unknown telemetry message '{name}'")
