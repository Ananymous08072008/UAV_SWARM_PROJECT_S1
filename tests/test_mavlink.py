"""Stage 8 tests: MAVLink telemetry (units, message contents, scheduler, GCS requests)."""

import socket

import pytest
from pymavlink.dialects.v20 import common as mavlink

from core.config import GeoOrigin
from telemetry.coordinate_converter import CoordinateConverter
from telemetry.mavlink_gateway import MavlinkGateway, TelemetryParams
from telemetry.mavlink_messages import COPTER_MODE, flight_mode
from telemetry.telemetry_scheduler import DEFAULT_RATES_HZ, TelemetryScheduler
from tests.helpers import make_sim, run_until


def test_converter_units():
    conv = CoordinateConverter(GeoOrigin(lat_deg=-35.363261, lon_deg=149.16523, alt_msl_m=584.0))
    lat, lon, alt_mm, rel_mm = conv.global_position(100.0, 200.0, 40.0)
    assert lat / 1e7 == pytest.approx(-35.363261 + 200 / 6378137 * 180 / 3.141592653589793, abs=1e-6)
    assert alt_mm == 624000 and rel_mm == 40000
    assert conv.velocity_ned_cm_s(1.0, 2.0, 3.0) == (200, 100, -300)   # north, east, down
    assert conv.heading_cdeg(90.0) == 9000 and conv.heading_cdeg(-10.0) == 35000
    assert conv.yaw_rad(270.0) == pytest.approx(-1.5707963, abs=1e-6)
    assert 14000 <= conv.battery_voltage_mv(0.0) < conv.battery_voltage_mv(100.0) <= 16800


def test_scheduler_respects_rates():
    scheduler = TelemetryScheduler({"HEARTBEAT": 1.0, "GLOBAL_POSITION_INT": 5.0})
    sent = {name: 0 for name in DEFAULT_RATES_HZ}
    for tick in range(100):                      # 10 s at 10 Hz
        for name in scheduler.due(tick * 0.1):
            sent[name] += 1
    assert sent["HEARTBEAT"] == 10
    assert sent["GLOBAL_POSITION_INT"] == 50


def test_scheduler_rejects_unknown_messages():
    with pytest.raises(ValueError, match="unknown telemetry message"):
        TelemetryScheduler({"NOT_A_MESSAGE": 1.0})


def test_flight_mode_mapping():
    sim = make_sim()
    run_until(sim, 40)
    snapshot = sim.world.snapshot()
    modes = {uav.role: flight_mode(uav) for uav in snapshot.uavs}
    for role, mode in modes.items():
        assert mode in COPTER_MODE.values()
    grounded = [u for u in snapshot.uavs if not u.airborne]
    assert all(flight_mode(u) == COPTER_MODE["STABILIZE"] for u in grounded)


def test_every_uav_is_published_and_decodes():
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(("127.0.0.1", 0))
    listener.setblocking(False)
    port = listener.getsockname()[1]
    sim = make_sim()
    gateway = MavlinkGateway(sim.world.params.geo_origin, TelemetryParams(target_port=port),
                             bus=sim.world.events)
    sim.gateway = gateway
    try:
        run_until(sim, 30)
        parser = mavlink.MAVLink(None, 255, 0)
        parser.robust_parsing = True
        types, systems = set(), set()
        while True:
            try:
                data, _ = listener.recvfrom(4096)
            except BlockingIOError:
                break
            for message in parser.parse_buffer(data) or []:
                types.add(message.get_type())
                systems.add(message.get_srcSystem())
        assert {"HEARTBEAT", "GLOBAL_POSITION_INT", "ATTITUDE", "VFR_HUD", "SYS_STATUS"} <= types
        assert systems == {u.uav_id for u in sim.world.state.uavs.values()}
        assert gateway.stats()["packets_sent"] > 0
    finally:
        gateway.close()
        listener.close()


def test_gateway_answers_the_requests_mission_planner_sends():
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(("127.0.0.1", 0))
    listener.setblocking(False)
    gateway = MavlinkGateway(GeoOrigin(), TelemetryParams(target_port=listener.getsockname()[1]))
    try:
        gcs = mavlink.MAVLink(None, 255, 0)
        for message in (gcs.param_request_list_encode(1, 1),
                        gcs.mission_request_list_encode(1, 1),
                        gcs.command_long_encode(1, 1, 520, 0, 0, 0, 0, 0, 0, 0, 0)):
            listener.sendto(message.pack(gcs), ("127.0.0.1", gateway.local_port))
            assert gateway.receive() == 1
        parser = mavlink.MAVLink(None, 255, 0)
        parser.robust_parsing = True
        replies = []
        while True:
            try:
                data, _ = listener.recvfrom(4096)
            except BlockingIOError:
                break
            replies += [m.get_type() for m in parser.parse_buffer(data) or []]
        assert replies == ["PARAM_VALUE", "MISSION_COUNT", "COMMAND_ACK"]
    finally:
        gateway.close()
        listener.close()


def test_mission_planner_connecting_after_the_simulation_started_gets_answers():
    """Usual workflow: start the simulation, open Mission Planner later, press CONNECT."""
    sim = make_sim()
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()                                  # nobody listens on `port` yet
    gateway = MavlinkGateway(sim.world.params.geo_origin, TelemetryParams(target_port=port))
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sim.start()
        for tick in range(30):                     # 3 s of telemetry into the void
            sim.tick()
            gateway.update(sim.world.snapshot(), now_s=tick * 0.1)
        listener.bind(("127.0.0.1", port))         # Mission Planner opens the port ...
        listener.settimeout(1.0)
        gcs = mavlink.MAVLink(None, 255, 190)
        listener.sendto(gcs.param_request_list_encode(1, 1).pack(gcs), ("127.0.0.1", gateway.local_port))
        sim.tick()
        gateway.update(sim.world.snapshot(), now_s=3.0)   # ... and is answered on the next step
        parser = mavlink.MAVLink(None, 255, 0)
        parser.robust_parsing = True
        replies = []
        while "PARAM_VALUE" not in replies:
            data, _ = listener.recvfrom(4096)
            replies += [m.get_type() for m in parser.parse_buffer(data) or []]
    finally:
        gateway.close()
        listener.close()


def test_replies_keep_mission_type_and_sequence_numbers():
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(("127.0.0.1", 0))
    listener.setblocking(False)
    sim = make_sim()
    gateway = MavlinkGateway(sim.world.params.geo_origin, TelemetryParams(target_port=listener.getsockname()[1]))
    try:
        sim.start()
        sim.tick()
        gateway.update(sim.world.snapshot(), now_s=0.0)
        gcs = mavlink.MAVLink(None, 255, 190)
        for mission_type in (0, 1, 2):                          # mission, fence, rally
            listener.sendto(gcs.mission_request_list_encode(1, 1, mission_type).pack(gcs),
                            ("127.0.0.1", gateway.local_port))
        listener.sendto(gcs.command_int_encode(1, 1, 0, 197, 0, 0, 0, 0, 0, 0, 0, 0, 0).pack(gcs),
                        ("127.0.0.1", gateway.local_port))
        gateway.receive()
        sim.tick()
        gateway.update(sim.world.snapshot(), now_s=1.0)
        parser = mavlink.MAVLink(None, 255, 0)
        parser.robust_parsing = True
        from_uav1 = []
        while True:
            try:
                data, _ = listener.recvfrom(4096)
            except BlockingIOError:
                break
            from_uav1 += [m for m in parser.parse_buffer(data) or [] if m.get_srcSystem() == 1]
        assert [m.mission_type for m in from_uav1 if m.get_type() == "MISSION_COUNT"] == [0, 1, 2]
        assert [m.get_type() for m in from_uav1].count("COMMAND_ACK") == 1
        seqs = [m.get_seq() for m in from_uav1]
        assert all((b - a) % 256 == 1 for a, b in zip(seqs, seqs[1:]))   # no gaps = no fake packet loss
    finally:
        gateway.close()
        listener.close()


def test_rates_follow_the_wall_clock_not_the_simulation_speed():
    """At 8x simulation speed Mission Planner must still get ~5 Hz positions, not 40 Hz."""
    sim = make_sim()
    gateway = MavlinkGateway(sim.world.params.geo_origin, TelemetryParams(target_port=9), bus=None)
    try:
        sim.start()
        sent_positions = 0
        for tick in range(80):                 # 8 s of simulation ...
            sim.tick()
            wall = tick * 0.0125               # ... squeezed into 1 s of wall clock (8x)
            before = gateway.scheduler.sent_counts["GLOBAL_POSITION_INT"]
            gateway.update(sim.world.snapshot(), now_s=wall)
            sent_positions += gateway.scheduler.sent_counts["GLOBAL_POSITION_INT"] - before
        assert 5 <= sent_positions <= 6        # 5 Hz over one wall-clock second
    finally:
        gateway.close()


def test_monitor_sees_every_vehicle_with_home_and_swarm_values():
    from telemetry.mavlink_monitor import MavlinkMonitor

    monitor = MavlinkMonitor(port=0, host="127.0.0.1")
    sim = make_sim()
    gateway = MavlinkGateway(sim.world.params.geo_origin, TelemetryParams(target_port=monitor.port),
                             bus=sim.world.events)
    try:
        sim.start()
        for tick in range(600):                # 60 s of simulation, 10 Hz wall clock
            sim.tick()
            gateway.update(sim.world.snapshot(), now_s=tick * 0.1)
            if tick % 20 == 0:
                monitor.poll()
        monitor.poll()
        assert set(monitor.vehicles) == set(sim.world.state.uavs)
        flying = [v for v in monitor.vehicles.values() if v.rel_alt_m and v.rel_alt_m > 5]
        assert flying and all(v.armed for v in flying)
        assert all(v.home is not None for v in monitor.vehicles.values())
        assert any(v.role in ("SURVEY", "RELAY") for v in monitor.vehicles.values())
        assert any(v.texts for v in monitor.vehicles.values())        # swarm decisions as STATUSTEXT
        assert "SYS" in monitor.table()
    finally:
        gateway.close()
        monitor.close()
