"""
main.py - UAV Swarm Project entry point.

    python main.py                                   headless run of config/scenario.yaml
    python main.py --demo                            dashboard + MAVLink + real time (the demo)
    python main.py --dashboard --realtime --speed 4  faster-than-real-time with the dashboard
    python main.py --scenario scenarios/obstacle.yaml --mode baseline
    python main.py --no-results --quiet              nothing written to disk, summary only

Layers (see docs/architecture.md):
    core/        world model, UAVs, PoIs, events        (execution + truth)
    simulation/  radio channel, obstacles, energy, data (physics)
    swarm/       allocation, relays, routing, recovery  (decisions)
    telemetry/   MAVLink gateway to Mission Planner
    dashboard/   live research dashboard
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
import webbrowser
from dataclasses import replace
from pathlib import Path
from typing import Optional

from core.config import ConfigError, Parameters, ScenarioConfig
from core.events import EventType
from simulation.runner import Simulation

PROJECT_ROOT = Path(__file__).resolve().parent
QUIET_EVENTS = {EventType.UAV_COMMANDED, EventType.ROUTE_CHANGED, EventType.POI_SURVEY_STARTED,
                EventType.UAV_ARRIVED, EventType.POI_CREATED, EventType.UAV_SPAWNED}


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Resilient UAV swarm simulation")
    parser.add_argument("--params", type=Path, default=PROJECT_ROOT / "config" / "parameters.yaml")
    parser.add_argument("--scenario", type=Path, default=PROJECT_ROOT / "config" / "scenario.yaml")
    parser.add_argument("--mode", choices=("adaptive", "baseline"), default="adaptive",
                        help="adaptive = the proposed system, baseline = static comparison system")
    parser.add_argument("--duration", type=float, help="override the scenario duration (s)")
    parser.add_argument("--seed", type=int, help="override the scenario seed")
    parser.add_argument("--realtime", action="store_true", help="pace the simulation against the wall clock")
    parser.add_argument("--speed", type=float, default=1.0, help="real-time multiplier (with --realtime)")
    parser.add_argument("--dashboard", action="store_true", help="serve the live dashboard")
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--open", action="store_true", help="open the dashboard in a browser")
    parser.add_argument("--mavlink", action="store_true", help="publish MAVLink telemetry (Mission Planner)")
    parser.add_argument("--mavlink-target", help="host:port for MAVLink (default from parameters.yaml)")
    parser.add_argument("--demo", action="store_true", help="shortcut: --dashboard --mavlink --realtime --open")
    parser.add_argument("--results-dir", type=Path, default=PROJECT_ROOT / "results")
    parser.add_argument("--no-results", action="store_true", help="do not write logs, metrics or the database")
    parser.add_argument("--keep-running", action="store_true",
                        help="run the full scenario duration even after the mission is complete")
    parser.add_argument("--all-events", action="store_true", help="print every event, not just the interesting ones")
    parser.add_argument("--quiet", action="store_true", help="print only the final summary")
    parser.add_argument("--log-level", default="WARNING", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args(argv)
    if args.speed <= 0:
        parser.error("--speed must be > 0")
    if args.demo:
        args.dashboard = args.mavlink = args.realtime = args.open = True
    return args


def build_simulation(args: argparse.Namespace):
    params = Parameters.load(args.params)
    scenario = ScenarioConfig.load(args.scenario)
    changes = {}
    if args.duration is not None:
        changes["duration_s"] = args.duration
        kept = tuple(t for t in scenario.timeline if t.at_s <= args.duration)
        if len(kept) != len(scenario.timeline):
            print(f"Note: --duration {args.duration:g} skips {len(scenario.timeline) - len(kept)} timeline trigger(s)")
        changes["timeline"] = kept
    if args.seed is not None:
        changes["seed"] = args.seed
    if changes:
        scenario = replace(scenario, **changes)

    hub = gateway = server = None
    if args.dashboard:
        from dashboard.api import DashboardServer
        from dashboard.websocket import LiveHub
        dash = params.section("dashboard")
        hub = LiveHub()
        server = DashboardServer(hub, args.host or dash.get("host", "127.0.0.1"),
                                 args.port or int(dash.get("port", 8000)),
                                 float(dash.get("push_interval_s", 0.2)))
    sim = Simulation(params, scenario, mode=args.mode,
                     results_dir=None if args.no_results else args.results_dir,
                     hub=hub, stop_when_complete=not args.keep_running)
    if args.mavlink:
        from telemetry.mavlink_gateway import MavlinkGateway
        gateway = MavlinkGateway.from_world(sim.world, sim.world.events)
        if args.mavlink_target:
            host, _, port = args.mavlink_target.partition(":")
            gateway.address = (host, int(port or 14550))
            gateway.writer.address = gateway.address
        sim.gateway = gateway
    return sim, server, gateway


def print_status(sim: Simulation) -> None:
    world = sim.world
    snap = world.snapshot()
    live = sim.metrics.live()
    done = sum(1 for p in snap.pois if p.status == "COMPLETED")
    print(f"\n--- t = {snap.t_s:6.1f}s | PoIs {done}/{len(snap.pois)} | "
          f"connected {live['connectivity_ratio']} | incidents {live['incidents']} "
          f"({live['incidents_open']} open) ---")
    print(f"{'UAV':<8}{'ROLE':<11}{'MODE':<12}{'TASK':<12}{'BAT%':>6}{'ALT':>6}{'PDR':>6}{'HOPS':>6}  ROUTE")
    for u in snap.uavs:
        route = " > ".join(str(n) for n in u.route) if u.connected else "no link"
        print(f"{u.name:<8}{u.role:<11}{u.mode:<12}{(u.assigned_poi or '-'):<12}"
              f"{u.battery_pct:6.0f}{u.z_m:6.0f}{u.pdr:6.2f}{(u.hop_count or 0):6d}  {route}")


def print_summary(summary: dict, wall_s: float, sim: Simulation) -> None:
    print("\n================ RUN SUMMARY ================")
    for group, values in summary.items():
        print(f"[{group}]")
        for key, value in values.items():
            print(f"  {key:<28} {value}")
    if sim.run_dir is not None:
        print(f"\nResults written to {sim.run_dir}")
    print(f"Wall clock: {wall_s:.1f}s for {sim.world.t:.0f}s of mission time "
          f"({sim.world.t / wall_s if wall_s else 0:.0f}x real time)")


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=args.log_level, format="%(levelname)s %(name)s: %(message)s")
    try:
        sim, server, gateway = build_simulation(args)
    except ConfigError as exc:
        print(f"Configuration error: {exc}", file=sys.stderr)
        return 2

    if not args.quiet:
        def printer(event):
            if args.all_events or event.type not in QUIET_EVENTS:
                print(event)
        sim.world.events.subscribe(printer)

    if server is not None:
        server.start()
        url = server.shareable_url
        print(f"Dashboard: {url}")
        if server.serves_network:
            print("  Reachable from other machines on this network at the address above.")
            print("  Windows blocks the port until you allow it once - see docs/running.md.")
        if args.open:
            webbrowser.open(url)
    if gateway is not None:
        stats = gateway.stats()
        print(f"MAVLink telemetry -> {stats['target']} (local port {stats['local_port']}); "
              f"in Mission Planner choose UDP and port {stats['target'].split(':')[1]}")

    interval = sim.world.params.simulation.status_interval_s
    wall_start = time.perf_counter()
    reason = "duration reached"
    try:
        sim.run(realtime=args.realtime, speed=args.speed,
                on_status=None if args.quiet else print_status, status_interval_s=interval)
    except KeyboardInterrupt:
        reason = "interrupted by user"
    summary = sim.finish(reason)
    print_summary(summary, time.perf_counter() - wall_start, sim)

    if server is not None:
        print("\nDashboard still serving the final state - press Ctrl+C to exit.")
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
