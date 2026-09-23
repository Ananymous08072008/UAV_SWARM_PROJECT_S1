"""
dashboard/server.py
Standalone multi-session server - the web app entry point.

    python -m dashboard.server
    python -m dashboard.server --host 0.0.0.0 --port 8000 --max-sessions 8

Unlike ``main.py --dashboard`` this starts no simulation of its own. It serves
the mission studio, and every visitor builds and runs their own world from the
browser. Use main.py when you want one shared demo, MAVLink telemetry, or
results written to disk.

Security: /api/sessions and /api/sessions/{id}/inject are unauthenticated, the
same as the single-simulation dashboard. --max-sessions bounds how many worlds
strangers can start, but put a reverse proxy with authentication in front of
this before exposing it to the internet.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path

import uvicorn

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from core.config import ConfigError, Parameters                      # noqa: E402
from dashboard.api import WILDCARD_HOSTS, create_app, shareable_url  # noqa: E402
from dashboard.session import MAX_SESSIONS, SessionManager           # noqa: E402


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UAV swarm mission studio (multi-session web app)")
    parser.add_argument("--params", type=Path, default=PROJECT_ROOT / "config" / "parameters.yaml")
    # Hosting platforms (Render, Railway, Fly.io, Spaces) inject PORT and expect the
    # process to bind it - they start the container, so there is no chance to pass a
    # flag. These are defaults, so an explicit flag still overrides the environment.
    parser.add_argument("--host", default=os.environ.get("HOST", "127.0.0.1"),
                        help="0.0.0.0 to accept connections from other machines")
    parser.add_argument("--port", type=int, default=int(os.environ.get("PORT", "8000")))
    parser.add_argument("--max-sessions", type=int,
                        default=int(os.environ.get("MAX_SESSIONS", MAX_SESSIONS)),
                        help="how many simulations may run at once")
    parser.add_argument("--idle-timeout", type=float,
                        default=float(os.environ.get("IDLE_TIMEOUT", "1800")),
                        help="seconds before an unwatched simulation is reaped")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser.parse_args(argv)


def build_app(args: argparse.Namespace):
    params = Parameters.load(args.params)
    manager = SessionManager(params, max_sessions=args.max_sessions,
                             idle_timeout_s=args.idle_timeout)
    app = create_app(manager=manager)

    @app.on_event("shutdown")
    def _shutdown() -> None:
        manager.shutdown()

    return app, manager


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=args.log_level, format="%(levelname)s %(name)s: %(message)s")
    try:
        app, _ = build_app(args)
    except ConfigError as exc:
        print(f"Configuration error: {exc}", file=sys.stderr)
        return 2

    print(f"Mission studio: {shareable_url(args.host, args.port)}")
    if args.host in WILDCARD_HOSTS:
        print("  Share that link with anyone on this network. Windows blocks the")
        print("  port until you allow it once - see docs/running.md.")
        print("  No authentication: everyone who can reach it can start simulations.")
    print(f"Up to {args.max_sessions} simultaneous simulations, "
          f"idle ones reaped after {args.idle_timeout:.0f}s.")
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning", access_log=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
