"""
dashboard/api.py
FastAPI application and the background server that serves the dashboard.

Two modes share this app.

Single simulation (``main.py --dashboard``) - one world, one shared view:

    GET  /                 the dashboard page
    GET  /api/state        latest world + network + metrics snapshot (JSON)
    GET  /api/events?since event log since a sequence number
    POST /api/inject       operator action: {"action": "...", "params": {...}}
    GET  /api/export       this run's mission metrics and event log (.xlsx)
    WS   /ws               live stream of state + events

Multi session (``python -m dashboard.server``) - one world per visitor, built
and driven from the browser:

    GET    /                          the mission studio page
    GET    /api/limits                mission builder constraints
    GET    /api/sessions              every live simulation on this server
    POST   /api/sessions              build and start one {uav_count, pois, ...}
    GET    /api/sessions/{id}         status of one simulation
    DELETE /api/sessions/{id}         stop and discard it
    GET    /api/sessions/{id}/state   snapshot
    GET    /api/sessions/{id}/events  event log since a sequence number
    POST   /api/sessions/{id}/control start|pause|resume|stop|restart|speed
    POST   /api/sessions/{id}/inject  operator action, scoped to that world
    GET    /api/sessions/{id}/export  that run's mission metrics and event log (.xlsx)
    WS     /ws/{id}                   live stream for that simulation

In single-simulation mode the server runs in a daemon thread so the simulation
keeps the main thread. In multi-session mode it is the other way round: the
server owns the process and each simulation gets its own thread.
"""

from __future__ import annotations

import asyncio
import logging
import socket
import threading
import time
from pathlib import Path
from typing import Any, Optional

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

from core.config import ConfigError
from dashboard.export import XLSX_MEDIA_TYPE, build_workbook
from dashboard.websocket import ALLOWED_ACTIONS, LiveHub, stream_state

BASE_DIR = Path(__file__).resolve().parent
log = logging.getLogger(__name__)

# Hosts that mean "every interface". Useful to bind, useless to type in a browser.
WILDCARD_HOSTS = ("0.0.0.0", "::", "")


def lan_address(fallback: str = "127.0.0.1") -> str:
    """This machine's address on the local network.

    Opening a UDP socket and connecting it to an off-machine address makes the OS
    pick the interface it would route through, and reports it via getsockname().
    Nothing is sent and the address need not exist, so this costs no traffic and
    does not need the host to be reachable.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("10.255.255.255", 1))
        return sock.getsockname()[0]
    except OSError:
        return fallback          # no network at all: loopback is all we have
    finally:
        sock.close()


def shareable_url(host: str, port: int) -> str:
    """A URL a browser on another machine can actually open.

    ``--host 0.0.0.0`` accepts connections on every interface, but
    ``http://0.0.0.0:8000`` resolves nowhere, so printing it back at the operator
    is no help when the point was to share the link. Substitute the LAN address.
    """
    if host in WILDCARD_HOSTS:
        return f"http://{lan_address()}:{port}"
    return f"http://{host}:{port}"


def _workbook_response(sim, session_id: Optional[str] = None,
                       extra: Optional[dict[str, Any]] = None) -> Response:
    """Serve one run's mission metrics and event log as an Excel workbook."""
    if sim is None:
        raise HTTPException(status_code=409,
                            detail="no simulation is attached to this dashboard")
    filename, blob = build_workbook(sim, session_id=session_id, extra=extra)
    # filename is assembled from a timestamp and export.safe_name(), so it cannot
    # break out of the quoted header value.
    return Response(content=blob, media_type=XLSX_MEDIA_TYPE,
                    headers={"Content-Disposition": f'attachment; filename="{filename}"'})


def _inject(hub: LiveHub, body: dict[str, Any]) -> dict[str, Any]:
    """Validate and queue one operator action. Shared by both modes."""
    action = str(body.get("action", ""))
    params = body.get("params") or {}
    if not isinstance(params, dict):
        raise HTTPException(status_code=400, detail="params must be an object")
    try:
        hub.submit_command(action, params)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from None
    return {"queued": True, "action": action, "params": params}


def create_app(hub: Optional[LiveHub] = None, push_interval_s: float = 0.2,
               manager: Optional["SessionManager"] = None) -> FastAPI:  # noqa: F821
    """
    Build the app.

    ``hub`` enables the single-simulation routes, ``manager`` the multi-session
    ones. Passing both is allowed and is what the tests use.
    """
    app = FastAPI(title="UAV Swarm Dashboard", docs_url=None, redoc_url=None)
    app.mount("/static", StaticFiles(directory=BASE_DIR / "static"), name="static")

    @app.get("/")
    def index() -> FileResponse:
        page = "studio.html" if manager is not None else "index.html"
        return FileResponse(BASE_DIR / "templates" / page)

    @app.get("/api/actions")
    def actions() -> JSONResponse:
        return JSONResponse({"actions": list(ALLOWED_ACTIONS)})

    # ---------------------------------------------------- single simulation
    if hub is not None:
        @app.get("/api/state")
        def state() -> JSONResponse:
            version, payload = hub.latest()
            return JSONResponse({"version": version, "state": payload})

        @app.get("/api/events")
        def events(since: int = 0) -> JSONResponse:
            return JSONResponse({"events": hub.events_since(since)})

        @app.post("/api/inject")
        def inject(body: dict[str, Any]) -> JSONResponse:
            return JSONResponse(_inject(hub, body))

        @app.get("/api/export")
        def export() -> Response:
            return _workbook_response(hub.simulation)

        @app.websocket("/ws")
        async def ws(websocket: WebSocket) -> None:
            await stream_state(websocket, hub, push_interval_s)

    # ------------------------------------------------------- multi session
    if manager is not None:
        from dashboard.mission import limits as mission_limits
        from dashboard.session import SessionError

        def _session(session_id: str):
            try:
                return manager.get(session_id)
            except KeyError:
                raise HTTPException(status_code=404,
                                    detail="no such simulation - it may have expired") from None

        @app.get("/api/limits")
        def limits() -> JSONResponse:
            # The builder map needs the geo origin to turn a click into local
            # ENU metres before a world exists to read it from.
            data = dict(mission_limits())
            origin = manager.params.geo_origin
            data["origin"] = {"lat_deg": origin.lat_deg, "lon_deg": origin.lon_deg,
                              "alt_msl_m": origin.alt_msl_m}
            return JSONResponse(data)

        @app.get("/api/sessions")
        def list_sessions() -> JSONResponse:
            return JSONResponse({"sessions": manager.list(),
                                 "capacity": manager.max_sessions})

        @app.post("/api/sessions")
        def create_session(body: Optional[dict[str, Any]] = None) -> JSONResponse:
            try:
                session = manager.create(body or {})
            except ConfigError as exc:
                raise HTTPException(status_code=400, detail=str(exc)) from None
            except SessionError as exc:
                raise HTTPException(status_code=503, detail=str(exc)) from None
            return JSONResponse(session.status(), status_code=201)

        @app.get("/api/sessions/{session_id}")
        def session_status(session_id: str) -> JSONResponse:
            return JSONResponse(_session(session_id).status())

        @app.delete("/api/sessions/{session_id}")
        def close_session(session_id: str) -> JSONResponse:
            _session(session_id)
            manager.delete(session_id)
            return JSONResponse({"closed": session_id})

        @app.get("/api/sessions/{session_id}/state")
        def session_state(session_id: str) -> JSONResponse:
            session = _session(session_id)
            version, payload = session.hub.latest()
            return JSONResponse({"version": version, "state": payload,
                                 "status": session.status()})

        @app.get("/api/sessions/{session_id}/events")
        def session_events(session_id: str, since: int = 0) -> JSONResponse:
            return JSONResponse({"events": _session(session_id).hub.events_since(since)})

        @app.get("/api/sessions/{session_id}/history")
        def session_history(session_id: str, every: int = 1) -> JSONResponse:
            """
            The metric samples so far, so somebody joining a running mission sees
            the curve that led to now instead of an empty chart.

            ``every`` thins the series (every=5 keeps one sample in five) to keep
            the payload small on a long run.
            """
            session = _session(session_id)
            step = max(1, int(every))
            rows = session.sim.metrics.rows()[::step]
            return JSONResponse({"samples": [
                {"t_s": r["t_s"],
                 "connectivity": r["connectivity_ratio"],
                 "pdr": r["mean_route_pdr"],
                 "latency": r["mean_latency_ms"]}
                for r in rows]})

        @app.get("/api/sessions/{session_id}/export")
        def session_export(session_id: str) -> Response:
            """This run's mission metrics and event log as an Excel workbook.

            Studio sessions write nothing to disk, so this is the only way their
            data leaves the server - and the session is reaped when idle.
            """
            session = _session(session_id)
            return _workbook_response(session.sim, session_id=session.id,
                                      extra={"state": session.state})

        @app.post("/api/sessions/{session_id}/inject")
        def session_inject(session_id: str, body: dict[str, Any]) -> JSONResponse:
            return JSONResponse(_inject(_session(session_id).hub, body))

        @app.post("/api/sessions/{session_id}/control")
        def session_control(session_id: str, body: dict[str, Any]) -> JSONResponse:
            session = _session(session_id)
            action = str(body.get("action", "")).lower()
            try:
                if action == "start":
                    session.start()
                elif action == "pause":
                    session.pause()
                elif action == "resume":
                    session.resume()
                elif action == "stop":
                    session.stop()
                elif action == "restart":
                    session.restart(body.get("spec"))
                elif action == "speed":
                    session.set_speed(body.get("speed"))
                else:
                    raise HTTPException(
                        status_code=400,
                        detail="action must be start, pause, resume, stop, restart or speed")
            except ConfigError as exc:
                raise HTTPException(status_code=400, detail=str(exc)) from None
            except SessionError as exc:
                raise HTTPException(status_code=409, detail=str(exc)) from None
            return JSONResponse(session.status())

        @app.websocket("/ws/{session_id}")
        async def session_ws(websocket: WebSocket, session_id: str) -> None:
            try:
                session = manager.get(session_id)
            except KeyError:
                await websocket.close(code=4004)
                return
            await stream_session(websocket, session, push_interval_s)

    return app


async def stream_session(websocket: WebSocket, session, interval_s: float = 0.2) -> None:
    """
    Push one session's state, events and control status until the client leaves.

    Unlike stream_state this re-reads ``session.hub`` every pass, because a
    restart swaps in a fresh hub whose event sequence begins at zero again. The
    generation counter tells us that happened so the client can drop the old
    event log instead of ignoring the new events as already-seen.
    """
    await websocket.accept()
    last_version, last_seq, generation = -1, 0, session.generation

    async def sender() -> None:
        nonlocal last_version, last_seq, generation
        while True:
            session.touch()
            if session.generation != generation:
                generation = session.generation
                last_version, last_seq = -1, 0
                await websocket.send_json({"type": "reset", "generation": generation})
            hub = session.hub
            version, payload = hub.latest()
            if version != last_version and payload:
                last_version = version
                events = hub.events_since(last_seq)
                if events:
                    last_seq = events[-1]["seq"]
                await websocket.send_json({"type": "state", "state": payload,
                                           "events": events, "status": session.status()})
            else:
                await websocket.send_json({"type": "status", "status": session.status()})
            await asyncio.sleep(interval_s)

    async def receiver() -> None:
        while True:
            await websocket.receive_text()

    tasks = [asyncio.create_task(sender()), asyncio.create_task(receiver())]
    try:
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    except WebSocketDisconnect:
        pass
    finally:
        for task in tasks:
            task.cancel()


class DashboardServer:
    """Runs uvicorn in a daemon thread next to the simulation."""

    def __init__(self, hub: LiveHub, host: str = "127.0.0.1", port: int = 8000,
                 push_interval_s: float = 0.2) -> None:
        self.hub = hub
        self.host = host
        self.port = port
        config = uvicorn.Config(create_app(hub, push_interval_s), host=host, port=port,
                                log_level="warning", access_log=False)
        self.server = uvicorn.Server(config)
        self._thread: Optional[threading.Thread] = None

    @property
    def url(self) -> str:
        """The bind address, as given. Used in diagnostics, not for sharing."""
        return f"http://{self.host}:{self.port}"

    @property
    def shareable_url(self) -> str:
        """The link to print or hand to someone on another machine."""
        return shareable_url(self.host, self.port)

    @property
    def serves_network(self) -> bool:
        """True when bound to every interface, so other machines can connect."""
        return self.host in WILDCARD_HOSTS

    def start(self, timeout_s: float = 10.0) -> str:
        self._thread = threading.Thread(target=self.server.run, name="dashboard", daemon=True)
        self._thread.start()
        deadline = time.perf_counter() + timeout_s
        while not self.server.started and self._thread.is_alive() and time.perf_counter() < deadline:
            time.sleep(0.05)
        if not self.server.started:
            raise RuntimeError(f"dashboard server did not start on {self.url}")
        return self.url

    def stop(self, timeout_s: float = 5.0) -> None:
        self.server.should_exit = True
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
