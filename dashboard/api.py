"""
dashboard/api.py
FastAPI application and the background server that serves the dashboard.

    GET  /                 the dashboard page
    GET  /api/state        latest world + network + metrics snapshot (JSON)
    GET  /api/events?since event log since a sequence number
    POST /api/inject       operator action: {"action": "...", "params": {...}}
    WS   /ws               live stream of state + events

The server runs in a daemon thread so the simulation keeps the main thread.
"""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Any, Optional

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from dashboard.websocket import ALLOWED_ACTIONS, LiveHub, stream_state

BASE_DIR = Path(__file__).resolve().parent
log = logging.getLogger(__name__)


def create_app(hub: LiveHub, push_interval_s: float = 0.2) -> FastAPI:
    app = FastAPI(title="UAV Swarm Dashboard", docs_url=None, redoc_url=None)
    app.mount("/static", StaticFiles(directory=BASE_DIR / "static"), name="static")

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(BASE_DIR / "templates" / "index.html")

    @app.get("/api/state")
    def state() -> JSONResponse:
        version, payload = hub.latest()
        return JSONResponse({"version": version, "state": payload})

    @app.get("/api/events")
    def events(since: int = 0) -> JSONResponse:
        return JSONResponse({"events": hub.events_since(since)})

    @app.get("/api/actions")
    def actions() -> JSONResponse:
        return JSONResponse({"actions": list(ALLOWED_ACTIONS)})

    @app.post("/api/inject")
    def inject(body: dict[str, Any]) -> JSONResponse:
        action = str(body.get("action", ""))
        params = body.get("params") or {}
        if not isinstance(params, dict):
            raise HTTPException(status_code=400, detail="params must be an object")
        try:
            hub.submit_command(action, params)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from None
        return JSONResponse({"queued": True, "action": action, "params": params})

    @app.websocket("/ws")
    async def ws(websocket: WebSocket) -> None:
        await stream_state(websocket, hub, push_interval_s)

    return app


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
        return f"http://{self.host}:{self.port}"

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
