"""
dashboard/websocket.py
Thread-safe bridge between the simulation loop and the browser.

The simulation thread calls ``publish()`` with the latest state and the new
events; websocket clients receive them as soon as the version changes.
Operator actions (inject a fault, add a PoI) travel the other way through a
queue that the simulation drains at the start of its next tick - the web
server never touches the world directly.
"""

from __future__ import annotations

import asyncio
import queue
import threading
from collections import deque
from typing import Any, Optional

from fastapi import WebSocket, WebSocketDisconnect

ALLOWED_ACTIONS = ("degrade_link", "restore_link", "fail_uav", "add_obstacle", "remove_obstacle",
                   "add_poi", "complete_poi", "set_battery")


class LiveHub:
    def __init__(self, max_events: int = 500) -> None:
        self._lock = threading.Lock()
        self._payload: dict[str, Any] = {}
        self._version = 0
        self._events: deque[dict[str, Any]] = deque(maxlen=max_events)
        self._commands: "queue.Queue[tuple[str, dict[str, Any]]]" = queue.Queue()

    # ---------------------------------------------------- simulation -> browser
    def publish(self, payload: dict[str, Any], events: list[dict[str, Any]]) -> None:
        with self._lock:
            self._payload = payload
            self._version += 1
            self._events.extend(events)

    def latest(self) -> tuple[int, dict[str, Any]]:
        with self._lock:
            return self._version, self._payload

    def events_since(self, seq: int) -> list[dict[str, Any]]:
        with self._lock:
            return [e for e in self._events if e["seq"] > seq]

    # ---------------------------------------------------- browser -> simulation
    def submit_command(self, action: str, params: Optional[dict[str, Any]] = None) -> None:
        if action not in ALLOWED_ACTIONS:
            raise ValueError(f"action must be one of {ALLOWED_ACTIONS}")
        self._commands.put((action, dict(params or {})))

    def pop_commands(self) -> list[tuple[str, dict[str, Any]]]:
        out = []
        while True:
            try:
                out.append(self._commands.get_nowait())
            except queue.Empty:
                return out


async def stream_state(websocket: WebSocket, hub: LiveHub, interval_s: float = 0.2) -> None:
    """Push state + new events to one client until it disconnects."""
    await websocket.accept()
    last_version, last_seq = -1, 0

    async def sender() -> None:
        nonlocal last_version, last_seq
        while True:
            version, payload = hub.latest()
            if version != last_version and payload:
                last_version = version
                events = hub.events_since(last_seq)
                if events:
                    last_seq = events[-1]["seq"]
                await websocket.send_json({"type": "state", "state": payload, "events": events})
            await asyncio.sleep(interval_s)

    async def receiver() -> None:
        while True:  # the client only pings; commands go through POST /api/inject
            await websocket.receive_text()

    tasks = [asyncio.create_task(sender()), asyncio.create_task(receiver())]
    try:
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    except WebSocketDisconnect:
        pass
    finally:
        for task in tasks:
            task.cancel()
