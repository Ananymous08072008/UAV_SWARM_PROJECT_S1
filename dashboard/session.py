"""
dashboard/session.py
One independent simulation per visitor, driven from the browser.

The single-simulation mode (``main.py --dashboard``) keeps the world in the main
thread and serves one shared view. That cannot give two people different
scenarios, so this module inverts the arrangement: the web server owns the
process and every session owns a daemon thread running its own World.

    SessionManager
      -> SimulationSession (id, spec)
           -> LiveHub          state + events for this session's viewers
           -> Simulation       its own World, Environment, MissionManager
           -> worker thread    start / pause / resume / stop / restart

Nothing is shared between sessions except the immutable Parameters, so one
viewer failing a relay cannot disturb anybody else's mission.

The worker thread is the only writer of its world. The API threads only read
through LiveHub (lock protected) and push commands onto its queue, which the
simulation drains at the top of its next tick - the same contract the
single-simulation mode already uses.
"""

from __future__ import annotations

import logging
import threading
import time
import uuid
from typing import Any, Mapping, Optional

from core.config import ConfigError, Parameters
from dashboard.mission import build_scenario
from dashboard.websocket import LiveHub
from simulation.runner import Simulation

log = logging.getLogger(__name__)

# A shared box has to bound what visitors can start.
MAX_SESSIONS = 8
IDLE_TIMEOUT_S = 1800.0      # reap a session nobody has polled for 30 minutes
MAX_SPEED = 20.0
MIN_SPEED = 0.25

IDLE, RUNNING, PAUSED, FINISHED, ERROR = "idle", "running", "paused", "finished", "error"


class SessionError(RuntimeError):
    """The requested control action does not apply in the current state."""


class SimulationSession:
    """One visitor's simulation, its own thread, controllable from the browser."""

    def __init__(self, session_id: str, spec: Mapping[str, Any], params: Parameters) -> None:
        self.id = session_id
        self.spec = dict(spec or {})
        self.params = params
        self.hub = LiveHub()
        self.state = IDLE
        self.error: Optional[str] = None
        self.generation = 0          # bumped on restart so viewers resync
        self.created_at = time.time()
        self.last_seen = time.time()

        self.mode = "baseline" if str(self.spec.get("mode", "")).lower() == "baseline" else "adaptive"
        self.speed = self._clamp_speed(self.spec.get("speed", 4.0))
        self.realtime = bool(self.spec.get("realtime", True))

        self._scenario = build_scenario(self.spec)       # raises ConfigError
        self.sim = self._new_sim()

        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._resume = threading.Event()
        self._resume.set()
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _clamp_speed(value: Any) -> float:
        try:
            speed = float(value)
        except (TypeError, ValueError):
            return 4.0
        return max(MIN_SPEED, min(MAX_SPEED, speed))

    def _new_sim(self) -> Simulation:
        # results_dir=None: a shared server should not litter the disk with a run
        # directory per visitor. Single-simulation mode still writes results.
        return Simulation(self.params, self._scenario, mode=self.mode,
                          results_dir=None, hub=self.hub,
                          stop_when_complete=not bool(self.spec.get("keep_running", False)))

    def touch(self) -> None:
        self.last_seen = time.time()

    # ------------------------------------------------------------------ controls
    def start(self) -> None:
        with self._lock:
            if self.state in (RUNNING, PAUSED):
                raise SessionError("already started - use restart")
            if self.state in (FINISHED, ERROR):
                raise SessionError("run has ended - use restart")
            self.state = RUNNING
            self._stop.clear()
            self._resume.set()
            self._thread = threading.Thread(target=self._loop, name=f"sim-{self.id[:8]}",
                                            daemon=True)
            self._thread.start()

    def pause(self) -> None:
        with self._lock:
            if self.state != RUNNING:
                raise SessionError(f"cannot pause while {self.state}")
            self.state = PAUSED
            self._resume.clear()

    def resume(self) -> None:
        with self._lock:
            if self.state != PAUSED:
                raise SessionError(f"cannot resume while {self.state}")
            self.state = RUNNING
            self._resume.set()

    def stop(self, reason: str = "stopped by operator") -> None:
        with self._lock:
            if self.state not in (RUNNING, PAUSED):
                return
            self._stop.set()
            self._resume.set()          # release the loop if it is parked in pause
            thread = self._thread
        if thread is not None:
            thread.join(timeout=10.0)
        self._finish(reason)

    def restart(self, spec: Optional[Mapping[str, Any]] = None) -> None:
        """Rebuild the world - optionally with a new mission - and run it again."""
        self.stop("restarting")
        with self._lock:
            if spec is not None:
                merged = dict(self.spec)
                merged.update(spec)
                self.spec = merged
                self.mode = "baseline" if str(merged.get("mode", "")).lower() == "baseline" else "adaptive"
                self.speed = self._clamp_speed(merged.get("speed", self.speed))
                self.realtime = bool(merged.get("realtime", self.realtime))
                self._scenario = build_scenario(merged)   # raises ConfigError
            self.generation += 1
            self.hub = LiveHub()        # fresh event sequence; viewers resync on generation
            self.sim = self._new_sim()
            self.state = IDLE
            self.error = None
        self.start()

    def set_speed(self, value: Any) -> float:
        self.speed = self._clamp_speed(value)
        return self.speed

    # ------------------------------------------------------------------ the loop
    def _loop(self) -> None:
        sim = self.sim
        try:
            sim.start()
            # Wall-clock origin for t=0. Shifted forward by however long we spend
            # paused so resuming does not make the simulation sprint to catch up.
            origin = time.perf_counter()
            while not self._stop.is_set() and not sim.is_finished:
                if not self._resume.is_set():
                    paused_at = time.perf_counter()
                    self._resume.wait()
                    origin += time.perf_counter() - paused_at
                    if self._stop.is_set():
                        break
                sim.tick()
                if self.realtime:
                    delay = origin + sim.world.t / self.speed - time.perf_counter()
                    if delay > 0:
                        # Wake early if someone stops us, instead of sleeping through it.
                        self._stop.wait(delay)
        except Exception as exc:                       # keep one bad run off the server
            log.exception("session %s failed", self.id)
            self.error = f"{type(exc).__name__}: {exc}"
            self.state = ERROR
            return
        if not self._stop.is_set():
            self._finish("mission complete" if sim.is_finished else "duration reached")

    def _finish(self, reason: str) -> None:
        with self._lock:
            if self.state in (FINISHED, ERROR):
                return
            try:
                self.sim.finish(reason)
            except Exception as exc:                   # a summary failure is not fatal
                log.warning("session %s summary failed: %s", self.id, exc)
            self.state = FINISHED

    # ------------------------------------------------------------------ reporting
    def status(self) -> dict[str, Any]:
        sim = self.sim
        return {
            "id": self.id,
            "state": self.state,
            "error": self.error,
            "generation": self.generation,
            "mode": self.mode,
            "speed": self.speed,
            "realtime": self.realtime,
            "t_s": round(sim.world.t, 1),
            "duration_s": self._scenario.duration_s,
            "scenario": self._scenario.name,
            "uav_count": len(sim.world.state.uavs),
            "fleet": dict(sim.world.fleet),
            "poi_count": len(sim.world.state.pois),
            "created_at": self.created_at,
            "age_s": round(time.time() - self.created_at, 1),
            "summary": sim.summary,
        }

    def shutdown(self) -> None:
        self._stop.set()
        self._resume.set()


class SessionManager:
    """Creates, tracks and reaps the per-visitor simulations."""

    def __init__(self, params: Optional[Parameters] = None,
                 max_sessions: int = MAX_SESSIONS,
                 idle_timeout_s: float = IDLE_TIMEOUT_S) -> None:
        self.params = params or Parameters()
        self.max_sessions = max_sessions
        self.idle_timeout_s = idle_timeout_s
        self._sessions: dict[str, SimulationSession] = {}
        self._lock = threading.Lock()

    def create(self, spec: Optional[Mapping[str, Any]] = None, autostart: bool = True) -> SimulationSession:
        # Validate the mission before anything else, so a malformed spec reports
        # what is actually wrong with it rather than "server is at capacity".
        spec = spec or {}
        build_scenario(spec)                                        # raises ConfigError
        self.reap()
        # Build outside the lock (constructing a World is not instant), then check
        # capacity and insert atomically so two simultaneous requests cannot both
        # pass the check and overshoot the limit.
        session = SimulationSession(uuid.uuid4().hex[:12], spec, self.params)
        with self._lock:
            if len(self._sessions) >= self.max_sessions:
                raise SessionError(
                    f"server is at capacity ({self.max_sessions} simulations). "
                    "Close one or wait for an idle session to expire.")
            self._sessions[session.id] = session
        if autostart:
            session.start()
        log.info("session %s created (%d UAVs, %d PoIs)", session.id,
                 len(session.sim.world.state.uavs), len(session.sim.world.state.pois))
        return session

    def get(self, session_id: str) -> SimulationSession:
        with self._lock:
            session = self._sessions.get(session_id)
        if session is None:
            raise KeyError(session_id)
        session.touch()
        return session

    def delete(self, session_id: str) -> None:
        with self._lock:
            session = self._sessions.pop(session_id, None)
        if session is not None:
            session.stop("closed by operator")
            session.shutdown()

    def list(self) -> list[dict[str, Any]]:
        with self._lock:
            sessions = list(self._sessions.values())
        return [s.status() for s in sessions]

    def reap(self) -> int:
        """Drop sessions nobody has looked at for idle_timeout_s."""
        cutoff = time.time() - self.idle_timeout_s
        with self._lock:
            stale = [sid for sid, s in self._sessions.items() if s.last_seen < cutoff]
            dropped = [self._sessions.pop(sid) for sid in stale]
        for session in dropped:
            log.info("reaping idle session %s", session.id)
            session.stop("idle timeout")
            session.shutdown()
        return len(dropped)

    def shutdown(self) -> None:
        with self._lock:
            sessions = list(self._sessions.values())
            self._sessions.clear()
        for session in sessions:
            session.shutdown()
