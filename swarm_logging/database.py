"""
swarm_logging/database.py
SQLite storage so several runs can be compared later with plain SQL.

    runs(run_id, started_at, scenario, mode, seed, duration_s, summary_json)
    events(run_id, seq, t_s, type, severity, uav_id, poi_id, message, data_json)
    samples(run_id, t_s, metric, value)        -- time series, long format
"""

from __future__ import annotations

import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional

SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    run_id       INTEGER PRIMARY KEY AUTOINCREMENT,
    started_at   TEXT NOT NULL,
    scenario     TEXT NOT NULL,
    mode         TEXT NOT NULL,
    seed         INTEGER,
    duration_s   REAL,
    summary_json TEXT
);
CREATE TABLE IF NOT EXISTS events (
    run_id    INTEGER NOT NULL REFERENCES runs(run_id),
    seq       INTEGER NOT NULL,
    t_s       REAL NOT NULL,
    type      TEXT NOT NULL,
    severity  TEXT NOT NULL,
    uav_id    INTEGER,
    poi_id    TEXT,
    message   TEXT,
    data_json TEXT,
    PRIMARY KEY (run_id, seq)
);
CREATE TABLE IF NOT EXISTS samples (
    run_id INTEGER NOT NULL REFERENCES runs(run_id),
    t_s    REAL NOT NULL,
    metric TEXT NOT NULL,
    value  REAL
);
CREATE INDEX IF NOT EXISTS idx_events_type ON events(run_id, type);
CREATE INDEX IF NOT EXISTS idx_samples_metric ON samples(run_id, metric);
"""


class RunDatabase:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.conn = sqlite3.connect(self.path)
        self.conn.executescript(SCHEMA)
        self.conn.commit()

    def start_run(self, scenario: str, mode: str, seed: Optional[int]) -> int:
        cur = self.conn.execute(
            "INSERT INTO runs (started_at, scenario, mode, seed) VALUES (?, ?, ?, ?)",
            (datetime.now(timezone.utc).isoformat(timespec="seconds"), scenario, mode, seed))
        self.conn.commit()
        return int(cur.lastrowid)

    def add_events(self, run_id: int, events: Iterable[Mapping[str, Any]]) -> None:
        self.conn.executemany(
            "INSERT OR REPLACE INTO events (run_id, seq, t_s, type, severity, uav_id, poi_id, message, data_json) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
            [(run_id, e["seq"], e["t_s"], e["type"], e["severity"], e["uav_id"], e["poi_id"], e["message"],
              json.dumps(e.get("data", {}), sort_keys=True)) for e in events])
        self.conn.commit()

    def add_samples(self, run_id: int, rows: Iterable[Mapping[str, Any]]) -> None:
        payload = [(run_id, row["t_s"], metric, float(value))
                   for row in rows for metric, value in row.items()
                   if metric != "t_s" and isinstance(value, (int, float))]
        self.conn.executemany("INSERT INTO samples (run_id, t_s, metric, value) VALUES (?, ?, ?, ?)", payload)
        self.conn.commit()

    def finish_run(self, run_id: int, duration_s: float, summary: Mapping[str, Any]) -> None:
        self.conn.execute("UPDATE runs SET duration_s = ?, summary_json = ? WHERE run_id = ?",
                          (duration_s, json.dumps(summary, sort_keys=True), run_id))
        self.conn.commit()

    def close(self) -> None:
        self.conn.close()
