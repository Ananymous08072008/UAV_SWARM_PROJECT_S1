"""
swarm_logging/event_logger.py
Writes every event of a run to disk: events.jsonl (full detail, one JSON object
per line) and events.csv (spreadsheet friendly). Keeps the records in memory so
the database writer can store them at the end of the run.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

from core.events import Event, EventBus

CSV_COLUMNS = ("seq", "t_s", "type", "severity", "uav_id", "poi_id", "message", "data")


class EventLogger:
    def __init__(self, run_dir: str | Path, bus: EventBus, flush_every: int = 25) -> None:
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.records: list[dict[str, Any]] = []
        self._flush_every = flush_every
        self._since_flush = 0
        self._jsonl = (self.run_dir / "events.jsonl").open("w", encoding="utf-8")
        self._csv_file = (self.run_dir / "events.csv").open("w", encoding="utf-8", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(CSV_COLUMNS)
        self._unsubscribe = bus.subscribe(self._on_event)

    def _on_event(self, event: Event) -> None:
        record = event.to_dict()
        self.records.append(record)
        self._jsonl.write(json.dumps(record) + "\n")
        self._csv.writerow([record["seq"], record["t_s"], record["type"], record["severity"],
                            record["uav_id"], record["poi_id"], record["message"],
                            json.dumps(record["data"], sort_keys=True)])
        self._since_flush += 1
        if self._since_flush >= self._flush_every:
            self.flush()

    def flush(self) -> None:
        self._jsonl.flush()
        self._csv_file.flush()
        self._since_flush = 0

    def close(self) -> None:
        self._unsubscribe()
        self.flush()
        self._jsonl.close()
        self._csv_file.close()
