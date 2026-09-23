"""
swarm_logging/event_logger.py
Writes every event of a run to disk: events.jsonl (full detail, one JSON object
per line) and events.csv (spreadsheet friendly). Keeps the records in memory so
the database writer can store them at the end of the run.

``run_dir=None`` records without writing anything, which is how the dashboard
keeps a whole run available for download. It cannot read the log back out of the
EventBus or the LiveHub: both are bounded ring buffers (2000 and 500 events), so
on any run longer than a couple of minutes they hold the tail and nothing else.
"""

from __future__ import annotations

import csv
import json
from collections import deque
from pathlib import Path
from typing import Any, Optional

from core.events import Event, EventBus

CSV_COLUMNS = ("seq", "t_s", "type", "severity", "uav_id", "poi_id", "message", "data")


class EventLogger:
    def __init__(self, run_dir: Optional[str | Path], bus: EventBus, flush_every: int = 25,
                 max_records: Optional[int] = None) -> None:
        self.run_dir = Path(run_dir) if run_dir is not None else None
        # deque(maxlen=None) is unbounded, so the on-disk path is unchanged.
        # A cap keeps the most recent events, matching EventBus and LiveHub.
        self.records: deque[dict[str, Any]] = deque(maxlen=max_records)
        self.dropped = 0
        self._flush_every = flush_every
        self._since_flush = 0
        self._jsonl = None
        self._csv_file = None
        self._csv = None
        if self.run_dir is not None:
            self.run_dir.mkdir(parents=True, exist_ok=True)
            self._jsonl = (self.run_dir / "events.jsonl").open("w", encoding="utf-8")
            self._csv_file = (self.run_dir / "events.csv").open("w", encoding="utf-8", newline="")
            self._csv = csv.writer(self._csv_file)
            self._csv.writerow(CSV_COLUMNS)
        self._unsubscribe = bus.subscribe(self._on_event)

    def _on_event(self, event: Event) -> None:
        record = event.to_dict()
        if self.records.maxlen is not None and len(self.records) == self.records.maxlen:
            self.dropped += 1          # the export says so rather than lying by omission
        self.records.append(record)
        if self._csv is None:
            return
        self._jsonl.write(json.dumps(record) + "\n")
        self._csv.writerow([record["seq"], record["t_s"], record["type"], record["severity"],
                            record["uav_id"], record["poi_id"], record["message"],
                            json.dumps(record["data"], sort_keys=True)])
        self._since_flush += 1
        if self._since_flush >= self._flush_every:
            self.flush()

    def flush(self) -> None:
        if self._jsonl is not None:
            self._jsonl.flush()
        if self._csv_file is not None:
            self._csv_file.flush()
        self._since_flush = 0

    def close(self) -> None:
        self._unsubscribe()
        self.flush()
        if self._jsonl is not None:
            self._jsonl.close()
        if self._csv_file is not None:
            self._csv_file.close()
