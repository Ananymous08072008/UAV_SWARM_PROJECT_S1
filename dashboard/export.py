"""
dashboard/export.py
Packs one simulation run into a single downloadable archive.

``main.py`` writes a directory per run under ``results/``. The mission studio
deliberately does not - a shared server would litter the disk with a directory
per visitor, and a hosted one loses the disk anyway - so the same files are built
in memory here and handed to the browser as one zip.

The names and columns match a ``results/<run>/`` directory exactly, so whatever
reads one reads the other: experiments/plot_results.py, a spreadsheet, pandas.

    <run>/summary.json      the grouped metrics used in the report
    <run>/timeseries.csv    one row per metric sample
    <run>/events.csv        every event, spreadsheet friendly
    <run>/events.jsonl      every event, full detail
    <run>/scenario.json     what was run, enough to run it again
    <run>/manifest.json     export provenance, including any truncation
"""

from __future__ import annotations

import csv
import io
import json
import re
import zipfile
from datetime import datetime, timezone
from typing import Any, Mapping, Optional

from swarm_logging.event_logger import CSV_COLUMNS

# Anything outside this is stripped from the download name. Scenario names reach
# us from YAML and from the browser, and the name goes into a Content-Disposition
# header - an unescaped quote or newline there is a header injection.
_UNSAFE = re.compile(r"[^A-Za-z0-9._-]+")


def safe_name(text: str, fallback: str = "run") -> str:
    cleaned = _UNSAFE.sub("-", str(text)).strip("-._")
    return cleaned[:60] or fallback


def _csv_bytes(rows: list[Mapping[str, Any]], columns: tuple[str, ...]) -> bytes:
    buf = io.StringIO(newline="")
    writer = csv.DictWriter(buf, fieldnames=list(columns), extrasaction="ignore")
    writer.writeheader()
    writer.writerows(rows)
    return buf.getvalue().encode("utf-8")


def _events_csv(records: list[Mapping[str, Any]]) -> bytes:
    buf = io.StringIO(newline="")
    writer = csv.writer(buf)
    writer.writerow(CSV_COLUMNS)
    for r in records:
        writer.writerow([r["seq"], r["t_s"], r["type"], r["severity"], r["uav_id"],
                         r["poi_id"], r["message"], json.dumps(r.get("data", {}), sort_keys=True)])
    return buf.getvalue().encode("utf-8")


def _json_bytes(payload: Any) -> bytes:
    return json.dumps(payload, indent=2, default=str).encode("utf-8")


def run_stem(sim, session_id: Optional[str] = None) -> str:
    """The archive's base name, built like a results/ run directory."""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    parts = [stamp, safe_name(sim.world.scenario.name, "scenario"), safe_name(sim.mode, "mode")]
    if session_id:
        parts.append(safe_name(session_id, "session"))
    return "_".join(parts)


def build_archive(sim, session_id: Optional[str] = None,
                  spec: Optional[Mapping[str, Any]] = None,
                  extra: Optional[Mapping[str, Any]] = None) -> tuple[str, bytes]:
    """
    Build the zip for ``sim`` and return ``(filename, bytes)``.

    Safe to call on a run that is still going: every list is snapshotted first,
    because the simulation thread owns them and keeps appending.
    """
    stem = run_stem(sim, session_id)
    rows = list(sim.metrics.rows())

    # A finished run already computed its summary; a running one is aggregated now.
    # A crashed run still has logs worth downloading, so a summary that cannot be
    # produced must not take the archive down with it.
    summary, summary_error = sim.summary, None
    if summary is None:
        try:
            summary = sim.metrics.summary()
        except Exception as exc:
            summary, summary_error = {}, f"{type(exc).__name__}: {exc}"

    logger = getattr(sim, "logger", None)
    if logger is not None:
        records = list(logger.records)
        dropped, source = logger.dropped, "event log"
    else:
        # No recorder attached: the bus keeps only its last 2000 events, so say so
        # rather than presenting the tail as the whole run.
        records = [e.to_dict() for e in sim.world.events.history()]
        dropped, source = 0, "event bus (bounded ring buffer - may be incomplete)"

    world = sim.world
    scenario = {"scenario": world.scenario.name, "mode": sim.mode, "seed": world.seed,
                "duration_s": world.duration_s}
    if spec is not None:
        # What the browser asked for: UAV count, PoI placement, speed. Without it a
        # studio run cannot be reproduced, since it came from clicks, not a file.
        scenario["mission_spec"] = dict(spec)

    manifest = {
        "exported_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "session_id": session_id,
        "scenario": world.scenario.name,
        "mode": sim.mode,
        "seed": world.seed,
        "sim_time_s": round(world.t, 2),
        "run_finished": sim.summary is not None,
        "samples": len(rows),
        "events": len(records),
        "events_source": source,
        "events_dropped": dropped,
        "complete": dropped == 0 and source == "event log",
        "files": ["summary.json", "timeseries.csv", "events.csv", "events.jsonl",
                  "scenario.json", "manifest.json"],
    }
    if summary_error is not None:
        manifest["summary_error"] = summary_error
    if extra:
        manifest.update(extra)

    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.writestr(f"{stem}/summary.json", _json_bytes(summary))
        if rows:
            zf.writestr(f"{stem}/timeseries.csv", _csv_bytes(rows, tuple(rows[0])))
        zf.writestr(f"{stem}/events.csv", _events_csv(records))
        zf.writestr(f"{stem}/events.jsonl",
                    "".join(json.dumps(r) + "\n" for r in records).encode("utf-8"))
        zf.writestr(f"{stem}/scenario.json", _json_bytes(scenario))
        zf.writestr(f"{stem}/manifest.json", _json_bytes(manifest))
    return f"{stem}.zip", buf.getvalue()
