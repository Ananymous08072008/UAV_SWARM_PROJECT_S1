"""
experiments/plot_results.py
Turns an experiment CSV into the figures used in the proposal.

    python experiments/plot_results.py                                  # newest experiment run
    python experiments/plot_results.py results/experiments/<stamp>/summary.csv

Produces, next to the CSV:
    fig_mission.png        PoI completion rate and mission time
    fig_communication.png  network availability, live imagery share, delivery delay
    fig_resilience.png     detection time, recovery time, relay changes
    fig_efficiency.png     distance flown, energy used, relay utilisation
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path
from statistics import fmean
from typing import Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parent.parent
COLORS = {"adaptive": "#4fc3f7", "baseline": "#f2984e"}

FIGURES = {
    "fig_mission": ("Mission performance", [
        ("mission.completion_rate", "PoI completion rate"),
        ("mission.mission_complete_s", "mission time (s)"),
        ("mission.high_priority_response_s", "response to new priority (s)"),
    ]),
    "fig_communication": ("Communication performance", [
        ("communication.network_availability", "UAVs connected (share of time)"),
        ("communication.data_live_ratio", "imagery delivered live"),
        ("communication.data_mean_delay_s", "imagery delay (s)"),
        ("communication.comm_downtime_s", "communication downtime (s)"),
    ]),
    "fig_resilience": ("Resilience", [
        ("resilience.mean_detection_time_s", "fault detection time (s)"),
        ("resilience.reconnected_share", "affected UAVs reconnected"),
        ("resilience.mean_reconnect_time_s", "time to reconnect (s)"),
        ("resilience.relay_changes", "relay changes"),
    ]),
    "fig_efficiency": ("Efficiency and safety", [
        ("efficiency.distance_total_m", "distance flown (m)"),
        ("efficiency.energy_consumed_pct", "energy used (battery %)"),
        ("efficiency.relay_utilisation", "relay utilisation"),
        ("safety.separation_violations", "separation violations"),
    ]),
}


def load(csv_path: Path) -> list[dict]:
    with csv_path.open(encoding="utf-8", newline="") as fh:
        rows = list(csv.DictReader(fh))
    for row in rows:
        for key, value in row.items():
            if key in ("scenario", "mode"):
                continue
            row[key] = float(value) if value not in ("", "None") else None
    return rows


def newest_csv() -> Optional[Path]:
    candidates = sorted((PROJECT_ROOT / "results" / "experiments").glob("*/summary.csv"))
    return candidates[-1] if candidates else None


def mean_by(rows: list[dict], metric: str) -> dict[tuple[str, str], float]:
    buckets: dict[tuple[str, str], list[float]] = defaultdict(list)
    for row in rows:
        if row.get(metric) is not None:
            buckets[(row["scenario"], row["mode"])].append(row[metric])
    return {key: fmean(values) for key, values in buckets.items()}


def draw(rows: list[dict], out_dir: Path) -> list[Path]:
    scenarios = sorted({r["scenario"] for r in rows})
    modes = sorted({r["mode"] for r in rows})
    written = []
    for name, (title, metrics) in FIGURES.items():
        fig, axes = plt.subplots(1, len(metrics), figsize=(4.6 * len(metrics), 4.2))
        axes = axes if len(metrics) > 1 else [axes]
        for axis, (metric, label) in zip(axes, metrics):
            values = mean_by(rows, metric)
            width = 0.8 / len(modes)
            for i, mode in enumerate(modes):
                heights = [values.get((s, mode), 0.0) for s in scenarios]
                axis.bar([x + i * width for x in range(len(scenarios))], heights, width,
                         label=mode, color=COLORS.get(mode))
            axis.set_title(label, fontsize=10)
            axis.set_xticks([x + 0.4 - width / 2 for x in range(len(scenarios))])
            axis.set_xticklabels([s.replace("_", "\n") for s in scenarios], fontsize=7, rotation=0)
            axis.grid(axis="y", alpha=0.25)
        axes[0].legend(fontsize=8)
        fig.suptitle(title)
        fig.tight_layout()
        path = out_dir / f"{name}.png"
        fig.savefig(path, dpi=150)
        plt.close(fig)
        written.append(path)
    return written


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Plot experiment results")
    parser.add_argument("csv", nargs="?", type=Path, help="summary.csv (default: newest experiment run)")
    args = parser.parse_args(argv)
    csv_path = args.csv or newest_csv()
    if csv_path is None or not csv_path.is_file():
        print("No experiment CSV found - run experiments/run_experiments.py first", file=sys.stderr)
        return 2
    rows = load(csv_path)
    for path in draw(rows, csv_path.parent):
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
