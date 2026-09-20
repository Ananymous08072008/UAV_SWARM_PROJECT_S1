"""
experiments/run_experiments.py
Runs every scenario in both modes over several seeds and writes one flat CSV.

    python experiments/run_experiments.py                       # all scenarios, 3 seeds, both modes
    python experiments/run_experiments.py --seeds 5 --jobs 4
    python experiments/run_experiments.py --scenarios scenarios/obstacle.yaml --modes adaptive

Output: results/experiments/<timestamp>/summary.csv  (+ a printed comparison table)
Feed that CSV to experiments/plot_results.py for the report figures.
"""

from __future__ import annotations

import argparse
import csv
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from statistics import fmean
from typing import Any, Iterable, Optional

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from core.config import Parameters, ScenarioConfig   # noqa: E402
from simulation.runner import Simulation             # noqa: E402

KEY_METRICS = [
    ("mission", "completion_rate"),
    ("mission", "mission_complete_s"),
    ("mission", "mean_completion_time_s"),
    ("mission", "high_priority_response_s"),
    ("communication", "network_availability"),
    ("communication", "mean_route_pdr"),
    ("communication", "mean_latency_ms"),
    ("communication", "comm_downtime_s"),
    ("communication", "data_live_ratio"),
    ("communication", "data_mean_delay_s"),
    ("resilience", "incidents"),
    ("resilience", "incidents_recovered"),
    ("resilience", "mean_detection_time_s"),
    ("resilience", "mean_recovery_time_s"),
    ("resilience", "reconnected_share"),
    ("resilience", "mean_reconnect_time_s"),
    ("resilience", "relay_changes"),
    ("safety", "separation_violations"),
    ("safety", "uavs_lost"),
    ("efficiency", "distance_total_m"),
    ("efficiency", "energy_consumed_pct"),
    ("efficiency", "relay_utilisation"),
]


def flatten(summary: dict[str, Any]) -> dict[str, Any]:
    row = {"scenario": summary["run"]["scenario"], "mode": summary["run"]["mode"],
           "seed": summary["run"]["seed"], "sim_duration_s": summary["run"]["duration_s"]}
    for group, key in KEY_METRICS:
        row[f"{group}.{key}"] = summary[group][key]
    return row


def run_one(job: tuple[str, str, int, str]) -> dict[str, Any]:
    parameters_path, scenario_path, seed, mode = job
    params = Parameters.load(parameters_path)
    scenario = replace(ScenarioConfig.load(scenario_path), seed=seed)
    sim = Simulation(params, scenario, mode=mode, results_dir=None)
    sim.run()
    return flatten(sim.finish())


def build_jobs(parameters: Path, scenarios: Iterable[Path], seeds: Iterable[int],
               modes: Iterable[str]) -> list[tuple[str, str, int, str]]:
    return [(str(parameters), str(scenario), seed, mode)
            for scenario in scenarios for seed in seeds for mode in modes]


def comparison_table(rows: list[dict[str, Any]], modes: list[str]) -> str:
    scenarios = sorted({row["scenario"] for row in rows})
    lines = []
    for group, key in KEY_METRICS:
        metric = f"{group}.{key}"
        lines.append(f"\n{metric}")
        lines.append(f"  {'scenario':26} " + "".join(f"{mode:>14}" for mode in modes))
        for scenario in scenarios:
            cells = []
            for mode in modes:
                values = [r[metric] for r in rows
                          if r["scenario"] == scenario and r["mode"] == mode and r[metric] is not None]
                cells.append(f"{fmean(values):14.3f}" if values else f"{'-':>14}")
            lines.append(f"  {scenario:26} " + "".join(cells))
    return "\n".join(lines)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Batch experiments over scenarios, modes and seeds")
    parser.add_argument("--params", type=Path, default=PROJECT_ROOT / "config" / "parameters.yaml")
    parser.add_argument("--scenarios", type=Path, nargs="*",
                        default=sorted((PROJECT_ROOT / "scenarios").glob("*.yaml")))
    parser.add_argument("--modes", nargs="*", default=["adaptive", "baseline"])
    parser.add_argument("--seeds", type=int, default=3, help="number of seeds (1, 2, 3, ...)")
    parser.add_argument("--jobs", type=int, default=1, help="parallel worker processes")
    parser.add_argument("--out", type=Path, default=PROJECT_ROOT / "results" / "experiments")
    args = parser.parse_args(argv)

    seeds = list(range(1, args.seeds + 1))
    jobs = build_jobs(args.params, args.scenarios, seeds, args.modes)
    print(f"Running {len(jobs)} simulations ({len(args.scenarios)} scenarios x {len(args.modes)} modes "
          f"x {len(seeds)} seeds) ...")

    if args.jobs > 1:
        with ProcessPoolExecutor(max_workers=args.jobs) as pool:
            rows = list(pool.map(run_one, jobs))
    else:
        rows = []
        for i, job in enumerate(jobs, start=1):
            rows.append(run_one(job))
            print(f"  [{i}/{len(jobs)}] {Path(job[1]).stem} seed={job[2]} mode={job[3]} "
                  f"-> completion {rows[-1]['mission.completion_rate']}")

    run_dir = args.out / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    csv_path = run_dir / "summary.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print(comparison_table(rows, list(args.modes)))
    print(f"\nWrote {csv_path}")
    print(f"Next: python experiments/plot_results.py {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
