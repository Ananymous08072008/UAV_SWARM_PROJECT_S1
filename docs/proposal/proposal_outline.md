# Stage 1 proposal outline (6-8 pages)

Judging criteria: feasibility, novelty, reproducibility, preliminary communication-aware autonomy.
Each section below says what to write and where the material comes from.

## 1. Mission understanding (0.5 page)
The disaster scenario, the GCS outside the area, and the six autonomy requirements from the
challenge: survey all PoIs, keep end-to-end communication, assign relays dynamically, reconfigure
on degradation / failure / recharge, prioritise emerging regions, finish safely within the allotted
time. State the operating envelope used in the simulation (area size, 12 UAVs, ~18 min endurance,
193 m UAV-UAV and 328 m UAV-GCS link range at 85 % PDR).

## 2. System architecture (1.5 pages)
Use the diagram in `docs/architecture.md` section 1 plus the module table. Emphasise the three
design rules: one authoritative world state, decisions separated from execution, and the research
network kept separate from the MAVLink telemetry link. Include the tick order and the interfaces
(World command API, event bus, snapshots).

## 3. Communication-aware autonomy (2 pages) - the core contribution
1. Link model and how link quality is *estimated* rather than assumed (`architecture.md` 3.1).
2. ETX routing with hysteresis and why it cannot loop (3.2).
3. Communication-aware task allocation: a survey is only started if the swarm can also staff the
   relays that keep it connected; otherwise it waits or is served in store-and-forward mode (3.3).
4. Relay placement as shortest-tree growth with quality-checked hops and obstacle detours (3.4).
5. Measurement-based fault diagnosis: measured vs predicted PDR separates a radio fault from
   geometry or a mapped obstacle; incidents record detection and recovery times (3.5).
6. Energy-aware relay hand-over: a relay only leaves once its replacement is on station (3.6).

Claim novelty carefully: the individual mechanisms exist in the literature; the contribution is
the closed loop - estimate, diagnose, re-plan, hand over - measured end to end on identical
scenarios against a static baseline.

## 4. Proof-of-concept simulation (1 page)
What runs today: `main.py`, the six scenarios, the dashboard, the MAVLink gateway with Mission
Planner, 97 tests. Include one dashboard screenshot and one Mission Planner screenshot.
Table: scenario -> what is injected -> what the swarm does.

## 5. Results (1.5 pages)
Run `python experiments/run_experiments.py --seeds 3` and paste the figures from
`experiments/plot_results.py`. Report, adaptive vs baseline:

* PoI completion rate and mission time
* share of UAVs connected to the GCS over time (network availability)
* imagery delivered live (within 10 s) and mean delivery delay
* fault detection time, share of affected UAVs reconnected and time to reconnect (the baseline
  detects nothing and reconnects far fewer UAVs)
* relay changes, hand-overs, distance and energy cost of the adaptive behaviour
* safety: minimum separation, violations, UAVs landed safely

State the trade-off honestly: the adaptive swarm flies further and uses more energy per surveyed
PoI, and can take longer, because it keeps the GCS informed while it works. Without faults (e.g.
the new-priority scenario) a static swarm can be marginally better connected, because the adaptive
one holds UAVs back as relays - the benefit shows under faults.

## 6. Reproducibility (0.5 page)
Fixed seeds, `parameters.yaml` / scenario files, per-run `summary.json` + `events.jsonl` +
`timeseries.csv` + `runs.sqlite`, the test suite, and the install/run commands from the README.
Point out that `--mode baseline` reruns the exact same scenario with the contributions disabled.

## 7. Plan to Stage 2 (0.5 page)
Decentralised decisions (per-UAV planners with consensus), ArduPilot SITL validation for a subset
of vehicles, learned link-quality prediction, trajectory-level deconfliction, hardware-in-the-loop.

## 8. Risks and limitations (0.5 page)
From `architecture.md` section 6: kinematic model, flat-earth conversion, central planner, mapped
obstacles. For each, state the mitigation or the Stage 2 plan.

---

### Figures to include
1. Architecture diagram (section 2).
2. Dashboard screenshot during a fault (map + network graph + event log).
3. Relay chain geometry sketch: GCS, relays, surveyors, hop ranges.
4. `fig_communication.png` and `fig_resilience.png` from the experiment run.
5. Mission Planner with several virtual UAVs connected.
