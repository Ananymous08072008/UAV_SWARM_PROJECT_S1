# Resilient UAV Swarm - simulation-first platform

[![tests](https://github.com/Ananymous08072008/UAV_SWARM_PROJECT_S1/actions/workflows/tests.yml/badge.svg)](https://github.com/Ananymous08072008/UAV_SWARM_PROJECT_S1/actions/workflows/tests.yml)
[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

A simulation-first resilient UAV swarm platform: Python generates virtual UAV states, a swarm
intelligence engine allocates survey and relay roles and reconfigures the communication network,
a MAVLink gateway exposes the virtual UAVs to Mission Planner, and a custom dashboard shows the
swarm's decisions live.

**Mission.** A Ground Control Station (GCS) sits outside an earthquake/landslide area. UAVs with
limited flight time survey Points of Interest (PoIs) inside it, collect imagery, and relay it back
to the GCS over a multi-hop aerial network. The swarm has to keep that link alive while radios
degrade, obstacles appear, UAVs fail or leave to recharge, and new high-priority regions emerge.

## Install

```bash
python -m venv .venv
.venv\Scripts\activate          # Windows;  source .venv/bin/activate on Linux/macOS
pip install -r requirements.txt
```

Python 3.10 or newer. Everything else (pymavlink, FastAPI, numpy) comes from `requirements.txt`.

## Run

```bash
python -m dashboard.server         # mission studio: build and run missions in the browser
python main.py                     # headless full-demo scenario, prints events + summary
python main.py --demo              # dashboard + MAVLink + real time (what to record for the video)
python main.py --dashboard --realtime --speed 4
python main.py --scenario scenarios/network_degradation.yaml --mode baseline
python -m pytest -q                # 200 tests
```

The default demo follows the mission constraints: a 45-minute mission over a 1000 x 1000 m area
75 m from the operational center, 5 m/s UAVs with a 100 m radio range and 20-minute batteries
(home at 20 %), and 10 PoIs that each appear at a random place and a random time in the first 30
minutes. It is different every run: it draws a new seed, the PoIs, and a random time for each fault
within its window. The seed is printed first; `--seed N` replays that run exactly. The scenarios in
`scenarios/` keep fixed seeds for controlled comparisons.

Useful flags: `--mode adaptive|baseline`, `--duration`, `--seed`, `--quiet`, `--all-events`,
`--no-results`, `--keep-running`, `--port`, `--mavlink-target host:port`.

Both servers listen on `127.0.0.1` by default, which accepts connections from this machine only.
To let other computers on your network open the dashboard, add `--host 0.0.0.0` and allow the port
through the firewall once — the server then prints the LAN link to share:
[docs/running.md](docs/running.md#letting-other-machines-reach-your-dashboard-lan).

### Mission studio (multi-user web app)

`python -m dashboard.server` serves <http://127.0.0.1:8000> and starts no simulation of its own -
every visitor builds and runs their own. Click the map to place Points of Interest and set their
priorities, pick adaptive or baseline, then launch - the fleet is sized to the mission (one UAV per
PoI, the relays that keep them connected, a spare and a fault reserve) unless you fix the count. Pause, resume, restart
and change speed while it runs; live charts plot connectivity, route PDR, latency and imagery
delivery over time, so a relay failure and the recovery are visible as they happen. Sessions are
independent worlds in their own threads, and the URL (`/?session=<id>`) can be shared so several
people watch or drive the same mission. **Download data (Excel)** saves the run's mission metrics
and event log as an `.xlsx` workbook, which for a studio session is the only copy, since sessions
keep nothing on disk. Full guide: [docs/running.md](docs/running.md).

### Deploy

`render.yaml` publishes the mission studio to [Render](https://render.com) from the existing
`Dockerfile` - push to GitHub, create a Blueprint Instance, and the studio is live on a public URL.
`HOST`, `PORT`, `MAX_SESSIONS` and `IDLE_TIMEOUT` are read from the environment for hosts that
start the container themselves. Netlify, Vercel and GitHub Pages **cannot** run this: it needs
WebSockets, simulation threads that live for minutes, and worlds held in RAM between requests,
and serverless platforms provide none of the three. Any container host works instead - Render,
Hugging Face Spaces, Railway, Fly.io. Deployed studios have no authentication; see
[docs/running.md](docs/running.md#path-c--deploy-on-the-internet-render).

### Dashboard

`--dashboard` serves <http://127.0.0.1:8000>: live map (UAVs, PoIs, relay links, obstacles),
UAV table, communication graph, event log, metrics, and operator buttons that inject faults into
the running simulation (degrade a relay's radio, drop debris on the backbone, add an urgent PoI,
drain a battery, fail a UAV). **Download data (Excel)** in the header saves the same workbook the
studio offers, which is the way to keep a run started with `--no-results`. One shared simulation, driven
from the command line - use the mission studio when people need their own.

### Mission Planner

1. Start with telemetry: `python main.py --demo` (or add `--mavlink`).
2. In Mission Planner choose **UDP** in the connection dropdown, port **14550**, and press Connect.
3. Every virtual UAV appears as its own MAVLink system id (1..N) - switch between them in the
   vehicle selector. Roles and swarm decisions arrive as status messages.

Check the telemetry without Mission Planner: `python -m telemetry.mavlink_monitor` prints one row
per vehicle (mode, role, position, battery, link quality). Positions stay at 5 Hz at any simulation
speed. Full guide and troubleshooting: [docs/mission_planner.md](docs/mission_planner.md).

The default map origin is the ArduPilot SITL home (CMAC, Canberra), so Mission Planner's map and
the dashboard map line up. Change `geo_origin` in `config/parameters.yaml` for your own site.

### Experiments

```bash
python experiments/run_experiments.py --seeds 3          # every scenario x adaptive/baseline
python experiments/plot_results.py                       # figures next to the CSV
```

Results land in `results/`: one folder per run (`events.jsonl`, `events.csv`, `timeseries.csv`,
`summary.json`), a shared `runs.sqlite`, and `results/experiments/<stamp>/` for batches.
Every run is queryable with plain SQL, for example mean connectivity per mode:

```sql
SELECT r.mode, AVG(s.value) FROM samples s JOIN runs r USING (run_id)
WHERE s.metric = 'connectivity_ratio' GROUP BY r.mode;
```

## Layout

```
main.py                 entry point and CLI
config/                 parameters.yaml (how the system behaves), scenario.yaml (the full demo)
core/                   world model: config, UAV, PoI, events, WorldState, World (execution)
simulation/             physics: radio channel, obstacles, battery estimates, imagery data, runner
swarm/                  decisions: allocation, relays, routing, roles, energy, priority, safety,
                        reconfiguration, mission_manager
telemetry/              MAVLink gateway, message builders, coordinate conversion, rate scheduler
dashboard/              mission studio (sessions, builder, charts, run download) + FastAPI/WebSocket
swarm_logging/          event log files, metrics, SQLite database
scenarios/              one file per demonstration scenario
experiments/            batch runs and figures
tests/                  200 tests across all layers
docs/                   architecture, Mission Planner guide, proposal outline, demo script
```

## How it works

Each simulation step (`dt = 0.1 s`):

1. **Environment** re-measures the radio channel (every 0.5 s): log-distance path loss, shadow
   fading, obstacle attenuation, per-UAV radio health -> PDR, latency, link up/down with hysteresis.
2. **Swarm** (every 1 s) rebuilds the network graph, runs Dijkstra over ETX (`1/PDR`) to route every
   UAV to the GCS, then decides: fault detection and re-planning, deadline recall, energy/RTH,
   task auction, priority pre-emption, relay placement, spares and data ferrying.
3. **World** executes: acceleration-limited motion, battery drain, survey progress, landing and
   charging, scenario triggers - and publishes every change as an event.
4. **Outputs**: metrics sample, MAVLink telemetry, dashboard push, log files.

Two things stay strictly separate: the **swarm research network** (simulated links between UAVs and
the GCS) and the **MAVLink telemetry link** used only to display the vehicles in Mission Planner.

`--mode baseline` disables the research contributions (communication-aware allocation, fault
detection, relay re-planning, hand-over, pre-emption) so every metric can be compared against a
static swarm. See [docs/architecture.md](docs/architecture.md) for the algorithms and metrics.

## Contributing

Branch off `main`, keep the tests green, open a pull request. Setup, branch
naming, commit style and the review checklist are in
[CONTRIBUTING.md](CONTRIBUTING.md).

## License

MIT - see [LICENSE](LICENSE).
