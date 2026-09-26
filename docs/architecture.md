# Software architecture

## 1. Layers and data flow

```text
config/*.yaml + scenarios/*.yaml
            |
            v
+------------------ SIMULATION LOOP (simulation/runner.py, every dt = 0.1 s) ------------------+
|                                                                                              |
|  1 simulation/   radio channel, obstacles, battery estimates, imagery data                   |
|          | measurements (link PDR, latency, obstruction)                                     |
|          v                                                                                   |
|  2 core/WorldState   <- single source of truth: UAVs, PoIs, links, time, events              |
|          | read-only view                                                                    |
|          v                                                                                   |
|  3 swarm/mission_manager   (decisions, every 1 s)                                            |
|      reconfiguration -> safety/deadline -> energy -> allocation -> priority ->               |
|      relay plan -> spares (backup, data ferry)                                               |
|          | commands: assign_poi / set_role / goto / return_home / complete_poi               |
|          v                                                                                   |
|  4 core/World   validates, executes (motion, battery, surveys, charging), publishes events    |
+----------------------------------------------------------------------------------------------+
            | snapshots + events (immutable)
     +------+-----------------+--------------------+------------------+
     v                        v                    v                  v
 telemetry/               dashboard/          swarm_logging/     experiments/
 MAVLink over UDP         FastAPI + WS        CSV/JSONL/SQLite   batch runs, figures
     v                        v
 Mission Planner          Browser (map, graph, log, metrics, controls)
```

Rules that keep the system understandable:

* **One authoritative state.** Only `core/World` mutates `WorldState`; everything outside the
  loop reads immutable `WorldSnapshot` objects.
* **Decisions are separate from execution.** `swarm/` never moves a UAV; it calls the World
  command API, which validates the command and publishes an event.
* **Two networks, never mixed.** The research network (`simulation/communication.py`) carries
  imagery to the GCS; MAVLink (`telemetry/`) only displays the vehicles in Mission Planner.
* **Extensible by registration.** Scenario actions, UAV selectors (`critical_relay`) and position
  selectors (`backbone_midpoint`) are registered by the layer that owns them, so `core/` never
  imports upwards.

## 2. Modules

| Module | Responsibility |
|---|---|
| `core/config.py` | typed, validated YAML loading; unknown keys are errors; later-stage sections stay raw |
| `core/uav.py` | UAV state and execution: acceleration-limited motion, take-off climb-out, battery drain |
| `core/poi.py` | PoI lifecycle `PENDING -> ASSIGNED -> IN_PROGRESS -> COMPLETED` and the PoI manager |
| `core/events.py` | event bus (history, filters, counters) and the scenario trigger schedule |
| `core/state.py` | `WorldState` plus JSON-ready `WorldSnapshot` (local ENU **and** lat/lon) |
| `core/world.py` | step loop, command API, RTH/charging, triggers, event publishing |
| `simulation/communication.py` | link model: path loss, fading, obstacles, radio health -> PDR, latency, up/down |
| `simulation/obstacles.py` | prism obstacles: exact segment-through-polygon test, attenuation, scenario hooks |
| `simulation/battery.py` | flight/hover/return cost estimates shared with the UAV drain formula |
| `simulation/data_model.py` | imagery capture, multi-hop offload, store-and-forward, pad download, delay stats |
| `simulation/environment.py` | one object holding the physics models; measured before decisions |
| `simulation/runner.py` | wires the layers, the tick order, real-time pacing, results writing |
| `swarm/network_manager.py` | graph view: neighbours, components, articulation points (critical relays) |
| `swarm/route_manager.py` | Dijkstra over ETX, route hysteresis (loop-free), writes each UAV's route |
| `swarm/task_allocator.py` | sequential auction with energy, deadline and communication budget |
| `swarm/relay_selector.py` | relay placement (shortest-tree growth + detours) and relay UAV choice |
| `swarm/role_manager.py` | allowed role transitions, flight levels, obstacle clearance |
| `swarm/energy_manager.py` | RTH thresholds, relay hand-over, idle recharge |
| `swarm/priority_manager.py` | ageing priorities and pre-emption for emerging regions |
| `swarm/safety_manager.py` | collisions, collision avoidance, geofence, obstacle avoidance, mission deadline recall |
| `swarm/reconfiguration.py` | fault detection, re-plan requests, incident and recovery measurement |
| `swarm/mission_manager.py` | fixed decision order; `adaptive` vs `baseline` mode |
| `telemetry/*` | MAVLink gateway (one socket, one system id per UAV), message builders, unit conversion, wall-clock rates, monitor tool |
| `dashboard/*` | FastAPI + WebSocket server and the browser front end |
| `swarm_logging/*` | event files, metrics collection, SQLite results database |

## 3. Algorithms

### 3.1 Radio channel (per link, every 0.5 s)

```
PL(d)  = PL0 + 10 n log10(d)                     PL0 = 40 dB, n = 2.6
Prx    = Ptx + Gtx + Grx - PL(d) - L_obstacle + N(0, sigma)
SNR    = Prx - noise_floor
PDR    = sigmoid((SNR - snr_mid) / snr_slope) * radio_health(a) * radio_health(b)
est    = EWMA(PDR)            link up if est >= 0.55, down if est < 0.45 (hysteresis)
latency = hop_latency + retx_latency * (1/PDR - 1)
```

With the shipped values a healthy UAV-UAV link has its maximum range at 100 m (85 % PDR, the
quality relays are planned for) and degrades beyond it (~73 % at 110 m, ~59 % at 120 m, dropping
out around 130 m), so relays are spaced 90 m apart. A UAV-GCS link reaches ~328 m thanks to the
high-gain ground antenna. Obstacles add 25-40 dB when the straight 3D
segment passes through their prism.

### 3.2 Routing

Edge cost `1/PDR` (expected transmissions, ETX) plus a penalty of 10 for links below 0.7 PDR;
Dijkstra from the GCS gives each UAV its next hop. Plain ETX prefers fewer hops even across a
marginal link (e.g. one 0.58 hop, 0.52 end to end) - with the penalty the router takes the longer
path of good links (0.84 end to end) and keeps weak links only as a last resort.
A UAV keeps its current next hop while that route costs no more than `1 + hysteresis` times the
best one **and** is strictly closer to the GCS, which prevents both flapping and loops.

### 3.3 Task allocation (sequential auction)

One PoI is set aside before any of this runs: the one farthest from the GCS, identified once at
mission start (adaptive mode only). It is held out of the auction until every other PoI is done, so
when its turn comes the whole fleet is free to build its relay chain - naturally the longest in the
mission - just for it: a multi-hop escort, not a lone data-ferry run.

For each remaining pending PoI in effective-priority order (base priority + ageing bonus):
feasible UAVs are IDLE/BACKUP with `battery >= fly + survey + return + reserve` and enough mission
time left (a slimmer, still-safe reserve is tried if no UAV clears the normal one, so a PoI is not
left waiting when one UAV could still just about do it). The winner has the lowest bid
`travel_time + w * battery_cost`. In adaptive mode the swarm also checks the **communication
budget**: surveyors + the relays needed to connect them must fit in the fleet, otherwise the PoI
waits for a relay path. Only once every PoI that fits the budget is tasked does a still-blocked PoI
get a last look: with UAVs left that have nothing else to do, and either it is high priority and has
stayed blocked for a while, or the mission deadline itself is close, it is surveyed disconnected and
its imagery ferried back (store-and-forward) - a deliberately rare fallback, not a timeout, and the
"deadline close" half of that or is what keeps a low-priority PoI a tight relay budget never reaches
from being abandoned for the whole mission.

### 3.4 Relay placement

Terminals are the survey waypoints, plus the next few queued PoIs (lookahead) so the backbone can
extend toward them ahead of time when that piggybacks on relays already in place. The tree grows
from the GCS, each step attaching the terminal
that needs the fewest relays (shortest-tree/Prim style), so chains share a backbone. Relays are
spaced so every planned hop predicts `>= min_planned_pdr` including obstacle attenuation; if a
straight chain is blocked, dog-leg detours (25/45/65 degrees) are tried. Chains are filled in
dependency order and only when the whole chain can be staffed - a partial chain connects nothing.
A UAV already relaying near a planned point keeps it (`stickiness_s`).

Re-planning follows "repair only what is broken": a new plan is made when the survey targets or
the eligible UAVs change, when a fault forces it, or periodically while a surveyor on station is
actually cut off. Routing alone often works around a change (for example a new obstacle); moving
relays that still carry traffic would break links while they fly.

### 3.5 Fault detection and recovery

| Fault | Detection | Reaction |
|---|---|---|
| Radio degradation | measured/predicted PDR `< 0.6` on `>= 2` links of one UAV for 2 s | exclude that UAV from the relay role, re-plan |
| UAV loss | no heartbeat for `failure_timeout_s` | release its task and relay point, re-plan |
| Obstacle | mapped when it appears | relays move only if a surveyor loses its route (planner detours) |
| Surveyor disconnected | on station without a route for 2 s | re-plan |

Comparing measurement with prediction is what separates a node fault (all links bad) from geometry
or a mapped obstacle (one link bad). Every impactful fault opens an **incident** holding the UAVs
that lost their route; it is recovered when each of them has a route with `PDR >= recovery_pdr`
again. `detection time = detected - onset`, `recovery time = recovered - onset`. Per affected UAV
the swarm also records whether it got its route back **while still on task** and how fast
(`reconnected_share`, `mean_reconnect_time_s`); a UAV that simply flies home past the GCS does not
count as a recovery.

### 3.6 Energy and safety

* RTH when `battery <= return_cost + reserve` (never below the critical floor).
* A relay inside the hand-over margin keeps relaying until its replacement is on station
  (or `max_handover_wait_s`), then flies home - connectivity is not interrupted by recharging.
* A full battery lasts 1200 s at most (hovering; ~15 min flying at cruise speed).
* Two airborne UAVs closer than 20 m **collide and are both lost**. Nothing flies above 100 m.
* Flight levels every 20 m (20, 40, 60, 80, 100 m), so UAVs on different levels can never collide.
  Waypoints take their role's level - survey 40, relay 60, return home 80 - unless another UAV is
  stationed within 30 m there, and are raised over obstacles. Launch pads are 30 m apart.
* Predictive avoidance, every tick after the swarm's decisions: each plan is projected 12 s ahead
  (including a returning UAV's landing descent). For a predicted conflict, the moving UAV (or else
  the higher id) takes the least disruptive plan that stays clear of everyone's projected path -
  carry on, change level, hold position, or stop - and returns to its level once clear.
* Every UAV is recalled early enough to land before the mission deadline; tasks that cannot finish
  in time are never assigned.

## 4. Metrics (`swarm_logging/metrics.py`)

| Group | Metrics |
|---|---|
| Mission | PoI completion rate and time, allocation runtime, response time to new high-priority regions, task reallocations, pre-emptions |
| Communication | route PDR, latency, share of UAVs connected (network availability), full-connectivity fraction, downtime, disconnections, route changes, imagery delivered / live share / delay |
| Resilience | incidents, detection time, recovery time, affected UAVs reconnected (share, time), unrecovered incidents, relay changes, hand-overs, re-plans |
| Safety | minimum separation, collisions, avoidance manoeuvres, geofence/obstacle violations, UAVs lost, UAVs still airborne at the end |
| Efficiency | distance flown, flight time, energy consumed, battery left, relay utilisation, RTH count |

## 5. Verification

`python -m pytest -q` runs 99 tests: channel and obstacle geometry, graph and routing (including
loop freedom), allocation rules, relay geometry (every planned hop meets the quality target),
energy and hand-over, fault detection and recovery, safety and imagery delivery, MAVLink units and message decoding, dashboard
API, and every shipped scenario end to end (completion, safety, reproducibility).

## 6. Known limitations

* Kinematic UAV model (no wind or attitude dynamics); no 4D path planning - separation comes from
  flight levels plus short-horizon predictive avoidance, not from planning whole trajectories.
* Flat-earth coordinate conversion (valid within a few km of the origin).
* Perfect knowledge of the swarm's own state: the decisions run centrally, as a GCS-side planner
  would; a decentralised version is future work.
* Obstacles are mapped the moment they appear (sensing them is out of scope).
