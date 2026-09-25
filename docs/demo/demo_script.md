# Demonstration video script (~6 minutes)

Setup before recording:

```bash
python main.py --demo --speed 4 --seed N     # dashboard + MAVLink + real time at 4x; N = your chosen seed
```

**Pin a seed for a recording.** Every run draws a new seed, which changes how many PoIs
there are, where they are, and when each event fires - so without `--seed` the video will
not match a rehearsal. Do a quick headless preview (`python main.py --quiet --seed N`), pick
a seed whose run you like, and record with that same `--seed N`. Its event times are printed
in the first log line (`SIM_STARTED`) and fire in the same order every time.

Open Mission Planner, connect **UDP port 14550**, and place it next to the browser.
(Check first with `python -m telemetry.mavlink_monitor` if unsure - see docs/mission_planner.md.)
The full demo scenario (`config/scenario.yaml`) injects everything below on its own timeline, so
the recording only needs narration. Operator buttons are there if you prefer to trigger manually.

Simulation times below are the windows each event is drawn from; with a pinned seed, read
the exact times from `SIM_STARTED`. The events can come in a different order than listed.

| Time | What is on screen | What to say |
|---|---|---|
| 0:00 | Dashboard, the fleet on its pads (sized to this run's PoIs), 4-8 PoIs in the disaster area | "GCS outside the area, PoIs placed at random 0.3-1.2 km inside it, far beyond a single radio hop." |
| 0:20 | UAVs launch; relay chains form; links turn green | "The allocator only starts surveys it can keep connected; the rest of the fleet becomes the relay backbone." |
| 0:50 | Mission Planner with several vehicles; switch vehicle | "The same virtual UAVs are live in Mission Planner - one MAVLink system id each, roles arriving as status text." |
| 1:20 | Network graph and UAV table; imagery-live metric | "Imagery is delivered to the GCS live over the multi-hop network, not after landing." |
| 1:40 | **t=70-140 s** relay radio degrades (or press *Degrade critical relay*) | "The relay's radio drops to 15 %. Nothing tells the swarm - it compares measured link quality with what the geometry predicts." |
| 2:00 | FAULT_DETECTED in the log, relay released, replacement flies out, links recover | "Detected in about 3 seconds, the relay role is re-assigned and the chain is rebuilt - the incident panel shows detection and recovery time." |
| 2:40 | **t=90-190 s** a PoI is completed early | "Scenario C: a PoI under survey is declared complete early; the UAV and the relays that served it are released for other work." |
| 3:00 | **t=90-150 s** debris appears across the backbone | "Scenario B: an obstacle drops 40 dB on the longest hop. The planner detours the chain around it and UAVs inside the footprint climb clear, then carry on to where they were going." |
| 3:40 | **t=150-320 s** POI-URGENT appears at a random spot | "A new priority-5 region: response time is measured from the moment it appears; if nobody is free, a low-priority survey is pre-empted. If the first wave has already landed, the swarm launches again." |
| 4:10 | **t=100-150 s** a relay is lost | "A UAV fails. After the heartbeat timeout the swarm re-plans; the dependent UAVs are back online within seconds." |
| 4:40 | **t=110-150 s** a relay's battery is drained | "Limited flight time. If the relay can afford to wait, it asks for a replacement and keeps relaying until it is on station; if it is too far out, it heads straight home and the relay planner fills the gap." |
| 5:10 | Metrics panel, mission completes, UAVs land | "All PoIs surveyed, everyone landed inside the allotted time - and no two UAVs ever came within 20 m, which here would be a collision." (`collisions` and `avoidance_manoeuvres` are in the safety metrics.) |
| 5:30 | Terminal summary, then `results/` folder | "Every run writes the event log, the time series and a metrics summary, plus a SQLite database." |
| 5:45 | `python experiments/plot_results.py` figures | "Adaptive versus baseline over all scenarios and seeds: connectivity, live imagery share, detection and recovery time." |

## Backup manual demo (if the timeline is not wanted)

```bash
python main.py --dashboard --realtime --speed 4 --scenario scenarios/normal.yaml --keep-running
```

Then use the operator buttons in this order: *Degrade critical relay* -> *Restore radios* ->
*Debris on backbone* -> *Clear obstacles* -> *New urgent PoI* -> select a relay row -> *Drain
selected UAV* -> *Fail critical relay*.

## Recording tips

* 1920x1080, browser at 100 % zoom; the dashboard layout is designed for that width.
* Keep the event log visible - it is the evidence that the swarm decided, not the operator.
* Mention the baseline: `python main.py --scenario scenarios/network_degradation.yaml --mode baseline`
  shows the same fault with no detection and no recovery.
