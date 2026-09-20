# Demonstration video script (~6 minutes)

Setup before recording:

```bash
python main.py --demo --speed 4        # dashboard + MAVLink + real time at 4x
```

Open Mission Planner, connect **UDP port 14550**, and place it next to the browser.
(Check first with `python -m telemetry.mavlink_monitor` if unsure - see docs/mission_planner.md.)
The full demo scenario (`config/scenario.yaml`) injects everything below on its own timeline, so
the recording only needs narration. Operator buttons are there if you prefer to trigger manually.

| Time | What is on screen | What to say |
|---|---|---|
| 0:00 | Dashboard, 12 UAVs on their pads, 6 PoIs in the disaster area | "GCS outside the area, PoIs 0.4-1.2 km inside it, far beyond a single radio hop." |
| 0:20 | UAVs launch; relay chains form; links turn green | "The allocator only starts surveys it can keep connected; the rest of the fleet becomes the relay backbone." |
| 0:50 | Mission Planner with several vehicles; switch vehicle | "The same virtual UAVs are live in Mission Planner - one MAVLink system id each, roles arriving as status text." |
| 1:20 | Network graph and UAV table; imagery-live metric | "Imagery is delivered to the GCS live over the multi-hop network, not after landing." |
| 1:40 | **t=80 s** relay radio degrades (or press *Degrade critical relay*) | "The relay's radio drops to 15 %. Nothing tells the swarm - it compares measured link quality with what the geometry predicts." |
| 2:00 | FAULT_DETECTED in the log, relay released, replacement flies out, links recover | "Detected in about 3 seconds, the relay role is re-assigned and the chain is rebuilt - the incident panel shows detection and recovery time." |
| 2:40 | **t=150 s** POI-5 completed early | "Scenario C: a PoI is declared complete early; the UAV and the relays that served it are released for other work." |
| 3:00 | **t=200 s** debris appears across the backbone | "Scenario B: an obstacle drops 40 dB on the longest hop. The planner detours the chain around it and UAVs inside the footprint climb clear." |
| 3:40 | **t=260 s** POI-URGENT appears | "A new priority-5 region: response time is measured from the moment it appears; if nobody is free, a low-priority survey is pre-empted." |
| 4:10 | **t=330 s** a relay is lost | "A UAV fails. After the heartbeat timeout the swarm re-plans; the dependent UAVs are back online within seconds." |
| 4:40 | **t=380 s** a relay's battery is drained | "Limited flight time: the relay asks for a replacement, keeps relaying until it is on station, then returns home and recharges - connectivity is never dropped for a battery swap." |
| 5:10 | Metrics panel, mission completes, UAVs land | "All PoIs surveyed, everyone landed inside the allotted time, no separation violations." |
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
