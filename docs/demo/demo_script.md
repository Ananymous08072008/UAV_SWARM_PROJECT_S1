# Demonstration video script (~6 minutes)

Setup before recording:

```bash
python main.py --demo --speed 8 --seed N     # dashboard + MAVLink + real time at 8x; N = your chosen seed
```

At 8x the 45-minute mission (usually finished after 34-40 min) takes 4-5 minutes to record.

**Pin a seed for a recording.** Every run draws a new seed, which changes where the 10 PoIs
are, when each one appears, and when each event fires - so without `--seed` the video will
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
| 0:00 | Dashboard, the fleet on its pads at the operational center (sized to this mission), the 1000 x 1000 m operational area 75 m away, no PoIs yet | "45-minute mission, 5 m/s UAVs with a 100 m radio and 20-minute batteries. Ten PoIs will appear at random places and random times; the swarm knows nothing about one until it appears." |
| 0:20 | The first PoI appears; a surveyor and its relay chain launch together; links turn green | "Every link is 100 m at most, the ground station included, so anything past the first ~80 m reports through a relay chain. The allocator only starts surveys it can keep connected; the rest of the fleet becomes the backbone." |
| 0:50 | Mission Planner with several vehicles; switch vehicle | "The same virtual UAVs are live in Mission Planner - one MAVLink system id each, roles arriving as status text." |
| 1:10 | A far PoI waiting while nearer ones are surveyed, then flown with a long multi-hop chain | "The farthest open PoI waits until the others are done, so the whole fleet is free to build its chain - up to a dozen hops - instead of sending one UAV out alone. Imagery comes back live, within 10 s." |
| 1:40 | **t=300-1500 s** relay radio degrades (or press *Degrade critical relay*) | "The relay's radio drops to 15 %. Nothing tells the swarm - it compares measured link quality with what the geometry predicts." |
| 2:00 | FAULT_DETECTED in the log, relay released, replacement flies out, links recover | "Detected in about 3 seconds, the relay role is re-assigned and the chain is rebuilt - the incident panel shows detection and recovery time." |
| 2:30 | **t=300-1500 s** debris appears across the backbone | "An obstacle drops 40 dB on the longest hop. The planner detours the chain around it and UAVs inside the footprint climb clear, then carry on to where they were going." |
| 3:00 | **t=300-1700 s** a PoI is completed early | "A PoI under survey is declared complete early; the UAV and the relays that served it are released for other work." |
| 3:20 | **t=300-1500 s** a relay is lost | "A UAV fails. After the heartbeat timeout the swarm re-plans; the dependent UAVs are back online as soon as a replacement reaches the gap." |
| 3:50 | **t=300-1500 s** a relay's battery drops to 25 % | "Limited flight time. If the relay can afford to wait, it asks for a replacement and keeps relaying until it is on station; if it is too far out, it heads straight home and the relay planner fills the gap." |
| 4:10 | UAVs finishing tasks return to hover over their pads; one lands at 20 % | "No UAV goes home early: until its battery reaches 20 % it keeps taking roles, and with nothing to do it waits airborne over its pad. Only a UAV far out leaves earlier, when the trip home needs more than 20 %." |
| 4:40 | A high-priority PoI appears while lower-priority ones are being flown | "Priority comes from geography - a PoI in the middle of a cluster is worth most. A new high-priority one is taken at once, pre-empting a lower-priority survey if nobody is free." |
| 5:10 | Metrics panel, mission completes, UAVs land | "All ten PoIs surveyed, everyone landed inside the 45 minutes - and no two UAVs ever came within 20 m, which here would be a collision." (`collisions` and `avoidance_manoeuvres` are in the safety metrics.) |
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
