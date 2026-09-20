# Connecting Mission Planner (Stage 11)

The simulation publishes every virtual UAV over MAVLink/UDP. Mission Planner is only the
display: all swarm decisions happen in Python.

## 1. Check the telemetry first (no Mission Planner needed)

```bash
python main.py --mavlink --realtime --speed 4           # terminal 1
python -m telemetry.mavlink_monitor                      # terminal 2 (listens on UDP 14550)
```

After a few seconds the monitor prints one row per UAV: flight mode, armed state, role, position,
altitude, battery, link quality to the GCS, hop count, position rate (~5 Hz) and the latest swarm
decision. If this works, Mission Planner will work. Close the monitor before step 2 - only one
program can listen on port 14550.

## 2. Connect Mission Planner

1. Start the simulation with telemetry: `python main.py --mavlink --realtime` (or `--demo`, which
   also opens the dashboard).
2. In Mission Planner, top right: choose **UDP** in the connection-type dropdown, click
   **CONNECT**, enter port **14550**, press OK.
3. The parameter download finishes almost immediately (the gateway answers with one parameter).
4. Use the vehicle selector (the drop-down next to CONNECT, or `Ctrl+X`) to switch between system
   ids 1..N - one per UAV.
5. Flight Data -> Messages shows the swarm's decisions (role changes, relay assignments, faults,
   recoveries) as status text.

What Mission Planner shows for each UAV:

| Mission Planner item | Comes from |
|---|---|
| position, altitude, heading, speed | GLOBAL_POSITION_INT, VFR_HUD, ATTITUDE (5 Hz / 2 Hz) |
| flight mode | GUIDED in transit, LOITER on station, RTL returning, LAND descending, STABILIZE on the pad |
| armed / disarmed | armed while airborne |
| battery % and voltage | SYS_STATUS (4S pack voltage derived from %) |
| GPS status | GPS_RAW_INT (3D fix, 12 satellites) |
| home icon | HOME_POSITION of the UAV's own pad |
| Messages tab | STATUSTEXT for swarm decisions |
| Quick tab (optional) | NAMED_VALUE_FLOAT `ROLE`, `LINKQ`, `HOPS` |

## 3. Troubleshooting

| Symptom | Fix |
|---|---|
| Monitor says "Cannot listen on UDP 14550" | Mission Planner or SITL already uses the port. Close it, or run `python main.py --mavlink --mavlink-target 127.0.0.1:14551` and `python -m telemetry.mavlink_monitor --port 14551`. |
| Mission Planner connects but shows nothing | Check the simulation is running with `--mavlink`; allow Python through the Windows firewall for private networks. |
| Only one vehicle visible | Open the vehicle selector - every UAV is a separate system id on the same link. |
| Vehicles do not move | Normal right after start (UAVs climb to their flight level first), or all tasks are finished / cannot finish before the deadline. Check the dashboard or the console events. |
| Mission Planner on another PC | `python main.py --mavlink --mavlink-target <that-PC-IP>:14550`. |
| Map in the wrong place | Mission Planner and the dashboard both use `geo_origin` in `config/parameters.yaml` (default: ArduPilot SITL home, Canberra). |

## 4. Recording tip

For the demonstration video run `python main.py --demo --speed 4`: the dashboard opens in the
browser, Mission Planner shows the same vehicles, and positions stay at 5 Hz even at 4x speed.
