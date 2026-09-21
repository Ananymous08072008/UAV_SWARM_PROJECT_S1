# Running the simulation

Three ways to run this project.

- **Mission studio** — the multi-user web app. Design a mission in the browser,
  run it, and let several people each drive their own. Start here if you want
  people to *use* the simulation rather than watch one fixed demo.
- **Path A (local Python)** — one shared simulation from the command line. The
  only way to reach Mission Planner, and what the demo video is recorded with.
- **Path B (Docker)** — a clean, reproducible box for deployment.

Every command below has been run on Windows 11 with Python 3.12 and verified.

---

## Prerequisites

- **Python 3.10 or newer** (CI tests 3.10, 3.11 and 3.12)
- For Path B only: **Docker Desktop**, which on Windows Home also needs WSL2

---

## Mission studio (multi-user web app)

```bash
python -m dashboard.server
```

Open <http://127.0.0.1:8000>. Unlike `main.py --dashboard`, this starts no
simulation of its own — every visitor builds and runs their own.

**Design a mission.** Drag the UAV count (1–40), duration and speed. Pick
adaptive or baseline. Click the map to drop Points of Interest, set each one's
priority, click a PoI to remove it. Leave the map empty to use the standard six.
Then press **Launch mission**.

**Drive it.** Pause, resume, restart, and change speed while it runs; inject
faults with the operator buttons. The charts plot connectivity, route PDR,
latency and imagery delivery over time, so a relay failure and the recovery that
follows are visible as they happen rather than only in the CSV afterwards.

**Share it.** The URL carries the session (`/?session=<id>`), so sending someone
the link puts them on the same mission. Anyone can also press **watch** on a run
listed under "Running on this server".

Every session is a separate world in its own thread. Pausing yours does not
touch anybody else's, and two people can run different scenarios at once.

Useful flags:

```bash
python -m dashboard.server --host 0.0.0.0 --port 8000   # reachable on the LAN
python -m dashboard.server --max-sessions 4             # bound concurrent worlds
python -m dashboard.server --idle-timeout 600           # reap unwatched runs sooner
```

Limits exist because the server is shared: at most 40 UAVs, 25 PoIs and one hour
of mission time per session, and `--max-sessions` (default 8) simultaneous
worlds. Idle sessions are reaped after 30 minutes.

> The studio has **no authentication** — anyone who can reach it can start
> simulations and inject faults. Fine on your machine or a trusted LAN; put a
> reverse proxy with a login in front of it before exposing it to the internet.

Mission Planner is not available in studio mode; use Path A for MAVLink.

---

## Path A — Run locally with Python

### 1. One-time setup

```bash
python -m venv .venv
.venv\Scripts\activate            # Windows
# source .venv/bin/activate       # Linux / macOS
pip install -r requirements.txt
```

### 2. Pick how you want to run it

**Headless — fastest, prints a full metrics summary and writes results to disk:**

```bash
python main.py
```

Takes about 7 seconds for a 512-second mission (roughly 78x real time).

**With the live dashboard — open <http://127.0.0.1:8000> in a browser:**

```bash
python main.py --dashboard --realtime --speed 4
```

**The full demo — dashboard + MAVLink + real time, opens the browser for you.**
This is what to record for the video:

```bash
python main.py --demo
```

**Compare against the non-adaptive baseline:**

```bash
python main.py --mode baseline
```

**Run one focused scenario** instead of the combined `full_demo`:

```bash
python main.py --scenario scenarios/network_degradation.yaml
```

Available: `normal.yaml`, `network_degradation.yaml`, `obstacle.yaml`,
`uav_failure_rth.yaml`, `new_priority_poi.yaml`, `poi_completion.yaml`.

**Batch experiments and figures:**

```bash
python experiments/run_experiments.py --seeds 3
python experiments/plot_results.py
```

### 3. Confirm it worked

A healthy headless run ends with a summary containing:

```
pois_completed               7
completion_rate              1.0
incidents_recovered          3
incidents_unrecovered        0
data_delivery_ratio          1.0
separation_violations        0
```

`incidents_unrecovered 0` and `completion_rate 1.0` are the two numbers that
say the swarm did its job.

To stop a dashboard or demo run, press **Ctrl+C**. The dashboard keeps serving
the final state after the mission ends, so Ctrl+C is how you exit.

---

## Path B — Run in Docker

Gives you a reproducible container. **Not usable with Mission Planner** — MAVLink
is UDP pushed to a target, which does not survive the container boundary. Use
Path A for Mission Planner.

### 1. Build

```bash
docker build -t uav-swarm .
```

First build takes a few minutes, most of it matplotlib. Result is a 619 MB image.

### 2. Start

```bash
docker compose up -d
```

Then open <http://127.0.0.1:8000>.

### 3. Check it is healthy

```bash
docker compose ps           # expect: Up (healthy)
docker compose logs -f      # expect: Dashboard: http://0.0.0.0:8000
```

### 4. Stop

```bash
docker compose down
```

**Do not skip this.** The service is set to `restart: unless-stopped`, so if you
leave it running it will come back every time Docker Desktop starts.

### Notes on the container

- The port is deliberately published on `127.0.0.1:8000`, not `0.0.0.0`, so the
  container is not reachable from your network by default. Start it with
  `BIND_ADDR=0.0.0.0 docker compose up -d` to share it — see the LAN section and
  its security note below.
- Run output is written to `./results` on your machine through a volume, but the
  default command uses `--no-results` and writes nothing. To keep a run, edit the
  `command:` block in `docker-compose.yml`.
- One container is one simulation with one shared world. Do not run replicas —
  a second container is a second, unrelated mission.

---

## Useful flags

| Flag | What it does |
|---|---|
| `--mode adaptive\|baseline` | `baseline` disables the research contributions for comparison |
| `--duration N` | Override the scenario length in seconds |
| `--seed N` | Override the random seed (default 42) for a repeatable run |
| `--realtime` | Pace the simulation against the wall clock |
| `--speed N` | Real-time multiplier, used with `--realtime` |
| `--dashboard` | Serve the live dashboard |
| `--host` / `--port` | Where the dashboard listens (default `127.0.0.1:8000`; use `--host 0.0.0.0` to accept connections from other machines) |
| `--mavlink` | Publish MAVLink telemetry for Mission Planner |
| `--mavlink-target host:port` | Send telemetry somewhere other than `127.0.0.1:14550` |
| `--demo` | Shortcut for `--dashboard --mavlink --realtime --open` |
| `--keep-running` | Play the full duration even after the mission completes |
| `--quiet` | Print only the final summary |
| `--all-events` | Print every event, not just the interesting ones |
| `--no-results` | Write nothing to disk |

---

## Mission Planner

1. Start the simulation with telemetry: `python main.py --demo`
2. In Mission Planner pick **UDP** in the connection dropdown, port **14550**, Connect.
3. Each virtual UAV appears as its own MAVLink system id (1..12). Switch between
   them in the vehicle selector.

To check telemetry without Mission Planner:

```bash
python -m telemetry.mavlink_monitor
```

This prints one row per vehicle. A healthy run shows 12 vehicles, `GUIDED` mode,
armed, with roles, battery and link quality updating. Mission Planner and this
monitor cannot both use port 14550 — close one first.

Full guide: [mission_planner.md](mission_planner.md).

---

## Letting other machines reach your dashboard (LAN)

The simulation keeps running on your computer; other people just open it in a
browser. Three things have to line up: the bind address, the firewall, and the
address you hand out.

### 1. Bind to every interface, not just loopback

The default `127.0.0.1` accepts connections from your machine *only* — that is
what makes a dashboard look unreachable from everywhere else. Pass `0.0.0.0`:

```bash
# one shared world, everyone watches the same UAVs
python main.py --dashboard --host 0.0.0.0 --port 8000 --realtime --keep-running

# mission studio: each visitor builds and runs their own world
python -m dashboard.server --host 0.0.0.0 --port 8000
```

Both print the link to share. When bound to `0.0.0.0` they substitute this
machine's LAN address, because `http://0.0.0.0:8000` resolves nowhere:

```
Mission studio: http://192.168.1.24:8000
```

In Docker, override the published address for the run — the compose file stays
bound to loopback by default:

```bash
BIND_ADDR=0.0.0.0 docker compose up --build -d
```

### 2. Open the firewall, once

This is the step that usually looks like "my code doesn't work": the server is
up and listening, and Windows silently drops the inbound connections. In an
**Administrator** PowerShell:

```powershell
New-NetFirewallRule -DisplayName "UAV Swarm Dashboard" -Direction Inbound -LocalPort 8000 -Protocol TCP -Action Allow
```

To undo it later: `Remove-NetFirewallRule -DisplayName "UAV Swarm Dashboard"`.

Rule the **port**, not the program. If you once clicked "Allow access" on a
Windows popup, that rule is bound to one interpreter's exact path — and a
virtualenv has its own `python.exe` at `.venv\Scripts\python.exe`, which is a
different file from the system `python.exe` the popup allowed. Running inside
the venv is then blocked even though "python is allowed". A port rule covers
every interpreter.

### 3. Hand out the address

Use the link the server printed, or find it with `ipconfig` (the IPv4 address of
your active adapter). Viewers browse to `http://<that-ip>:8000`.

Both machines must be on the same network. Check what Windows thinks that
network is:

```powershell
Get-NetConnectionProfile | Select-Object Name, NetworkCategory
```

A `NetworkCategory` of **Public** is the safe default Windows picks for Wi-Fi it
does not recognise, and it makes the firewall stricter. On your *own* network you
can set it to Private (`Set-NetConnectionProfile -Name "<name>" -NetworkCategory
Private` as Administrator). Do not do this on shared building, campus or café
Wi-Fi — it loosens the firewall toward everyone else on that network.

Shared Wi-Fi has a second problem the firewall rule cannot solve: many access
points enable **client isolation**, which blocks machine-to-machine traffic at
the router. The server is fine, the firewall is fine, and the connection still
never arrives. Test it with a plain `ping <host-ip>` from the other machine — if
ping fails too, the network is isolating clients and no amount of configuration
on your side will help. Use a phone hotspot, an ethernet switch, or a tunnel
such as `ngrok http 8000` instead (put authentication in front of it first —
see the security note below).

> **Security.** `POST /api/inject` has no authentication and no rate limiting.
> Anyone who can reach the port can fail relays, drop obstacles and drain
> batteries. The action list is whitelisted and all parameters are validated, so
> there is no arbitrary code path — but it is trivial to disrupt a live demo. On
> a trusted LAN this is usually fine. Do not expose it to the internet without
> putting a reverse proxy with authentication in front of it.

---

## Troubleshooting

**`ModuleNotFoundError`** — the virtual environment is not active. Re-run
`.venv\Scripts\activate`.

**It works on my machine but no other computer can open it** — work through the
three steps in the LAN section above, in order. Quick check from the *other*
machine: `curl http://<host-ip>:8000/api/state`. "Connection refused" means the
server is still bound to `127.0.0.1` (step 1); a hang or timeout means the
firewall is dropping it (step 2); a wrong-looking page means you have the wrong
address (step 3).

**Dashboard shows nothing / no live updates** — the page needs the `/ws`
WebSocket. Behind a reverse proxy you must forward the `Upgrade` and
`Connection` headers or the stream silently dies.

**The map is blank but the rest of the page works** — Leaflet and the map tiles
load from a CDN, so the *browser* needs internet even when the simulation is
local.

**`uvicorn dashboard.api:app` fails** — it is supposed to. There is no
module-level ASGI app; the FastAPI object is built inside `create_app(hub)` and
needs a live simulation feeding it. Always start through `main.py`.

**An operator action does nothing** — check the event log for `TRIGGER_REJECTED`.
The `uav_id` parameter takes a **numeric id** (`2`), a registered selector
(`critical_relay`), or a role (`role:RELAY`). The display name `UAV-02` is not
valid. Note that `critical_relay` can legitimately match nothing while the relay
chain is being rebuilt.

**Docker: `failed to connect to the docker API`** — the daemon is not running.
On Windows Home, Docker Desktop needs WSL2: run `wsl --install` in an
Administrator PowerShell, reboot, then launch Docker Desktop once from the Start
menu and accept the licence prompt.

**Docker: `docker: command not found`** — Docker Desktop may have installed
per-user. Add this to your PATH:
`C:\Users\<you>\AppData\Local\Programs\DockerDesktop\resources\bin`

**Short runs report zero flight metrics** — below roughly 150 simulated seconds,
`flight_time_total_s` and `distance_total_m` read `0.0` even though UAVs are
airborne. Use `--duration 300` or more for meaningful efficiency numbers. This is
a reporting quirk, not a simulation fault, and it is identical locally and in
Docker.
