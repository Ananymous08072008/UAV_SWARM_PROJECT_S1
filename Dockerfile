# Resilient UAV Swarm - dashboard demo image.
#
# This image runs the SIMULATION, not a bare web server. dashboard/api.py has no
# module-level ASGI app: create_app(hub) needs a live LiveHub that a running
# simulation feeds, and DashboardServer runs uvicorn in a daemon thread while the
# simulation keeps the main thread. So `uvicorn dashboard.api:app` does not work -
# the entry point is always main.py.
#
# One container = one simulation = one shared world. Every viewer sees the same
# UAVs, and a second replica would be a second unrelated mission. Do not scale
# this horizontally.
#
#   docker build -t uav-swarm .
#   docker run --rm -p 8000:8000 uav-swarm

FROM python:3.12-slim

# MPLBACKEND=Agg: matplotlib (experiments/plot_results.py) has no display here.
# PYTHONUNBUFFERED: stream simulation output to `docker logs` instead of buffering.
ENV PYTHONUNBUFFERED=1 \
    MPLBACKEND=Agg

WORKDIR /app

# Dependencies first so edits to the simulation code reuse this cached layer.
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

# Run unprivileged. results/ must stay writable - the simulation writes its run
# directory, metrics and runs.sqlite there.
RUN useradd --create-home --uid 1000 app \
    && mkdir -p results \
    && chown -R app:app /app
USER app

EXPOSE 8000

# --host 0.0.0.0 is mandatory in a container: the 127.0.0.1 default in
# config/parameters.yaml is unreachable from outside the container.
# --keep-running plays the full scenario duration even after the mission completes.
# Override at run time, e.g.  docker run uav-swarm python main.py --help
CMD ["python", "main.py", "--dashboard", "--host", "0.0.0.0", "--port", "8000", \
     "--realtime", "--keep-running", "--duration", "3600", "--quiet", "--no-results"]

# Uses urllib rather than curl, which python:3.12-slim does not ship.
# Probes /api/actions, not /api/state: create_app only registers /api/state when a
# LiveHub is present, so it 404s under `python -m dashboard.server` (studio mode).
# Reads PORT because hosting platforms assign one rather than using 8000.
HEALTHCHECK --interval=30s --timeout=5s --start-period=20s --retries=3 \
    CMD python -c "import os, urllib.request; urllib.request.urlopen('http://127.0.0.1:' + os.environ.get('PORT', '8000') + '/api/actions', timeout=4).read(1)" || exit 1
