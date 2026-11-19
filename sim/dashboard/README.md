# Python dashboard: T800 planar base under NMPC

Dash + Plotly dashboard for `sim/t800/planar_base.py`. It runs one episode at a time in a background thread, and the page refreshes every 500 ms.

**Shows:** the scene from above (robot pelvis and path, persons with keep-out rings, objects, goal, commanded acceleration); charts for NMPC solve time, pelvis tilt, distance to the nearest person and object, speed with tracking error, and commanded acceleration; headline numbers, including the measured real-time factor.

## Run on a laptop or the sandbox

From the repository root:

```
git submodule update --init --depth 1 third_party/T800-PROJECT
python3 -m venv .venv-dash && . .venv-dash/bin/activate
pip install -r sim/dashboard/requirements.txt
python sim/dashboard/app.py --host 0.0.0.0 --port 8050
```

Open http://localhost:8050. Set a seed and a maximum time, then press **Start episode**. **Stop** ends the current run early.

Requirements: Python 3.10–3.12 (MuJoCo 3.2.3 wheels), and the T800 submodule. Port 8050 is the default. The sandbox preview uses `--host 0.0.0.0`.

## Run in Docker (recommended on your machine)

The sim image now includes Dash and Plotly, and compose has a `dashboard` service in the `sim-only` profile. Your `.env` must already be set up for the base image (see `docker/README.md`).

```
git submodule update --init --depth 1 third_party/T800-PROJECT
docker compose --profile sim-only build base dashboard
docker compose --profile sim-only up dashboard
```

Open http://localhost:8050. The repo's `sim/`, `planning/` and the T800 submodule are mounted read-only, so editing the code on the host needs a restart of the service, not a rebuild.

Not yet run: this Docker path was written and checked for paths and YAML syntax only. Docker is not available in the sandbox, so the image was not built. Please report any build or run error.

## Tests run in the sandbox

- A 3 s run completes, and the scene, KPI and chart builders run on its output.
- A 30 s run stopped after about 6 s ends with `stopped: true`.
- A new run started after a stop is not stopped by the previous run's stop request.
- The Dash layout and all three callbacks are registered (`/_dash-layout`, `/_dash-dependencies`).

Not checked: the rendered layout in a browser (none is available in the sandbox).

## Honesty notes

- The real-time factor is measured here. In the sandbox one simulated second takes about 3–5 wall seconds, because each NMPC solve takes 0.25–0.6 s at 10 Hz. The dashboard does not claim real time.
- Solve times are measurements on the machine that runs the dashboard, not target-hardware numbers.
- The base is driven by a weld to a mocap body. The legs hold a PD standing pose. Nothing balances or walks.
- Persons do not react to the robot (Stage 0 scenario S2).
- The `d_min` values are ASSUMPTIONs from `planning/stage0/config.py`.

## Balance panel (T800 standing)

The panel reads the telemetry buffer that `sim/t800/viewer.py` writes in the same process. Run the viewer (`scripts/run_viewer.sh` in a container, or `python sim/t800/viewer.py` on a host venv) and the panel fills in. Without the viewer, it shows "not started".
