# Real-time dashboard: T800 planar base under NMPC

Browser dashboard for `sim/t800/planar_base.py`. It runs one episode at a time in a background thread and streams every control step over a WebSocket.

**Shows:** the scene from above (robot pelvis and trail, persons with their keep-out rings, objects, goal, commanded acceleration); live charts for NMPC solve time, pelvis tilt, distance to the nearest person and object, speed and tracking error, and commanded acceleration; headline numbers (max tilt, closest person, violation steps, solve p50/p99, infeasible fraction, tracking error, real-time factor).

**Not shown:** a 3D render. The MuJoCo viewer needs a display, so it is not used here.

## Run

```
pip install -r sim/dashboard/requirements.txt   # plus the T800 submodule (see sim/t800/README.md)
cd sim/dashboard && python -m uvicorn server:app --host 0.0.0.0 --port 8000
```

Open the page on port 8000. Set a seed and a maximum time, then press **Start episode**. A new browser connection receives the current run from the beginning.

API:

- `POST /api/run?seed=N&max_time=T` starts a run (stops the current one).
- `GET /api/status` shows the run ID and state.
- `WS /ws` streams `meta`, `step`, `done` or `error` messages.

## Honesty notes

- The real-time factor is measured. In the sandbox, one simulated second takes about 3 wall-clock seconds, because the NMPC solve (about 0.25–0.5 s) runs at 10 Hz. The page does not claim real time.
- Solve times and the real-time factor are sandbox measurements, not target-hardware numbers.
- The base is driven by a weld to a mocap body. The legs hold a PD standing pose. Nothing balances or walks.
- Persons don't react to the robot, as in Stage 0.
- The browser layout was not checked visually: no browser is available in the sandbox. The page was checked for syntax and served over HTTP, and the WebSocket stream was tested end to end with a Python client.
