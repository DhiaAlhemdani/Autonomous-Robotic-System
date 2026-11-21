# T800 simulation: viewer and model check

Sandbox simulation only. Nothing here is validated on hardware. There is no controller in this directory, and no balance, standing or walking claim is made.

Contents:
- `viewer.py`: MuJoCo viewer for the T800 model, native window or headless. No controller is applied: the robot is passive and falls under gravity.
- `../../scripts/check_t800_model.py`: loads the vendored model and runs one forward step.
- `../../vendor/t800/`: the vendored T800 config and robot assets. Licence and NOTICE are kept there.

## Native 3D viewer (run on your laptop)

`viewer.py` opens MuJoCo's own 3D window. Run it natively, not in Docker, because the window needs a display.

```
.venv/bin/python -m pip install mujoco==3.2.3 numpy==1.26.4   # first time only
.venv/bin/python sim/t800/viewer.py                          # passive; falls from the start pose
.venv/bin/python sim/t800/viewer.py --push-x -150 --push-t 5 # 150 N backward push at 5 s
```

Options: `--push-x`, `--push-y` (N), `--push-t` (start, s), `--push-dur` (s, default 0.2), `--T` (stop after T s). Closing the window ends the run. Stop with Ctrl+C or `--T`.

Headless check (no window, runs anywhere): `.venv/bin/python sim/t800/viewer.py --headless --T 3`. The pelvis drops from 0.87 m within about 1 s, which is the expected passive behaviour.

## Linux laptop: viewer in a container (X11)

Requires a Linux desktop session (X11, not Wayland-only) and Docker with the compose plugin.

```bash
scripts/run_viewer.sh
```

The script allows local X connections (`xhost +local:`) and starts the `viewer` service, which opens the MuJoCo 3D window on your display. Stop with `docker compose --profile sim-only down`.

Not tested in the build sandbox: the window path (no display there) and the X11 mount on a real laptop. The headless path is the one checked.

## Model check

`scripts/check_t800_model.py` loads `vendor/t800` and prints the body, joint and actuator counts. The mass it prints is the model value, not a measurement.
