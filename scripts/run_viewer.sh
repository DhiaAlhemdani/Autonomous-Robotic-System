#!/usr/bin/env bash
# Start the T800 MuJoCo 3D viewer in the sim container (Linux host with X11).
# Usage (from a desktop terminal, repo root):  scripts/run_viewer.sh
set -euo pipefail
cd "$(dirname "$0")/.."

: "${DISPLAY:?no DISPLAY set: run this from your desktop session, not over ssh}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
command -v xhost >/dev/null || { echo "install xhost first: sudo apt install x11-xserver-utils"; exit 1; }

# Let local containers talk to the X server (the container user is not your user).
xhost +local: >/dev/null

docker compose --profile sim-only build viewer
docker compose --profile sim-only up -d --force-recreate viewer

sleep 5
echo "3D window status (from the container log):"
docker compose --profile sim-only logs --tail 20 viewer 2>/dev/null | grep -E "3D window|no DISPLAY|failed|Error" \
  || echo "  (no window message yet; see: docker compose --profile sim-only logs viewer)"
echo "viewer log: docker compose --profile sim-only logs -f viewer"
echo "stop:       docker compose --profile sim-only stop viewer"
