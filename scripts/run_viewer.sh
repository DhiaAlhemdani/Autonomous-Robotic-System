#!/usr/bin/env bash
# Start the T800 MuJoCo viewer in the sim container (Linux host with X11) and open the dashboard.
# Usage (from a desktop terminal, repo root):  scripts/run_viewer.sh
set -euo pipefail
cd "$(dirname "$0")/.."

: "${DISPLAY:?no DISPLAY set: run this from your desktop session, not over ssh}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
command -v xhost >/dev/null || { echo "install xhost first: sudo apt install x11-xserver-utils"; exit 1; }
command -v curl >/dev/null || { echo "install curl first: sudo apt install curl"; exit 1; }

# Let local containers talk to the X server (the container user is not your user).
xhost +local: >/dev/null

docker compose --profile sim-only build base viewer
docker compose --profile sim-only up -d viewer

echo "waiting for the dashboard on http://localhost:8050 ..."
for _ in $(seq 1 90); do
  if curl -fs -o /dev/null http://localhost:8050/_dash-layout; then break; fi
  sleep 1
done
if curl -fs -o /dev/null http://localhost:8050/_dash-layout; then
  xdg-open http://localhost:8050 >/dev/null 2>&1 || echo "open http://localhost:8050 in your browser"
else
  echo "dashboard did not start; see: docker compose --profile sim-only logs viewer"
  exit 1
fi
echo "viewer log: docker compose --profile sim-only logs -f viewer"
echo "stop:       docker compose --profile sim-only stop viewer"
