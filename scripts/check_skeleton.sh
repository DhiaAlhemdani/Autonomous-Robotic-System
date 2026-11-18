#!/usr/bin/env bash
# Static checks that run without Docker or ROS. Run before every commit that touches the skeleton.
set -euo pipefail
cd "$(dirname "$0")/.."
fail=0
check() { if "$@"; then echo "ok:   $*"; else echo "FAIL: $*"; fail=1; fi; }

for f in docker/base/Dockerfile docker/dev/Dockerfile docker-compose.yml scripts/pin_base_digest.sh \
         ros2_ws/src/ars_bringup/package.xml ros2_ws/src/ars_bringup/setup.py \
         ros2_ws/src/ars_bringup/ars_bringup/heartbeat.py ros2_ws/src/ars_bringup/launch/heartbeat.launch.py; do
  check test -f "$f"
done

# No floating tags anywhere (docs/03 §1 rule 4)
check bash -c '! grep -RIn --include=Dockerfile --include=*.yml -E "(FROM|image:).*:latest" docker docker-compose.yml'
check bash -c '! grep -RIn -E "ubuntu:24\.04|ros:[a-z]" docker/base/Dockerfile'

# Scripts parse
check bash -n scripts/pin_base_digest.sh
check bash -n scripts/check_skeleton.sh

# Python parses
check python3 -m py_compile ros2_ws/src/ars_bringup/setup.py ros2_ws/src/ars_bringup/ars_bringup/heartbeat.py ros2_ws/src/ars_bringup/launch/heartbeat.launch.py

# Compose parses, if PyYAML is available
if python3 -c "import yaml" 2>/dev/null; then
  check python3 -c "import yaml,sys; yaml.safe_load(open('docker-compose.yml'))"
else
  echo "skip: compose YAML parse (PyYAML not installed)"
fi

# Secret scan on tracked skeleton files
check bash -c '! grep -RIn --exclude=check_skeleton.sh -E "(api[_-]?key|password|BEGIN (RSA|OPENSSH|PRIVATE) KEY|ghp_|github_pat_)" docker docker-compose.yml scripts ros2_ws/src'

exit $fail
