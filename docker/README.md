# Development environment: laptop setup

This skeleton implements the first part of `docs/03` (base image, dev image, Compose profile `sim-only`, and one ROS 2 package). It has been checked for syntax and structure only. **It has not yet been built or run.** The first build on the laptop is the real test.

## Prerequisites

- Docker Engine with the Compose plugin.
- The NVIDIA Container Toolkit is not needed for this skeleton. It becomes necessary when the `ml-train` image is added.
- Internet access to Docker Hub and to `api.github.com` (needed once, by the pinning script).

## Steps

1. **Resolve the base-image digest once** (needs Docker). This rewrites `docker/base/pins.env`, which is committed:

   ```bash
   ./scripts/pin_base_digest.sh --update
   git diff docker/base/pins.env      # review, then commit it
   git commit -am "Pin Ubuntu 24.04 base digest" && git push
   ```

   Until this is done, the build refuses to run, by design. On later days, run `pin_base_digest.sh` without `--update` to write `.env` from the committed pins.

2. **Build the base image** (fails if `.env` is missing):

   ```bash
   docker compose build base
   ```

3. **Start a development shell**:

   ```bash
   docker compose --profile sim-only run --rm dev
   ```

4. **Inside the container**, build the workspace and run the heartbeat node:

   ```bash
   cd ~/ws
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch ars_bringup heartbeat.launch.py
   ```

5. **In a second shell** on the laptop (or another container shell), check that discovery works:

   ```bash
   docker compose --profile sim-only run --rm dev bash -lc 'ros2 topic echo /heartbeat'
   ```

   You should see `data: alive N` once per second. Host networking is used, so both shells share the DDS domain (`ROS_DOMAIN_ID`, default 42).

## Notes and limits

- The workspace is bind-mounted from `./ros2_ws`. Build outputs (`build/`, `install/`, `log/`) are git-ignored.
- The container user is `robot` with UID 1000. If your laptop user has a different UID, the build outputs will be owned differently on the host. Set `ROBOT_UID` in the build arguments to match.
- Not yet written: the `sim`, `control`, `ml-train`, `ml-deploy` and `ci` images from `docs/03` §1, the CI pipeline (§5), the Python lockfile, and any humanoid model. The humanoid platform is not yet chosen (`docs/11` §5), so no robot model is included.
- The heartbeat node has no safety function. It only tests the toolchain.

## Troubleshooting

Two problems came up while testing this skeleton on the laptop. Both are fixed in the repository, so they should not recur.

- **`ros2: command not found` under `bash -lc`.** The ROS setup was in `~/.bashrc`, which Ubuntu's default file skips for non-interactive shells. The setup now lives in `/etc/profile.d/ros2.sh`. If you see this, rebuild the images with `docker compose build base dev`.
- **Topics appear (`ros2 topic list` shows `/heartbeat`) but `ros2 topic echo` prints nothing across containers.** Discovery worked, but data did not. Each container had its own IPC namespace, so ROS 2 shared-memory transport could not reach the other container. The Compose file sets `ipc: host` on the ROS services to fix this. Restart the containers after pulling, because the setting applies when a container starts.

To diagnose a silent topic, run these in a separate container while the publisher runs:

```bash
ros2 topic info /heartbeat -v   # Publisher count should be 1
timeout 5 ros2 topic hz /heartbeat
```

If the publisher count is 0, the publishing node is not running. If the count is 1 but `hz` reports nothing, check the IPC setting.
