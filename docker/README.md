# Development environment: laptop setup

This skeleton implements the first part of `docs/03` (base image, dev image, Compose profile `sim-only`, and one ROS 2 package). It has been checked for syntax and structure only. **It has not yet been built or run.** The first build on the laptop is the real test.

## Prerequisites

- Docker Engine with the Compose plugin.
- The NVIDIA Container Toolkit is not needed for this skeleton. It becomes necessary when the `ml-train` image is added.
- Internet access to Docker Hub and to `api.github.com` (needed once, by the pinning script).

## Steps

1. **Pin the base image and the ROS apt-source version** (writes `.env` at the repository root; `.env` is not committed):

   ```bash
   ./scripts/pin_base_digest.sh
   ```

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
