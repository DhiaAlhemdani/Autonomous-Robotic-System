# Base image: Ubuntu 24.04 + ROS 2 Jazzy (LTS) + build tools + non-root user.
# Implements docs/03 §1. Build with scripts/pin_base_digest.sh first, which writes .env.
#
# Fail-closed: BASE_IMAGE and ROS_APT_SOURCE_VERSION have no defaults. A placeholder digest
# makes FROM fail, and scripts/pin_base_digest.sh refuses to write .env for it. The build stops
# if either is missing, so an unpinned image cannot be produced by accident.
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG ROS_APT_SOURCE_VERSION
ARG ROBOT_UID=1000

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

RUN test -n "${ROS_APT_SOURCE_VERSION}" || (echo "ROS_APT_SOURCE_VERSION is required" >&2 && exit 1)

# Locale and universe repository (ROS 2 install guide, Ubuntu deb packages)
RUN apt-get update \
 && apt-get install -y --no-install-recommends curl ca-certificates software-properties-common locales \
 && locale-gen en_US en_US.UTF-8 \
 && add-apt-repository -y universe \
 && rm -rf /var/lib/apt/lists/*

# ROS 2 apt source, pinned to ROS_APT_SOURCE_VERSION (not resolved at build time)
RUN . /etc/os-release \
 && curl -fsSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb" \
 && apt-get update && apt-get install -y /tmp/ros2-apt-source.deb \
 && rm -f /tmp/ros2-apt-source.deb \
 && rm -rf /var/lib/apt/lists/*

# ROS 2 Jazzy base install plus build tools for colcon packages
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-base \
      python3-colcon-common-extensions \
      python3-pytest \
 && rm -rf /var/lib/apt/lists/*

# Non-root user (docs/03 §6). Ubuntu 24.04 ships a default 'ubuntu' user at UID 1000.
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
 && useradd -m -u "${ROBOT_UID}" -s /bin/bash robot

USER robot
WORKDIR /home/robot/ws
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/robot/.bashrc

CMD ["bash"]
