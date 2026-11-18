# Simulation image: base image plus MuJoCo in a virtualenv. Adds no robot code.
ARG BASE_IMAGE_TAG=ars/base:local
FROM ${BASE_IMAGE_TAG}

USER root
RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-venv libgl1 libegl1 \
 && rm -rf /var/lib/apt/lists/*

USER robot
# Pinned to the MuJoCo version the T800 project uses (3.2.3).
RUN python3 -m venv /home/robot/simvenv \
 && /home/robot/simvenv/bin/pip install --no-cache-dir mujoco==3.2.3 numpy==1.26.4
ENV PATH=/home/robot/simvenv/bin:$PATH
WORKDIR /home/robot/ws