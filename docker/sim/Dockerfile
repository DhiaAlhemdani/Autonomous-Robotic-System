# Simulation image: base image plus MuJoCo and the dashboard packages in a virtualenv.
# Adds no robot code; the T800 model and the repo code are mounted by docker-compose.yml.
ARG BASE_IMAGE_TAG=ars/base:local
FROM ${BASE_IMAGE_TAG}

USER root
RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-venv libgl1 libegl1 \
    libx11-6 libxext6 libxrandr2 libxinerama1 libxcursor1 libxi6 libxrender1 \
    libgl1-mesa-dri libglx-mesa0 libegl-mesa0 \
 && rm -rf /var/lib/apt/lists/*

USER robot
# Pinned to the MuJoCo version the T800 project uses (3.2.3). Dash and Plotly are for sim/dashboard.
RUN python3 -m venv /home/robot/simvenv \
 && /home/robot/simvenv/bin/pip install --no-cache-dir \
    mujoco==3.2.3 numpy==1.26.4 scipy==1.13.1 pytest==8.3.3 \
    dash==4.5.0 plotly==7.1.0
ENV PATH=/home/robot/simvenv/bin:$PATH
WORKDIR /home/robot/ws
