#!/usr/bin/env bash
# Start (or re-enter) the Chain SLAM ROS1 container with GUI access and the
# repo bind-mounted at its real host path.
#
#   ./docker/run.sh                 # create & enter the container
#   ./docker/run.sh                 # if it already exists, just re-enters it
#
# The repo is mounted at the SAME path inside the container as on the host, so
# every path in commands.txt / launch files resolves identically.
set -euo pipefail

IMAGE="chainslam_ros1:latest"
CONTAINER="chainslam_ros1"

# Repo root = parent of this script's directory.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Allow the container to talk to the host X server (idempotent).
if command -v xhost >/dev/null 2>&1; then
  xhost +local:docker >/dev/null 2>&1 || true
fi

# If the container already exists, start + exec into it.
if docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  docker start "${CONTAINER}" >/dev/null
  exec docker exec -it \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e QT_X11_NO_MITSHM=1 \
    "${CONTAINER}" bash
fi

# Optional NVIDIA GPU passthrough (only if the toolkit is present).
GPU_FLAG=""
if docker info 2>/dev/null | grep -qi nvidia; then
  GPU_FLAG="--gpus all"
fi

# First run: create the container.
#   --network host + --privileged : ROS networking + device (LiDAR/USB) access
#   /home /mnt /media mounts        : broad filesystem access for bags/outputs
exec docker run -it \
  --name "${CONTAINER}" \
  --network host \
  --privileged \
  ${GPU_FLAG} \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${HOME}/.Xauthority:/root/.Xauthority:rw" \
  -v /home:/home \
  -v /mnt:/mnt \
  -v /media:/media \
  "${IMAGE}" \
  bash
