#!/usr/bin/env bash
# Build the catkin workspace INSIDE the container.
#
# Run this from inside the container (after ./docker/run.sh):
#   ./docker/build_ws.sh
#
# It is idempotent: clones livox_ros_driver only if missing, then builds the
# livox driver first (it generates CustomMsg.h that the node includes) and
# finally the whole workspace.
set -euo pipefail

# Workspace root = repo root = parent of this script's dir.
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/noetic/setup.bash

# --- livox_ros_driver: required, provides livox_ros_driver/CustomMsg.h ---
if [ ! -d "${WS_ROOT}/src/livox_ros_driver" ]; then
  echo ">>> Cloning livox_ros_driver into src/ ..."
  git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git \
    "${WS_ROOT}/src/livox_ros_driver"
fi

cd "${WS_ROOT}"

# Build the livox driver first so its generated messages exist before the node.
echo ">>> Building livox_ros_driver ..."
catkin_make --pkg livox_ros_driver -DPYTHON_EXECUTABLE=/usr/bin/python3

# Full workspace build. Flags mirror commands.txt (system Boost, python3).
echo ">>> Building full workspace ..."
catkin_make \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DBOOST_ROOT=/usr \
  -DBoost_NO_BOOST_CMAKE=ON

echo ">>> Done. Source the overlay with:  source ${WS_ROOT}/devel/setup.bash"
