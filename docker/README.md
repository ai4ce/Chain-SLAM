# Chain-SLAM — Docker environment

A reproducible ROS1 (Noetic, Ubuntu 20.04) environment for building and running
Chain-SLAM. The image bakes in the system dependencies and GTSAM; the workspace
and Open3D are built once inside the container.

## Prerequisites
- Docker
- An X server on the host (for rviz). Optional NVIDIA GPU passthrough is enabled
  automatically when the NVIDIA Container Toolkit is installed.

## Files
| File | Where it runs | Purpose |
|------|---------------|---------|
| `Dockerfile`        | host  | Builds the image: apt deps + GTSAM 4.0.3 from source. |
| `run.sh`            | host  | Creates/re-enters the `chainslam_ros1` container with GUI + mounts. |
| `build_ws.sh`       | container | Clones `livox_ros_driver` and builds the catkin workspace. |
| `install_open3d.sh` | container | Builds Open3D 0.15.1 (C++) from source with the libstdc++ ABI. |

## Quick start

```bash
# 1. Build the image (from the repo root)
docker build -t chainslam_ros1:latest docker/

# 2. Create & enter the container (re-run later to re-enter)
./docker/run.sh

# --- inside the container ---

# 3. Build Open3D from source (once)
./docker/install_open3d.sh

# 4. Build the catkin workspace
./docker/build_ws.sh

# 5. Source the overlay and run (see the top-level README for the pipeline)
source devel/setup.bash
```

## Notes
- The repo is bind-mounted at the **same path** inside the container as on the
  host, so paths in launch files and the pipeline resolve identically.
- `run.sh` also mounts `/home`, `/mnt`, and `/media` for access to rosbags and
  output directories.
- GTSAM is pinned to 4.0.3 and Open3D to 0.15.1 — the versions that compile
  cleanly on this Noetic/GCC 9 image (see comments in `install_open3d.sh`).
