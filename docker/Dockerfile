FROM osrf/ros:noetic-desktop-full

ARG GTSAM_VERSION=4.0.3
ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# 1. System / apt dependencies
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      git cmake build-essential \
      libeigen3-dev libboost-all-dev libtbb-dev \
      libgeographic-dev \
      ros-noetic-pcl-ros \
      libgl1-mesa-glx libgl1-mesa-dri \
      wget unzip vim nano \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. GTSAM from source (commands.txt: "need to build gtsam from source")
# ---------------------------------------------------------------------------
RUN cd /opt \
    && git clone --branch ${GTSAM_VERSION} --depth 1 https://github.com/borglab/gtsam.git \
    && cd gtsam && mkdir build && cd build \
    && cmake \
         -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DCMAKE_BUILD_TYPE=Release \
         .. \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && cd /opt && rm -rf gtsam/build

# ---------------------------------------------------------------------------
# 3. Shell convenience: auto-source ROS in every interactive shell
# ---------------------------------------------------------------------------
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

CMD ["bash"]
