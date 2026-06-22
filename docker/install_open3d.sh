#!/usr/bin/env bash
# Build & install Open3D (C++) from source INSIDE the container.
#
# Run once after first entering the container (./docker/run.sh):
#   ./docker/install_open3d.sh
#
# Why from source (not the release binary):
#   The official Open3D release binaries are compiled against libc++, which is
#   ABI-incompatible with the libstdc++ ROS/PCL/GTSAM stack — linking the node
#   against them fails with "undefined reference to std::__1::*". Building from
#   source with -DGLIBCXX_USE_CXX11_ABI=ON produces libstdc++-compatible
#   std:: symbols.
#
# Why 0.15.1:
#   It is the last Open3D release that compiles cleanly on Ubuntu 20.04 / GCC 9
#   (bundled fmt 8.x and Eigen 3.4). Newer Open3D (0.16+) needs a newer
#   toolchain and fails on this image (fmt static_assert, Eigen::Map::begin).
#   0.15.1 still has every registration API the node uses (FPFH, RANSAC-based
#   feature matching, correspondence checkers, normal estimation).
set -euo pipefail

OPEN3D_VERSION="${OPEN3D_VERSION:-0.15.1}"
JOBS="${JOBS:-$(nproc)}"

# Open3D 0.15 wants CMake >= 3.18; Noetic ships 3.16. Install a newer CMake
# from Kitware's binary release if needed.
need_cmake=1
if command -v cmake >/dev/null 2>&1; then
  ver="$(cmake --version | head -1 | awk '{print $3}')"
  major="${ver%%.*}"; rest="${ver#*.}"; minor="${rest%%.*}"
  if [ "${major}" -gt 3 ] || { [ "${major}" -eq 3 ] && [ "${minor}" -ge 18 ]; }; then
    need_cmake=0
  fi
fi
if [ "${need_cmake}" -eq 1 ]; then
  echo ">>> Installing newer CMake (3.27.9) ..."
  cd /tmp
  wget -qO cmake.sh "https://github.com/Kitware/CMake/releases/download/v3.27.9/cmake-3.27.9-linux-x86_64.sh"
  bash cmake.sh --skip-license --prefix=/usr/local
  rm -f cmake.sh
fi
echo ">>> Using $(cmake --version | head -1)"

echo ">>> Cloning Open3D v${OPEN3D_VERSION} ..."
cd /opt
if [ ! -d Open3D ]; then
  git clone --branch "v${OPEN3D_VERSION}" --depth 1 https://github.com/isl-org/Open3D.git
fi
cd Open3D

echo ">>> Installing Open3D's Ubuntu build dependencies ..."
bash util/install_deps_ubuntu.sh assume-yes

echo ">>> Configuring (libstdc++ ABI, C++ lib only) ..."
rm -rf build && mkdir build && cd build
cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DGLIBCXX_USE_CXX11_ABI=ON \
  -DBUILD_PYTHON_MODULE=OFF \
  -DBUILD_GUI=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_UNIT_TESTS=OFF \
  -DBUILD_BENCHMARKS=OFF \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  ..

echo ">>> Building with -j${JOBS} ..."
make -j"${JOBS}"
make install
ldconfig

echo ">>> Done. Verifying ABI (expect libstdc++, NOT libc++):"
ldd /usr/local/lib/libOpen3D.so | grep -E "libstdc\+\+|libc\+\+" || true
echo ">>> Open3D installed to /usr/local. CMake config:"
find /usr/local -name "Open3DConfig.cmake" 2>/dev/null | head -1
