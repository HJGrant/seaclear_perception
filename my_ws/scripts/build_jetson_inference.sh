#!/usr/bin/env bash
set -euo pipefail

# Build jetson-inference inside the running container to a local prefix.
# Usage: bash my_ws/scripts/build_jetson_inference.sh

JI_DIR=${JI_DIR:-/workspaces/oakd_isaac_ros/my_ws/3rdparty/jetson-inference}
JI_INSTALL=${JI_INSTALL:-"${JI_DIR}/install"}

# --- Configuration for libnpymath fix ---
# This path is where libnpymath.a (the NumPy math library) is typically installed
# when python3-numpy is installed via apt on Ubuntu/Debian systems.
NUMPY_LIB_PATH_DEFAULT="/usr/lib/python3.10/dist-packages/numpy/core/lib"
NUMPY_LIB_PATH=${NUMPY_LIB_PATH_OVERRIDE:-$NUMPY_LIB_PATH_DEFAULT}

echo "[build_jetson_inference] JI_DIR=${JI_DIR}"
echo "[build_jetson_inference] JI_INSTALL=${JI_INSTALL}"
echo "[build_jetson_inference] NumPy Lib Path: ${NUMPY_LIB_PATH}"

mkdir -p "$(dirname "${JI_DIR}")"

if [[ ! -d "${JI_DIR}" ]]; then
  echo "[build_jetson_inference] Cloning jetson-inference..."
  git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference "${JI_DIR}"
else
  echo "[build_jetson_inference] Using existing repo at ${JI_DIR}"
  pushd "${JI_DIR}" >/dev/null
  git submodule update --init --recursive
  popd >/dev/null
fi

mkdir -p "${JI_DIR}/build"
pushd "${JI_DIR}/build" >/dev/null

echo "[build_jetson_inference] Configuring CMake..."
# THE FIX: We pass the NumPy library directory (-L) to the linker flags
# so it can find the required 'libnpymath.a' library.
# The variable is now explicitly defined via a default/override pattern.
cmake ../ \
    -DCMAKE_INSTALL_PREFIX="${JI_INSTALL}" \
    -DCMAKE_SHARED_LINKER_FLAGS="-L${NUMPY_LIB_PATH}"

echo "[build_jetson_inference] Building core targets..."
make -j$(nproc)

echo "[build_jetson_inference] Installing to ${JI_INSTALL} and system Python paths (requires sudo)..."
# FIX 1: Use 'sudo' to gain permission to write to /usr/lib/python3.10/dist-packages/
sudo make install

echo "[build_jetson_inference] Updating shared library cache (requires sudo)..."
# FIX 2: Use 'sudo' to gain permission to write to /etc/ld.so.cache~
sudo ldconfig

echo "[build_jetson_inference] Done. To use it now, run:"
echo "  source my_ws/scripts/source_jetson_inference.sh"

popd >/dev/null