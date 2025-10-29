#!/usr/bin/env bash
set -euo pipefail

# Build jetson-inference inside the running container to a local prefix.
# Usage: bash my_ws/scripts/build_jetson_inference.sh

JI_DIR=${JI_DIR:-/workspaces/oakd_isaac_ros/my_ws/3rdparty/jetson-inference}
JI_INSTALL=${JI_INSTALL:-"${JI_DIR}/install"}

echo "[build_jetson_inference] JI_DIR=${JI_DIR}"
echo "[build_jetson_inference] JI_INSTALL=${JI_INSTALL}"

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
cmake ../

echo "[build_jetson_inference] Building core targets..."
make -j$(nproc)

echo "[build_jetson_inference] Installing to ${JI_INSTALL}..."
make install

ldconfig

echo "[build_jetson_inference] Done. To use it now, run:"
echo "  source my_ws/scripts/source_jetson_inference.sh"

popd >/dev/null
