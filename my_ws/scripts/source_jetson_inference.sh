#!/usr/bin/env bash

# Source local jetson-inference install for development.
# Usage: source my_ws/scripts/source_jetson_inference.sh

JI_DIR=${JI_DIR:-/workspaces/oakd_isaac_ros/my_ws/3rdparty/jetson-inference}
JI_INSTALL=${JI_INSTALL:-"${JI_DIR}/install"}

if [[ ! -d "${JI_INSTALL}" ]]; then
  echo "[source_jetson_inference] Install dir not found: ${JI_INSTALL}" >&2
  echo "Run: bash my_ws/scripts/build_jetson_inference.sh" >&2
  return 1 2>/dev/null || exit 1
fi

export CMAKE_PREFIX_PATH="${JI_INSTALL}:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${JI_INSTALL}/lib:${LD_LIBRARY_PATH:-}"
export LIBRARY_PATH="${JI_INSTALL}/lib:${LIBRARY_PATH:-}"
export CPATH="${JI_INSTALL}/include:${CPATH:-}"

# Python bindings typically installed to ${JI_INSTALL}/python
if [[ -d "${JI_INSTALL}/python" ]]; then
  export PYTHONPATH="${JI_INSTALL}/python:${PYTHONPATH:-}"
fi

echo "[source_jetson_inference] Env set. CMAKE_PREFIX_PATH includes ${JI_INSTALL}"
echo "[source_jetson_inference] PYTHONPATH includes ${JI_INSTALL}/python"
