#!/usr/bin/env bash
set -euo pipefail

# Exec into the running dev container with jetson-inference env pre-sourced.
# Usage: bash my_ws/scripts/attach_with_jetson_inference.sh [container_name]

CONTAINER_NAME=${1:-isaac_ros_dev-aarch64-container}
SRC_SCRIPT=/workspaces/oakd_isaac_ros/my_ws/scripts/source_jetson_inference.sh

echo "[attach] attaching to ${CONTAINER_NAME}"
docker exec -it "${CONTAINER_NAME}" bash -lc "\
  if [[ -f ${SRC_SCRIPT} ]]; then \
    source ${SRC_SCRIPT}; \
  else \
    echo '[attach] warn: source script not found:' ${SRC_SCRIPT} ; \
  fi; \
  exec bash -i"
