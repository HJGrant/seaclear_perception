#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found. This script targets ROS2 Humble."
  exit 1
fi

if [[ ! -x .venv/bin/python ]]; then
  echo "ERROR: .venv/bin/python missing. Run ./scripts/uv_bootstrap.sh first."
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

.venv/bin/python -m colcon build --merge-install "$@"
