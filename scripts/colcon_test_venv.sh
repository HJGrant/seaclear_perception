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

source "$(dirname "$0")/source_ros_env_safe.sh"

.venv/bin/python -m colcon test --merge-install "$@"
.venv/bin/python -m colcon test-result --verbose

# Short-term guard: run package pytest suites explicitly so tests are never silently skipped.
.venv/bin/python -m pytest -q \
  src/sc2_visual_servo_controller/test \
  src/sc2_visual_servo_perception/test \
  src/sc2_visual_servo_bringup/test
