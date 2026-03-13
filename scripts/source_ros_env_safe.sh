#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found. This script targets ROS2 Humble."
  exit 1
fi

# Guard against nounset shells when ROS/colcon setup scripts reference these vars.
: "${COLCON_TRACE:=}"
: "${COLCON_CURRENT_PREFIX:=}"
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=}"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
