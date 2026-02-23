#!/usr/bin/env bash
set -euo pipefail

BAG_PATH="${1:-}"
if [[ -z "${BAG_PATH}" ]]; then
  echo "Usage: $0 <bag_path> [rate]"
  exit 1
fi
RATE="${2:-1.0}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found."
  exit 1
fi

source "$(dirname "$0")/source_ros_env_safe.sh"

echo "Playing bag=${BAG_PATH} rate=${RATE}"
ros2 bag play "${BAG_PATH}" --clock --rate "${RATE}"
