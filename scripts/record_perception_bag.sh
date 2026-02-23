#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${1:-perception_bag_$(date +%Y%m%d_%H%M%S)}"
DURATION_SEC="${2:-60}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found."
  exit 1
fi

source "$(dirname "$0")/source_ros_env_safe.sh"

echo "Recording ${DURATION_SEC}s to ${OUT_DIR}"
timeout "${DURATION_SEC}" ros2 bag record -o "${OUT_DIR}" \
  /detections_output \
  /oak/stereo/image_raw \
  /oak/stereo/camera_info \
  /oak/rgb/image_raw \
  /oak/rgb/image_raw_rgb8 \
  /oak/rgb/camera_info \
  /tf /tf_static

echo "Bag recording completed: ${OUT_DIR}"
