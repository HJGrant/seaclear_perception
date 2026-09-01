#!/usr/bin/env bash
set -euo pipefail

# Records a KF calibration/validation bag by running the fixed maneuver
# sequence (settle -> hold_still -> translate_x -> translate_y -> yaw -> stop)
# via calibration_maneuver_node, while ros2 bag record captures the topics
# the calibration analysis needs. Same procedure is meant to be reproduced
# later on the real vehicle -- see the maneuver node's docstring.
#
# Usage: scripts/record_kf_calibration_bag.sh [output_dir]

OUT_DIR="${1:-rosbags/kf_calibration_$(date +%Y%m%d_%H%M%S)}"

source "$(dirname "$0")/source_ros_env_safe.sh"

WS_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
  set -u
else
  echo "ERROR: ${WS_ROOT}/install/setup.bash not found -- build the workspace first." >&2
  exit 1
fi

TOPICS=(
  /best_detection
  /sc2/odometry_ekf
  /sc2/state_estimation_node/estimated_velocity
  /waterlinked_dvl_driver/velocity_report
  /sc2/dvl
  /tf_static
  /tf
  /oak/rgb/image_raw
  /oak/rgb/camera_info
  /mask_overlay
  /sc2/visual_servo/calibration_phase
)

echo "Recording to ${OUT_DIR}"
echo "Topics: ${TOPICS[*]}"
ros2 bag record -o "${OUT_DIR}" "${TOPICS[@]}" &
BAG_PID=$!

cleanup() {
  if kill -0 "${BAG_PID}" 2>/dev/null; then
    echo "Stopping bag recording..."
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Give the recorder a moment to actually subscribe before commands start.
sleep 2

echo "Running calibration maneuver..."
ros2 run sc2_visual_servo_controller calibration_maneuver_node

echo "Maneuver complete. Bag: ${OUT_DIR}"
