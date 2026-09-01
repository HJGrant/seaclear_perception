#!/usr/bin/env bash
set -euo pipefail

# Play a recorded bag through the *live* visual_servo_controller_node (with
# the centroid Kalman filter enabled) while foxglove_bridge runs alongside,
# so /sc2/visual_servo/centroid_error (instantaneous) and
# /sc2/visual_servo/filtered_centroid_error (KF) can be watched live in
# Foxglove -- connect to ws://localhost:8765 once this is running.
#
# Usage: scripts/run_centroid_kf_bag_debug.sh <bag_path> [rate]

BAG_PATH="${1:-}"
if [[ -z "${BAG_PATH}" ]]; then
  echo "Usage: $0 <bag_path> [rate]"
  echo "  e.g.: $0 rosbags/<your_bag_name>"
  exit 1
fi
RATE="${2:-1.0}"

source "$(dirname "$0")/source_ros_env_safe.sh"

WS_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  # colcon's generated setup scripts aren't nounset-safe (e.g.
  # COLCON_PYTHON_EXECUTABLE is referenced with no default) -- relax -u just
  # for sourcing, standard practice for ROS2/colcon setup scripts.
  set +u
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
  set -u
else
  echo "ERROR: ${WS_ROOT}/install/setup.bash not found -- build the workspace first." >&2
  exit 1
fi

PIDS=()
cleanup() {
  echo "Shutting down..."
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Starting foxglove_bridge (ws://localhost:8765)..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml >/tmp/foxglove_bridge_debug.log 2>&1 &
PIDS+=("$!")

echo "Starting visual_servo_controller_node with centroid_kf_enable:=true..."
ros2 run sc2_visual_servo_controller visual_servo_controller_node \
  --ros-args -p centroid_kf_enable:=true \
  >/tmp/visual_servo_controller_debug.log 2>&1 &
PIDS+=("$!")

# Give both a moment to come up before the bag starts publishing.
sleep 2

echo "Playing bag=${BAG_PATH} rate=${RATE}"
echo "Watch /sc2/visual_servo/centroid_error (instantaneous) and"
echo "      /sc2/visual_servo/filtered_centroid_error (KF) in Foxglove."
echo "Node/bridge logs: /tmp/visual_servo_controller_debug.log /tmp/foxglove_bridge_debug.log"
ros2 bag play "${BAG_PATH}" --clock --rate "${RATE}"

echo "Bag playback finished."
