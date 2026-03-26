#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PROFILE="no_hw"
DO_BUILD=1
SKIP_BOOTSTRAP=0
LAUNCH_SECONDS=12
LOGDIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="$2"
      shift 2
      ;;
    --logdir)
      LOGDIR="$2"
      shift 2
      ;;
    --launch-seconds)
      LAUNCH_SECONDS="$2"
      shift 2
      ;;
    --skip-build)
      DO_BUILD=0
      shift
      ;;
    --skip-bootstrap)
      SKIP_BOOTSTRAP=1
      shift
      ;;
    *)
      echo "Unknown argument: $1"
      echo "Usage: $0 [--profile no_hw|jetson_live] [--logdir DIR] [--launch-seconds N] [--skip-build] [--skip-bootstrap]"
      exit 1
      ;;
  esac
done

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found. Run this inside the ROS2 Humble container."
  exit 1
fi

if [[ -z "$LOGDIR" ]]; then
  TS="$(date +%Y%m%d_%H%M%S)"
  LOGDIR="$ROOT/diagnoses_oakd_system/$TS"
fi
mkdir -p "$LOGDIR"

{
  echo "timestamp=$(date --iso-8601=seconds)"
  echo "profile=$PROFILE"
  echo "pwd=$ROOT"
  echo "commit=$(git rev-parse HEAD)"
} | tee "$LOGDIR/meta.log"

if [[ "$SKIP_BOOTSTRAP" -eq 0 ]]; then
  "$ROOT/scripts/ensure_dev_container_ready.sh" 2>&1 | tee "$LOGDIR/bootstrap.log"
fi

if [[ "$DO_BUILD" -eq 1 ]]; then
  "$ROOT/scripts/colcon_build_venv.sh" \
    --packages-up-to \
    oakd_isaac_ros \
    sc2_visual_servo_interfaces \
    sc2_visual_servo_perception \
    sc2_visual_servo_controller \
    sc2_visual_servo_bringup \
    2>&1 | tee "$LOGDIR/colcon_build_oakd_system.log"
fi

echo "Sourcing ROS environment..." | tee "$LOGDIR/phase.log"
source "$ROOT/scripts/source_ros_env_safe.sh"
echo "Sourcing workspace install/setup.bash..." | tee -a "$LOGDIR/phase.log"
source "$ROOT/scripts/source_workspace_env_safe.sh" install/setup.bash > >(tee -a "$LOGDIR/phase.log") 2>&1
echo "Workspace environment sourced." | tee -a "$LOGDIR/phase.log"

if [[ "$PROFILE" == "jetson_live" ]]; then
  {
    echo "=== model files ==="
    ls -lah /workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear 2>/dev/null || true
    file /workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.onnx 2>/dev/null || true
    file /workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.plan 2>/dev/null || true
    echo
    echo "=== detection package hints from workspace ==="
    find /workspaces/oakd_isaac_ros/src/oakd_isaac_ros/launch -maxdepth 2 -type f | sort | grep -E 'detection|object' || true
  } | tee "$LOGDIR/perception_preflight.log"
fi

case "$PROFILE" in
  no_hw)
    LAUNCH_ARGS=(
      run_oakd:=false
      enable_detection:=false
      enable_nvblox:=false
      enable_visualization:=false
      enable_visual_servo:=true
      enable_target_point_estimator:=false
      enable_debug_viz:=false
      enable_mavros_bridge:=false
      controller_mode:=ibvs
    )
    ;;
  jetson_live)
    LAUNCH_ARGS=(
      run_oakd:=true
      enable_detection:=true
      enable_nvblox:=false
      enable_visualization:=false
      enable_visual_servo:=true
      enable_target_point_estimator:=true
      enable_debug_viz:=true
      enable_mavros_bridge:=false
      controller_mode:=bbox_target_point
    )
    ;;
  *)
    echo "Invalid profile: $PROFILE"
    exit 1
    ;;
esac

LAUNCH_LOG="$LOGDIR/oakd_system_launch.log"
echo "Launching oakd_system.launch.py with profile=$PROFILE" | tee -a "$LOGDIR/phase.log"
stdbuf -oL -eL ros2 launch oakd_isaac_ros oakd_system.launch.py "${LAUNCH_ARGS[@]}" \
  > >(tee "$LAUNCH_LOG") 2>&1 &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
    kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" || true
  fi
}
trap cleanup EXIT

sleep "$LAUNCH_SECONDS"

if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
  echo "Launch process exited early. See $LAUNCH_LOG" | tee -a "$LOGDIR/meta.log"
  exit 1
fi

echo "Launch stayed alive for ${LAUNCH_SECONDS}s; collecting ROS graph state." | tee -a "$LOGDIR/phase.log"
ros2 node list | sort | tee "$LOGDIR/ros2_node_list.log"
ros2 topic list | sort | tee "$LOGDIR/ros2_topic_list.log"

{
  echo "=== /sc2/visual_servo/target_obs ==="
  ros2 topic info -v /sc2/visual_servo/target_obs || true
  echo
  echo "=== /sc2/visual_servo/target_point ==="
  ros2 topic info -v /sc2/visual_servo/target_point || true
  echo
  echo "=== /sc2/visual_servo/cmd_vel_body ==="
  ros2 topic info -v /sc2/visual_servo/cmd_vel_body || true
  echo
  echo "=== /detections_output ==="
  ros2 topic info -v /detections_output || true
} | tee "$LOGDIR/topic_info_core.log"

timeout 8s ros2 topic hz /sc2/visual_servo/target_obs 2>&1 | tee "$LOGDIR/hz_target_obs.log" || true
timeout 8s ros2 topic hz /sc2/visual_servo/target_point 2>&1 | tee "$LOGDIR/hz_target_point.log" || true
timeout 8s ros2 topic hz /sc2/visual_servo/cmd_vel_body 2>&1 | tee "$LOGDIR/hz_cmd_vel_body.log" || true

ros2 topic echo /sc2/visual_servo/status --once 2>&1 | tee "$LOGDIR/echo_status_once.log" || true
ros2 topic echo /sc2/visual_servo/cmd_vel_body --once 2>&1 | tee "$LOGDIR/echo_cmd_vel_once.log" || true

echo "ROS graph collection complete." | tee -a "$LOGDIR/phase.log"
echo "Logs written to: $LOGDIR" | tee -a "$LOGDIR/meta.log"
