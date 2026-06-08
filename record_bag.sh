#!/usr/bin/env bash

set -eo pipefail

BAG_BASE_DIR="${1:-/workspaces/oakd_isaac_ros/rosbags_dubrovnik/no-slew/beams}"

TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
BAG_NAME="ctrl_bag_${TIMESTAMP}-staged_pid_xy-kp_0.05-kd_0.0008-maxvxvy20ms"
BAG_PATH="${BAG_BASE_DIR}/${BAG_NAME}"

TOPICS=(
  /best_detection
  /detections_output
  /oak/rgb/camera_info
  /oak/rgb/image_raw
  /sc2/imu/imu
  /sc2/imu/data_ned
  /sc2/dvl/dvl
  /sc2/dvl/odometry
  /sc2/odometry_ekf
  /waterlinked_dvl_driver/velocity_report
  /sc2/altitude
  /tf
  /tf_static
  /sc2/visual_servo/status
  /sc2/visual_servo/target_locked
  /sc2/visual_servo/centroid_error
  /sc2/visual_servo/target_point
  /sc2/visual_servo/velocity_camera_image_plane_frame
  /sc2/visual_servo/velocity_sc2_base_link
  /sc2/velocity_controller_node/outer/console_signals
)

source /opt/ros/humble/setup.bash

if [ -f /mnt/orin_ssd/workspaces/oakd_isaac_ros/install/setup.bash ]; then
  source /mnt/orin_ssd/workspaces/oakd_isaac_ros/install/setup.bash
elif [ -f /workspaces/oakd_isaac_ros/install/setup.bash ]; then
  source /workspaces/oakd_isaac_ros/install/setup.bash
elif [ -f ./install/setup.bash ]; then
  source ./install/setup.bash
else
  echo "ERROR: install/setup.bash not found"
  exit 1
fi

mkdir -p "$BAG_BASE_DIR"

echo "Recording ROS 2 control bag..."
echo "BAG_PATH: $BAG_PATH"
echo
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo
echo "Topics:"
printf '  %s\n' "${TOPICS[@]}"
echo

echo "Checking visible topics..."
ros2 topic list > /tmp/record_bag_topic_list.txt

MISSING=0
for t in "${TOPICS[@]}"; do
  if ! grep -qx "$t" /tmp/record_bag_topic_list.txt; then
    echo "WARN: topic not currently visible: $t"
    MISSING=1
  fi
done

echo
if [ "$MISSING" -eq 1 ]; then
  echo "Some topics are not visible. ros2 bag may still start, but missing topics will not record until visible."
fi

echo
echo "Press Ctrl+C to stop recording."
echo

ros2 bag record \
  --storage sqlite3 \
  -o "$BAG_PATH" \
  "${TOPICS[@]}"