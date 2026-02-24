#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WITH_MAVROS=0

if [[ "${1:-}" == "--with-mavros" ]]; then
  WITH_MAVROS=1
fi

if [[ -f "${SCRIPT_DIR}/source_ros_env_safe.sh" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/source_ros_env_safe.sh"
else
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ERROR: /opt/ros/humble/setup.bash not found."
    exit 1
  fi
  : "${COLCON_TRACE:=}"
  : "${AMENT_TRACE_SETUP_FILES:=}"
  : "${AMENT_PYTHON_EXECUTABLE:=}"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 CLI not found after sourcing ROS environment."
  exit 1
fi

has_mavros_msgs() {
  if ros2 interface show mavros_msgs/msg/OverrideRCIn >/dev/null 2>&1; then
    return 0
  fi
  if python3 -c "import mavros_msgs" >/dev/null 2>&1; then
    return 0
  fi
  return 1
}

if has_mavros_msgs; then
  echo "ROS apt deps check: mavros_msgs already available."
  exit 0
fi

if [[ "${SC2_SKIP_APT:-0}" == "1" ]]; then
  echo "ERROR: mavros_msgs is missing and SC2_SKIP_APT=1 is set."
  echo "Install manually: sudo apt-get install -y ros-humble-mavros-msgs"
  exit 1
fi

if ! command -v sudo >/dev/null 2>&1; then
  echo "ERROR: sudo not found; cannot install ros-humble-mavros-msgs automatically."
  echo "Install manually: apt-get update --allow-releaseinfo-change && apt-get install -y ros-humble-mavros-msgs"
  exit 1
fi

echo "Installing required ROS apt package: ros-humble-mavros-msgs"
sudo apt-get update --allow-releaseinfo-change
sudo apt-get install -y ros-humble-mavros-msgs

if [[ "${SC2_INSTALL_MAVROS:-0}" == "1" || "${WITH_MAVROS}" == "1" ]]; then
  echo "Optional install enabled: ros-humble-mavros"
  sudo apt-get install -y ros-humble-mavros
fi

if ! has_mavros_msgs; then
  echo "ERROR: mavros_msgs is still missing after apt installation."
  echo "Run checks:"
  echo "  ros2 interface show mavros_msgs/msg/OverrideRCIn"
  echo "  python3 -c \"import mavros_msgs\""
  echo "If this persists, restart the shell/container and rerun this script."
  exit 1
fi

echo "ROS apt deps check: mavros_msgs is available."
