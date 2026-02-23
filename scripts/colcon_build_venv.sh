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

.venv/bin/python -m colcon build --merge-install "$@"

# Ensure ROS2 launch can resolve Python nodes via package libexec path.
# Some setuptools/pip install paths place console_scripts under install/bin.
INSTALL_ROOT="$(pwd)/install"
ensure_libexec() {
  local pkg="$1"
  local exe="$2"
  local pkg_libexec="${INSTALL_ROOT}/lib/${pkg}"
  local bin_exe="${INSTALL_ROOT}/bin/${exe}"

  mkdir -p "${pkg_libexec}"
  if [[ -x "${bin_exe}" && ! -e "${pkg_libexec}/${exe}" ]]; then
    ln -s "../../bin/${exe}" "${pkg_libexec}/${exe}"
    echo "Created libexec shim: lib/${pkg}/${exe} -> ../../bin/${exe}"
  fi
}

ensure_libexec sc2_visual_servo_controller visual_servo_controller_node
ensure_libexec sc2_visual_servo_controller target_obs_publisher
ensure_libexec sc2_visual_servo_controller fake_vehicle_node
ensure_libexec sc2_visual_servo_controller csv_logger_node
ensure_libexec sc2_visual_servo_controller cmd_vel_to_mavlink_stub
ensure_libexec sc2_visual_servo_perception target_observer_node

echo "Verified libexec directories:"
find "${INSTALL_ROOT}/lib" -maxdepth 2 -type d | grep -E "sc2_visual_servo_(controller|perception|bringup)" || true
