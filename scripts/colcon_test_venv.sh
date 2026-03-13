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

# Re-source runtime environments before direct pytest runs.
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

# Ensure installed Python packages are importable from merged install.
export PYTHONPATH="$PWD/install/lib/python3.10/site-packages:${PYTHONPATH:-}"

# Guard check: fail early if package import wiring is broken.
.venv/bin/python -c "import sc2_visual_servo_controller, sc2_visual_servo_perception"

# Explicit unit test file list (no discovery dependency).
.venv/bin/python -m pytest -q \
  src/sc2_visual_servo_controller/test/test_controller_sign.py \
  src/sc2_visual_servo_perception/test/test_conversion.py \
  src/sc2_visual_servo_perception/test/test_target_observer_integration.py \
  src/sc2_visual_servo_perception/test/test_grasp_pose_estimator_integration.py
