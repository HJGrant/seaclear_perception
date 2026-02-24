#!/usr/bin/env bash
set -euo pipefail

WITH_MPPI=0
if [[ "${1:-}" == "--with-mppi" ]]; then
  WITH_MPPI=1
fi

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is not installed. Install: https://docs.astral.sh/uv/getting-started/installation/"
  exit 1
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # Source ROS env with nounset-safe guards.
  # shellcheck disable=SC1091
  source "$(dirname "$0")/source_ros_env_safe.sh"
else
  echo "Warning: /opt/ros/humble/setup.bash not found in this environment."
fi

# Ensure required ROS apt deps for MAVROS rc_override workflow.
if [[ -f /opt/ros/humble/setup.bash ]]; then
  "$(dirname "$0")/ensure_ros_apt_deps.sh"
fi

if ! command -v python3.10 >/dev/null 2>&1; then
  echo "python3.10 not found; attempting to install via uv..."
  uv python install 3.10
fi

uv venv .venv --python 3.10 --system-site-packages

if [[ "$WITH_MPPI" -eq 1 ]]; then
  uv sync --python .venv/bin/python --group dev --group mppi
else
  uv sync --python .venv/bin/python --group dev
fi

mkdir -p .venv
: > .venv/COLCON_IGNORE

echo "Bootstrap complete."

echo "Verify interpreter:"
.venv/bin/python -c 'import sys; print(sys.version); print(sys.executable)'

echo "Verify ROS imports (requires ROS2 Humble + system-site-packages):"
.venv/bin/python - <<'PY'
import importlib
for m in ("rclpy", "cv_bridge"):
    try:
        importlib.import_module(m)
        print(f"ok: {m}")
    except Exception as exc:
        print(f"warn: {m}: {exc}")
PY
