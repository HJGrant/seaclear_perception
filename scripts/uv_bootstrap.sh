#!/usr/bin/env bash
set -euo pipefail

WITH_MPPI=0
if [[ "${1:-}" == "--with-mppi" ]]; then
  WITH_MPPI=1
fi

UV_BIN="$("$(dirname "$0")/ensure_uv.sh")"
export PATH="$(dirname "$UV_BIN"):$PATH"

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
  "$UV_BIN" python install 3.10
fi

# Recreate deterministically when a stale or partially-created venv already exists.
"$UV_BIN" venv .venv --clear --python 3.10 --system-site-packages

if [[ "$WITH_MPPI" -eq 1 ]]; then
  "$UV_BIN" sync --python .venv/bin/python --group dev --group mppi
else
  "$UV_BIN" sync --python .venv/bin/python --group dev
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
