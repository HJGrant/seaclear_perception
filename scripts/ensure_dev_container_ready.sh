#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

WITH_MPPI=0
FORCE_BOOTSTRAP=0

for arg in "$@"; do
  case "$arg" in
    --with-mppi)
      WITH_MPPI=1
      ;;
    --force-bootstrap)
      FORCE_BOOTSTRAP=1
      ;;
    *)
      echo "Unknown argument: $arg"
      echo "Usage: $0 [--with-mppi] [--force-bootstrap]"
      exit 1
      ;;
  esac
done

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found. Run this inside the ROS2 Humble container."
  exit 1
fi

source "$ROOT/scripts/source_ros_env_safe.sh"

need_bootstrap="$FORCE_BOOTSTRAP"

if [[ ! -x .venv/bin/python ]]; then
  need_bootstrap=1
fi

if [[ "$need_bootstrap" -eq 0 ]]; then
  if ! .venv/bin/python - <<'PY' >/dev/null 2>&1
import importlib
for module in ("rclpy", "cv_bridge"):
    importlib.import_module(module)
PY
  then
    need_bootstrap=1
  fi
fi

if [[ "$need_bootstrap" -eq 1 ]]; then
  echo "Preparing .venv with uv_bootstrap.sh ..."
  if [[ "$WITH_MPPI" -eq 1 ]]; then
    "$ROOT/scripts/uv_bootstrap.sh" --with-mppi
  else
    "$ROOT/scripts/uv_bootstrap.sh"
  fi
else
  echo "Existing .venv is usable; skipping uv_bootstrap.sh"
  .venv/bin/python -c 'import sys; print(sys.executable)'
fi
