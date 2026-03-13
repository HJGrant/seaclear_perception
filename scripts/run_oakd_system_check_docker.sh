#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PROFILE="no_hw"
LOGDIR=""
LAUNCH_SECONDS=12
SKIP_BUILD=0
SKIP_BOOTSTRAP=0
CONTAINER_NAME=""
START_CONTAINER=1

if [[ -f /opt/ros/humble/setup.bash ]] && ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: run_oakd_system_check_docker.sh is a host-side wrapper and should not be run inside the ROS container."
  echo "Run this inside the container instead:"
  echo "  cd /workspaces/oakd_isaac_ros"
  echo "  ./scripts/run_oakd_system_check.sh --profile no_hw"
  exit 1
fi

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
    --container-name)
      CONTAINER_NAME="$2"
      shift 2
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --skip-bootstrap)
      SKIP_BOOTSTRAP=1
      shift
      ;;
    --no-start-container)
      START_CONTAINER=0
      shift
      ;;
    *)
      echo "Unknown argument: $1"
      echo "Usage: $0 [--profile no_hw|jetson_live] [--logdir DIR] [--launch-seconds N] [--container-name NAME] [--skip-build] [--skip-bootstrap] [--no-start-container]"
      exit 1
      ;;
  esac
done

detect_container_name() {
  if [[ -n "$CONTAINER_NAME" ]]; then
    echo "$CONTAINER_NAME"
    return 0
  fi

  local suffix=""
  if [[ -f "$ROOT/.isaac_ros_common-config" ]]; then
    # shellcheck disable=SC1090
    source "$ROOT/.isaac_ros_common-config"
    suffix="${CONFIG_CONTAINER_NAME_SUFFIX:-}"
  fi
  if [[ -f "$HOME/.isaac_ros_common-config" ]]; then
    # shellcheck disable=SC1090
    source "$HOME/.isaac_ros_common-config"
    suffix="${CONFIG_CONTAINER_NAME_SUFFIX:-$suffix}"
  fi

  local arch
  arch="$(uname -m)"
  local base="isaac_ros_dev-$arch"
  if [[ -n "$suffix" ]]; then
    base="$base-$suffix"
  fi
  echo "${base}-container"
}

CONTAINER_NAME="$(detect_container_name)"

if ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  if [[ "$START_CONTAINER" -eq 0 ]]; then
    echo "ERROR: container '$CONTAINER_NAME' is not running."
    echo "Start it first with src/isaac_ros_common/scripts/run_dev.sh"
    exit 1
  fi

  echo "Starting Isaac ROS dev container: $CONTAINER_NAME"
  "$ROOT/src/isaac_ros_common/scripts/run_dev.sh" \
    --isaac_ros_dev_dir "$ROOT" \
    /bin/true
fi

docker exec -it -u admin --workdir /workspaces/oakd_isaac_ros \
  "$CONTAINER_NAME" \
  bash -lc "$(
    printf './scripts/run_oakd_system_check.sh --profile %q ' "$PROFILE"
    if [[ -n "$LOGDIR" ]]; then
      printf -- '--logdir %q ' "$LOGDIR"
    fi
    printf -- '--launch-seconds %q ' "$LAUNCH_SECONDS"
    if [[ "$SKIP_BUILD" -eq 1 ]]; then
      printf -- '--skip-build '
    fi
    if [[ "$SKIP_BOOTSTRAP" -eq 1 ]]; then
      printf -- '--skip-bootstrap '
    fi
  )"
