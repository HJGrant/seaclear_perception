#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${PKG_DIR}/../../.." && pwd)"

PARAMS_FILE="${PKG_DIR}/config/oak_d/oakd_rgbd.yaml"
ENABLE_VSLAM="true"
IFACE="eno1"
DURATION_SEC="0"
LOG_ROOT="${WS_ROOT}/logs/oakd_diag"
RESTART_DAEMON="true"

if command -v rg >/dev/null 2>&1; then
  FILTER_I_CMD="rg -i"
  FILTER_Q_CMD="rg -q"
else
  FILTER_I_CMD="grep -Ei"
  FILTER_Q_CMD="grep -Eq"
fi

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]

Options:
  --params-file PATH      Camera yaml passed to launch (default: ${PARAMS_FILE})
  --enable-vslam BOOL     true/false, forwarded to launch (default: ${ENABLE_VSLAM})
  --iface NAME            NIC interface for counters (default: ${IFACE})
  --duration-sec N        Stop after N seconds, 0 means run until Ctrl-C (default: ${DURATION_SEC})
  --log-root PATH         Base directory for logs (default: ${LOG_ROOT})
  --restart-daemon BOOL   true/false restart ros2 daemon before run (default: ${RESTART_DAEMON})
  -h, --help              Show this help
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --params-file) PARAMS_FILE="$2"; shift 2 ;;
    --enable-vslam) ENABLE_VSLAM="$2"; shift 2 ;;
    --iface) IFACE="$2"; shift 2 ;;
    --duration-sec) DURATION_SEC="$2"; shift 2 ;;
    --log-root) LOG_ROOT="$2"; shift 2 ;;
    --restart-daemon) RESTART_DAEMON="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ ! -f "${PARAMS_FILE}" ]]; then
  echo "params file not found: ${PARAMS_FILE}" >&2
  exit 1
fi

mkdir -p "${LOG_ROOT}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${LOG_ROOT}/${RUN_ID}"
SNAPSHOT_DIR="${RUN_DIR}/snapshots"
mkdir -p "${RUN_DIR}" "${SNAPSHOT_DIR}"

PIDS=()

cleanup() {
  set +e
  echo "[cleanup] stopping background processes" | tee -a "${RUN_DIR}/meta.log"
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
  wait >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

log_cmd() {
  echo "[$(date -Iseconds)] $*" >> "${RUN_DIR}/meta.log"
}

start_bg() {
  local name="$1"
  shift
  log_cmd "start ${name}: $*"
  "$@" >"${RUN_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

start_loop() {
  local name="$1"
  local interval="$2"
  shift 2
  log_cmd "start loop ${name}: every ${interval}s :: $*"
  (
    while true; do
      {
        echo "===== $(date -Iseconds) ====="
        "$@"
      } >>"${RUN_DIR}/${name}.log" 2>&1
      sleep "${interval}"
    done
  ) &
  PIDS+=("$!")
}

snapshot_now() {
  local reason="$1"
  local stamp
  stamp="$(date +%Y%m%d_%H%M%S)"
  local out="${SNAPSHOT_DIR}/${stamp}_${reason}.log"

  {
    echo "snapshot_time=${stamp}"
    echo "reason=${reason}"
    echo

    echo "## ros env"
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
    echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
    echo

    echo "## ros2 daemon status"
    ros2 daemon status || true
    echo

    echo "## ros2 node list"
    ros2 node list || true
    echo

    echo "## ros2 topic list"
    ros2 topic list || true
    echo

    echo "## ros2 node list --no-daemon"
    ros2 node list --no-daemon || true
    echo

    echo "## ros2 topic list --no-daemon"
    ros2 topic list --no-daemon || true
    echo

    echo "## ip -s link show ${IFACE}"
    ip -s link show "${IFACE}" || true
    echo

    echo "## ethtool -S ${IFACE} (filtered)"
    ethtool -S "${IFACE}" 2>/dev/null | ${FILTER_I_CMD} "crc|drop|error|miss|timeout|fifo|frame|carrier|overrun|rx_.*err|tx_.*err" || true
    echo

    echo "## dmesg tail"
    dmesg | tail -n 120 || true
    echo

    echo "## launch tail"
    tail -n 200 "${RUN_DIR}/launch.log" || true
  } >"${out}" 2>&1

  echo "[$(date -Iseconds)] snapshot saved: ${out}" >> "${RUN_DIR}/events.log"
}

watch_driver_errors() {
  local pattern='X_LINK_ERROR|ping was missed|Camera diagnostics error: No Data|double free|bad_alloc|corruption'
  tail -n 0 -F "${RUN_DIR}/launch.log" | while read -r line; do
    if echo "${line}" | ${FILTER_Q_CMD} "${pattern}"; then
      echo "[$(date -Iseconds)] trigger: ${line}" >> "${RUN_DIR}/events.log"
      snapshot_now "driver_event"
    fi
  done
}

{
  echo "run_id=${RUN_ID}"
  echo "params_file=${PARAMS_FILE}"
  echo "enable_vslam=${ENABLE_VSLAM}"
  echo "iface=${IFACE}"
  echo "duration_sec=${DURATION_SEC}"
  echo "host=$(hostname)"
  echo "pwd=$(pwd)"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
} > "${RUN_DIR}/run.env"

if [[ "${RESTART_DAEMON}" == "true" ]]; then
  ros2 daemon stop || true
  ros2 daemon start || true
fi
ros2 daemon status > "${RUN_DIR}/ros2_daemon_status_pre.log" 2>&1 || true

start_bg "launch" ros2 launch oakd_isaac_ros oakd_visual_slam.launch.py params_file:="${PARAMS_FILE}" enable_vslam:="${ENABLE_VSLAM}"

# topic monitors
start_bg "hz_left_raw" ros2 topic hz /oakd_camera/left/image_raw
start_bg "hz_right_raw" ros2 topic hz /oakd_camera/right/image_raw
start_bg "hz_left_info" ros2 topic hz /oakd_camera/left/camera_info
start_bg "hz_right_info" ros2 topic hz /oakd_camera/right/camera_info
start_bg "hz_left_rect" ros2 topic hz /oakd_camera/left/image_rect
start_bg "hz_right_rect" ros2 topic hz /oakd_camera/right/image_rect
start_bg "bw_left_raw" ros2 topic bw /oakd_camera/left/image_raw
start_bg "bw_right_raw" ros2 topic bw /oakd_camera/right/image_raw

# system/network monitors
start_bg "dmesg_follow" dmesg -w
start_loop "ip_link_stats" 1 ip -s link show "${IFACE}"
start_loop "ethtool_stats" 1 bash -lc "ethtool -S '${IFACE}' 2>/dev/null | ${FILTER_I_CMD} 'crc|drop|error|miss|timeout|fifo|frame|carrier|overrun|rx_.*err|tx_.*err' || true"

if command -v docker >/dev/null 2>&1; then
  start_loop "docker_stats" 1 docker stats --no-stream --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.NetIO}}\t{{.PIDs}}"
fi

watch_driver_errors > "${RUN_DIR}/error_watch.log" 2>&1 &
PIDS+=("$!")

echo "Logs: ${RUN_DIR}"
echo "Press Ctrl-C to stop."

if [[ "${DURATION_SEC}" != "0" ]]; then
  sleep "${DURATION_SEC}"
  exit 0
fi

wait
