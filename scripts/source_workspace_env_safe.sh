#!/usr/bin/env bash
set -euo pipefail

_sc2_sourced=0
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  _sc2_sourced=1
fi

_sc2_fail() {
  local code="$1"
  shift
  echo "$*"
  if [[ "$_sc2_sourced" -eq 1 ]]; then
    return "$code"
  fi
  exit "$code"
}

if [[ $# -ne 1 ]]; then
  _sc2_fail 1 "Usage: ${BASH_SOURCE[0]} <workspace-setup-bash>"
fi

SETUP_FILE="$1"
if [[ ! -f "$SETUP_FILE" ]]; then
  _sc2_fail 1 "ERROR: workspace setup file not found: $SETUP_FILE"
fi

# Guard variables commonly referenced by colcon/ament setup scripts.
: "${COLCON_TRACE:=}"
: "${COLCON_CURRENT_PREFIX:=}"
: "${COLCON_PYTHON_EXECUTABLE:=}"
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=}"

restore_errexit=0
restore_nounset=0
if [[ $- == *e* ]]; then
  restore_errexit=1
fi
if [[ $- == *u* ]]; then
  restore_nounset=1
fi

set +e
set +u
# shellcheck disable=SC1090
source "$SETUP_FILE"
source_rc=$?

if [[ "$restore_errexit" -eq 1 ]]; then
  set -e
fi
if [[ "$restore_nounset" -eq 1 ]]; then
  set -u
fi

if [[ "$source_rc" -ne 0 ]]; then
  _sc2_fail "$source_rc" "ERROR: sourcing workspace setup failed: $SETUP_FILE"
fi

WORKSPACE_ROOT="$(pwd)"
VENV_PYTHON="${WORKSPACE_ROOT}/.venv/bin/python"
if [[ -x "$VENV_PYTHON" ]]; then
  export PATH="${WORKSPACE_ROOT}/.venv/bin:${PATH}"
  export COLCON_PYTHON_EXECUTABLE="$VENV_PYTHON"
  export AMENT_PYTHON_EXECUTABLE="$VENV_PYTHON"

  VENV_SITE_PACKAGES="$("$VENV_PYTHON" - <<'PY'
import sysconfig
print(sysconfig.get_path("purelib"))
PY
)"
  if [[ -n "$VENV_SITE_PACKAGES" ]]; then
    export PYTHONPATH="${VENV_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"
  fi
fi

unset _sc2_sourced
unset -f _sc2_fail
