#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UV_ROOT="$ROOT/.tools/uv"
UV_BIN="$UV_ROOT/uv"

if command -v uv >/dev/null 2>&1; then
  command -v uv
  exit 0
fi

if [[ -x "$UV_BIN" ]]; then
  if "$UV_BIN" --version >/dev/null 2>&1; then
    echo "$UV_BIN"
    exit 0
  fi
  echo "Removing incompatible workspace-local uv at $UV_BIN" >&2
  rm -f "$UV_BIN"
fi

mkdir -p "$UV_ROOT"

if ! command -v curl >/dev/null 2>&1; then
  echo "ERROR: curl is required to install uv automatically."
  exit 1
fi

echo "Installing workspace-local uv into $UV_ROOT ..." >&2
curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR="$UV_ROOT" sh >&2

if [[ ! -x "$UV_BIN" ]]; then
  echo "ERROR: uv installation finished but $UV_BIN was not created." >&2
  exit 1
fi

echo "$UV_BIN"
