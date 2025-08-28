#!/usr/bin/env bash
set -e

# This script is sourced by the base image entrypoint additions mechanism.
# It starts dbus and avahi-daemon in the foreground/background without systemd.

start_service() {
  local name="$1"
  if command -v service >/dev/null 2>&1; then
    service "$name" start || true
  elif command -v /etc/init.d/$name >/dev/null 2>&1; then
    /etc/init.d/$name start || true
  fi
}

# dbus sometimes needs /run/dbus
mkdir -p /run/dbus || true
start_service dbus
start_service avahi-daemon

true
