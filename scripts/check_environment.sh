#!/usr/bin/env bash
set -euo pipefail

if [[ ! -r /etc/os-release ]]; then
  echo "ERROR: /etc/os-release not found" >&2
  exit 1
fi

source /etc/os-release
if [[ "${VERSION_ID:-}" != "24.04" ]]; then
  echo "ERROR: Ubuntu 24.04 is required; detected ${PRETTY_NAME:-unknown}" >&2
  exit 1
fi

if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: ROS 2 Jazzy is not installed at /opt/ros/jazzy" >&2
  exit 1
fi

if ! command -v webots >/dev/null 2>&1; then
  echo "ERROR: Webots is not on PATH" >&2
  exit 1
fi

webots_version="$(timeout 10s webots --version 2>&1 || true)"
if [[ "$webots_version" != *"R2025a"* ]]; then
  echo "ERROR: Webots R2025a is required; detected: $webots_version" >&2
  exit 1
fi

echo "Environment OK: ${PRETTY_NAME}, ROS 2 Jazzy, Webots R2025a, $(uname -m)"
