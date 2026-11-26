#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$repo_root/scripts/check_environment.sh"

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$repo_root/ros2_ws"
colcon build --symlink-install --packages-up-to webots_autoware_demo
colcon test --packages-select webots_autoware_demo --event-handlers console_direct+
colcon test-result --verbose
"$repo_root/scripts/verify_safety_environment.sh"
