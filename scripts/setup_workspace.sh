#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$repo_root/scripts/check_environment.sh"

for command_name in vcs rosdep colcon; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "ERROR: missing command '$command_name'" >&2
    echo "Install python3-vcstool, python3-rosdep, and python3-colcon-common-extensions." >&2
    exit 1
  fi
done

vcs import "$repo_root/ros2_ws/src" < "$repo_root/dependencies/autoware.repos"

set +u
source /opt/ros/jazzy/setup.bash
set -u
rosdep update
rosdep install \
  --from-paths "$repo_root/ros2_ws/src" \
  --ignore-src \
  --rosdistro jazzy \
  -y

echo "Dependencies imported and installed. Run scripts/build.sh next."
