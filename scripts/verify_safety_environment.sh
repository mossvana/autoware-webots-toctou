#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="${AUTOWARE_TOCTOU_WS:-$repo_root/ros2_ws}"

set +u
source /opt/ros/jazzy/setup.bash
source "$workspace_root/install/setup.bash"
set -u

safety_packages=(
  autoware_control_validator
  autoware_autonomous_emergency_braking
  autoware_operation_mode_transition_manager
  autoware_diagnostic_graph_aggregator
  autoware_mrm_handler
  autoware_mrm_emergency_stop_operator
)

for package_name in "${safety_packages[@]}"; do
  prefix="$(python3 -c \
    'import sys; from ament_index_python.packages import get_package_prefix; print(get_package_prefix(sys.argv[1]))' \
    "$package_name")"
  echo "OK: $package_name -> $prefix"
done

universe="$workspace_root/src/universe/autoware_universe"
expected_revision="815d3f1b5c532eaccab33f0479819af8535919d2"
actual_revision="$(git -C "$universe" rev-parse HEAD)"
if [[ "$actual_revision" != "$expected_revision" ]]; then
  echo "ERROR: unexpected autoware_universe revision: $actual_revision" >&2
  exit 1
fi

sparse_paths="$(git -C "$universe" sparse-checkout list)"
for sparse_path in \
  control/autoware_control_validator \
  control/autoware_autonomous_emergency_braking \
  control/autoware_operation_mode_transition_manager \
  system/autoware_diagnostic_graph_aggregator \
  system/autoware_mrm_handler \
  system/autoware_mrm_emergency_stop_operator; do
  if ! rg -Fxq "$sparse_path" <<< "$sparse_paths"; then
    echo "ERROR: missing sparse path: $sparse_path" >&2
    exit 1
  fi
done

echo "Safety environment OK: six pinned Autoware safety packages are installed."
