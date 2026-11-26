#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$repo_root/scripts/check_environment.sh"

for command_name in git rosdep colcon; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "ERROR: missing command '$command_name'" >&2
    echo "Install git, python3-rosdep, and python3-colcon-common-extensions." >&2
    exit 1
  fi
done

clone_sparse() {
  local relative_path="$1"
  local url="$2"
  local revision="$3"
  shift 3
  local destination="$repo_root/ros2_ws/src/$relative_path"

  if [[ ! -d "$destination/.git" ]]; then
    mkdir -p "$(dirname "$destination")"
    git clone --filter=blob:none --no-checkout "$url" "$destination"
  fi

  git -C "$destination" sparse-checkout init --cone
  git -C "$destination" sparse-checkout set "$@"
  git -C "$destination" fetch --filter=blob:none origin "$revision"
  git -C "$destination" checkout --detach "$revision"
}

# Only packages in the dependency closure of the demo and the six safety
# packages are checked out. The revisions match dependencies/autoware.repos.
clone_sparse core/autoware_adapi_msgs \
  https://github.com/autowarefoundation/autoware_adapi_msgs.git \
  35a7aeb97dfbbbc363c49fc032ded0c8ba172e24 \
  autoware_adapi_v1_msgs autoware_adapi_version_msgs

clone_sparse core/autoware_cmake_repo \
  https://github.com/autowarefoundation/autoware_cmake.git \
  7796130d9fe9e45d8ec5e24815db9364dc4300f0 \
  autoware_cmake autoware_lint_common

clone_sparse core/autoware_core \
  https://github.com/autowarefoundation/autoware_core.git \
  384bcd47e6d2fd21e5954a915153aa5303b34a53 \
  api/autoware_adapi_specs \
  common/autoware_agnocast_wrapper \
  common/autoware_component_interface_specs \
  common/autoware_component_interface_utils \
  common/autoware_geography_utils \
  common/autoware_interpolation \
  common/autoware_lanelet2_utils \
  common/autoware_motion_utils \
  common/autoware_object_recognition_utils \
  common/autoware_osqp_interface \
  common/autoware_point_types \
  common/autoware_signal_processing \
  common/autoware_vehicle_info_utils \
  map/autoware_map_loader \
  map/autoware_map_projection_loader \
  planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module \
  planning/motion_velocity_planner/autoware_motion_velocity_planner \
  planning/motion_velocity_planner/autoware_motion_velocity_planner_common

clone_sparse core/autoware_internal_msgs \
  https://github.com/autowarefoundation/autoware_internal_msgs.git \
  4144effa727d194554ad78d343683201bbbbc427 \
  autoware_internal_debug_msgs \
  autoware_internal_localization_msgs \
  autoware_internal_metric_msgs \
  autoware_internal_msgs \
  autoware_internal_perception_msgs \
  autoware_internal_planning_msgs

clone_sparse core/autoware_lanelet2_extension_repo \
  https://github.com/autowarefoundation/autoware_lanelet2_extension.git \
  b1df9ed04d361562b08b1a0ac33451c2031b8f8c \
  autoware_lanelet2_extension

clone_sparse core/autoware_msgs \
  https://github.com/autowarefoundation/autoware_msgs.git \
  bb8e7bf5d97168663e0e7b357929e4fcdbd3a967 \
  autoware_common_msgs \
  autoware_control_msgs \
  autoware_localization_msgs \
  autoware_map_msgs \
  autoware_perception_msgs \
  autoware_planning_msgs \
  autoware_system_msgs \
  autoware_vehicle_msgs

clone_sparse core/autoware_utils \
  https://github.com/autowarefoundation/autoware_utils.git \
  9cb6f102ab4fce01a1ba1b847c9a57e822184268 \
  autoware_utils \
  autoware_utils_debug \
  autoware_utils_diagnostics \
  autoware_utils_geometry \
  autoware_utils_logging \
  autoware_utils_math \
  autoware_utils_pcl \
  autoware_utils_rclcpp \
  autoware_utils_system \
  autoware_utils_tf \
  autoware_utils_uuid \
  autoware_utils_visualization

clone_sparse universe/autoware_universe \
  https://github.com/autowarefoundation/autoware_universe.git \
  815d3f1b5c532eaccab33f0479819af8535919d2 \
  common/autoware_component_interface_specs_universe \
  common/autoware_fake_test_node \
  common/tier4_api_utils \
  control/autoware_autonomous_emergency_braking \
  control/autoware_control_validator \
  control/autoware_mpc_lateral_controller \
  control/autoware_operation_mode_transition_manager \
  control/autoware_pid_longitudinal_controller \
  control/autoware_pure_pursuit \
  control/autoware_trajectory_follower_base \
  control/autoware_trajectory_follower_node \
  control/autoware_vehicle_cmd_gate \
  system/autoware_diagnostic_graph_aggregator \
  system/autoware_mrm_emergency_stop_operator \
  system/autoware_mrm_handler

clone_sparse universe/external/tier4_autoware_msgs \
  https://github.com/tier4/tier4_autoware_msgs.git \
  ed0b9e86ff72598178c9030ec1b52515ef98d69a \
  tier4_control_msgs \
  tier4_debug_msgs \
  tier4_external_api_msgs \
  tier4_metric_msgs \
  tier4_planning_msgs \
  tier4_system_msgs \
  tier4_vehicle_msgs

set +u
source /opt/ros/jazzy/setup.bash
set -u
rosdep update
rosdep install \
  --from-paths "$repo_root/ros2_ws/src" \
  --ignore-src \
  --rosdistro jazzy \
  -y

echo "Sparse Autoware sources and system dependencies are ready. Run scripts/build.sh next."
