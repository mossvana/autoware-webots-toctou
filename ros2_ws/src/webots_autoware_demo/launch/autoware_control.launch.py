import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    demo_share = get_package_share_directory("webots_autoware_demo")
    mpc_share = get_package_share_directory("autoware_mpc_lateral_controller")
    pid_share = get_package_share_directory("autoware_pid_longitudinal_controller")

    controller_params = os.path.join(demo_share, "config", "controller.param.yaml")
    vehicle_params = os.path.join(demo_share, "config", "vehicle_info.param.yaml")
    mpc_params = os.path.join(mpc_share, "param", "lateral_controller_defaults.param.yaml")
    pid_params = os.path.join(
        pid_share, "config", "autoware_pid_longitudinal_controller.param.yaml"
    )

    controller = Node(
        package="autoware_trajectory_follower_node",
        executable="controller_node_exe",
        namespace="control/trajectory_follower",
        name="controller_node",
        output="screen",
        parameters=[controller_params, vehicle_params, mpc_params, pid_params],
        remappings=[
            ("~/input/reference_trajectory", "/planning/trajectory"),
            ("~/input/current_odometry", "/localization/kinematic_state"),
            ("~/input/current_steering", "/vehicle/status/steering_status"),
            ("~/input/current_accel", "/localization/acceleration"),
            ("~/input/current_operation_mode", "/system/operation_mode/state"),
            ("~/output/control_cmd", "/control/trajectory_follower/control_cmd"),
        ],
    )

    return LaunchDescription([controller])
