import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    demo_share = get_package_share_directory("webots_autoware_demo")
    controller_launch = os.path.join(
        demo_share, "launch", "autoware_control.launch.py"
    )
    vehicle_info = os.path.join(demo_share, "config", "vehicle_info.param.yaml")
    gate_params = os.path.join(demo_share, "config", "vehicle_cmd_gate.param.yaml")

    adapter = Node(
        package="webots_autoware_demo",
        executable="vehicle_adapter",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "control_topic": "/control/command/control_cmd",
                "publish_demo_engage": True,
            }
        ],
    )
    trajectory = Node(
        package="webots_autoware_demo",
        executable="straight_trajectory_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    gate = ComposableNodeContainer(
        name="gate_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="autoware_vehicle_cmd_gate",
                plugin="autoware::vehicle_cmd_gate::VehicleCmdGate",
                name="vehicle_cmd_gate",
                namespace="/control",
                parameters=[gate_params, vehicle_info],
                remappings=[
                    ("input/auto/control_cmd", "/control/trajectory_follower/control_cmd"),
                    ("input/steering", "/vehicle/status/steering_status"),
                    ("input/acceleration", "/localization/acceleration"),
                    ("input/operation_mode", "/system/operation_mode/state"),
                    ("input/engage", "/autoware/engage"),
                    ("output/control_cmd", "/control/command/control_cmd"),
                ],
            )
        ],
    )
    controller = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(controller_launch))
        ],
    )
    return LaunchDescription([adapter, trajectory, gate, controller])
