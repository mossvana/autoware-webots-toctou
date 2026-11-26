import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context):
    attack_point = LaunchConfiguration("attack_point").perform(context)
    if attack_point not in ("before_gate", "after_gate"):
        raise RuntimeError("attack_point must be 'before_gate' or 'after_gate'")

    hold_count = int(LaunchConfiguration("hold_count").perform(context))
    replay_period = float(LaunchConfiguration("replay_period").perform(context))
    result_csv = LaunchConfiguration("result_csv").perform(context)

    share = get_package_share_directory("webots_autoware_demo")
    planner_launch = os.path.join(share, "launch", "obstacle_planner_only.launch.py")
    controller_launch = os.path.join(share, "launch", "autoware_control.launch.py")
    vehicle_info = os.path.join(share, "config", "vehicle_info.param.yaml")
    gate_params = os.path.join(share, "config", "vehicle_cmd_gate.param.yaml")

    if attack_point == "before_gate":
        attack_input = "/control/trajectory_follower/control_cmd"
        gate_auto_input = "/attack/control_cmd"
        adapter_input = "/control/command/control_cmd"
    else:
        attack_input = "/control/command/control_cmd"
        gate_auto_input = "/control/trajectory_follower/control_cmd"
        adapter_input = "/attack/control_cmd"

    planner = IncludeLaunchDescription(PythonLaunchDescriptionSource(planner_launch))
    adapter = Node(
        package="webots_autoware_demo",
        executable="vehicle_adapter",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "control_topic": adapter_input,
            "publish_demo_engage": True,
        }],
    )
    attack = Node(
        package="webots_autoware_demo",
        executable="n_command_delay",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "input_topic": attack_input,
            "output_topic": "/attack/control_cmd",
            "attack_point": attack_point,
            "hold_count": hold_count,
            "replay_period": replay_period,
        }],
    )
    evaluator = Node(
        package="webots_autoware_demo",
        executable="n_command_evaluator",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "attack_point": attack_point,
            "hold_count": hold_count,
            "result_csv": result_csv,
        }],
    )
    probe = Node(
        package="webots_autoware_demo",
        executable="trajectory_probe",
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
                    ("input/auto/control_cmd", gate_auto_input),
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
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(controller_launch))],
    )
    return [planner, adapter, attack, evaluator, probe, gate, controller]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("attack_point", default_value="before_gate"),
        DeclareLaunchArgument("hold_count", default_value="30"),
        DeclareLaunchArgument("replay_period", default_value="0.03"),
        DeclareLaunchArgument("result_csv", default_value="n_command_result.csv"),
        OpaqueFunction(function=launch_setup),
    ])
