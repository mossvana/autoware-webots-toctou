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

    config = lambda name: os.path.join(share, "config", name)
    graph_share = get_package_share_directory("autoware_diagnostic_graph_aggregator")

    if attack_point == "before_gate":
        attack_input = "/control/trajectory_follower/control_cmd"
        gate_auto_input = "/attack/control_cmd"
        adapter_input = "/control/command/control_cmd"
    else:
        attack_input = "/control/command/control_cmd"
        gate_auto_input = "/control/trajectory_follower/control_cmd"
        adapter_input = "/attack/control_cmd"

    agnocast_env = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("autoware_agnocast_wrapper"),
                "launch",
                "agnocast_env.launch.py",
            )
        )
    )
    common_env = {"LD_PRELOAD": LaunchConfiguration("ld_preload_value")}

    planner = IncludeLaunchDescription(PythonLaunchDescriptionSource(planner_launch))
    adapter = Node(
        package="webots_autoware_demo",
        executable="vehicle_adapter",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "control_topic": adapter_input,
            "publish_demo_engage": False,
            "publish_demo_operation_mode": False,
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
            "evaluation_time": 4.0,
        }],
    )
    probe = Node(
        package="webots_autoware_demo",
        executable="trajectory_probe",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    gate = ComposableNodeContainer(
        name="safety_gate_container",
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
                parameters=[config("vehicle_cmd_gate_safety.param.yaml"), vehicle_info],
                remappings=[
                    ("input/auto/control_cmd", gate_auto_input),
                    ("input/steering", "/vehicle/status/steering_status"),
                    ("input/acceleration", "/localization/acceleration"),
                    ("input/operation_mode", "/system/operation_mode/state"),
                    ("input/engage", "/autoware/engage"),
                    ("input/mrm_state", "/system/fail_safe/mrm_state"),
                    ("input/emergency/control_cmd", "/system/emergency/control_cmd"),
                    ("input/emergency/gear_cmd", "/system/emergency/gear_cmd"),
                    (
                        "input/emergency/turn_indicators_cmd",
                        "/system/emergency/turn_indicators_cmd",
                    ),
                    (
                        "input/emergency/hazard_lights_cmd",
                        "/system/emergency/hazard_lights_cmd",
                    ),
                    ("output/control_cmd", "/control/command/control_cmd"),
                    (
                        "output/vehicle_cmd_emergency",
                        "/control/command/vehicle_emergency",
                    ),
                    (
                        "output/operation_mode",
                        "/control/vehicle_cmd_gate/operation_mode",
                    ),
                    ("output/gear_cmd", "/control/command/gear_cmd"),
                ],
            )
        ],
    )

    control_validator = Node(
        package="autoware_control_validator",
        executable="autoware_control_validator_node",
        name="control_validator",
        output="screen",
        additional_env=common_env,
        parameters=[
            config("safety_control_validator.param.yaml"),
            vehicle_info,
            {"use_sim_time": True},
        ],
        remappings=[
            ("~/input/control_cmd", "/control/command/control_cmd"),
            ("~/input/kinematics", "/localization/kinematic_state"),
            ("~/input/operational_mode_state", "/system/operation_mode/state"),
            ("~/input/measured_acceleration", "/localization/acceleration"),
            ("~/input/reference_trajectory", "/planning/trajectory"),
            (
                "~/input/predicted_trajectory",
                "/control/trajectory_follower/lateral/predicted_trajectory",
            ),
        ],
    )

    aeb = Node(
        package="autoware_autonomous_emergency_braking",
        executable="autoware_autonomous_emergency_braking",
        name="autonomous_emergency_braking",
        output="screen",
        additional_env=common_env,
        parameters=[config("safety_aeb.param.yaml"), vehicle_info, {"use_sim_time": True}],
        remappings=[
            ("~/input/pointcloud", "/sensing/lidar/no_ground/pointcloud"),
            ("~/input/velocity", "/vehicle/status/velocity_status"),
            ("~/input/imu", "/sensing/imu/imu_data"),
            ("~/input/odometry", "/localization/kinematic_state"),
            (
                "~/input/predicted_trajectory",
                "/control/trajectory_follower/lateral/predicted_trajectory",
            ),
            ("~/input/objects", "/perception/object_recognition/objects"),
        ],
    )

    operation_mode = Node(
        package="autoware_operation_mode_transition_manager",
        executable="autoware_operation_mode_transition_manager_exe",
        name="autoware_operation_mode_transition_manager",
        output="screen",
        parameters=[
            config("safety_operation_mode.param.yaml"),
            vehicle_info,
            {"use_sim_time": True},
        ],
        remappings=[
            ("kinematics", "/localization/kinematic_state"),
            ("steering", "/vehicle/status/steering_status"),
            ("trajectory", "/planning/trajectory"),
            ("control_cmd", "/control/command/control_cmd"),
            (
                "trajectory_follower_control_cmd",
                "/control/trajectory_follower/control_cmd",
            ),
            ("control_mode_report", "/vehicle/status/control_mode"),
            ("gate_operation_mode", "/control/vehicle_cmd_gate/operation_mode"),
            ("is_autonomous_available", "/control/is_autonomous_available"),
            ("control_mode_request", "/control/control_mode_request"),
        ],
    )

    aggregator = Node(
        package="autoware_diagnostic_graph_aggregator",
        executable="aggregator_node",
        name="diagnostic_graph_aggregator",
        output="screen",
        additional_env=common_env,
        parameters=[
            os.path.join(graph_share, "config", "default.param.yaml"),
            {
                "use_sim_time": True,
                "graph_file": config("safety_diagnostic_graph.yaml"),
                "graph_vars": "",
            },
        ],
        remappings=[
            ("~/diagnostics", "/diagnostics"),
            ("~/availability", "/system/command_mode/availability"),
        ],
    )
    converter = Node(
        package="autoware_diagnostic_graph_aggregator",
        executable="converter_node",
        name="operation_mode_availability_converter",
        output="screen",
        additional_env=common_env,
        parameters=[
            os.path.join(graph_share, "config", "converter.param.yaml"),
            {"use_sim_time": True},
        ],
        remappings=[
            ("~/command_mode/availability", "/system/command_mode/availability"),
            ("~/operation_mode/availability", "/system/operation_mode/availability"),
        ],
    )

    mrm_handler = Node(
        package="autoware_mrm_handler",
        executable="autoware_mrm_handler_node",
        name="mrm_handler",
        output="screen",
        additional_env=common_env,
        parameters=[config("safety_mrm_handler.param.yaml"), {"use_sim_time": True}],
        remappings=[
            ("~/input/operation_mode_availability", "/system/operation_mode/availability"),
            ("~/input/odometry", "/localization/kinematic_state"),
            ("~/input/control_mode", "/vehicle/status/control_mode"),
            ("~/input/mrm/emergency_stop/status", "/system/mrm/emergency_stop/status"),
            ("~/input/api/operation_mode/state", "/system/operation_mode/state"),
            ("~/input/gear", "/control/command/gear_cmd"),
            ("~/output/gear", "/system/emergency/gear_cmd"),
            ("~/output/turn_indicators", "/system/emergency/turn_indicators_cmd"),
            ("~/output/hazard", "/system/emergency/hazard_lights_cmd"),
            ("~/output/mrm/state", "/system/fail_safe/mrm_state"),
            ("~/output/emergency_holding", "/system/emergency_holding"),
            ("~/output/mrm/emergency_stop/operate", "/system/mrm/emergency_stop/operate"),
        ],
    )
    emergency_operator = Node(
        package="autoware_mrm_emergency_stop_operator",
        executable="autoware_mrm_emergency_stop_operator_node",
        name="mrm_emergency_stop_operator",
        output="screen",
        additional_env=common_env,
        parameters=[
            config("safety_mrm_emergency_stop.param.yaml"),
            {"use_sim_time": True},
        ],
        remappings=[
            ("~/input/mrm/emergency_stop/operate", "/system/mrm/emergency_stop/operate"),
            ("~/input/control/control_cmd", "/control/command/control_cmd"),
            ("~/output/mrm/emergency_stop/status", "/system/mrm/emergency_stop/status"),
            ("~/output/mrm/emergency_stop/control_cmd", "/system/emergency/control_cmd"),
        ],
    )

    controller = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(controller_launch))],
    )
    bootstrap = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="webots_autoware_demo",
                executable="safety_mode_bootstrap",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    return [
        agnocast_env,
        planner,
        adapter,
        attack,
        evaluator,
        probe,
        gate,
        control_validator,
        aeb,
        operation_mode,
        aggregator,
        converter,
        mrm_handler,
        emergency_operator,
        controller,
        bootstrap,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("attack_point", default_value="before_gate"),
        DeclareLaunchArgument("hold_count", default_value="30"),
        DeclareLaunchArgument("replay_period", default_value="0.03"),
        DeclareLaunchArgument("result_csv", default_value="n_command_safety_result.csv"),
        OpaqueFunction(function=launch_setup),
    ])
