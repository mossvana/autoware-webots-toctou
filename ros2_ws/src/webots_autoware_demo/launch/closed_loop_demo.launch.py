import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    control_launch = os.path.join(
        get_package_share_directory("webots_autoware_demo"),
        "launch",
        "autoware_control.launch.py",
    )

    adapter = Node(
        package="webots_autoware_demo",
        executable="vehicle_adapter",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    trajectory = Node(
        package="webots_autoware_demo",
        executable="straight_trajectory_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    # Give the adapter time to switch the Webots bridge from P to D before
    # the follower can publish a non-zero speed command.
    controller = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(control_launch))
        ],
    )
    return LaunchDescription([adapter, trajectory, controller])
