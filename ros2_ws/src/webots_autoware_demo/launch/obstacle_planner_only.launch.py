import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('webots_autoware_demo')
    config = lambda name: os.path.join(share, 'config', name)

    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'map_only.launch.py')
        )
    )

    inputs = Node(
        package='webots_autoware_demo',
        executable='planner_test_inputs',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    planner = Node(
        package='autoware_motion_velocity_planner',
        executable='autoware_motion_velocity_planner_node',
        namespace='planning',
        name='motion_velocity_planner',
        output='screen',
        parameters=[
            config('planner_common.param.yaml'),
            os.path.join(
                get_package_share_directory('autoware_velocity_smoother'),
                'config',
                'default_velocity_smoother.param.yaml',
            ),
            config('analytical_velocity_smoother.param.yaml'),
            config('planner_nearest.param.yaml'),
            config('vehicle_info.param.yaml'),
            config('motion_velocity_planner.param.yaml'),
            config('obstacle_stop.param.yaml'),
            config('planner_rc_override.param.yaml'),
            {
                'use_sim_time': True,
                'launch_modules': [
                    'autoware::motion_velocity_planner::ObstacleStopModule'
                ],
            },
        ],
        remappings=[
            ('~/input/trajectory', '/planning/trajectory_raw'),
            ('~/input/vector_map', '/map/vector_map'),
            ('~/input/vehicle_odometry', '/localization/kinematic_state'),
            ('~/input/accel', '/localization/acceleration'),
            ('~/input/dynamic_objects',
             '/perception/object_recognition/objects'),
            ('~/input/no_ground_pointcloud',
             '/sensing/lidar/no_ground/pointcloud'),
            ('~/output/trajectory', '/planning/trajectory'),
        ],
    )

    return LaunchDescription([map_launch, inputs, planner])
