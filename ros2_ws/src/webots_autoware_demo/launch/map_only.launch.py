import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    map_dir = os.path.join(get_package_share_directory("webots_autoware_demo"), "map")
    map_file = os.path.join(map_dir, "lanelet2_map.osm")
    projector_file = os.path.join(map_dir, "map_projector_info.yaml")

    projection = Node(
        package="autoware_map_projection_loader",
        executable="autoware_map_projection_loader_node",
        namespace="map",
        name="map_projection_loader",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "map_projector_info_path": projector_file,
                "lanelet2_map_path": map_file,
            }
        ],
    )
    lanelet = Node(
        package="autoware_map_loader",
        executable="autoware_lanelet2_map_loader",
        namespace="map",
        name="lanelet2_map_loader",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "lanelet2_map_path": map_file,
                "lanelet2_map_metadata_path": "",
                "center_line_resolution": 0.2,
                "use_waypoints": False,
                "allow_unsupported_version": True,
                "enable_selected_map_loading": False,
            }
        ],
    )
    return LaunchDescription([projection, lanelet])
