import math

import rclpy
from autoware_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


STOP_VELOCITY_MPS = 0.01


def first_stop_x(trajectory):
    for point in trajectory.points:
        if abs(float(point.longitudinal_velocity_mps)) <= STOP_VELOCITY_MPS:
            return float(point.pose.position.x)
    return math.nan


class TrajectoryProbe(Node):
    """Observe, but never modify, the LiDAR-to-planned-stop closed loop."""

    def __init__(self):
        super().__init__("trajectory_probe")
        self.raw_stop_x = math.nan
        self.planned_stop_x = math.nan
        self.ego_x = math.nan
        self.point_count = 0
        self.cloud_bounds = None
        self.processed_point_count = 0
        self.processed_cloud_bounds = None
        self.last_summary = None

        self.create_subscription(
            Trajectory, "/planning/trajectory_raw", self.on_raw_trajectory, 10
        )
        self.create_subscription(
            Trajectory, "/planning/trajectory", self.on_planned_trajectory, 10
        )
        self.create_subscription(
            PointCloud2,
            "/sensing/lidar/no_ground/pointcloud",
            self.on_pointcloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/planning/motion_velocity_planner/debug/processed_pointcloud",
            self.on_processed_pointcloud,
            10,
        )
        self.create_subscription(
            Odometry, "/localization/kinematic_state", self.on_odometry, 10
        )
        self.create_timer(0.5, self.report)

    def on_raw_trajectory(self, msg):
        self.raw_stop_x = first_stop_x(msg)

    def on_planned_trajectory(self, msg):
        self.planned_stop_x = first_stop_x(msg)

    def on_pointcloud(self, msg):
        self.point_count = int(msg.width) * int(msg.height)
        points = list(
            point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
        )
        if points:
            xs = [float(point[0]) for point in points]
            ys = [float(point[1]) for point in points]
            zs = [float(point[2]) for point in points]
            self.cloud_bounds = (
                min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)
            )

    def on_odometry(self, msg):
        self.ego_x = float(msg.pose.pose.position.x)

    @staticmethod
    def bounds(msg):
        points = list(
            point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
        )
        if not points:
            return None
        xs = [float(point[0]) for point in points]
        ys = [float(point[1]) for point in points]
        zs = [float(point[2]) for point in points]
        return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)

    def on_processed_pointcloud(self, msg):
        self.processed_point_count = int(msg.width) * int(msg.height)
        self.processed_cloud_bounds = self.bounds(msg)

    def report(self):
        if not all(math.isfinite(value) for value in (self.raw_stop_x, self.planned_stop_x)):
            return
        shift = self.raw_stop_x - self.planned_stop_x
        summary = (
            round(self.raw_stop_x, 3),
            round(self.planned_stop_x, 3),
            round(shift, 3),
            self.point_count,
        )
        if summary == self.last_summary:
            return
        self.last_summary = summary
        obstacle_stop_active = shift > 0.02
        self.get_logger().info(
            "[OBSTACLE STOP] "
            f"active={obstacle_stop_active} raw_stop_x={self.raw_stop_x:.3f}m "
            f"planned_stop_x={self.planned_stop_x:.3f}m shift={shift:.3f}m "
            f"ego_x={self.ego_x:.3f}m lidar_points={self.point_count}"
        )
        if self.cloud_bounds is not None:
            min_x, max_x, min_y, max_y, min_z, max_z = self.cloud_bounds
            self.get_logger().info(
                "[LIDAR FRAME] "
                f"x=[{min_x:.3f},{max_x:.3f}] "
                f"y=[{min_y:.3f},{max_y:.3f}] "
                f"z=[{min_z:.3f},{max_z:.3f}]"
            )
        if self.processed_cloud_bounds is not None:
            min_x, max_x, min_y, max_y, min_z, max_z = self.processed_cloud_bounds
            self.get_logger().info(
                "[PLANNER CLOUD] "
                f"points={self.processed_point_count} "
                f"x=[{min_x:.3f},{max_x:.3f}] "
                f"y=[{min_y:.3f},{max_y:.3f}] "
                f"z=[{min_z:.3f},{max_z:.3f}]"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
