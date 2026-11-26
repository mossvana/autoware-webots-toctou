import rclpy
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from rclpy.node import Node


class StraightTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("straight_trajectory_publisher")
        self.publisher = self.create_publisher(Trajectory, "/planning/trajectory", 10)
        self.timer = self.create_timer(0.1, self.publish_trajectory)

    def publish_trajectory(self):
        trajectory = Trajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = "map"

        start_x = -0.3
        end_x = 1.5
        spacing = 0.05
        point_count = int(round((end_x - start_x) / spacing)) + 1

        for index in range(point_count):
            x = start_x + spacing * index
            point = TrajectoryPoint()
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = index * 100_000_000
            if point.time_from_start.nanosec >= 1_000_000_000:
                point.time_from_start.sec = point.time_from_start.nanosec // 1_000_000_000
                point.time_from_start.nanosec %= 1_000_000_000

            point.pose.position.x = x
            point.pose.position.y = 0.0
            point.pose.position.z = 0.1
            point.pose.orientation.w = 1.0

            if x < 0.7:
                target_velocity = 0.25
            elif x < 1.2:
                target_velocity = max(0.0, 0.25 * (1.2 - x) / 0.5)
            else:
                target_velocity = 0.0

            point.longitudinal_velocity_mps = float(target_velocity)
            point.lateral_velocity_mps = 0.0
            point.acceleration_mps2 = 0.0
            point.heading_rate_rps = 0.0
            point.front_wheel_angle_rad = 0.0
            point.rear_wheel_angle_rad = 0.0
            trajectory.points.append(point)

        self.publisher.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = StraightTrajectoryPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
