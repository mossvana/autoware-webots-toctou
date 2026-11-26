import rclpy
from rclpy.node import Node

from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_perception_msgs.msg import PredictedObjects


class PlannerTestInputs(Node):
    def __init__(self):
        super().__init__('planner_test_inputs')
        self.trajectory_pub = self.create_publisher(
            Trajectory, '/planning/trajectory_raw', 10)
        self.objects_pub = self.create_publisher(
            PredictedObjects, '/perception/object_recognition/objects', 10)
        self.timer = self.create_timer(0.1, self.publish_inputs)

    def publish_inputs(self):
        stamp = self.get_clock().now().to_msg()

        trajectory = Trajectory()
        trajectory.header.stamp = stamp
        trajectory.header.frame_id = 'map'

        for index in range(37):
            x = -0.3 + index * 0.05
            point = TrajectoryPoint()
            point.pose.position.x = x
            point.pose.position.z = 0.1
            point.pose.orientation.w = 1.0
            point.longitudinal_velocity_mps = 0.25 if index < 36 else 0.0
            trajectory.points.append(point)

        objects = PredictedObjects()
        objects.header.stamp = stamp
        objects.header.frame_id = 'map'

        self.trajectory_pub.publish(trajectory)
        self.objects_pub.publish(objects)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerTestInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
