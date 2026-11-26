import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import (
    ControlModeReport,
    Engage,
    GearReport,
    SteeringReport,
    VelocityReport,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class VehicleAdapter(Node):
    def __init__(self):
        super().__init__("webots_autoware_vehicle_adapter")
        self.declare_parameter(
            "control_topic", "/control/trajectory_follower/control_cmd"
        )
        self.declare_parameter("publish_demo_engage", False)
        control_topic = self.get_parameter("control_topic").value
        publish_demo_engage = self.get_parameter("publish_demo_engage").value
        self.latest_odom = None
        self.latest_steer = 0.0
        self.current_gear = None

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, "/webots/drive_cmd", 10
        )
        self.gear_cmd_pub = self.create_publisher(String, "/webots/gear_cmd", 10)
        self.velocity_pub = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 10
        )
        self.steering_pub = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 10
        )
        self.gear_report_pub = self.create_publisher(
            GearReport, "/vehicle/status/gear_status", 10
        )
        self.control_mode_pub = self.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", 10
        )
        operation_mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.operation_mode_pub = self.create_publisher(
            OperationModeState,
            "/system/operation_mode/state",
            operation_mode_qos,
        )
        self.engage_pub = (
            self.create_publisher(Engage, "/autoware/engage", operation_mode_qos)
            if publish_demo_engage
            else None
        )

        self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.on_odometry,
            10,
        )
        self.create_subscription(
            JointState,
            "/webots/joint_states",
            self.on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, "/webots/gear_state", self.on_gear_state, 10
        )
        self.create_subscription(
            Control,
            control_topic,
            self.on_control,
            10,
        )
        # The Webots bridge starts in P and accepts a gear change only while
        # the target speed is zero. Retry until its feedback confirms D.
        self.gear_init_timer = self.create_timer(0.2, self.initialize_drive_gear)
        self.create_timer(0.05, self.publish_status)

    def initialize_drive_gear(self):
        if self.current_gear == "D":
            return
        gear = String()
        gear.data = "D"
        self.gear_cmd_pub.publish(gear)

    def on_gear_state(self, msg):
        if msg.data == "D" and self.current_gear != "D":
            self.get_logger().info("Webots gear confirmed D")
        self.current_gear = msg.data

    def on_odometry(self, msg):
        self.latest_odom = msg

    def on_joint_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        left = float(positions.get("front_left_steer", 0.0))
        right = float(positions.get("front_right_steer", 0.0))
        self.latest_steer = 0.5 * (left + right)

    def on_control(self, msg):
        if self.current_gear != "D":
            return
        velocity = float(msg.longitudinal.velocity)
        steering = float(msg.lateral.steering_tire_angle)
        if not math.isfinite(velocity) or not math.isfinite(steering):
            self.get_logger().error("Rejected non-finite Autoware control command")
            return

        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.drive.speed = abs(velocity)
        drive.drive.steering_angle = steering
        self.drive_pub.publish(drive)

    def publish_status(self):
        now = self.get_clock().now().to_msg()

        if self.latest_odom is not None:
            velocity = VelocityReport()
            velocity.header = self.latest_odom.header
            velocity.longitudinal_velocity = float(
                self.latest_odom.twist.twist.linear.x
            )
            velocity.lateral_velocity = float(self.latest_odom.twist.twist.linear.y)
            velocity.heading_rate = float(self.latest_odom.twist.twist.angular.z)
            self.velocity_pub.publish(velocity)

        steering = SteeringReport()
        steering.stamp = now
        steering.steering_tire_angle = float(self.latest_steer)
        self.steering_pub.publish(steering)

        gear = GearReport()
        gear.stamp = now
        gear.report = GearReport.DRIVE
        self.gear_report_pub.publish(gear)

        control_mode = ControlModeReport()
        control_mode.stamp = now
        control_mode.mode = ControlModeReport.AUTONOMOUS
        self.control_mode_pub.publish(control_mode)

        operation_mode = OperationModeState()
        operation_mode.stamp = now
        operation_mode.mode = OperationModeState.AUTONOMOUS
        operation_mode.is_autoware_control_enabled = True
        operation_mode.is_in_transition = False
        operation_mode.is_stop_mode_available = True
        operation_mode.is_autonomous_mode_available = True
        operation_mode.is_local_mode_available = False
        operation_mode.is_remote_mode_available = False
        self.operation_mode_pub.publish(operation_mode)

        if self.engage_pub is not None:
            engage = Engage()
            engage.stamp = now
            engage.engage = self.current_gear == "D" and self.latest_odom is not None
            self.engage_pub.publish(engage)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleAdapter()
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
