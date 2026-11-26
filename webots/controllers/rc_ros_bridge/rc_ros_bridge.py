import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time
from geometry_msgs.msg import AccelWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from controller import Robot


WHEEL_RADIUS = 0.05
WHEELBASE = 0.22
TRACK_WIDTH = 0.21
MAX_WHEEL_SPEED = 20.0
MAX_CENTER_STEER = 0.45
DRIVE_TORQUE = 5.0
COMMAND_TIMEOUT = 0.5


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def seconds_to_time(seconds):
    stamp = Time()
    stamp.sec = int(seconds)
    stamp.nanosec = int((seconds - stamp.sec) * 1_000_000_000)
    return stamp


def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def ackermann_angles(center_angle):
    if abs(center_angle) < 1.0e-6:
        return 0.0, 0.0

    radius = WHEELBASE / math.tan(center_angle)
    left_angle = math.atan(WHEELBASE / (radius - TRACK_WIDTH / 2.0))
    right_angle = math.atan(WHEELBASE / (radius + TRACK_WIDTH / 2.0))
    return left_angle, right_angle


class WebotsRosBridge(Node):
    def __init__(self, robot):
        super().__init__("webots_vehicle_interface")
        self.robot = robot
        self.target_speed = 0.0
        self.target_steer = 0.0
        self.gear = "P"
        self.last_command_time = None

        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.clock_pub = self.create_publisher(Clock, "/clock", clock_qos)
        self.odom_pub = self.create_publisher(
            Odometry, "/localization/kinematic_state", 10
        )
        self.accel_pub = self.create_publisher(
            AccelWithCovarianceStamped, "/localization/acceleration", 10
        )
        self.imu_pub = self.create_publisher(
            Imu, "/sensing/imu/imu_data", qos_profile_sensor_data
        )
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            "/sensing/lidar/no_ground/pointcloud",
            qos_profile_sensor_data,
        )
        self.range_pub = self.create_publisher(
            Range, "/webots/front_range", qos_profile_sensor_data
        )
        self.joint_pub = self.create_publisher(
            JointState, "/webots/joint_states", qos_profile_sensor_data
        )
        self.gear_pub = self.create_publisher(String, "/webots/gear_state", 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_lidar_static_transform()

        self.create_subscription(
            AckermannDriveStamped,
            "/webots/drive_cmd",
            self.on_drive_command,
            10,
        )

    def publish_lidar_static_transform(self):
        transform = TransformStamped()
        transform.header.stamp = seconds_to_time(self.robot.getTime())
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "lidar_link"
        transform.transform.translation.x = 0.13
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.08
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)
        self.create_subscription(
            String,
            "/webots/gear_cmd",
            self.on_gear_command,
            10,
        )

    def on_drive_command(self, msg):
        self.target_speed = max(0.0, float(msg.drive.speed))
        self.target_steer = clamp(
            float(msg.drive.steering_angle),
            -MAX_CENTER_STEER,
            MAX_CENTER_STEER,
        )
        self.last_command_time = self.robot.getTime()

    def on_gear_command(self, msg):
        requested = msg.data.strip().upper()
        if requested not in ("P", "R", "N", "D"):
            self.get_logger().warning(f"Rejected unknown gear: {requested}")
            return

        if self.target_speed > 1.0e-6:
            self.get_logger().warning("Stop the target speed before changing gear")
            return

        self.gear = requested


def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    wheel_motors = [
        robot.getDevice("front_left_motor"),
        robot.getDevice("front_right_motor"),
        robot.getDevice("rear_left_motor"),
        robot.getDevice("rear_right_motor"),
    ]
    front_left_steer = robot.getDevice("front_left_steer")
    front_right_steer = robot.getDevice("front_right_steer")

    gps = robot.getDevice("gps")
    imu_sensor = robot.getDevice("imu")
    gyro = robot.getDevice("gyro")
    accelerometer = robot.getDevice("accelerometer")
    front_distance = robot.getDevice("front_distance")
    lidar = robot.getDevice("lidar")

    joint_sensors = [
        robot.getDevice("front_left_steer_sensor"),
        robot.getDevice("front_right_steer_sensor"),
        robot.getDevice("front_left_wheel_sensor"),
        robot.getDevice("front_right_wheel_sensor"),
        robot.getDevice("rear_left_wheel_sensor"),
        robot.getDevice("rear_right_wheel_sensor"),
    ]

    for motor in wheel_motors:
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)

    front_left_steer.setVelocity(4.0)
    front_right_steer.setVelocity(4.0)
    front_left_steer.setPosition(0.0)
    front_right_steer.setPosition(0.0)

    for sensor in [gps, imu_sensor, gyro, accelerometer, front_distance, *joint_sensors]:
        sensor.enable(time_step)
    lidar.enable(time_step)
    lidar.enablePointCloud()

    rclpy.init(args=None)
    node = WebotsRosBridge(robot)

    print("ROS bridge ready")
    print("Sub: /webots/drive_cmd, /webots/gear_cmd")
    print("Pub: /clock, /localization/kinematic_state, /localization/acceleration")
    print("Pub: /sensing/imu/imu_data, /sensing/lidar/no_ground/pointcloud")
    print("Pub: /tf, /tf_static, /webots/joint_states, /webots/front_range")

    try:
        while robot.step(time_step) != -1:
            now = robot.getTime()
            stamp = seconds_to_time(now)

            clock = Clock()
            clock.clock = stamp
            node.clock_pub.publish(clock)

            rclpy.spin_once(node, timeout_sec=0.0)

            if (
                node.last_command_time is None
                or now - node.last_command_time > COMMAND_TIMEOUT
            ):
                safe_speed = 0.0
            else:
                safe_speed = node.target_speed

            left_steer, right_steer = ackermann_angles(node.target_steer)
            front_left_steer.setPosition(left_steer)
            front_right_steer.setPosition(right_steer)

            wheel_speed = clamp(safe_speed / WHEEL_RADIUS, 0.0, MAX_WHEEL_SPEED)
            if node.gear == "R":
                wheel_speed = -wheel_speed
            elif node.gear not in ("D", "R"):
                wheel_speed = 0.0

            for motor in wheel_motors:
                if node.gear == "N":
                    motor.setAvailableTorque(0.0)
                else:
                    motor.setAvailableTorque(DRIVE_TORQUE)
                motor.setVelocity(wheel_speed)

            xyz = gps.getValues()
            velocity_vector = gps.getSpeedVector()
            roll, pitch, yaw = imu_sensor.getRollPitchYaw()
            gx, gy, gz = gyro.getValues()
            ax, ay, az = accelerometer.getValues()
            qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)

            longitudinal_velocity = (
                velocity_vector[0] * math.cos(yaw)
                + velocity_vector[1] * math.sin(yaw)
            )
            lateral_velocity = (
                -velocity_vector[0] * math.sin(yaw)
                + velocity_vector[1] * math.cos(yaw)
            )

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = xyz[0]
            odom.pose.pose.position.y = xyz[1]
            odom.pose.pose.position.z = xyz[2]
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            odom.twist.twist.linear.x = longitudinal_velocity
            odom.twist.twist.linear.y = lateral_velocity
            odom.twist.twist.angular.z = gz
            node.odom_pub.publish(odom)

            map_to_base = TransformStamped()
            map_to_base.header.stamp = stamp
            map_to_base.header.frame_id = "map"
            map_to_base.child_frame_id = "base_link"
            map_to_base.transform.translation.x = xyz[0]
            map_to_base.transform.translation.y = xyz[1]
            map_to_base.transform.translation.z = xyz[2]
            map_to_base.transform.rotation.x = qx
            map_to_base.transform.rotation.y = qy
            map_to_base.transform.rotation.z = qz
            map_to_base.transform.rotation.w = qw
            node.tf_broadcaster.sendTransform(map_to_base)

            accel_msg = AccelWithCovarianceStamped()
            accel_msg.header.stamp = stamp
            accel_msg.header.frame_id = "base_link"
            accel_msg.accel.accel.linear.x = ax
            accel_msg.accel.accel.linear.y = ay
            accel_msg.accel.accel.linear.z = az
            accel_msg.accel.accel.angular.x = gx
            accel_msg.accel.accel.angular.y = gy
            accel_msg.accel.accel.angular.z = gz
            node.accel_pub.publish(accel_msg)

            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = "base_link"
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            node.imu_pub.publish(imu_msg)

            cloud_header = Header()
            cloud_header.stamp = stamp
            cloud_header.frame_id = "lidar_link"
            lidar_points = []
            for point in lidar.getPointCloud():
                if (
                    math.isfinite(point.x)
                    and math.isfinite(point.y)
                    and math.isfinite(point.z)
                ):
                    lidar_points.append((float(point.x), float(point.y), float(point.z)))
            cloud_msg = point_cloud2.create_cloud_xyz32(cloud_header, lidar_points)
            node.pointcloud_pub.publish(cloud_msg)

            range_msg = Range()
            range_msg.header.stamp = stamp
            range_msg.header.frame_id = "front_distance_link"
            range_msg.radiation_type = Range.INFRARED
            range_msg.field_of_view = 0.01
            range_msg.min_range = 0.0
            range_msg.max_range = 2.0
            range_msg.range = float(front_distance.getValue())
            node.range_pub.publish(range_msg)

            joint_msg = JointState()
            joint_msg.header.stamp = stamp
            joint_msg.name = [
                "front_left_steer",
                "front_right_steer",
                "front_left_wheel",
                "front_right_wheel",
                "rear_left_wheel",
                "rear_right_wheel",
            ]
            joint_msg.position = [float(sensor.getValue()) for sensor in joint_sensors]
            node.joint_pub.publish(joint_msg)

            gear_msg = String()
            gear_msg.data = node.gear
            node.gear_pub.publish(gear_msg)
    finally:
        for motor in wheel_motors:
            motor.setVelocity(0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
