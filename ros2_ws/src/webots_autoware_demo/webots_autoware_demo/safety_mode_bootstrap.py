import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_system_msgs.srv import ChangeAutowareControl, ChangeOperationMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class SafetyModeBootstrap(Node):
    """Request AUTONOMOUS mode through the real transition manager services."""

    def __init__(self):
        super().__init__("safety_mode_bootstrap")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state = None
        self.pending = False
        self.mode_client = self.create_client(
            ChangeOperationMode, "/system/operation_mode/change_operation_mode"
        )
        self.control_client = self.create_client(
            ChangeAutowareControl,
            "/system/operation_mode/change_autoware_control",
        )
        self.create_subscription(
            OperationModeState,
            "/system/operation_mode/state",
            self.on_state,
            qos,
        )
        self.create_timer(1.0, self.tick)

    def on_state(self, msg):
        self.state = msg

    def request_finished(self, future, label):
        self.pending = False
        try:
            response = future.result()
            if response.status.success:
                self.get_logger().info(f"{label} request accepted")
            else:
                self.get_logger().warning(
                    f"{label} rejected: {response.status.message}"
                )
        except Exception as error:  # pragma: no cover
            self.get_logger().warning(f"{label} failed: {error}")

    def tick(self):
        if self.pending:
            return
        if self.state is None or self.state.mode != OperationModeState.AUTONOMOUS:
            if not self.mode_client.service_is_ready():
                return
            request = ChangeOperationMode.Request()
            request.mode = ChangeOperationMode.Request.AUTONOMOUS
            self.pending = True
            future = self.mode_client.call_async(request)
            future.add_done_callback(
                lambda result: self.request_finished(result, "AUTONOMOUS mode")
            )
            return
        if not self.state.is_autoware_control_enabled:
            if not self.control_client.service_is_ready():
                return
            request = ChangeAutowareControl.Request()
            request.autoware_control = True
            self.pending = True
            future = self.control_client.call_async(request)
            future.add_done_callback(
                lambda result: self.request_finished(result, "Autoware control")
            )


def main(args=None):
    rclpy.init(args=args)
    node = SafetyModeBootstrap()
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
