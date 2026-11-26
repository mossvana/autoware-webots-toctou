import csv
import json
from pathlib import Path

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from autoware_adapi_v1_msgs.msg import MrmState
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import VelocityReport
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tier4_system_msgs.msg import OperationModeAvailability
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped


class NCommandEvaluator(Node):
    """Independently evaluate stale-command acceptance and physical response."""

    def __init__(self):
        super().__init__("n_command_evaluator")
        self.declare_parameter("attack_point", "before_gate")
        self.declare_parameter("hold_count", 30)
        self.declare_parameter("result_csv", "n_command_result.csv")
        self.declare_parameter("evaluation_time", 2.0)
        self.declare_parameter("positive_velocity_threshold", 0.05)
        self.declare_parameter("movement_threshold", 0.01)
        self.attack_point = str(self.get_parameter("attack_point").value)
        self.hold_count = int(self.get_parameter("hold_count").value)
        self.result_csv = str(self.get_parameter("result_csv").value)
        self.evaluation_ns = int(
            float(self.get_parameter("evaluation_time").value) * 1_000_000_000
        )
        self.velocity_threshold = float(
            self.get_parameter("positive_velocity_threshold").value
        )
        self.movement_threshold = float(
            self.get_parameter("movement_threshold").value
        )

        self.release_started = False
        self.release_start_ns = None
        self.release_end_ns = None
        self.start_x = None
        self.latest_x = None
        self.max_speed_mps = 0.0
        self.max_forward_distance_m = 0.0
        self.released_count = 0
        self.attack_output_positive = 0
        self.gate_output_positive = 0
        self.drive_output_positive = 0
        self.released_ages_ms = []
        self.last_attack_stamp_ns = None
        self.out_of_order_count = 0
        self.aeb_error_before_attack = False
        self.aeb_error_after_attack = False
        self.validator_error_before_attack = False
        self.validator_error_after_attack = False
        self.autonomous_unavailable_after_attack = False
        self.mrm_emergency_before_attack = False
        self.mrm_emergency_after_attack = False
        self.emergency_control_count = 0
        self.gate_emergency_after_attack = False
        self.done = False

        self.create_subscription(
            String, "/experiment/attack_events", self.on_event, 100
        )
        self.create_subscription(
            Control, "/attack/control_cmd", self.on_attack_output, 100
        )
        self.create_subscription(
            Control, "/control/command/control_cmd", self.on_gate_output, 100
        )
        self.create_subscription(
            AckermannDriveStamped, "/webots/drive_cmd", self.on_drive_output, 100
        )
        self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self.on_velocity, 10
        )
        self.create_subscription(
            Odometry, "/localization/kinematic_state", self.on_odometry, 10
        )
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self.on_diagnostics, 100
        )
        self.create_subscription(
            OperationModeAvailability,
            "/system/operation_mode/availability",
            self.on_availability,
            10,
        )
        self.create_subscription(
            MrmState, "/system/fail_safe/mrm_state", self.on_mrm_state, 10
        )
        self.create_subscription(
            Control,
            "/system/emergency/control_cmd",
            self.on_emergency_control,
            100,
        )
        self.create_subscription(
            VehicleEmergencyStamped,
            "/control/command/vehicle_emergency",
            self.on_gate_emergency,
            10,
        )
        self.result_pub = self.create_publisher(
            String, "/experiment/n_command_result", 10
        )
        self.create_timer(0.05, self.evaluate)

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def in_attack_window(self):
        return self.release_started and not self.done

    def on_event(self, msg):
        event = json.loads(msg.data)
        kind = event["kind"]
        if kind == "release_start":
            self.release_started = True
            self.release_start_ns = int(event["t_ns"])
            self.start_x = self.latest_x
            self.get_logger().warning(
                f"[EVAL] stale replay started at x={self.start_x}"
            )
        elif kind == "release_command":
            self.released_count += 1
            self.released_ages_ms.append(float(event["age_ms"]))
        elif kind == "release_end":
            self.release_end_ns = int(event["t_ns"])

    def on_attack_output(self, msg):
        stamp_ns = int(msg.stamp.sec) * 1_000_000_000 + int(msg.stamp.nanosec)
        if (
            self.in_attack_window()
            and self.last_attack_stamp_ns is not None
            and stamp_ns < self.last_attack_stamp_ns
        ):
            self.out_of_order_count += 1
        self.last_attack_stamp_ns = stamp_ns
        if self.in_attack_window() and msg.longitudinal.velocity > self.velocity_threshold:
            self.attack_output_positive += 1

    def on_gate_output(self, msg):
        if self.in_attack_window() and msg.longitudinal.velocity > self.velocity_threshold:
            self.gate_output_positive += 1

    def on_drive_output(self, msg):
        if self.in_attack_window() and msg.drive.speed > self.velocity_threshold:
            self.drive_output_positive += 1

    def on_velocity(self, msg):
        if self.in_attack_window():
            self.max_speed_mps = max(
                self.max_speed_mps, abs(float(msg.longitudinal_velocity))
            )

    def on_odometry(self, msg):
        self.latest_x = float(msg.pose.pose.position.x)
        if self.in_attack_window() and self.start_x is not None:
            self.max_forward_distance_m = max(
                self.max_forward_distance_m, self.latest_x - self.start_x
            )

    def on_diagnostics(self, msg):
        for status in msg.status:
            if status.level < DiagnosticStatus.ERROR:
                continue
            if "aeb_emergency_stop" in status.name:
                if self.release_started:
                    self.aeb_error_after_attack = True
                else:
                    self.aeb_error_before_attack = True
            if "control_validation_" in status.name:
                if self.release_started:
                    self.validator_error_after_attack = True
                else:
                    self.validator_error_before_attack = True

    def on_availability(self, msg):
        if self.release_started and not msg.autonomous:
            self.autonomous_unavailable_after_attack = True

    def on_mrm_state(self, msg):
        is_emergency = (
            msg.behavior == MrmState.EMERGENCY_STOP
            and msg.state
            in (MrmState.MRM_OPERATING, MrmState.MRM_SUCCEEDED, MrmState.MRM_FAILED)
        )
        if not is_emergency:
            return
        if self.release_started:
            self.mrm_emergency_after_attack = True
        else:
            self.mrm_emergency_before_attack = True

    def on_emergency_control(self, _msg):
        if self.in_attack_window():
            self.emergency_control_count += 1

    def on_gate_emergency(self, msg):
        if self.in_attack_window() and msg.emergency:
            self.gate_emergency_after_attack = True

    def evaluate(self):
        if self.done or self.release_end_ns is None:
            return
        if self.now_ns() - self.release_end_ns < self.evaluation_ns:
            return

        recipient_positive_count = (
            self.gate_output_positive
            if self.attack_point == "before_gate"
            else self.drive_output_positive
        )
        stale_accepted = recipient_positive_count > 0
        physical_reacceleration = (
            self.max_speed_mps > self.velocity_threshold
            and self.max_forward_distance_m > self.movement_threshold
        )
        safety_pretrigger = (
            self.aeb_error_before_attack
            or self.validator_error_before_attack
            or self.mrm_emergency_before_attack
        )
        if safety_pretrigger:
            safety_outcome = "INVALID_PRETRIGGER"
        elif not physical_reacceleration:
            safety_outcome = "MITIGATED"
        elif self.attack_point == "after_gate" and self.gate_emergency_after_attack:
            safety_outcome = "BYPASSED_AFTER_GATE"
        elif self.mrm_emergency_after_attack:
            safety_outcome = "MRM_ACTIVATED_WITH_IMPACT"
        elif self.aeb_error_after_attack or self.validator_error_after_attack:
            safety_outcome = "DETECTED_ONLY"
        else:
            safety_outcome = "NOT_DETECTED"
        row = {
            "attack_point": self.attack_point,
            "configured_hold_count": self.hold_count,
            "released_count": self.released_count,
            "min_released_age_ms": round(min(self.released_ages_ms), 1),
            "max_released_age_ms": round(max(self.released_ages_ms), 1),
            "out_of_order_count": self.out_of_order_count,
            "attack_output_positive_count": self.attack_output_positive,
            "gate_output_positive_count": self.gate_output_positive,
            "drive_output_positive_count": self.drive_output_positive,
            "max_reaccel_speed_mps": round(self.max_speed_mps, 4),
            "max_forward_distance_m": round(self.max_forward_distance_m, 4),
            "stale_accepted_by_recipient": stale_accepted,
            "physical_reacceleration": physical_reacceleration,
            "attack_success": stale_accepted and physical_reacceleration,
            "aeb_error_before_attack": self.aeb_error_before_attack,
            "aeb_error_after_attack": self.aeb_error_after_attack,
            "validator_error_before_attack": self.validator_error_before_attack,
            "validator_error_after_attack": self.validator_error_after_attack,
            "autonomous_unavailable_after_attack": self.autonomous_unavailable_after_attack,
            "mrm_emergency_before_attack": self.mrm_emergency_before_attack,
            "mrm_emergency_after_attack": self.mrm_emergency_after_attack,
            "emergency_control_count": self.emergency_control_count,
            "gate_emergency_after_attack": self.gate_emergency_after_attack,
            "safety_outcome": safety_outcome,
        }
        output = Path(self.result_csv).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=row.keys())
            writer.writeheader()
            writer.writerow(row)
        self.result_pub.publish(String(data=json.dumps(row, sort_keys=True)))
        self.get_logger().warning(f"[N-COMMAND RESULT] {row}; saved {output}")
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = NCommandEvaluator()
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
