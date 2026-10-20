"""Select one velocity owner before smoothing and collision protection."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


class VelocitySelector(Node):
    """Keep direct docking/holding commands exclusive of Nav2 commands."""

    def __init__(self):
        super().__init__("competition_velocity_selector")
        self.direct = False
        self.changed_at = time.monotonic()
        self.nav_message = None
        self.direct_message = None
        self.nav_stamp = self.direct_stamp = 0.0
        self.manip_message = self.arm_message = None
        self.manip_stamp = self.arm_stamp = 0.0
        self.publisher = self.create_publisher(
            Twist, "/competition/cmd_vel_selected", 10)
        self.create_subscription(
            Twist, "/cmd_vel_nav", self.nav_callback, 10)
        self.create_subscription(
            Twist, "/competition/cmd_vel_direct", self.direct_callback, 10)
        self.create_subscription(
            Twist, "/competition/cmd_vel_manipulation",
            self.manip_callback, 10)
        self.create_subscription(
            Twist, "/competition/cmd_vel_arm_stop", self.arm_callback, 10)
        qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, "/competition/navigation/direct_control",
            self.mode_callback, qos)
        self.create_timer(0.05, self.tick)

    def mode_callback(self, message):
        """Switch owner without replaying its cached previous command."""
        value = bool(message.data)
        if value != self.direct:
            self.direct = value
            self.changed_at = time.monotonic()
            self.publisher.publish(Twist())

    def nav_callback(self, message):
        """Store the latest navigation command."""
        self.nav_message = message
        self.nav_stamp = time.monotonic()

    def direct_callback(self, message):
        """Store the latest direct command."""
        self.direct_message = message
        self.direct_stamp = time.monotonic()

    def manip_callback(self, message):
        """Receive short-lived task-coordinator base adjustments."""
        self.manip_message = message
        self.manip_stamp = time.monotonic()

    def arm_callback(self, message):
        """Receive the arm worker's explicit stop pulses."""
        self.arm_message = message
        self.arm_stamp = time.monotonic()

    def tick(self):
        """Publish only fresh commands from the selected owner, else stop."""
        stamp = self.direct_stamp if self.direct else self.nav_stamp
        command = self.direct_message if self.direct else self.nav_message
        now = time.monotonic()
        if command is None or stamp < self.changed_at or now - stamp > 0.35:
            command = Twist()
        # Arm stop pulses have priority; task adjustments precede Nav2.
        # Neither source may replay a command after its short lease expires.
        manip_stamp = getattr(self, "manip_stamp", 0.0)
        arm_stamp = getattr(self, "arm_stamp", 0.0)
        if 0.0 <= now - manip_stamp <= 0.35:
            command = self.manip_message
        if 0.0 <= now - arm_stamp <= 0.35:
            command = self.arm_message
        self.publisher.publish(command)


def main(args=None):
    """Run the exclusive velocity selector."""
    rclpy.init(args=args)
    node = VelocitySelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
