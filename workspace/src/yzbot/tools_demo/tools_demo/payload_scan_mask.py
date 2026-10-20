"""Mask the carried payload's own lidar returns while the arm holds a cube.

The carried cube rides ahead of the chassis with part of the payload assembly
inside the lidar plane at a range just past ``range_min`` (0.38 m), so every
scan consumer marked the robot's own payload as an obstacle: the local costmap
made RPP report "collision ahead" during the whole delivery leg, the global
dynamic layer accumulated a lethal trail behind the robot, and NavFn then
refused the next plan with "Starting point in lethal space". Height filtering
cannot separate the payload because a 2D lidar places every return at the
sensor's own height, and raising ``range_min`` would blind the robot to real
obstacles during close encounters.

This node republishes the scan with returns inside the payload box removed
while the mission is in a cube-holding state, and passes every other return
through unchanged. All real obstacle sources stay live: the static map, the
placed-cubes layer, the moving-obstacle truth used by the safety limiter, and
lidar returns outside the payload box. The masked box is unobserved, not free
space: NaN beams cannot clear obstacles behind the payload. State is latched
until the next coordinator event; only before the first event is scan raw.
"""

import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def in_payload_box(
    range_value: float,
    angle: float,
    forward_min: float,
    forward_max: float,
    lateral_half: float,
) -> bool:
    """Return whether one return falls inside the laser-frame payload box."""
    forward = range_value * math.cos(angle)
    lateral = range_value * math.sin(angle)
    return (
        forward_min <= forward <= forward_max
        and abs(lateral) <= lateral_half
    )


def should_mask(
    state: Optional[str],
    holding_states: frozenset,
) -> bool:
    """Return whether the payload box applies for the given mission state.

    The mission state is level information published on change with
    transient-local durability, not a heartbeat: a long docked picking
    phase must keep the mask active even when no message arrives for
    seconds. Fail-open applies only before the first state message
    arrives (the coordinator never started or died before publishing).
    """
    if state is None:
        return False
    return state in holding_states


def masked_scan(
    scan: LaserScan,
    forward_min: float,
    forward_max: float,
    lateral_half: float,
) -> LaserScan:
    """Copy the scan with payload-box beams marked unobserved (NaN)."""
    masked = LaserScan()
    masked.header = scan.header
    masked.angle_min = scan.angle_min
    masked.angle_max = scan.angle_max
    masked.angle_increment = scan.angle_increment
    masked.time_increment = scan.time_increment
    masked.scan_time = scan.scan_time
    masked.range_min = scan.range_min
    masked.range_max = scan.range_max
    masked.ranges = [
        float("nan")
        if in_payload_box(
            range_value, scan.angle_min + index * scan.angle_increment,
            forward_min, forward_max, lateral_half,
        )
        else range_value
        for index, range_value in enumerate(scan.ranges)
    ]
    masked.intensities = list(scan.intensities)
    return masked


class PayloadScanMask(Node):
    """Republish /scan without the carried payload's own returns."""

    def __init__(self) -> None:
        super().__init__("payload_scan_mask")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_payload_masked")
        self.declare_parameter("state_topic", "/competition/mission/state")
        self.declare_parameter("forward_min", 0.30)
        self.declare_parameter("forward_max", 0.48)
        self.declare_parameter("lateral_half", 0.24)
        self.declare_parameter(
            "holding_states",
            [
                "picking",
                "navigating_departure",
                "navigating_destination",
                "placing",
                "verifying",
                "releasing",
            ],
        )

        self.forward_min = float(self.get_parameter("forward_min").value)
        self.forward_max = float(self.get_parameter("forward_max").value)
        self.lateral_half = float(self.get_parameter("lateral_half").value)
        if not 0.0 <= self.forward_min < self.forward_max:
            raise ValueError(
                "payload box needs 0 <= forward_min < forward_max")
        if self.lateral_half < 0.0:
            raise ValueError("lateral_half must be nonnegative")
        self.holding_states = frozenset(
            str(value)
            for value in self.get_parameter("holding_states").value
        )

        self.mission_state: Optional[str] = None
        self.masked_returns = 0
        self.published_scans = 0

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Match the coordinator's reliable transient-local state publisher:
        # the current state is level information, and a mask node that joins
        # mid-carry must receive it without waiting for the next change.
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            LaserScan, str(self.get_parameter("output_topic").value),
            sensor_qos
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("input_topic").value),
            self.scan_callback,
            sensor_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("state_topic").value),
            self.state_callback,
            state_qos,
        )

    def state_callback(self, message: String) -> None:
        """Track the latest mission state."""
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        if not isinstance(payload, dict):
            return
        state = payload.get("state")
        if not isinstance(state, str):
            return
        self.mission_state = state

    def scan_callback(self, scan: LaserScan) -> None:
        """Republish the scan, masking the payload box only while holding."""
        if should_mask(self.mission_state, self.holding_states):
            output = masked_scan(
                scan, self.forward_min, self.forward_max, self.lateral_half
            )
        else:
            output = scan
        self.publisher.publish(output)
        self.published_scans += 1


def main(args=None) -> None:
    """Run the payload scan mask node."""
    rclpy.init(args=args)
    node = PayloadScanMask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
