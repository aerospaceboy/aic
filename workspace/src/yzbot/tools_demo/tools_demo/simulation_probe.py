"""Read-only, bounded proof of a progressing simulation, not a cached graph."""

import json
import math
import time


class SimulationEvidence:
    """Require two advancing clock stamps and two valid laser scans."""

    def __init__(self):
        self.first_clock = None
        self.last_clock = None
        self.scans = 0

    def clock(self, stamp):
        if self.first_clock is None:
            self.first_clock = stamp
        self.last_clock = stamp

    def scan(self, ranges, range_min, range_max):
        if ranges and any(math.isfinite(x) and range_min <= x <= range_max
                          for x in ranges):
            self.scans += 1

    def ready(self):
        return (self.first_clock is not None
                and self.last_clock > self.first_clock and self.scans >= 2)


def main(args=None):
    """Subscribe only; never create a service client against the simulator."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan

    rclpy.init(args=args)
    node = Node('yzbot_startup_simulation_probe')
    evidence = SimulationEvidence()
    node.create_subscription(
        Clock, '/clock', lambda msg: evidence.clock(
            msg.clock.sec * 1_000_000_000 + msg.clock.nanosec),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        LaserScan, '/scan', lambda msg: evidence.scan(
            msg.ranges, msg.range_min, msg.range_max),
        qos_profile_sensor_data,
    )
    deadline = time.monotonic() + 8
    try:
        while time.monotonic() < deadline and not evidence.ready():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({'simulation_live': evidence.ready(),
                      'first_clock_ns': evidence.first_clock,
                      'last_clock_ns': evidence.last_clock,
                      'valid_scan_messages': evidence.scans}))
    return 0 if evidence.ready() else 1
