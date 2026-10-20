"""Publish the moving partitions' swept bands as a global keepout cloud.

The two preview partitions patrol back and forth forever, so waiting for
them to leave their band never succeeds: wall 2 sweeps x=[6.5, 7.9] across
the only 2 m free strip between the trays and the south wall, and both the
0.1 m north and 0.4 m south gaps beside its 1.6 m segment are narrower than
the robot. A global path through the band therefore always depends on
timing; the 2026-10-05 rerun took the shorter south lane and its body
passed the sweeping wall face at 0.043 m surface clearance.

This node republishes each partition's swept rectangle - derived from the
frozen sweep parameters in competition_zones.yaml, expanded by the planning
margin - as a dense map-frame PointCloud2. The costmap inflation layer supplies
the robot radius separately; this node must not count that radius twice. The global
costmap marks it in a dedicated never-cleared layer, so planning routes
around the band (the proven north corridor at y ~ -2.97) instead of through
it. The live wall position stays marked by the dynamic layer for local
avoidance, and the original corridor movers are deliberately not banded:
crossing their patrol line is the designed avoidance interaction.
"""

from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import struct


def parse_bands(specifications: List[str]) -> List[Tuple[str, float, float, float, float]]:
    """Parse "name:min_x:min_y:max_x:max_y" keepout specifications."""
    bands: List[Tuple[str, float, float, float, float]] = []
    for specification in specifications:
        parts = str(specification).split(":")
        if len(parts) != 5:
            continue
        name = parts[0]
        try:
            values = [float(value) for value in parts[1:]]
        except ValueError:
            continue
        min_x, min_y, max_x, max_y = values
        if not min_x < max_x or not min_y < max_y:
            continue
        bands.append((name, min_x, min_y, max_x, max_y))
    return bands


def band_points(
    min_x: float,
    min_y: float,
    max_x: float,
    max_y: float,
    margin: float,
    resolution: float,
) -> List[Tuple[float, float, float]]:
    """Grid the inflated band rectangle at the given resolution."""
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    points: List[Tuple[float, float, float]] = []
    x = min_x - margin
    while x <= max_x + margin + resolution / 2.0:
        y = min_y - margin
        while y <= max_y + margin + resolution / 2.0:
            points.append((x, y, 0.0))
            y += resolution
        x += resolution
    return points


def keepout_message(
    points: List[Tuple[float, float, float]],
    frame_id: str,
    stamp_sec: float,
) -> PointCloud2:
    """Pack one dense keepout cloud in the map frame."""
    message = PointCloud2()
    message.header.frame_id = frame_id
    message.header.stamp.sec = int(stamp_sec)
    message.header.stamp.nanosec = int((stamp_sec % 1.0) * 1e9)
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name=name, offset=index * 4,
                   datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(("x", "y", "z"))
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * len(points)
    message.is_dense = True
    message.data = b"".join(
        struct.pack("<fff", *point) for point in points
    )
    return message


class SweepBandKeepout(Node):
    """Republish the partitions' swept bands for the global costmap."""

    def __init__(self) -> None:
        super().__init__("sweep_band_keepout")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "/competition/sweep_band_keepout")
        self.declare_parameter("bands", [""])
        self.declare_parameter("margin", 0.38)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("publish_rate_hz", 1.0)

        self.margin = float(self.get_parameter("margin").value)
        self.resolution = float(self.get_parameter("resolution").value)
        if self.margin < 0.0:
            raise ValueError("margin must be nonnegative")

        self.bands = parse_bands(
            list(self.get_parameter("bands").value)
        )
        if not self.bands:
            self.get_logger().warn(
                "no valid keepout bands configured; publishing empty cloud"
            )
        points: List[Tuple[float, float, float]] = []
        for name, min_x, min_y, max_x, max_y in self.bands:
            points.extend(
                band_points(min_x, min_y, max_x, max_y,
                            self.margin, self.resolution)
            )
        self.points = points

        reliable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter("topic").value), reliable
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self.publish_count = 0
        self.create_timer(1.0 / rate, self.tick)

    def tick(self) -> None:
        """Republish the static keepout cloud on the node clock."""
        stamp = self.get_clock().now()
        message = keepout_message(
            self.points,
            str(self.get_parameter("frame_id").value),
            stamp.nanoseconds * 1e-9,
        )
        self.publisher.publish(message)
        self.publish_count += 1


def main(args=None) -> None:
    """Run the sweep-band keepout node."""
    rclpy.init(args=args)
    node = SweepBandKeepout()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
