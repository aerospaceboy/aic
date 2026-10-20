"""Publish sensor-derived dynamic returns for global route replanning."""

import math
import json
import struct
import time
from collections import deque

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from tools_demo.dynamic_obstacle_filter import static_return, transform_point
from tools_demo.dynamic_prediction import MotionTracker, scan_detections


class DynamicObstacleNode(Node):
    """Keep known walls out of live global obstacle marking."""

    def __init__(self):
        super().__init__('competition_dynamic_obstacles')
        self.declare_parameter('static_match_tolerance', 0.18)
        self.declare_parameter('maximum_range', 4.0)
        self.declare_parameter('publish_rate_hz', 5.0)
        # The competition launch feeds the payload-masked scan so the carried
        # cube cannot mark the global dynamic layer on the robot's own trail.
        self.declare_parameter('scan_topic', '/scan_payload_masked')
        self.declare_parameter('prediction_exclusion_bounds', [0.0, 0.0, 0.0, 0.0])
        self.prediction_exclusion = list(
            self.get_parameter('prediction_exclusion_bounds').value)
        if len(self.prediction_exclusion) != 4 or not all(
                math.isfinite(v) for v in self.prediction_exclusion):
            raise ValueError('Prediction exclusion must contain four finite bounds')
        self.map = None
        self.last_publish = 0.0
        self.pending = deque(maxlen=20)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, '/competition/dynamic_obstacles', 10)
        self.prediction_publisher = self.create_publisher(
            PointCloud2, '/competition/dynamic_predictions', 1)
        self.track_publisher = self.create_publisher(
            String, '/competition/dynamic_tracks', 1)
        self.tracker = MotionTracker()
        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self.queue_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.process_pending)

    def queue_scan(self, scan):
        """Retain a bounded queue for scans arriving just before their TF."""
        self.pending.append((time.monotonic(), scan))

    def process_pending(self):
        """Wait for stamped TF without blocking its subscription callbacks."""
        while self.pending and time.monotonic() - self.pending[0][0] > 1.0:
            self.pending.popleft()
        for index in range(len(self.pending) - 1, -1, -1):
            if self.scan_callback(self.pending[index][1]):
                for _ in range(index + 1):
                    self.pending.popleft()
                break

    def map_callback(self, message):
        """Retain only a well-formed authoritative static map."""
        if (message.info.resolution > 0 and
                len(message.data) == message.info.width * message.info.height):
            self.map = message

    def scan_callback(self, scan):
        """Filter static returns; raw scan separately clears old live marks."""
        now = time.monotonic()
        rate = float(self.get_parameter('publish_rate_hz').value)
        if self.map is None or rate <= 0 or now - self.last_publish < 1 / rate:
            return False
        grid = self.map
        try:
            transform = self.buffer.lookup_transform(
                grid.header.frame_id, scan.header.frame_id,
                Time.from_msg(scan.header.stamp)).transform
        except TransformException:
            # Never publish guessed map coordinates when TF is unavailable.
            return False
        self.last_publish = now
        tr = transform.translation
        rot = transform.rotation
        translation = (tr.x, tr.y, tr.z)
        quaternion = (rot.x, rot.y, rot.z, rot.w)
        origin = grid.info.origin
        q = origin.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        points = []
        indexed_world_points = []
        maximum = min(scan.range_max,
                      float(self.get_parameter('maximum_range').value))
        tolerance = float(self.get_parameter('static_match_tolerance').value)
        for index, distance in enumerate(scan.ranges):
            if (not math.isfinite(distance) or
                    not scan.range_min <= distance <= maximum):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            point = (distance * math.cos(angle),
                     distance * math.sin(angle), 0.0)
            wx, wy, _ = transform_point(point, translation, quaternion)
            # A conservative static raster can overlap a mover's real free
            # space. Track circular shape+motion from ALL masked live rays;
            # stationary map-frame circles produce no directional forecast.
            # Static filtering below still applies unchanged to live marks.
            indexed_world_points.append((index, wx, wy))
            if not static_return(
                    grid.data, grid.info.width, grid.info.height,
                    grid.info.resolution,
                    (origin.position.x, origin.position.y, yaw),
                    wx, wy, tolerance):
                points.append(point)
        message = PointCloud2()
        # Keeping the laser frame makes the obstacle layer use the true sensor
        # origin. Using map-frame coordinates as a sensor origin is incorrect.
        message.header = scan.header
        message.height = 1
        message.width = len(points)
        message.fields = [PointField(name=name, offset=i * 4,
                                     datatype=PointField.FLOAT32, count=1)
                          for i, name in enumerate(('x', 'y', 'z'))]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = 12 * len(points)
        message.is_dense = True
        message.data = b''.join(
            struct.pack('<fff', *point) for point in points)
        self.publisher.publish(message)
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        x0, y0, x1, y1 = self.prediction_exclusion
        detections = [d for d in scan_detections(indexed_world_points)
                      if not (x0 <= d.x <= x1 and y0 <= d.y <= y1)]
        if self.tracker.update(stamp, detections):
            predicted = self.tracker.cost_points(stamp)
            forecast = PointCloud2()
            forecast.header.stamp = scan.header.stamp
            forecast.header.frame_id = grid.header.frame_id
            forecast.height = 1
            forecast.width = len(predicted)
            forecast.fields = [PointField(name=name, offset=i*4,
                                          datatype=PointField.FLOAT32,count=1)
                               for i,name in enumerate(('x','y','z','cost'))]
            forecast.is_bigendian = False
            forecast.point_step = 16
            forecast.row_step = 16 * len(predicted)
            forecast.is_dense = True
            forecast.data = b''.join(struct.pack('<ffff', *p) for p in predicted)
            # A replacing snapshot, not persistent obstacle-layer marks. Empty
            # directions are retired after reversal. A briefly occluded known
            # mover retains a bounded reachable disc, not its obsolete heading.
            self.prediction_publisher.publish(forecast)
            self.track_publisher.publish(String(data=json.dumps({
                'source': 'laser_stamped_map_tf', 'stamp': stamp,
                'horizon': self.tracker.horizon,
                'tracks': self.tracker.snapshot(stamp),
            })))
        return True


def main(args=None):
    """Run the sensor filter alongside navigation."""
    rclpy.init(args=args)
    node = DynamicObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
