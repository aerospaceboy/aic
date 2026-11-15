"""Derived local-only static clearance; original map and moving bodies stay intact."""
import copy
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
import json


def static_margin_cells(data, width, height, resolution, margin=.20):
    if (len(data) != width*height or width <= 0 or height <= 0
            or not math.isfinite(resolution) or resolution <= 0
            or not .10 <= margin <= .30):
        raise ValueError('Invalid static clearance map')
    cells = np.asarray(data, dtype=np.int16).reshape(height, width)
    # Unknown cells remain unknown; their border is not an unproved shortcut.
    free = ((cells >= 0) & (cells < 65)).astype(np.uint8)
    distances = cv2.distanceTransform(free, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)*resolution
    padded = cells.copy()
    padded[(cells >= 0) & (distances <= margin+1.e-7)] = 100
    return padded.astype(np.int8).ravel().tolist()


def preserve_spawn_corridor(original, padded, width, height, resolution, origin):
    """Keep the existing scored entry gates and their original static geometry."""
    ox, oy, yaw = origin
    iy, ix = np.indices((height, width))
    x = (ix+.5)*resolution
    y = (iy+.5)*resolution
    wx = ox+math.cos(yaw)*x-math.sin(yaw)*y
    wy = oy+math.sin(yaw)*x+math.cos(yaw)*y
    mask = (wx >= 3.8) & (wx <= 14.) & (wy >= -1.6) & (wy <= 1.)
    result = np.asarray(padded, dtype=np.int8).reshape(height, width).copy()
    source = np.asarray(original, dtype=np.int8).reshape(height, width)
    result[mask] = source[mask]
    return result.ravel().tolist()


class DenseStaticMargin(Node):
    def __init__(self):
        super().__init__('competition_dense_static_margin')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid, '/competition/dense_static_map', qos)
        self.global_publisher = self.create_publisher(OccupancyGrid, '/competition/dense_global_static_map', qos)
        self.region_publisher = self.create_publisher(String, '/competition/static_clearance', 10)
        self.map = None
        self.distance = None
        self.odom = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(OccupancyGrid, '/map', self.receive, qos)
        self.create_subscription(Odometry, '/odom', lambda message: setattr(self, 'odom', message), 20)
        self.create_timer(.1, self.region)

    def receive(self, message):
        try:
            info = message.info
            q = info.origin.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            origin = (info.origin.position.x, info.origin.position.y, yaw)
            maps = []
            for margin in (.20, .10):
                cells = static_margin_cells(message.data, info.width, info.height, info.resolution, margin)
                maps.append(preserve_spawn_corridor(message.data, cells, info.width,
                                                    info.height, info.resolution, origin))
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        derived = copy.deepcopy(message)
        derived.data = maps[0]
        self.publisher.publish(derived)
        derived.data = maps[1]
        self.global_publisher.publish(derived)
        self.map = message
        cells = np.asarray(message.data).reshape(info.height, info.width)
        free = ((cells >= 0) & (cells < 65)).astype(np.uint8)
        self.distance = cv2.distanceTransform(free, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)*info.resolution
        self.get_logger().info('dense_static_clearance local=0.20 global=0.10 original_spawn_preserved=true')

    def region(self):
        now = self.get_clock().now().nanoseconds*1.e-9
        value = dict(stamp_sec=now, available=False, clearance_m=0.)
        message = self.odom
        if self.map is not None and self.distance is not None and message is not None:
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1.e-9
            if 0 <= now-stamp <= .35:
                try:
                    p = message.pose.pose.position
                    x, y = p.x, p.y
                    if message.header.frame_id != self.map.header.frame_id:
                        t = self.tf.lookup_transform(self.map.header.frame_id, message.header.frame_id, Time())
                        q = t.transform.rotation
                        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                        x, y = (t.transform.translation.x+math.cos(yaw)*x-math.sin(yaw)*y,
                                t.transform.translation.y+math.sin(yaw)*x+math.cos(yaw)*y)
                    info = self.map.info
                    q = info.origin.orientation
                    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                    dx, dy = x-info.origin.position.x, y-info.origin.position.y
                    ix = math.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/info.resolution)
                    iy = math.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/info.resolution)
                    if 0 <= ix < info.width and 0 <= iy < info.height:
                        # Both query-cell and occupied-cell quantization.
                        clearance = max(0., float(self.distance[iy, ix])-math.sqrt(2)*info.resolution)
                        value.update(available=True, clearance_m=clearance, stamp_sec=stamp)
                except (TransformException, ValueError, OverflowError):
                    pass
        self.region_publisher.publish(String(data=json.dumps(value)))


def main(args=None):
    rclpy.init(args=args)
    node = DenseStaticMargin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
