"""Derived local-only static clearance; original map and moving bodies stay intact."""
import copy
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid


def static_margin_cells(data, width, height, resolution, margin=.20):
    if (len(data) != width*height or width <= 0 or height <= 0
            or not math.isfinite(resolution) or resolution <= 0
            or not .20 <= margin <= .30):
        raise ValueError('Invalid static clearance map')
    cells = np.asarray(data, dtype=np.int16).reshape(height, width)
    # Unknown cells remain unknown; their border is not an unproved shortcut.
    free = ((cells >= 0) & (cells < 65)).astype(np.uint8)
    distances = cv2.distanceTransform(free, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)*resolution
    padded = cells.copy()
    padded[(cells >= 0) & (distances <= margin+1.e-7)] = 100
    return padded.astype(np.int8).ravel().tolist()


class DenseStaticMargin(Node):
    def __init__(self):
        super().__init__('competition_dense_static_margin')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid, '/competition/dense_static_map', qos)
        self.create_subscription(OccupancyGrid, '/map', self.receive, qos)

    def receive(self, message):
        try:
            cells = static_margin_cells(message.data, message.info.width,
                                        message.info.height, message.info.resolution)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        derived = copy.deepcopy(message)
        derived.data = cells
        self.publisher.publish(derived)
        self.get_logger().info('dense_static_clearance margin=0.20 original_map_unchanged=true')


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
