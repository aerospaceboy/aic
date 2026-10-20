"""Detect red and blue RGB-D targets and publish transformed 3D poses."""

import json
import math
from typing import Optional, Sequence, Tuple

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from tools_demo.perception_math import build_hsv_mask
from tools_demo.perception_math import CameraIntrinsics
from tools_demo.perception_math import connected_color_regions
from tools_demo.perception_math import depth_to_meters
from tools_demo.perception_math import project_pixel_to_camera
from tools_demo.perception_math import transform_point
from tools_demo.perception_math import valid_depth_median


class ColorTargetDetector(Node):
    """Synchronize RGB-D data and publish validated color target positions."""

    def __init__(self) -> None:
        super().__init__("competition_color_target_detector")
        self.declare_parameter(
            "color_topic",
            "/camera_rgbd/camera_rgbd/image_raw",
        )
        self.declare_parameter(
            "depth_topic",
            "/camera_rgbd/camera_rgbd/depth/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera_rgbd/camera_rgbd/camera_info",
        )
        self.declare_parameter(
            "targets_topic",
            "/competition/perception/targets",
        )
        self.declare_parameter(
            "debug_image_topic",
            "/competition/perception/debug_image",
        )
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("minimum_area", 40)
        self.declare_parameter("maximum_area", 50000)
        self.declare_parameter("minimum_depth", 0.05)
        self.declare_parameter("maximum_depth", 3.0)
        self.declare_parameter("maximum_stamp_delta", 0.12)
        self.declare_parameter("tf_timeout", 0.20)
        self.declare_parameter("morphology_size", 3)
        self.declare_parameter("maximum_targets_per_color", 5)
        self.declare_parameter("red_lower_1", [0, 100, 70])
        self.declare_parameter("red_upper_1", [10, 255, 255])
        self.declare_parameter("red_lower_2", [170, 100, 70])
        self.declare_parameter("red_upper_2", [179, 255, 255])
        self.declare_parameter("blue_lower", [95, 100, 60])
        self.declare_parameter("blue_upper", [135, 255, 255])

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.minimum_area = int(self.get_parameter("minimum_area").value)
        self.maximum_area = int(self.get_parameter("maximum_area").value)
        self.minimum_depth = float(
            self.get_parameter("minimum_depth").value
        )
        self.maximum_depth = float(
            self.get_parameter("maximum_depth").value
        )
        self.maximum_stamp_delta = float(
            self.get_parameter("maximum_stamp_delta").value
        )
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.morphology_size = int(
            self.get_parameter("morphology_size").value
        )
        self.maximum_targets_per_color = int(
            self.get_parameter("maximum_targets_per_color").value
        )
        self.hsv_ranges = {
            "red": (
                self._hsv_range("red_lower_1", "red_upper_1"),
                self._hsv_range("red_lower_2", "red_upper_2"),
            ),
            "blue": (
                self._hsv_range("blue_lower", "blue_upper"),
            ),
        }
        self.colors = {
            "red": (0, 0, 255),
            "blue": (255, 0, 0),
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.targets_publisher = self.create_publisher(
            String,
            str(self.get_parameter("targets_topic").value),
            10,
        )
        self.debug_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("debug_image_topic").value),
            10,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.color_subscriber = message_filters.Subscriber(
            self,
            Image,
            str(self.get_parameter("color_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_subscriber = message_filters.Subscriber(
            self,
            Image,
            str(self.get_parameter("depth_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.color_subscriber, self.depth_subscriber],
            queue_size=10,
            slop=self.maximum_stamp_delta,
        )
        self.synchronizer.registerCallback(self.image_callback)
        self.last_warning_key: Optional[str] = None

    def _hsv_range(
        self,
        lower_name: str,
        upper_name: str,
    ) -> Tuple[Sequence[int], Sequence[int]]:
        lower = tuple(self.get_parameter(lower_name).value)
        upper = tuple(self.get_parameter(upper_name).value)
        if len(lower) != 3 or len(upper) != 3:
            raise ValueError("HSV parameters must each have three values")
        return lower, upper

    def warn_changed(self, key: str, message: str) -> None:
        """Log a repeated frame warning only when its reason changes."""
        if key != self.last_warning_key:
            self.get_logger().warn(message)
            self.last_warning_key = key

    def camera_info_callback(self, message: CameraInfo) -> None:
        """Store the newest pinhole calibration."""
        self.camera_info = message

    def intrinsics(self) -> Optional[CameraIntrinsics]:
        """Build validated intrinsics from the current CameraInfo."""
        info = self.camera_info
        if info is None or len(info.k) != 9:
            return None
        try:
            return CameraIntrinsics(
                fx=float(info.k[0]),
                fy=float(info.k[4]),
                cx=float(info.k[2]),
                cy=float(info.k[5]),
                width=int(info.width),
                height=int(info.height),
            )
        except ValueError:
            return None

    def lookup_rigid_transform(
        self,
        source_frame: str,
        stamp,
    ) -> Optional[Tuple[Tuple[float, ...], Tuple[float, ...]]]:
        """Look up target-frame translation and quaternion."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException:
            # The stamped lookup can extrapolate a few tens of ms into the
            # future while the wrist TF lags under load. The scene is static
            # during a scan, so retry with the latest available transform
            # before giving up on this frame.
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout),
                )
            except TransformException as exception:
                self.warn_changed("tf", f"TF unavailable: {exception}")
                return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def image_callback(
        self,
        color_message: Image,
        depth_message: Image,
    ) -> None:
        """Process one approximately synchronized RGB-D frame pair."""
        intrinsics = self.intrinsics()
        if intrinsics is None:
            self.warn_changed(
                "camera_info",
                "Waiting for valid CameraInfo",
            )
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(
                color_message,
                desired_encoding="bgr8",
            )
            depth_raw = self.bridge.imgmsg_to_cv2(
                depth_message,
                desired_encoding="passthrough",
            )
            depth_meters = depth_to_meters(
                np.asarray(depth_raw),
                depth_message.encoding,
            )
        except (ValueError, TypeError, CvBridgeError, cv2.error) as exception:
            self.warn_changed(
                "conversion",
                f"Image conversion failed: {exception}",
            )
            return
        if bgr.shape[:2] != depth_meters.shape:
            self.warn_changed(
                "dimensions",
                "RGB and depth image dimensions differ",
            )
            return

        source_frame = color_message.header.frame_id
        rigid_transform = self.lookup_rigid_transform(
            source_frame,
            color_message.header.stamp,
        )
        if rigid_transform is None:
            return
        translation, quaternion = rigid_transform

        debug_image = bgr.copy()
        targets = []
        for color, ranges in self.hsv_ranges.items():
            mask = build_hsv_mask(
                bgr,
                ranges,
                morphology_size=self.morphology_size,
            )
            regions = connected_color_regions(
                mask,
                minimum_area=self.minimum_area,
                maximum_area=self.maximum_area,
            )
            for region in regions[:self.maximum_targets_per_color]:
                depth = valid_depth_median(
                    depth_meters,
                    region.mask,
                    self.minimum_depth,
                    self.maximum_depth,
                )
                if depth is None:
                    continue
                try:
                    camera_point = project_pixel_to_camera(
                        region.centroid_u,
                        region.centroid_v,
                        depth,
                        intrinsics,
                    )
                except ValueError:
                    continue
                target_point = transform_point(
                    camera_point,
                    translation,
                    quaternion,
                )
                targets.append(
                    {
                        "color": color,
                        "area_px": region.area,
                        "pixel": {
                            "u": round(region.centroid_u, 2),
                            "v": round(region.centroid_v, 2),
                        },
                        "depth_m": round(depth, 5),
                        "position": {
                            "x": round(target_point[0], 5),
                            "y": round(target_point[1], 5),
                            "z": round(target_point[2], 5),
                        },
                    }
                )
                self.draw_region(
                    debug_image,
                    color,
                    region.bounding_box,
                    depth,
                )

        targets.sort(
            key=lambda target: (
                target["color"],
                math.sqrt(
                    sum(
                        target["position"][axis] ** 2
                        for axis in ("x", "y", "z")
                    )
                ),
            )
        )
        payload = {
            "schema_version": "1.0",
            "frame_id": self.target_frame,
            "source_frame": source_frame,
            "stamp": {
                "sec": color_message.header.stamp.sec,
                "nanosec": color_message.header.stamp.nanosec,
            },
            "targets": targets,
        }
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        self.targets_publisher.publish(message)
        debug_message = self.bridge.cv2_to_imgmsg(
            debug_image,
            encoding="bgr8",
        )
        debug_message.header = color_message.header
        self.debug_publisher.publish(debug_message)
        self.last_warning_key = None

    def draw_region(
        self,
        image: np.ndarray,
        color: str,
        bounding_box: Tuple[int, int, int, int],
        depth: float,
    ) -> None:
        """Draw one labeled component on the debug image."""
        x_value, y_value, width, height = bounding_box
        cv2.rectangle(
            image,
            (x_value, y_value),
            (x_value + width, y_value + height),
            self.colors[color],
            2,
        )
        cv2.putText(
            image,
            f"{color} {depth:.2f}m",
            (x_value, max(y_value - 5, 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            self.colors[color],
            1,
            cv2.LINE_AA,
        )


def main(args=None) -> None:
    """Run the RGB-D color target detector."""
    rclpy.init(args=args)
    node = ColorTargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
