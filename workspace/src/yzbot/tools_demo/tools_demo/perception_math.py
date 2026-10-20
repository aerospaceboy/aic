"""Reusable RGB-D segmentation, projection, and transform helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """Minimal pinhole camera calibration."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def __post_init__(self) -> None:
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("Camera focal lengths must be positive")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Camera dimensions must be positive")


@dataclass(frozen=True)
class ColorRegion:
    """One connected color component and its binary support mask."""

    centroid_u: float
    centroid_v: float
    area: int
    bounding_box: Tuple[int, int, int, int]
    mask: np.ndarray


def depth_to_meters(depth_image: np.ndarray, encoding: str) -> np.ndarray:
    """Convert ROS depth encodings into float32 meters."""
    if depth_image.ndim != 2:
        raise ValueError("Depth image must be two-dimensional")
    normalized_encoding = encoding.upper()
    if normalized_encoding == "32FC1":
        return depth_image.astype(np.float32, copy=True)
    if normalized_encoding in ("16UC1", "MONO16"):
        return depth_image.astype(np.float32) * 0.001
    raise ValueError(f"Unsupported depth encoding: {encoding}")


def valid_depth_median(
    depth_meters: np.ndarray,
    mask: np.ndarray,
    minimum_depth: float,
    maximum_depth: float,
) -> Optional[float]:
    """Return the median finite depth inside a component mask."""
    if depth_meters.shape != mask.shape:
        raise ValueError("Depth image and mask dimensions differ")
    if minimum_depth <= 0.0 or maximum_depth <= minimum_depth:
        raise ValueError("Invalid depth range")
    valid = (
        (mask > 0)
        & np.isfinite(depth_meters)
        & (depth_meters >= minimum_depth)
        & (depth_meters <= maximum_depth)
    )
    values = depth_meters[valid]
    if values.size == 0:
        return None
    return float(np.median(values))


def project_pixel_to_camera(
    pixel_u: float,
    pixel_v: float,
    depth: float,
    intrinsics: CameraIntrinsics,
) -> Tuple[float, float, float]:
    """Project an aligned depth pixel into the optical camera frame."""
    if not math.isfinite(depth) or depth <= 0.0:
        raise ValueError("Projection depth must be finite and positive")
    if not 0.0 <= pixel_u < intrinsics.width:
        raise ValueError("Pixel u is outside the calibrated image")
    if not 0.0 <= pixel_v < intrinsics.height:
        raise ValueError("Pixel v is outside the calibrated image")
    return (
        (pixel_u - intrinsics.cx) * depth / intrinsics.fx,
        (pixel_v - intrinsics.cy) * depth / intrinsics.fy,
        depth,
    )


def normalize_quaternion(
    quaternion: Sequence[float],
) -> Tuple[float, float, float, float]:
    """Normalize an x-y-z-w quaternion."""
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError("Quaternion norm is zero")
    return tuple(float(value) / norm for value in quaternion)


def rotate_vector(
    quaternion: Sequence[float],
    vector: Sequence[float],
) -> Tuple[float, float, float]:
    """Rotate a vector using an x-y-z-w quaternion."""
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def transform_point(
    point: Sequence[float],
    translation: Sequence[float],
    quaternion: Sequence[float],
) -> Tuple[float, float, float]:
    """Apply a rigid transform to one point."""
    rotated = rotate_vector(quaternion, point)
    return tuple(
        rotated[index] + float(translation[index])
        for index in range(3)
    )


def build_hsv_mask(
    bgr_image: np.ndarray,
    ranges: Iterable[Tuple[Sequence[int], Sequence[int]]],
    morphology_size: int = 3,
) -> np.ndarray:
    """Build a combined HSV mask, including hue-wrapping colors."""
    if bgr_image.ndim != 3 or bgr_image.shape[2] != 3:
        raise ValueError("Color image must have BGR shape HxWx3")
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    combined = np.zeros(bgr_image.shape[:2], dtype=np.uint8)
    range_count = 0
    for lower, upper in ranges:
        lower_array = np.asarray(lower, dtype=np.uint8)
        upper_array = np.asarray(upper, dtype=np.uint8)
        if lower_array.shape != (3,) or upper_array.shape != (3,):
            raise ValueError("Every HSV bound must contain three values")
        combined = cv2.bitwise_or(
            combined,
            cv2.inRange(hsv, lower_array, upper_array),
        )
        range_count += 1
    if range_count == 0:
        raise ValueError("At least one HSV range is required")
    if morphology_size > 1:
        size = int(morphology_size)
        kernel = np.ones((size, size), dtype=np.uint8)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
    return combined


def connected_color_regions(
    mask: np.ndarray,
    minimum_area: int,
    maximum_area: Optional[int] = None,
) -> Tuple[ColorRegion, ...]:
    """Extract area-filtered connected components from a binary mask."""
    if mask.ndim != 2:
        raise ValueError("Mask must be two-dimensional")
    if minimum_area <= 0:
        raise ValueError("minimum_area must be positive")
    if maximum_area is not None and maximum_area < minimum_area:
        raise ValueError("maximum_area is smaller than minimum_area")
    binary = np.where(mask > 0, 255, 0).astype(np.uint8)
    count, labels, statistics, centroids = cv2.connectedComponentsWithStats(
        binary,
        connectivity=8,
    )
    regions = []
    for label in range(1, count):
        area = int(statistics[label, cv2.CC_STAT_AREA])
        if area < minimum_area:
            continue
        if maximum_area is not None and area > maximum_area:
            continue
        x_value = int(statistics[label, cv2.CC_STAT_LEFT])
        y_value = int(statistics[label, cv2.CC_STAT_TOP])
        width = int(statistics[label, cv2.CC_STAT_WIDTH])
        height = int(statistics[label, cv2.CC_STAT_HEIGHT])
        regions.append(
            ColorRegion(
                centroid_u=float(centroids[label, 0]),
                centroid_v=float(centroids[label, 1]),
                area=area,
                bounding_box=(x_value, y_value, width, height),
                mask=np.where(labels == label, 255, 0).astype(np.uint8),
            )
        )
    regions.sort(key=lambda region: region.area, reverse=True)
    return tuple(regions)
