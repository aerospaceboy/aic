"""Tests for RGB-D segmentation, projection, and transforms."""

import math

import cv2
import numpy as np
import pytest

from tools_demo.perception_math import build_hsv_mask
from tools_demo.perception_math import CameraIntrinsics
from tools_demo.perception_math import connected_color_regions
from tools_demo.perception_math import depth_to_meters
from tools_demo.perception_math import project_pixel_to_camera
from tools_demo.perception_math import transform_point
from tools_demo.perception_math import valid_depth_median


def test_depth_encodings_convert_to_meters():
    """Convert uint16 millimeters and preserve float meters."""
    millimeters = np.array([[500, 1250]], dtype=np.uint16)
    meters = np.array([[0.5, 1.25]], dtype=np.float32)

    assert np.allclose(
        depth_to_meters(millimeters, "16UC1"),
        meters,
    )
    assert np.allclose(depth_to_meters(meters, "32FC1"), meters)


def test_depth_median_ignores_nan_zero_and_out_of_range():
    """Use only finite in-range depth samples inside the mask."""
    depth = np.array(
        [
            [math.nan, 0.0, 0.4],
            [0.5, 0.6, 9.0],
        ],
        dtype=np.float32,
    )
    mask = np.full(depth.shape, 255, dtype=np.uint8)

    result = valid_depth_median(depth, mask, 0.1, 3.0)

    assert result == pytest.approx(0.5)


def test_pinhole_projection_at_center_and_offset():
    """Project the principal point and one horizontal offset."""
    intrinsics = CameraIntrinsics(
        fx=500.0,
        fy=500.0,
        cx=320.0,
        cy=240.0,
        width=640,
        height=480,
    )

    assert project_pixel_to_camera(
        320.0,
        240.0,
        2.0,
        intrinsics,
    ) == pytest.approx((0.0, 0.0, 2.0))
    assert project_pixel_to_camera(
        420.0,
        240.0,
        2.0,
        intrinsics,
    ) == pytest.approx((0.4, 0.0, 2.0))


def test_rigid_transform_rotates_then_translates():
    """Apply a 90-degree yaw followed by translation."""
    half_angle = math.pi / 4.0
    quaternion = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))

    result = transform_point(
        (1.0, 0.0, 0.0),
        (1.0, 2.0, 3.0),
        quaternion,
    )

    assert result == pytest.approx((1.0, 3.0, 3.0))


def test_red_hue_wrap_and_blue_components_are_detected():
    """Segment synthetic red and blue regions while rejecting small noise."""
    image = np.zeros((120, 180, 3), dtype=np.uint8)
    cv2.rectangle(image, (10, 20), (49, 59), (0, 0, 255), -1)
    cv2.rectangle(image, (100, 40), (149, 89), (255, 0, 0), -1)
    image[5, 5] = (0, 0, 255)
    red_mask = build_hsv_mask(
        image,
        (
            ((0, 100, 70), (10, 255, 255)),
            ((170, 100, 70), (179, 255, 255)),
        ),
        morphology_size=3,
    )
    blue_mask = build_hsv_mask(
        image,
        (((95, 100, 60), (135, 255, 255)),),
        morphology_size=3,
    )

    red_regions = connected_color_regions(red_mask, minimum_area=100)
    blue_regions = connected_color_regions(blue_mask, minimum_area=100)

    assert len(red_regions) == 1
    assert red_regions[0].area == 1600
    assert red_regions[0].centroid_u == pytest.approx(29.5)
    assert len(blue_regions) == 1
    assert blue_regions[0].area == 2500
    assert blue_regions[0].centroid_v == pytest.approx(64.5)


@pytest.mark.parametrize(
    "encoding",
    ("8UC1", "rgb8", ""),
)
def test_unsupported_depth_encoding_is_rejected(encoding):
    """Reject encodings whose metric scale is ambiguous."""
    with pytest.raises(ValueError):
        depth_to_meters(np.ones((2, 2), dtype=np.uint8), encoding)
