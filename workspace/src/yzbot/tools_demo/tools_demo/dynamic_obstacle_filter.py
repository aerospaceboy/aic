"""Separate live free-space returns from known static map geometry."""

import math


def distance_to_box(x, y, center_x, center_y, size_x, size_y):
    """Return distance to an axis-aligned physical wall, not its centre."""
    return math.hypot(
        max(abs(x - center_x) - size_x / 2.0, 0.0),
        max(abs(y - center_y) - size_y / 2.0, 0.0),
    )


def static_return(grid, width, height, resolution, origin, x, y,
                  tolerance=0.18):
    """Match a return to static occupied cells without marking unknown free."""
    ox, oy, yaw = origin
    dx, dy = x - ox, y - oy
    gx = (math.cos(yaw) * dx + math.sin(yaw) * dy) / resolution
    gy = (-math.sin(yaw) * dx + math.cos(yaw) * dy) / resolution
    cx, cy = math.floor(gx), math.floor(gy)
    radius = math.ceil(tolerance / resolution) + 1
    for row in range(max(0, cy - radius), min(height, cy + radius + 1)):
        for col in range(max(0, cx - radius), min(width, cx + radius + 1)):
            if grid[row * width + col] < 65:
                continue
            if math.hypot((col + 0.5 - gx) * resolution,
                          (row + 0.5 - gy) * resolution) <= tolerance:
                return True
    return False


def transform_point(point, translation, quaternion):
    """Apply a complete rigid transform, including laser roll and pitch."""
    x, y, z = point
    qx, qy, qz, qw = quaternion
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty + translation[0],
        y + qw * ty + qz * tx - qx * tz + translation[1],
        z + qw * tz + qx * ty - qy * tx + translation[2],
    )
