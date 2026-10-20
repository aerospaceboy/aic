"""Pure collision-avoidance math shared by the safety limiter and its tests."""

from __future__ import annotations

import math
from typing import Dict, Optional, Sequence, Tuple


def safe_crossing_phase(current_y: float, previous_y: Optional[float]) -> bool:
    """Admit the east corridor only as mover 1 retreats south below -0.65."""
    return (
        previous_y is not None
        and math.isfinite(current_y)
        and math.isfinite(previous_y)
        and current_y <= -0.65
        and previous_y - current_y >= 0.01
    )


def compute_scale(
    robot: Optional[Tuple[float, float, float]],
    obstacles: Dict[str, Tuple[float, float]],
    danger_radius: float,
    caution_radius: float,
    cone_half_angle: float = math.pi,
    motion_sign: float = 1.0,
) -> float:
    """
    Return the minimum velocity scale for forward obstacles.

    The scale is 0.0 (stop) when any obstacle is inside ``danger_radius``,
    ramps linearly from 0 to 1 across the caution band, and stays 1.0 for
    obstacles farther than ``caution_radius`` or outside the forward cone.
    """
    if robot is None or not obstacles or abs(motion_sign) < 1e-6:
        return 1.0
    robot_x, robot_y, yaw = robot
    if motion_sign < 0.0:
        yaw += math.pi
    best_scale = 1.0
    for obstacle_x, obstacle_y in obstacles.values():
        dx = obstacle_x - robot_x
        dy = obstacle_y - robot_y
        distance = math.hypot(dx, dy)
        if distance > caution_radius:
            continue
        heading = math.atan2(dy, dx)
        error = abs(
            math.atan2(
                math.sin(heading - yaw),
                math.cos(heading - yaw),
            )
        )
        if error > cone_half_angle:
            continue
        if distance <= danger_radius:
            return 0.0
        scale = (distance - danger_radius) / (
            caution_radius - danger_radius
        )
        best_scale = min(best_scale, scale)
    return best_scale


def relative_positions(
    robot_pose: Sequence[float],
    obstacle_xy: Sequence[float],
) -> Tuple[float, float]:
    """Return one obstacle's position in the robot frame."""
    robot_x, robot_y, yaw = (float(v) for v in robot_pose)
    dx = float(obstacle_xy[0]) - robot_x
    dy = float(obstacle_xy[1]) - robot_y
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    return (
        cos_y * dx + sin_y * dy,
        -sin_y * dx + cos_y * dy,
    )
