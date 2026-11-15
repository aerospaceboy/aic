"""Relocate occupied virtual route anchors within a statically proved tube."""
import math
from tools_demo.navigation_math import ZonePose


def resolve_transit_anchor(pose, previous, following, cost_at, line_clear):
    """Never shift a dock, pickup, corridor admission gate, or final target.

    The live planner still owns the motion. Only an intermediate, statically
    free waypoint whose live cell became occupied can be moved, by <= 1 m.
    Both neighbouring segments and the lateral shift must clear the full body.
    Missing/unknown geometry preserves the original route (no unproved bypass).
    """
    allowed = pose.zone == 'universal_origin_staging' or pose.zone in (
        'A_WALL_NORTH_CLEAR', 'A_WALL_SOUTH_CLEAR',
        'A_WALL_CENTRAL_CLEAR', 'A_WALL_RED2_NORTH_CLEAR')
    if not allowed or previous is None or following is None:
        return pose
    center = (pose.x, pose.y)
    cost = cost_at(*center)
    if cost is None or cost < 65 or not line_clear(center, center):
        return pose
    candidates = []
    for radius in (.4, .6, .8, 1.0):
        for index in range(16):
            angle = index * math.pi / 8
            point = (pose.x+radius*math.cos(angle), pose.y+radius*math.sin(angle))
            live_cost = cost_at(*point)
            if live_cost is None or not 0 <= live_cost <= 30:
                continue
            if not (line_clear(center, point) and line_clear(previous, point)
                    and line_clear(point, following)):
                continue
            score = math.dist(previous, point)+math.dist(point, following)+.3*radius
            candidates.append((score, point))
    if not candidates:
        return pose
    _, point = min(candidates)
    return ZonePose(pose.zone, *point, pose.yaw)
