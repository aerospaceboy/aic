"""Keep southeast B deliveries below the continuous obstacle sweep."""
import math
from tools_demo.navigation_math import ZonePose


def b_south_waypoints(x, y, dock):
    """Apply only to the red4 southeast departure and an east-side B dock."""
    if x is None or y is None or x <= 10.0 or y >= -3.6 or dock.x <= 9.0:
        return []
    points = [(9.7, -2.4), (3.8, -2.6)]
    following = points[1:] + [(dock.x, dock.y)]
    return [ZonePose(f'B_SOUTH_SWEEP_{i}', p[0], p[1],
                     math.atan2(q[1] - p[1], q[0] - p[0]))
            for i, (p, q) in enumerate(zip(points, following))]


def b_northeast_waypoints(x, y, dock):
    """Route a north-east start around the central pickup trays.

    The moved red_cube_2 (0, 3) and red_cube_4 (0, -1) trays sit in the
    central corridor and the rock at (1.73, 1.63) closes the short crossing
    beside them.  For a start north of the y ~ 1.0 wall band the only
    zero-cost passage therefore runs above the (0, 3) tray at y ~ 4.0 and
    then south through the lane west of both trays.  Without this guidance
    Smac hugs the rock's inflation ceiling near y ~ 2.6; the controller
    tracks that path into contact range, the lidar then marks lethal cells
    around the robot and every candidate trajectory is rejected.  Measured
    2026-09-10: blue_cube_1 -> B aborted twice with the robot's own cell at
    cost 100 while it carried the cube, which failed the round.
    """
    if x is None or y is None or dock is None:
        return []
    if x <= 0.9 or y <= 1.5:
        return []
    # Climb in the free lane WEST of the blue pickup tray (x = 4.1 .. 5.0 at
    # y = 1.6 .. 2.5), not beside it. Measured 2026-09-12 (AB3): the east dock
    # start (5.08,1.92) was clamped to x = 4.6, inside that tray's NearSlow
    # polygon, so the collision monitor toggled 179 slowdowns in one 84.8 s
    # leg (normal 20-30 s) and RPP reported "collision ahead" 36 times.
    ascent_x = min(max(float(x), 3.0), 3.7)
    points = [(ascent_x, 4.0), (-1.6, 4.0)]
    following = points[1:] + [(dock.x, dock.y)]
    return [ZonePose(f'B_NE_CORRIDOR_{i}', p[0], p[1],
                     math.atan2(q[1] - p[1], q[0] - p[0]))
            for i, (p, q) in enumerate(zip(points, following))]


def b_central_south_waypoints(x, y, dock):
    """Guide central-shelf departures down the free west lane to B.

    Starts near the r4/r2 trays (|x|<2, -3<y<3) otherwise send Smac
    straight through the middle corridor where NearSlow toggles on the
    tray/obstacle inflation.  Stay just east of Wall_57 (x=-3.2) at
    x=-2.2, then drop south toward the B bay.
    """
    if x is None or y is None or dock is None:
        return []
    if not (-2.0 <= float(x) <= 1.2):
        return []
    if not (-3.0 <= float(y) <= 3.0):
        return []
    if dock.y > -8.0:
        return []
    points = [(-2.20, -1.00), (-2.20, -6.00)]
    following = points[1:] + [(dock.x, dock.y)]
    return [
        ZonePose(
            f"B_CENTRAL_SOUTH_{i}",
            p[0],
            p[1],
            math.atan2(q[1] - p[1], q[0] - p[0]),
        )
        for i, (p, q) in enumerate(zip(points, following))
    ]


def b_central_north_waypoints(x, y, dock):
    """Clear the red2 tray west before descending to B.

    A loaded base can finish the red2 departure at about (-0.4, 3.0),
    inside the tray's inflated costmap fringe.  A direct B goal then starts
    with a diagonal southward path through that fringe.  This route first
    moves west into the open lane, then descends before turning for B.  Keep
    the guard north-only: the older south-start route was measured slower
    and remains disabled for ordinary r4/r2 deliveries.
    """
    if x is None or y is None or dock is None:
        return []
    if not (-1.2 <= float(x) <= 0.9):
        return []
    if not (1.5 <= float(y) <= 3.6) or dock.y > -8.0:
        return []
    points = [(-2.20, float(y)), (-2.20, -1.00), (-2.20, -6.00)]
    following = points[1:] + [(dock.x, dock.y)]
    return [
        ZonePose(
            f"B_CENTRAL_NORTH_{i}",
            p[0],
            p[1],
            math.atan2(q[1] - p[1], q[0] - p[0]),
        )
        for i, (p, q) in enumerate(zip(points, following))
    ]
