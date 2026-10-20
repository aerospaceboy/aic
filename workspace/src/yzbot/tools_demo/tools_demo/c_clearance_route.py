"""Opt-in C delivery corridors; require flight validation before promotion."""
import math
from tools_demo.navigation_math import ZonePose


def c_clearance_waypoints(x, y, dock):
    if x is None or y is None or y >= 0.5:
        return []
    if x > 8.8 and y < -7.0:
        points = [(9.65, -6.0), (7.2, -6.0), (4.6, -6.5),
                  (3.1, -6.75), (2.55, -6.25), (-1.4, -4.5),
                  (-1.4, -2.4), (-2.1, 0.0), (-2.1, 1.8)]
    elif 3.8 < x < 5.2 and y < -10.5:
        points = [(4.8, -11.65), (3.3, -11.65), (2.5, -11.1),
                  (0.8, -10.4), (-0.1, -8.8), (0.0, -6.0),
                  (-1.4, -4.5), (-1.4, -2.4), (-2.1, 0.0), (-2.1, 1.8)]
    elif -9.9 < x < -8.8 and -3.3 < y < -2.7:
        points = [(-5.0, -3.2), (-4.1, -2.2), (-4.1, 0.0),
                  (-2.1, 0.0), (-2.1, 1.8)]
    else:
        return []
    targets = points[1:] + [(dock.x, dock.y)]
    return [ZonePose(f'C_clearance_{i}', p[0], p[1],
                     math.atan2(q[1]-p[1], q[0]-p[0]))
            for i, (p, q) in enumerate(zip(points, targets))]
