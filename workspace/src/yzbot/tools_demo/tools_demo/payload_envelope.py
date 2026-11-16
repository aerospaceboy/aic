"""Conservative planar navigation envelope for the carried arm and load."""
import math


def navigation_footprint(loaded):
    """Use a circumscribed base polygon, plus the forward carried assembly."""
    radius = .28 / math.cos(math.pi / 16)
    points = [(radius * math.cos(i * math.pi / 8),
               radius * math.sin(i * math.pi / 8)) for i in range(16)]
    if loaded:
        points.extend(((.46, -.12), (.46, .12)))
    # Monotonic-chain convex hull preserves the base in every orientation.
    ordered = sorted(set(points))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]
