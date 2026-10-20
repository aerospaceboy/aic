"""Conservative admission for a bounded emergency retreat, not planning.

The existing truth-backed final safety channel may approve a retreat only
when its whole duration is collision-free and its waiting endpoint is outside
both complete patrols. Unknown velocity uses an all-direction reach bound.
Laser-based Nav2 remains responsible for selecting the normal route.
"""

import math


def distance_to_patrol(point, flat_points):
    """Return distance to a finite XY polyline, rejecting malformed input."""
    if len(flat_points) < 4 or len(flat_points) % 2:
        raise ValueError('A patrol requires at least two XY points')
    if not all(math.isfinite(v) for v in (*point, *flat_points)):
        raise ValueError('Non-finite patrol geometry')
    points = list(zip(flat_points[::2], flat_points[1::2]))
    distances = []
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        squared = dx*dx+dy*dy
        if squared <= 1e-12:
            raise ValueError('Degenerate patrol segment')
        ratio = max(0.0, min(1.0, ((point[0]-a[0])*dx+
                                 (point[1]-a[1])*dy)/squared))
        distances.append(math.hypot(point[0]-a[0]-ratio*dx,
                                    point[1]-a[1]-ratio*dy))
    return min(distances)


def retreat_admitted(pose, distance, speed, current_speed, actors, patrols):
    """Validate braking, reverse sweep and an indefinitely safe endpoint.

    Actors are (x, y, radius, source_age_seconds). Speeds are conservatively
    bounded by 0.30 m/s (above both current 0.25 m/s patrols), independent of
    inferred heading. The 0.10 m physical safety margin is never reduced.
    """
    try:
        numbers = (*pose, distance, speed, current_speed,
                   *(value for actor in actors for value in actor))
        if not all(math.isfinite(v) for v in numbers):
            return False
        if (len(pose) != 3 or len(actors) < 2 or len(patrols) != 2 or
                not 0.0 < distance <= .50 or not 0.0 < speed <= .25 or
                not 0.0 <= current_speed <= 1.0):
            return False
        if any(len(a) != 4 or a[2] <= 0 or not 0 <= a[3] <= .75
               for a in actors):
            return False
        x, y, yaw = pose
        cosine, sine = math.cos(yaw), math.sin(yaw)
        endpoint = (x-distance*cosine, y-distance*sine)
        if any(distance_to_patrol(endpoint, patrol) < .60 for patrol in patrols):
            return False
        reaction = .20
        coast = current_speed*reaction
        duration = reaction+(distance+coast)/speed
        steps = math.ceil(duration/.025)
        for index in range(steps+1):
            elapsed = duration*index/steps
            forward = (current_speed*elapsed if elapsed <= reaction else
                       coast-speed*(elapsed-reaction))
            bx, by = x+forward*cosine, y+forward*sine
            for ax, ay, radius, age in actors:
                reachable = .30*(age+elapsed)
                # Sampling between 25 ms endpoints also needs a speed-bound
                # allowance; no inferred stationary interval earns credit.
                allowance = (.30+max(speed, current_speed))*.025
                if math.hypot(bx-ax, by-ay)-reachable-allowance < .28+radius+.10:
                    return False
                # The final guard slows reverse commands near the rear. Its
                # unchanged 1.60 m caution radius must not stretch this timed
                # admission while an actor approaches from the front.
                front = (ax-bx)*cosine+(ay-by)*sine
                if (front-reachable-allowance <= 0 and
                        math.hypot(bx-ax, by-ay)-reachable-allowance < 1.60):
                    return False
        return True
    except (TypeError, ValueError, IndexError):
        return False
