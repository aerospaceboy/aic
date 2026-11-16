"""Continuous bounded patrols on a pre-audited open polyline."""
import math

from tools_demo.navigation_math import triangle_wave


def admit_proximity_guarded_step(
    current,
    candidate,
    robot,
    stop_distance,
    release_distance,
    paused,
    *,
    paused_sec=0.0,
    freeze_timeout_sec=0.0,
):
    """Admit one patrol step without letting an actor press into the robot.

    Closing steps stop at the physical margin. Separating steps are always
    allowed, including during a guard episode; a stopped robot must not be
    required to release the actor. Timeout arguments remain accepted for
    configuration compatibility. Zero stop distance preserves original motion.
    """
    if stop_distance <= 0.0 or robot is None or current is None:
        return True, False
    if release_distance < stop_distance:
        raise ValueError("release_distance must not be below stop_distance")
    current_distance = math.dist(current, robot)
    candidate_distance = math.dist(candidate, robot)
    if paused and current_distance < release_distance:
        if candidate_distance > current_distance + 1e-8:
            return True, True
        return False, True
    if candidate_distance < stop_distance:
        if candidate_distance > current_distance + 1e-8:
            return True, True
        return False, True
    return True, False


class PolylinePatrol:
    """Constant arclength speed, reversible without position discontinuities."""

    def __init__(self, coordinates, period):
        if len(coordinates) < 4 or len(coordinates) % 2:
            raise ValueError('Patrol needs at least two XY points')
        if not all(math.isfinite(v) for v in coordinates):
            raise ValueError('Patrol coordinates must be finite')
        triangle_wave(0, period)
        self.points = tuple(zip(coordinates[::2], coordinates[1::2]))
        self.period = period
        self.lengths = tuple(math.dist(a,b) for a,b in zip(self.points,self.points[1:]))
        if min(self.lengths) <= 0:
            raise ValueError('Patrol segments cannot have zero length')
        self.length = sum(self.lengths)
        self.maximum_speed = 2*self.length/period

    def position(self, elapsed):
        remaining = self.length*(triangle_wave(elapsed,self.period)+1)/2
        for (a,b),length in zip(zip(self.points,self.points[1:]),self.lengths):
            if remaining <= length:
                fraction = remaining/length
                return (a[0]+fraction*(b[0]-a[0]),a[1]+fraction*(b[1]-a[1]))
            remaining -= length
        return self.points[-1]
