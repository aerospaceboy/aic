"""Shared spawn-corridor admission with independent live protection."""

import math

from tools_demo.navigation_math import (
    CORRIDOR_PATROL_MIN_Y, CORRIDOR_PATROL_MAX_Y, CORRIDOR_PATROL_SPEED,
    CORRIDOR_TRANSIT_SPEED, UNIVERSAL_EAST_WAIT_Y,
    corridor_crossing_clear, corridor_lane_release_clear,
)
from tools_demo.safety_math import compute_scale


def guarded_corridor_crossing_clear(
    pose: tuple[float, float, float], actor: tuple[float, float],
    velocity: float, delay: float,
) -> bool:
    """Prove the variable-speed side-pass policy, including its slowdown."""
    x, y, yaw = pose
    ox, oy = actor
    name = ('competition_moving_obstacle_1' if abs(ox - 8.5) <= .05
            else 'competition_moving_obstacle_2')
    direction = math.copysign(CORRIDOR_PATROL_SPEED, velocity)
    span = CORRIDOR_PATROL_MAX_Y - CORRIDOR_PATROL_MIN_Y
    phase_start = (oy - CORRIDOR_PATROL_MIN_Y if direction > 0.
                   else 2 * span - (oy - CORRIDOR_PATROL_MIN_Y))
    for step in range(241):
        t = step * .025
        phase = (phase_start + CORRIDOR_PATROL_SPEED * t) % (2 * span)
        ay = CORRIDOR_PATROL_MIN_Y + (phase if phase <= span else 2 * span - phase)
        if math.hypot(x - ox, y - ay) < .60:
            return False
        if x <= ox - .60:
            return True
        if t < delay:
            continue
        actors = {name: (ox, ay)}
        scale = compute_scale((x, y, yaw), actors, .75, 1.6, math.radians(65))
        travel = CORRIDOR_TRANSIT_SPEED * scale * .025
        x += math.cos(yaw) * travel
        y += math.sin(yaw) * travel
    return False


def wait_anchor_velocity(
    pose: tuple[float, float, float], anchor: tuple[float, float],
) -> tuple[float, float]:
    """Merge into the westbound lane while translating, before stopping."""
    x, y, yaw = pose
    target_x, target_y = anchor
    # Point pursuit retains the bearing to a tiny lateral residual even at
    # arrival. Align to the lane instead, so admission needs no parked turn.
    target_yaw = math.pi - math.atan(2.5 * (target_y - y))
    error = math.remainder(target_yaw - yaw, 2 * math.pi)
    remaining = max(0., x - target_x)
    # The checked approach may start 2.85 m before the waiting anchor.
    # Reserve the original precision controller for its final 0.70 m;
    # blend up to 0.70 m/s on the open prefix instead of crawling all of it.
    # At 0.70 m/s, even 0.25 s observation/command lag plus braking at
    # 1 m/s^2 fits within that unchanged slow-down reserve (0.42 m).
    # Wall-tip completion can still carry angular velocity from Nav2.
    # Merge at the original speed until both lateral and heading error
    # settle; accelerating that turn overshot into the static-map guard.
    settled = max(0., min(
        1. - abs(target_y - y) / .10,
        1. - abs(math.remainder(yaw - math.pi, 2 * math.pi)) / .30))
    cruise = min(.35 + .35 * settled,
                 .35 + .50 * max(0., remaining - .70))
    speed = min(cruise, max(.02, 1.2 * remaining))
    # Bound lateral travel while an initial heading correction is settling.
    # This shrinks speed near the checked lane edge, rather than allowing
    # heading latency to carry the body outside the proved rectangle.
    lateral = math.sin(yaw)
    if abs(lateral) > 1e-6:
        margin = .60 - y if lateral > 0. else y - .20
        speed = min(speed, max(0., 2. * margin / abs(lateral)))
    return speed * max(0., math.cos(error)), max(-.6, min(.6, 2.6 * error))


def spawn_corridor_admission(pose, actor, velocity, rolling=False):
    """Prove the full slow/fast crossing window for the actual control mode."""
    if not all(math.isfinite(v) for v in (*pose, *actor, velocity)):
        return False
    x, y, yaw = pose
    ox, oy = actor
    if (abs(y - UNIVERSAL_EAST_WAIT_Y) > .07
            or abs(math.remainder(yaw - math.pi, 2 * math.pi)) > .04
            or abs(velocity) < .02):
        return False
    if abs(ox - 8.5) < .05 and 9.3 <= x <= 10.5:
        release = 1.0
    elif abs(ox - 5.) < .05 and 5.8 <= x <= 7.2:
        release = .60
    else:
        return False
    moving_away = (oy - y) * velocity > 0.
    if rolling:
        # A moving base need not wait for a preferred phase if every full
        # trajectory below proves the returning actor remains clear.
        if abs(oy - y) < release:
            return False
    elif not corridor_lane_release_clear(y, oy, moving_away, release):
        return False
    delays = (0., .20, .40) if rolling else (0., .40, .80)
    return all(corridor_crossing_clear(
        x, y, yaw, ox, oy, velocity, crossing_speed=speed,
        dispatch_delay=delay, patrol_speed=CORRIDOR_PATROL_SPEED)
        for speed in (.56, .80, .85, CORRIDOR_TRANSIT_SPEED)
        for delay in delays) and all(guarded_corridor_crossing_clear(
            pose, actor, velocity, delay) for delay in delays)
