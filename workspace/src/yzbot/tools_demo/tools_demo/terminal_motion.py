"""Bounded holonomic delivery entry, separate from ordinary path pursuit."""
import math

from tools_demo.navigation_math import ZonePose


def delivery_entry(dock, backoff=.65):
    """Finish path tracking outside the pad before turning beside its cubes."""
    if not all(math.isfinite(v) for v in (dock.x, dock.y, dock.yaw, backoff)):
        raise ValueError('Nonfinite delivery entry')
    if not .30 <= backoff <= .8:
        raise ValueError('Unbounded delivery entry')
    return ZonePose(dock.zone + '_delivery_entry',
                    dock.x - backoff * math.cos(dock.yaw),
                    dock.y - backoff * math.sin(dock.yaw), dock.yaw)


def terminal_command(pose, dock, maximum_speed=.40,
                     heading_tolerance=.045, arrival_tolerance=.025,
                     lateral_first=True):
    """Align outside, trim the slot line, then advance with constant heading.

    A holonomic base need not curve or reverse its heading to remove a
    lateral residual. A 180 degree turn belongs at the external entry.
    """
    x, y, yaw = pose
    if not all(math.isfinite(v) for v in
               (x, y, yaw, dock.x, dock.y, dock.yaw, maximum_speed)):
        raise ValueError('Nonfinite terminal observation')
    error = math.remainder(dock.yaw - yaw, 2 * math.pi)
    dx, dy = dock.x - x, dock.y - y
    along = dx * math.cos(dock.yaw) + dy * math.sin(dock.yaw)
    across = -dx * math.sin(dock.yaw) + dy * math.cos(dock.yaw)
    angular = max(-1.5, min(1.5, 3.5 * error))
    if abs(error) > heading_tolerance:
        return (0., 0., angular), 'heading'
    if math.hypot(dx, dy) <= arrival_tolerance:
        return (0., 0., 0.), 'arrived'
    # Trim lateral error while there is still room outside the occupied pad.
    # After entering, retain small lateral feedback without changing heading.
    forward = max(-.20, min(maximum_speed, 1.8 * along))
    lateral = max(-.20, min(.20, 2.0 * across))
    if lateral_first and abs(across) > .02 and along > .30:
        # Trimming beside a zone wall can put the .38 m body tube against the
        # static clearance bound and stall the dock. The caller may ask for a
        # forward-first variant that keeps advancing to open the corner first.
        forward = 0.
    vx = forward * math.cos(error) - lateral * math.sin(error)
    vy = forward * math.sin(error) + lateral * math.cos(error)
    return (vx, vy, angular), 'translation'


def ground_cubes_clear(pose, cubes):
    """Conservative wheel/body rectangle versus validated ground cubes.

    URDF: 0.32 m chassis, wheels at x=.14 with .065 m radius and y=.185
    with .025 m half-width. The rectangle encloses both. Cube circumradius
    .050 plus .015 margin expands it. Carried cubes are above these cubes;
    static obstacles still use the independent loaded .38 m tube.
    """
    x, y, yaw = pose
    if not all(math.isfinite(v) for v in pose):
        return False
    c, s = math.cos(yaw), math.sin(yaw)
    for ox, oy in cubes:
        if not math.isfinite(ox + oy):
            return False
        dx, dy = ox - x, oy - y
        if abs(c * dx + s * dy) <= .275 and abs(-s * dx + c * dy) <= .285:
            return False
    return True


def command_sweep(pose, command, horizon=.35):
    """Sample a low-speed command through reaction and braking distance."""
    x, y, yaw = pose
    vx, vy, wz = command
    if not all(math.isfinite(v) for v in (*pose, *command, horizon)):
        raise ValueError('Invalid terminal sweep')
    if math.hypot(vx, vy) > .50 or abs(wz) > 1.5 or not .3 <= horizon <= .5:
        raise ValueError('Terminal command outside bounded dynamics')
    poses = [(x, y, yaw)]
    count = max(1, math.ceil(horizon / .025))
    dt = horizon / count
    for _ in range(count):
        middle = yaw + .5 * wz * dt
        x += (vx * math.cos(middle) - vy * math.sin(middle)) * dt
        y += (vx * math.sin(middle) + vy * math.cos(middle)) * dt
        yaw += wz * dt
        poses.append((x, y, yaw))
    return poses


def terminal_route(pose, dock):
    """Prove rotation, lateral trim and the remaining dock stroke separately."""
    x, y, yaw = pose
    error = math.remainder(dock.yaw - yaw, 2 * math.pi)
    points = [(x, y, yaw + error * i / max(1, math.ceil(abs(error) / .04)))
              for i in range(max(1, math.ceil(abs(error) / .04)) + 1)]
    across = -(dock.x - x) * math.sin(dock.yaw) + (dock.y - y) * math.cos(dock.yaw)
    trim = (x - across * math.sin(dock.yaw), y + across * math.cos(dock.yaw))
    for start, end in [((x, y), trim), (trim, (dock.x, dock.y)),
                       ((x, y), (dock.x, dock.y))]:
        count = max(1, math.ceil(math.hypot(end[0] - start[0], end[1] - start[1]) / .025))
        points.extend((start[0] + (end[0] - start[0]) * i / count,
                       start[1] + (end[1] - start[1]) * i / count, dock.yaw)
                      for i in range(count + 1))
    return points, trim
