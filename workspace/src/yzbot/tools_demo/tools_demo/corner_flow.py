"""Build continuous, wall-clear arcs at the two deployed corridor corners."""

import math
from typing import List, Sequence, Tuple

from tools_demo.navigation_math import ZonePose


def has_corner_flow(poses: Sequence[ZonePose]) -> bool:
    """Identify only explicitly constructed corner routes."""
    return any(p.zone.startswith('FLOW_') for p in poses)


def flow_prefix_size(poses: Sequence[ZonePose]) -> int:
    """Return the first continuous corner group, excluding later stages."""
    if not poses or not poses[0].zone.startswith('FLOW_'):
        return 0
    family = poses[0].zone.split('_')[1]
    count = 0
    for pose in poses:
        if not pose.zone.startswith(f'FLOW_{family}_'):
            break
        count += 1
    return count


def _tag(family, index, pose):
    return ZonePose(f'FLOW_{family}_{index:02d}::{pose.zone}',
                    pose.x, pose.y, pose.yaw)


def _arc(family, center, radius, first, last, original=None):
    count = max(1, math.ceil(abs(last - first) / (math.pi / 12)))
    sign = 1 if last > first else -1
    poses = []
    for index in range(count + 1):
        angle = first + (last - first) * index / count
        pose = ZonePose(f'arc_{index}', center[0] + radius * math.cos(angle),
                        center[1] + radius * math.sin(angle), angle + sign*math.pi/2)
        if original is not None and math.hypot(pose.x-original.x, pose.y-original.y) < 1e-6:
            pose = original
        poses.append(_tag(family, index, pose))
    return poses


def _north_arc(anchor, target, reverse=False):
    # Start at the ORIGINAL north anchor, then turn above the wall. A
    # conventional fillet cuts inside it and can erase the physical proof.
    center = (anchor.x - 1.0, anchor.y)
    dx, dy = target.x - center[0], target.y - center[1]
    distance = math.hypot(dx, dy)
    if distance <= 1.0:
        return []
    angle = math.atan2(dy, dx) - math.acos(1.0 / distance)
    if not 0 < angle < math.pi/2:
        return []
    poses = _arc('A', center, 1.0, 0., angle, anchor)
    if reverse:
        poses.reverse()
        poses = [ZonePose(p.zone.replace('FLOW_A_', 'FLOW_NORTH_'),
                          p.x, p.y, p.yaw + math.pi) for p in poses]
    return poses


def round_navigation_corners(
        start: Tuple[float, float], poses: Sequence[ZonePose],
        zone: str = '') -> List[ZonePose]:
    """Round validated north/east corners, retaining their physical anchors.

    Keep narrow S stages, patrol gates, docks and all other layouts intact.
    Only free approach endpoints join a batch; precision docking stays last.
    """
    result = list(poses)
    if has_corner_flow(result) or not result or None in start:
        return result
    sx, sy = start
    # A entry: preserve the north pole, then use the tangent to a free
    # approach point. The original south red5 route already owns its prefix.
    if (zone == 'A' and -5.2 <= result[-1].x <= -4.8
            and 3.4 <= result[-1].y <= 4.1):
        matches = [i for i, p in enumerate(result)
                   if abs(p.x+2.1) < .001 and abs(p.y-1.85) < .001]
        if matches:
            index = matches[-1]
            old_south = result[0].zone == 'A_SOUTH_OPEN_CLEAR'
            target = ZonePose('A_FLOW_APPROACH', -4.15, result[-1].y, math.pi)
            previous = result[index-1] if index else ZonePose('start', sx, sy, 0.)
            incoming_north = abs(previous.x + 2.1) < .25 and previous.y < 1.0
            arc = _north_arc(result[index], target) if incoming_north else []
            if old_south and arc:
                # Keep the original south route identity and arrival guards.
                extra = [ZonePose(f'FLOW_A_ARC_{i}', p.x, p.y, p.yaw)
                         for i, p in enumerate(arc[1:])]
                result[index+1:index+1] = extra
            elif index == len(result)-2 and (arc or not incoming_north):
                group = arc or [_tag('A', 0, result[index])]
                result[index:] = group + [_tag('A', 99, target), result[-1]]
    # A -> lower west pickup: turn before the north pole and leave it
    # heading south, instead of stopping there to turn another 60 degrees.
    if -5.3 <= sx <= -3.7 and 3.2 <= sy <= 4.1:
        for index in range(len(result)-1):
            north, south = result[index:index+2]
            if (abs(north.x+2.1) < .001 and 1.79 <= north.y <= 1.86
                    and abs(south.x+2.1) < .001 and abs(south.y) < .001
                    and not north.zone.startswith('FLOW_')):
                arc = _north_arc(north, ZonePose('start', sx, sy, 0.), reverse=True)
                if arc:
                    result[index:index+2] = arc + [_tag('NORTH', 99, south)]
                break
    # The upper-right blue1 tray reaches the same wall end without the
    # lower_east helper (which only owns upper-left transfers). Its direct
    # Nav2 goal otherwise keeps the original sharp corner. Cover both
    # directions using the same physical wall gate, never a detour west.
    if not any(p.zone == 'lower_east_bay_gate' for p in result):
        end = result[-1]
        to_b = (5.0 <= sx <= 8.5 and sy >= 1.5
                and 10.8 <= end.x <= 11.8 and -5.5 <= end.y <= -4.5)
        from_b = (9.5 <= sx <= 12.0 and -5.8 <= sy <= -4.2
                  and 5.0 <= end.x <= 8.5 and end.y >= 1.5)
        if to_b:
            group = _arc('EAST', (13.4, -3.5), 1., math.pi/2, -math.pi/2)
            # A single far-east goal rounds the roof's west tip toward
            # mover 2, whose crossing recovery can put the base into that
            # roof. First reach the existing open south lane, then follow
            # its eastbound tangent. A separate FLOW family keeps this
            # exact, physically confirmed approach ahead of the arc batch.
            approach = _tag('APPROACH', 0, ZonePose(
                'lower_east_lane', 4.4, -2.5, 0.))
            result = [approach] + group + [_tag('EAST', 99, ZonePose(
                'exit', 12.4, -4.6, math.pi))] + result
        elif from_b:
            group = _arc('EAST', (13.4, -3.5), 1., -math.pi/2, math.pi/2)
            # Stay below mover 2, then turn northwest on a tangent arc.
            # Ending at (4.4,-2.5) forced the next path's rotation shim to
            # stop for a northward entry. Finish already aligned to the
            # open west flank, outside the actor's whole patrol instead.
            turn = _arc('EAST', (5.1, -1.3), 1.2, -math.pi/2, -3*math.pi/4)
            turn = [_tag('EAST', 100+i, ZonePose(
                f'west_arc_{i}', p.x, p.y, p.yaw)) for i, p in enumerate(turn)]
            target = ZonePose('west_pickup_clear', 3.2,
                              turn[-1].x + turn[-1].y - 3.2, 3*math.pi/4)
            result = group + [_tag('EAST', 99, ZonePose(
                'exit', 12.4, -2.5, math.pi))] + turn + [
                    _tag('EAST', 199, target)] + result
    # Wall_72 ends at (12.508,-3.608). Radius 1m fits outside its east
    # tip and inside Wall_10 at x=15.923, with the full tracking reserve.
    for index, pose in enumerate(result):
        if pose.zone != 'lower_east_bay_gate':
            continue
        outgoing = index > 0 and result[index-1].zone == 'lower_east_lane'
        incoming = index+1 < len(result) and result[index+1].zone == 'lower_east_lane'
        if outgoing:
            last = 0. if result[-1].y < -7.5 else -math.pi/2
            group = _arc('EAST', (13.4, -3.5), 1., math.pi/2, last, pose)
            tail = ((14.4, -4.5, -math.pi/2) if last == 0.
                    else (12.4, -4.6, math.pi))
        elif incoming:
            first = 0. if sy < -7.5 else -math.pi/2
            group = _arc('EAST', (13.4, -3.5), 1., first, math.pi/2, pose)
            tail = (12.4, -2.5, math.pi)
        else:
            break
        result[index:index+1] = group + [_tag('EAST', 99, ZonePose('exit', *tail))]
        break
    return result
