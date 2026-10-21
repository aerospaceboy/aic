"""Shared, ROS-independent routes for the deployed distributed-tray layout."""

import math
from typing import List

from tools_demo.navigation_math import (
    ZonePose, dynamic_gate_waypoints, lower_east_corridor_waypoints,
    bay_pocket_waypoints, wall57_pocket_waypoints, a_wall_tip_waypoints,
    layout_a_sweep_waypoints, c_southeast_transport_waypoints,
)


CENTRAL_MOVER_GATE_WEST = ZonePose(
    "CENTRAL_MOVER_GATE_WEST", -0.60, -4.70, 0.0)
CENTRAL_MOVER_GATE_EAST = ZonePose(
    "CENTRAL_MOVER_GATE_EAST", 1.00, -4.70, math.pi)
CENTRAL_RED2_NORTH_ENTRY = ZonePose(
    "central_red2_north_entry", 1.90, -4.70, -math.pi / 2.0)
CENTRAL_PATROL_NORTH_FLANK = ZonePose(
    "central_patrol_north_flank", -1.20, -2.20, -math.pi / 2.0)
CENTRAL_PATROL_SOUTH_FLANK = ZonePose(
    # Nav2 may stop 0.20 m before its goal. Aim 0.20 m farther south on
    # this open flank, so even that early arrival starts below the -4.70
    # transit line rather than rounding its approach toward the carts.
    "central_patrol_south_flank", -1.20, -4.90, 0.0)

# Obstacle_2 sweeps x=[-1.3, 0.1] at y=1.0 beside the west end of the long
# upper wall. A north-to-south NavFn route otherwise cuts between the moving
# board and that wall. Cross on the board's west side with 0.8 m lateral
# clearance, then continue on the appropriate side of the lower L-wall.
OBSTACLE_2_NORTH_SOUTH_GATES = (
    ZonePose("obstacle_2_north_gate", -2.10, 1.80, -math.pi / 2.0),
    ZonePose("obstacle_2_south_gate", -2.10, 0.00, -1.48),
)
OBSTACLE_2_LOWER_RIGHT_GATE = ZonePose(
    "obstacle_2_lower_right", -1.40, -7.60, 3.03
)
OBSTACLE_2_WEST_UPPER_GATE = ZonePose(
    "obstacle_2_west_upper", -4.20, 0.00, -2.58
)


def continuous_a_route(poses):
    """Identify the south departure's ordered, physically checked corridor."""
    return (len(poses) in (4, 5)
            and [(p.zone, p.x, p.y) for p in poses[:3]] == [
                ("A_SOUTH_OPEN_CLEAR", -1.20, -4.55),
                ("A_WALL_SOUTH_CLEAR", -2.10, -.30),
                ("A_WALL_NORTH_CLEAR", -2.10, 1.85)]
            and poses[3].zone == "A_SOUTH_DOCK_APPROACH"
            and poses[3].x == -4.15 and 3.4 <= poses[3].y <= 4.1
            and (len(poses) == 4 or (
                -5.2 <= poses[-1].x <= -4.8
                and poses[-1].y == poses[3].y)))


def bay_exit_staging(start, place, distance):
    """Keep the existing south-east departure on its open aisle."""
    x, y = start
    if distance <= 0.0 or x is None or y is None or not (
            x > 10.4 and y < -11.0):
        return None
    if place.y == y:
        return None
    staging_y = y + math.copysign(distance, place.y - y)
    if abs(place.y - staging_y) < 0.5:
        return None
    return ZonePose("bay_exit_staging", x, staging_y, math.pi / 2.0)


def current_pickup_guides(zone, start_x, start_y, entry,
                          responses_complete=False, log=lambda _msg: None):
    """Return the layout's validated pickup guides without ROS state."""
    # Original spawn corridor and its two movers did not move.
    # Retain initial staging, disable only old resource/zone guides.
    if start_x is None or start_y is None or entry is None:
        return []
    if zone == "RED_CUBE_1":
        pocket = wall57_pocket_waypoints(
            (start_x, start_y), (entry.x, entry.y))
        if pocket:
            log("Wall57_pocket_pickup_route active")
            return list(pocket)
    if (zone == "BLUE_CUBE_1"
            and -5.2 <= start_x <= -3.8 and 3.4 <= start_y <= 4.1
            and 6.3 <= entry.x <= 6.7
            and 3.4 <= entry.y <= 3.8):
        # A -> blue1 must pass SOUTH of the low red3 tray and the
        # Wall_29 tip, but north of Wall_31. The direct north-row
        # goal reached (-0.49,3.40), then repeatedly predicted tray
        # contact (v20). Preserve a physically checked clear lane.
        return [ZonePose("north_pickup_tray_west_clear", -1.2, 3.0, 0.),
                ZonePose("north_pickup_tray_east_clear", 6.5, 3.0,
                         math.pi / 2.0)]
    # Wall/patrol clearance guides protect the route. Do not add an out/back
    # solely to manufacture avoidance evidence; telemetry reports what occurs.
    central_gates = []
    # A retry that starts beside the dock must not re-cross the mover
    # corridor: the first attempt already earned the crossing, and
    # re-running it burns ~20 s and ends in "no clear escape"
    # (20261014T123209: second attempt started 0.42 m from the dock,
    # drove back to x=-0.20 through both movers, then aborted).
    near_dock = math.hypot(
        entry.x - start_x, entry.y - start_y
    ) <= 1.5
    # These fixed gates serve the former south-side red-2 tray.
    # Binding them only to the entity name sent the relocated tray
    # at (-0.8, 0.8) through y=-4.7 and back north before docking.
    south_red2_entry = (
        zone == "RED_CUBE_2"
        and entry.y < CENTRAL_MOVER_GATE_EAST.y
    )
    if south_red2_entry and not near_dock:
        central_gates = [
            CENTRAL_MOVER_GATE_WEST, CENTRAL_MOVER_GATE_EAST]
        # During route construction dock_pose is the tray-clear
        # ENTRY (1.9,-5.4); next_dock_pose is the final grasp dock.
        if (1.7 <= entry.x <= 2.1
                and abs(entry.y + 5.4) <= .05):
            central_gates.append(CENTRAL_RED2_NORTH_ENTRY)
    guides = list(dynamic_gate_waypoints(
        (start_x, start_y), (entry.x, entry.y)))
    if not guides:
        guides = list(lower_east_corridor_waypoints(
            (start_x, start_y),
            (entry.x, entry.y),
        ))
    if not guides:
        guides = list(bay_pocket_waypoints(
            (start_x, start_y),
            (entry.x, entry.y),
        ))
    # The original movers did not move, so their bypass gates still
    # apply. Dropping them sent a north-side start on a straight
    # diagonal through the shelf rows (20261013T225015: blue_cube_2
    # clipped the bottom shelf of its rack and RPP reported collision
    # ahead for the whole leg).
    bypass: List[ZonePose] = []
    if (
        zone.startswith("BLUE_CUBE_")
        and start_y >= 1.5
        and entry.y < 0.5
    ):
        bypass = list(OBSTACLE_2_NORTH_SOUTH_GATES)
        if zone == "BLUE_CUBE_2":
            bypass.append(OBSTACLE_2_WEST_UPPER_GATE)
        # BLUE_CUBE_3 now docks at (-1.64,-6), not the legacy
        # (-6,-7) bay. Its central gate already supplies the safe
        # southern access. The old (-1.4,-7.6) anchor sends it past
        # the dock, back north through the carts, then south again.
        log(
            f"obstacle_2_west_bypass zone={zone}"
        )
    # Existing wall/spawn guides must run first. The short mover gate
    # is the final transit immediately before the precision entry.
    prefix = guides + bypass
    if zone == "BLUE_CUBE_3" and not near_dock:
        flank_start = prefix[-1] if prefix else ZonePose(
            "start", start_x, start_y, 0.)
        if flank_start.y >= CENTRAL_PATROL_NORTH_FLANK.y:
            # Descend outside both complete patrols, even without response
            # evidence. Never turn east from this safe western approach.
            prefix += [CENTRAL_PATROL_NORTH_FLANK,
                       CENTRAL_PATROL_SOUTH_FLANK]
        elif (flank_start.y > CENTRAL_MOVER_GATE_WEST.y + .08
              and flank_start.x <= -.8):
            prefix.append(CENTRAL_PATROL_SOUTH_FLANK)
        elif (flank_start.y > CENTRAL_MOVER_GATE_WEST.y + .08
              and flank_start.x > 1.2):
            # An eastern start needs a south-side clearance point before
            # going west to the pickup, not an out-and-back demonstration.
            prefix.append(CENTRAL_MOVER_GATE_EAST)
        log("blue3_encounter_detour_disabled task_route_only")
        return prefix
    if central_gates:
        flank_start = prefix[-1] if prefix else ZonePose(
            "start", start_x, start_y, 0.0)
        if flank_start.y >= -2.20 and flank_start.x <= 1.20:
            # A safe crossing endpoint does not make its approach
            # safe: the old diagonal reached (0.662,-2.892) beside
            # cart 2's north reversal, leaving only 0.0776 m. The
            # eastern flank is a static wall, not an available gap.
            # Descend west of BOTH complete sweeps, then cross on
            # the southern line. Serial safety anchors prevent RPP
            # rounding the whole approach back into that diagonal.
            prefix += [CENTRAL_PATROL_NORTH_FLANK,
                       CENTRAL_PATROL_SOUTH_FLANK]
    return prefix + central_gates


# The south-east pickup bay opens only on its west side: a wall runs from
# x=10.7 east at y=-7.65 and the direct line from the bay to zone B crosses
# it (map clearance 0.00 m). AB8-3/FX1 lost a round because the direct goal
# put RPP's projection into that wall for ~50 s. These anchors leave the bay
# on the widest line; measured map clearance per leg is 0.60/0.60/0.86/0.87 m.
BAY_POCKET_NORTH_ANCHORS = (
    (12.30, -8.40, math.pi / 2.0),
    (9.90, -8.30, math.pi),
    (9.75, -7.70, math.pi / 2.0),
    (10.00, -6.30, math.pi / 2.0),
)


def bay_pocket_north_route(start, place, zone=None):
    """Return the exit anchors when a zone-B delivery starts in the bay.

    Anchors the base has already passed are trimmed, so a start that is
    already west of the wall or north of the bay's first anchor does not
    drive backwards to rejoin the full chain.
    """
    if zone is not None and zone != "B":
        return ()
    x, y = start
    if x is None or y is None:
        return ()
    if not (9.40 < x < 13.40 and -10.80 < y < -7.55):
        return ()
    if place is None or place.y < -7.0:
        return ()
    anchors = list(enumerate(BAY_POCKET_NORTH_ANCHORS, 1))
    if x < 10.40:
        anchors = anchors[2:]          # already past the west run
    elif y > -8.60:
        anchors = anchors[1:]          # already north of the bay anchor
    return tuple(
        ZonePose(f"bay_pocket_north_{index}", px, py, yaw)
        for index, (px, py, yaw) in anchors
    )


def current_delivery_route(zone, start_x, start_y, place, backup=False,
                           bay_distance=2.0, base_guides=(),
                           log=lambda _msg: None):
    """Return validated delivery poses and the A-corridor ownership flag."""
    pending = list(base_guides) + [place]
    dedicated = False
    if (zone == "A" and start_x is not None and start_y is not None
            and -4.3 <= start_x <= -3.3 and -13.0 <= start_y <= -11.5
            and -5.2 <= place.x <= -4.8 and 3.4 <= place.y <= 4.1):
        # The lower diagonal joins Wall_107 on its west side. Plan around
        # its east tip, then continue north outside the carts' west sweep.
        # The old west and south-tip anchors sent this departure back west
        # across open space after it had already cleared the diagonal.
        pending = [
            ZonePose("A_SOUTH_OPEN_CLEAR", -1.20, -4.55, math.pi / 2.0),
            ZonePose("A_WALL_SOUTH_CLEAR", -2.10, -.30, math.pi / 2.0),
            ZonePose("A_WALL_NORTH_CLEAR", -2.10, 1.85, math.pi / 2.0),
            ZonePose("A_SOUTH_DOCK_APPROACH", -4.15, place.y, math.pi),
            place]
        log("A_south_continuous_route active")
        return pending, True
    pocket = (wall57_pocket_waypoints(
        (start_x, start_y), (place.x, place.y))
        if zone == "A" and start_x is not None and start_y is not None
        else ())
    if pocket:
        pending = list(pocket) + [place]
        dedicated = True
        log("Wall57_pocket_return_route active")
        return pending, dedicated
    if (zone == "A" and start_x is not None and start_y is not None
            and 6.3 <= start_x <= 6.7 and 3.4 <= start_y <= 3.8
            and -5.2 <= place.x <= -4.8 and 3.6 <= place.y <= 4.0):
        # The same low red3 tray must be cleared while CARRYING
        # back from blue1, not only on the empty outbound journey.
        pending = [
            ZonePose("north_delivery_tray_east_clear", 6.5, 3.0, math.pi),
            ZonePose("north_delivery_tray_west_clear", -1.2, 3.0, math.pi),
            place]
        dedicated = True
        log("A_north_tray_clearance_route active")
        return pending, dedicated
    a_guides = ()
    if (zone == "A" and start_x is not None
            and start_y is not None):
        a_guides = a_wall_tip_waypoints(
            (start_x, start_y), (place.x, place.y))
    if a_guides:
        corridor = lower_east_corridor_waypoints(
            (start_x, start_y), (place.x, place.y))
        if (not corridor and not backup
                and -1.5 <= start_x <= 2.5
                and start_y < CENTRAL_MOVER_GATE_WEST.y):
            # South-central cargo must clear the west of the full
            # patrol before climbing toward A. The former diagonal
            # stalled at (-0.64,-3.72), retried and spent 37.5 s on
            # this leg (20261015 1rC_4bA). Nav2 still plans both legs;
            # the original A wall-tip route and pickup gates remain.
            corridor = (ZonePose(
                "A_WALL_CENTRAL_CLEAR", -1.20, -4.55,
                math.pi / 2.0),)
            if (1.7 <= start_x <= 2.1
                    and -5.6 <= start_y <= -5.2):
                # Mirror red2's tray-clear inbound corner before
                # carrying west. A direct northwest diagonal from
                # its east entry grazes the low, lidar-invisible tray.
                corridor = (ZonePose(
                    "A_WALL_RED2_NORTH_CLEAR", 1.90, -4.70,
                    math.pi),) + corridor
        pending = (list(corridor) + list(a_guides)
                   + pending)
        dedicated = True
        log("A_wall_tip_clearance_route active")
    elif (zone == "A" and place.x < -10.0
            and start_x is not None
            and start_y is not None):
        guides = list(layout_a_sweep_waypoints(
            start_x, start_y))
        if guides:
            pending = guides + [place]
            dedicated = True
        log("A_sweep_clearance_route active")
    elif start_x is not None and start_y is not None:
        pocket_route = bay_pocket_north_route(
            (start_x, start_y), place, zone)
        if pocket_route:
            log(
                "bay_pocket_north_route active "
                f"from=({start_x:.2f},{start_y:.2f})"
            )
            return list(pocket_route) + [place], True
        c_guides = (c_southeast_transport_waypoints(
            (start_x, start_y), (place.x, place.y))
            if zone == "C" else ())
        if c_guides:
            pending = list(c_guides) + [place]
            log("C_south_wall_clearance_route active")
            return pending, dedicated
        guides = list(lower_east_corridor_waypoints(
            (start_x, start_y), (place.x, place.y)))
        if guides:
            pending = guides + [place]
            dedicated = True
            log(
                "lower_east_corridor_route active")
        elif (staging := bay_exit_staging(
                (start_x, start_y), place, bay_distance)) is not None:
            pending = [staging, place]
            log(
                "bay_exit_staging active "
                f"from=({start_x:.2f},{start_y:.2f}) "
                f"via=({staging.x:.2f},{staging.y:.2f})"
            )
    return pending, dedicated
