"""Pure validation and motion helpers for competition navigation."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence


VALID_ZONES = ("A", "B", "C")
VALID_ZONE_EXITS = tuple(f"{zone}_EXIT" for zone in VALID_ZONES)
VALID_PICKUPS = ("RED", "BLUE")
VALID_DEPARTURES = ("RED_EXIT", "BLUE_EXIT")
# The 2026 layout distributes every cube on its own tray, so navigation
# targets one cube at a time instead of a shared per-colour pickup dock.
VALID_CUBES = tuple(
    f"{color}_CUBE_{number}"
    for color in ("RED", "BLUE")
    for number in range(1, 6)
)
VALID_CUBE_ENTRIES = tuple(f"{cube}_ENTRY" for cube in VALID_CUBES)
VALID_LOCATIONS = (
    VALID_ZONES
    + VALID_ZONE_EXITS
    + VALID_PICKUPS
    + VALID_DEPARTURES
    + VALID_CUBES
    + VALID_CUBE_ENTRIES
)


class NavigationValueError(ValueError):
    """Raised when a navigation command or parameter is invalid."""


def navigation_progress_stalled(
    elapsed_sec: float,
    translation_m: float,
    window_sec: float,
    minimum_translation_m: float,
) -> bool:
    """Return true when a goal crawls too little distance in its window."""
    values = tuple(float(value) for value in (
        elapsed_sec, translation_m, window_sec, minimum_translation_m,
    ))
    if not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Progress measurements must be finite")
    if any(value < 0.0 for value in values):
        raise NavigationValueError("Progress measurements cannot be negative")
    if window_sec <= 0.0 or minimum_translation_m <= 0.0:
        raise NavigationValueError("Progress thresholds must be positive")
    return elapsed_sec >= window_sec and translation_m < minimum_translation_m


@dataclass(frozen=True)
class ZonePose:
    """A named planar navigation pose."""

    zone: str
    x: float
    y: float
    yaw: float


def normalize_zone(value: str) -> str:
    """Normalize a zone command and reject unsupported values."""
    if not isinstance(value, str):
        raise NavigationValueError("Zone command must be text")
    zone = value.strip().upper()
    if zone.startswith("ZONE_"):
        zone = zone[5:]
    elif zone.startswith("ZONE "):
        zone = zone[5:].strip()
    if zone not in VALID_ZONES:
        raise NavigationValueError(
            f"Unsupported zone {value!r}; expected A, B, or C"
        )
    return zone


def normalize_location(value: str) -> str:
    """Normalize a destination zone or color pickup location."""
    if not isinstance(value, str):
        raise NavigationValueError("Navigation location must be text")
    location = value.strip().upper().replace("-", "_").replace(" ", "_")
    for prefix in ("ZONE_", "PICKUP_", "SOURCE_"):
        if location.startswith(prefix):
            location = location[len(prefix):]
            break
    if location not in VALID_LOCATIONS:
        raise NavigationValueError(
            "Unsupported navigation location "
            f"{value!r}; expected A, B, C, RED, BLUE, "
            "RED_EXIT, BLUE_EXIT, a cube such as RED_CUBE_1, "
            "or a cube entry such as RED_CUBE_1_ENTRY"
        )
    return location


def parse_zone_pose(zone: str, values: Sequence[float]) -> ZonePose:
    """Validate one [x, y, yaw] parameter sequence."""
    normalized = normalize_zone(zone)
    if isinstance(values, (str, bytes)) or len(values) != 3:
        raise NavigationValueError(
            f"Zone {normalized} pose must contain x, y, yaw"
        )
    numbers = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in numbers):
        raise NavigationValueError(
            f"Zone {normalized} pose must contain finite values"
        )
    return ZonePose(normalized, *numbers)


def parse_location_pose(
    location: str,
    values: Sequence[float],
) -> ZonePose:
    """Validate one named destination or pickup pose."""
    normalized = normalize_location(location)
    if isinstance(values, (str, bytes)) or len(values) != 3:
        raise NavigationValueError(
            f"Location {normalized} pose must contain x, y, yaw"
        )
    numbers = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in numbers):
        raise NavigationValueError(
            f"Location {normalized} pose must contain finite values"
        )
    return ZonePose(normalized, *numbers)


def navigation_handoff_reached(
    current_x: float,
    current_y: float,
    goal_x: float,
    goal_y: float,
    handoff_distance: float,
) -> bool:
    """Return whether odometry is inside the bounded Nav2 handoff radius."""
    values = (
        float(current_x),
        float(current_y),
        float(goal_x),
        float(goal_y),
        float(handoff_distance),
    )
    if not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Navigation handoff values must be finite")
    if handoff_distance <= 0.0:
        raise NavigationValueError(
            "Navigation handoff distance must be positive"
        )
    return math.hypot(goal_x - current_x, goal_y - current_y) <= (
        handoff_distance
    )


def target_is_behind(
    current_x: float,
    current_y: float,
    current_yaw: float,
    target_x: float,
    target_y: float,
) -> bool:
    """Return whether a target lies in the robot's rear half-plane."""
    values = (
        float(current_x),
        float(current_y),
        float(current_yaw),
        float(target_x),
        float(target_y),
    )
    if not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Navigation heading values must be finite")
    dx = target_x - current_x
    dy = target_y - current_y
    if math.hypot(dx, dy) <= 1e-6:
        return False
    heading_error = math.atan2(
        math.sin(math.atan2(dy, dx) - current_yaw),
        math.cos(math.atan2(dy, dx) - current_yaw),
    )
    return abs(heading_error) > math.pi / 2.0


def should_use_zone_exit_waypoints(zone: str, current_yaw: float) -> bool:
    """Return whether a delivery exit still needs its configured waypoint."""
    normalized = normalize_zone(zone)
    yaw = float(current_yaw)
    if not math.isfinite(yaw):
        raise NavigationValueError("Zone exit yaw must be finite")
    # Zone A can be docked from either side. From the west dock (yaw near
    # zero), the bounded 0.45 m reverse already clears the delivery marker;
    # adding its north waypoint forces a 90-degree Nav2 turn in the bay and
    # has produced a 111 s crawl. The east dock faces west, so it keeps the
    # waypoint to route around the placed cubes instead of crossing them.
    if normalized == "A" and math.cos(yaw) > 0.0:
        return False
    # C is always delivered from its east dock while facing west. Reversing
    # 0.45 m therefore moves east, directly away from the placed cube and
    # out of the bay. The configured north-west waypoint is only needed for
    # an unexpected west-side dock; using it here adds a turn and sends the
    # robot in the opposite direction from the remaining southern pickups.
    if normalized == "C" and math.cos(yaw) < 0.0:
        return False
    return True


def forward_dock_remaining(
    current_x: float,
    current_y: float,
    dock_x: float,
    dock_y: float,
    dock_yaw: float,
) -> float:
    """Project dock displacement onto the forward-only approach axis."""
    values = tuple(
        float(value)
        for value in (current_x, current_y, dock_x, dock_y, dock_yaw)
    )
    if not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Dock projection values must be finite")
    dx = dock_x - current_x
    dy = dock_y - current_y
    return dx * math.cos(dock_yaw) + dy * math.sin(dock_yaw)


def pickup_dock_drive_distance(
    forward: float,
    lateral: float,
    arrival_tolerance: float,
    lateral_tolerance: float,
) -> float:
    """Drive distance still owed to a pickup dock; 0.0 means arrived.

    A forward-only arrival test accepted a 0.25 m cross-track miss
    (20261013T213956): the mover stopped the approach early, the base was
    declared docked, the cube sat outside the wrist camera's view and every
    scan found nothing, ending the round as target_unavailable. A pickup dock
    is therefore only complete when the cross-track miss is inside its own
    band as well.
    """
    forward = max(0.0, float(forward))
    lateral = abs(float(lateral))
    if forward <= arrival_tolerance and lateral <= lateral_tolerance:
        return 0.0
    return max(forward, lateral)


def reverse_dock_errors(
    current_x: float,
    current_y: float,
    target_x: float,
    target_y: float,
    dock_yaw: float,
) -> tuple[float, float]:
    """Return remaining reverse distance and lateral miss to a tray entry."""
    values = tuple(float(value) for value in (
        current_x, current_y, target_x, target_y, dock_yaw
    ))
    if not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Departure projection values must be finite")
    dx = target_x - current_x
    dy = target_y - current_y
    axis_x = -math.cos(dock_yaw)
    axis_y = -math.sin(dock_yaw)
    return (
        dx * axis_x + dy * axis_y,
        abs(dx * axis_y - dy * axis_x),
    )


def needs_pre_navigation_alignment(
    distance_to_goal: float,
    heading_error: float,
) -> bool:
    """Pre-turn hard Nav2 legs while the base is still in open space."""
    distance = float(distance_to_goal)
    error = abs(float(heading_error))
    if not math.isfinite(distance) or not math.isfinite(error):
        raise NavigationValueError("Pre-alignment values must be finite")
    if distance < 0.0:
        raise NavigationValueError("Pre-alignment distance must be positive")
    if error <= 0.35:
        return False
    # Hard turns always pre-align. Medium heading error only on longer legs:
    # RPP already absorbs short hops, and a 1.25 m gate fired ~10 times per
    # AB round (~20 s) without a matching Nav2 stall. The 0.55 / 1.10 pair
    # trims the same cost on the long A<->C rounds: ~11 legs each paid an
    # in-place turn, and the 300 s stops at 300.1/300.2 s (20261014 matrix
    # 3rA_2bC and 4rA_1bC) were entirely alignment plus docking idle time.
    return distance >= 3.0 or error >= 0.90


# ---------------------------------------------------------------------------
# Delivery-slot reach contract.
#
# The arm node (`mybot/pick_place_moveit.py`) is the single authority on
# whether a delivery point is reachable; it fails closed and keeps the cube.
# These bounds only let the navigator CHOOSE a final dock that is reachable by
# construction, from the same world slot the arm will be given.
#
# Why the dock must follow the slot: the coordinator spreads a zone's jobs
# across +/-0.160 m of WORLD Y, but the arm's checked tool window is only
# +/-0.12 m of BASE-FRAME Y. A dock that keeps the zone-centre lateral
# coordinate puts every end slot 0.152-0.168 m off-axis, which the arm
# correctly refuses (measured 2026-10-16: 24 of 42 zone/count/slot end-slot
# combinations rejected; v15 died on `place_failed:holding_cube` at A slot
# 4/4 after 291.2 s and 4/5 delivered).
#
# Why the dock is moved sideways rather than turned toward the slot: turning
# the base to face an end slot does centre the tool lateral coordinate, but it
# also swings the arm's forward shortfall into world Y. The arm's own
# `Dock is too far from the physical delivery area` guard allows only 0.030 m
# of world-Y error, and at the real v15 dock pose the turned solution measured
# 0.0558 m (corr_x 0.1245). A sideways shift keeps the validated pad-axis
# heading, so the slot's world-Y error stays 0.0000 and only the already
# tolerated forward shortfall (<= 0.18 m) remains.
DELIVERY_DOCK_MAX_LATERAL = 0.20
# Forward distance from the shifted dock to the slot. The configured zone
# docks stand 0.40 m off the zone centre; the band leaves room for the
# documented layout without ever asking the arm for an out-of-window point.
DELIVERY_DOCK_MIN_FORWARD = 0.25
DELIVERY_DOCK_MAX_FORWARD = 0.54


def delivery_slot_dock(
    place,
    slot_xy,
    max_lateral: float = DELIVERY_DOCK_MAX_LATERAL,
    min_forward: float = DELIVERY_DOCK_MIN_FORWARD,
    max_forward: float = DELIVERY_DOCK_MAX_FORWARD,
):
    """Return ``(dock_pose, reason)`` for one fixed world delivery slot.

    The zone dock keeps its along-axis standoff and its validated heading and
    is moved sideways until its lateral coordinate equals the job's slot line.
    The robot then faces the pad exactly as before but stands opposite its own
    slot column, so the arm's base-frame lateral coordinate for that slot is
    the residual docking error instead of the slot's full 0.16 m spread.

    ``reason`` is ``"aligned"`` when the moved dock is returned and
    ``"fallback"`` when the configured dock must be kept because the slot is
    outside the bounded band. The caller must log the refusal: an unreachable
    slot has to reach the arm as a fail-closed refusal, never as a silently
    repeated unreachable request.
    """
    values = (
        float(place.x), float(place.y),
        float(slot_xy[0]), float(slot_xy[1]),
    )
    if not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Delivery slot geometry must be finite")
    for name, limit in (("max_lateral", max_lateral),
                        ("min_forward", min_forward),
                        ("max_forward", max_forward)):
        if not math.isfinite(limit) or limit < 0.0:
            raise NavigationValueError(f"Delivery dock {name} must be >= 0")
    lateral = values[3] - values[1]
    forward = abs(values[2] - values[0])
    if abs(lateral) > max_lateral:
        return place, "fallback"
    if not (min_forward <= forward <= max_forward):
        return place, "fallback"
    if lateral == 0.0:
        return place, "aligned"
    return (
        ZonePose(f"{place.zone}_slot", place.x, values[3], place.yaw),
        "aligned",
    )


def pickup_dock_velocity(
    distance: float,
    yaw_error: float,
    max_speed: float,
    deceleration: float,
    max_angular_speed: float,
    lateral_error: float = 0.0,
    forward_distance: float | None = None,
) -> tuple[float, float]:
    """Turn then approach a tray in one forward-only dock control loop.

    ``yaw_error`` alone keeps the chassis parallel to the docking axis.  It
    cannot remove a cross-track error inherited from Nav2, so the old loop
    drove that error all the way to the tray.  Aim at the dock centre while
    there is room, then let the correction naturally decay to the requested
    final yaw.  A large correction still rotates in place before translating.
    """
    if not all(math.isfinite(value) for value in (
        distance, yaw_error, max_speed, deceleration, max_angular_speed,
        lateral_error,
    )):
        raise NavigationValueError("Pickup dock values must be finite")
    if forward_distance is None:
        forward_distance = distance
    if not math.isfinite(float(forward_distance)):
        raise NavigationValueError("Pickup dock values must be finite")
    if distance < 0.0 or min(max_speed, deceleration, max_angular_speed) <= 0:
        raise NavigationValueError("Pickup dock limits must be positive")
    forward_distance = max(0.0, float(forward_distance))
    # Do not chase an angularly singular point at the dock plane.  The caller
    # rejects a remaining lateral miss before that plane; this floor gives the
    # entry-to-dock arc a stable, bounded lookahead.
    cross_track_heading = math.atan2(
        float(lateral_error), max(0.25, forward_distance)
    )
    steering_error = yaw_error + cross_track_heading
    if abs(steering_error) > 0.12:
        angular = max(-max_angular_speed, min(
            max_angular_speed, 3.0 * steering_error,
        ))
        # Keep a floor so the turn does not crawl out near the 0.12 gate.
        if abs(angular) < 0.25:
            angular = math.copysign(0.25, angular)
        return 0.0, angular
    speed = min(max_speed, max(0.03, deceleration * distance))
    angular = max(-0.40, min(0.40, 1.5 * steering_error))
    return speed, angular


def costmap_static_layer_ready(
    values: Sequence[int],
    width: int,
    height: int,
    minimum_occupied_cells: int = 100,
) -> bool:
    """Return whether a complete costmap contains rendered static obstacles."""
    if width <= 0 or height <= 0 or len(values) != width * height:
        return False
    return sum(1 for value in values if value >= 50) >= minimum_occupied_cells


# ---------------------------------------------------------------------------
# Dynamic cube-approach planning.
#
# A cube can be grasped from any side whose Nav2 entry and final dock stay
# clear of the pickup trays, rocks and walls. Instead of always staging west
# of the red shelf (the fixed "outside" approach), the navigator builds one
# candidate per enabled cardinal direction and chooses the cheapest by path
# length, chassis rotation, and a small preference for the "inside" side
# (toward the robot's working area between the shelves) when it is feasible.
# ---------------------------------------------------------------------------

APPROACH_WEST = "west"
APPROACH_EAST = "east"
APPROACH_NORTH = "north"
APPROACH_SOUTH = "south"
APPROACH_SIDES = (
    APPROACH_WEST,
    APPROACH_EAST,
    APPROACH_NORTH,
    APPROACH_SOUTH,
)

# Unit vector from the approach dock toward the cube plus the yaw the base
# must hold at the dock so the cube sits dead ahead (base +X).
APPROACH_DIRECTIONS = {
    APPROACH_WEST: ((1.0, 0.0), 0.0),
    APPROACH_EAST: ((-1.0, 0.0), math.pi),
    APPROACH_NORTH: ((0.0, -1.0), -math.pi / 2.0),
    APPROACH_SOUTH: ((0.0, 1.0), math.pi / 2.0),
}

# Dock standoff and entry staging distance (metres) mirroring the layout
# generator geometry (config/final_mission_layout.yaml).
CUBE_DOCK_STANDOFF = 0.36
CUBE_ENTRY_OFFSET = 0.54

# Every mission first reaches one open staging pose near the map origin. The
# full leg is planned by Nav2, passes the two live crossing actors, and is
# shared by every colour/destination distribution. After staging, ordinary
# shortest-path pickup selection resumes from the robot's real pose.
def mover_reverse_segment_clear(
    start_xy: Sequence[float],
    end_xy: Sequence[float],
    actor_x: float,
    minimum_y: float = -0.80,
    maximum_y: float = 0.10,
    minimum_distance: float = 0.60,
) -> bool:
    """Admit a reverse only outside the mover's complete swept segment.

    Both competition movers have fixed X and patrol Y[-0.8,0.1]. Checking
    the currently observed Y alone can back the robot into their next pose.
    A 2 cm spatial sample plus a 1 cm distance buffer bounds interpolation
    error; endpoints and the occupied starting pose are included.
    """
    sx, sy = (float(v) for v in start_xy)
    ex, ey = (float(v) for v in end_xy)
    if not all(math.isfinite(v) for v in (
        sx, sy, ex, ey, actor_x, minimum_y, maximum_y, minimum_distance,
    )) or minimum_y > maximum_y or minimum_distance <= 0.0:
        return False
    steps = max(1, int(math.ceil(math.hypot(ex - sx, ey - sy) / 0.02)))
    for index in range(steps + 1):
        fraction = index / steps
        x_value = sx + fraction * (ex - sx)
        y_value = sy + fraction * (ey - sy)
        nearest_y = min(maximum_y, max(minimum_y, y_value))
        if math.hypot(x_value - actor_x, y_value - nearest_y) < (
            minimum_distance + 0.01
        ):
            return False
    return True


UNIVERSAL_SPAWN_MIN_X = 13.00
UNIVERSAL_SPAWN_MIN_Y = -2.80
UNIVERSAL_SPAWN_MAX_Y = -0.80
UNIVERSAL_CORRIDOR_ENTRY_X = 12.60
UNIVERSAL_CORRIDOR_ENTRY_Y = -0.60
UNIVERSAL_WALL_TIP_CLEAR_X = 12.60
UNIVERSAL_WALL_TIP_CLEAR_Y = 0.45
# Complete the waiting goal OUTSIDE the final gate's x<=9.75 band. At 9.50
# the final limiter stopped Nav2 before arrival, then staged admission waited
# for the next patrol cycle a second time (v15 1rC_4bA-r2). The full slow/fast
# crossing model still admits from the measured, slightly more distant stop;
# neither the final gate nor the separation threshold is weakened.
UNIVERSAL_EAST_WAIT_X = 9.825
UNIVERSAL_EAST_WAIT_ANCHOR_Y = .525
# At the old y=0.45 anchor, the observed 4 cm arrival error made the
# shared admission window shorter than one 5 Hz actor update. This lane
# widens the window while retaining 0.149 m clearance to Wall_40 for the
# conservative 0.28 m base circle. The wall-tip turn stays at y=0.45.
UNIVERSAL_EAST_WAIT_Y = 0.55
# End the first straight crossing before mover 2's final gate, not at 7.25
# where an intervening unconstrained Nav2 path could drift into y~0.1.
# At x=6.65 even 0.20m arrival error remains >1m clear of mover 2's sweep.
UNIVERSAL_MOVER_1_CLEAR_X = 6.65
EAST_LANE_RELEASE_DISTANCE = 1.00
UNIVERSAL_SPAWN_EXIT_X = 0.00
UNIVERSAL_SPAWN_EXIT_Y = 0.00
TRAY_CROSSING_CLEARANCE = 0.65
# The northern wall's inner face in the current competition layout. The
# ordinary 0.65 m tray bypass at y=4.45 leaves only 0.005 m after the base
# radius and the existing 0.20 m Nav2 tracking/arrival reserve are deducted.
A_NORTH_WALL_INNER_Y = 4.935
WALL_TRAY_CROSSING_CLEARANCE = 0.85
TRAY_TRACKING_RESERVE = 0.20
TRAY_BASE_RADIUS = 0.28
TRAY_REQUIRED_CLEARANCE = 0.10

# The known world slot can correct small dock errors before scanning. The
# forward cap keeps at least 5 cm of chassis clearance from the pickup tray;
# the reverse/lateral caps remain inside the arm's validated reach window.
DOCK_REALIGN_TARGET_X = CUBE_DOCK_STANDOFF
DOCK_REALIGN_FORWARD_TOLERANCE = 0.010
# 25 mm still sits inside GRASP_XY_LIMIT (35 mm) and the 33 mm cache
# band, so a 3 cm shelf slot no longer forces a 0.5 s micro-realign.
DOCK_REALIGN_LATERAL_TOLERANCE = 0.025
DOCK_REALIGN_MAX_FORWARD = 0.025
DOCK_REALIGN_MAX_REVERSE = 0.040
DOCK_REALIGN_MAX_LATERAL = 0.035


def bounded_dock_realign(
    slot_x: float,
    slot_y: float,
    desired_x: float = DOCK_REALIGN_TARGET_X,
) -> tuple[float, float]:
    """Return a conservative base-frame correction toward a cube dock."""
    values = (float(slot_x), float(slot_y), float(desired_x))
    if not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Dock realignment values must be finite")
    if desired_x <= 0.0:
        raise NavigationValueError("Dock realignment target X must be positive")

    forward = slot_x - desired_x
    lateral = slot_y
    if abs(forward) <= DOCK_REALIGN_FORWARD_TOLERANCE:
        forward = 0.0
    if abs(lateral) <= DOCK_REALIGN_LATERAL_TOLERANCE:
        lateral = 0.0
    return (
        max(
            -DOCK_REALIGN_MAX_REVERSE,
            min(DOCK_REALIGN_MAX_FORWARD, forward),
        ),
        max(
            -DOCK_REALIGN_MAX_LATERAL,
            min(DOCK_REALIGN_MAX_LATERAL, lateral),
        ),
    )


# Per-cube enabled approach sides, validated against the world geometry by
# test_layout_config.py (every enabled entry clears trays/rocks by more than
# the costmap inflation radius). South of the large rock the corridor between
# the shelves is too tight, so the sides that would thread it are disabled.
CUBE_APPROACH_SIDES = {
    # West and east are both feasible: the base (0.32 m square) clears the
    # 0.24 m tray by ~0.08 m on either side. The coordinator and navigator
    # score every enabled side with cube_delivery_cost (path length, turns,
    # corridor risk, grasp difficulty, prior failures) and pick the side
    # with the shorter/faster route each round instead of always wrapping
    # around the same side. North/south remain disabled: those docks sit
    # inside the tray footprint and the planner stalls there.
    "red_cube_1": (APPROACH_WEST, APPROACH_EAST),
    "red_cube_2": (APPROACH_WEST, APPROACH_EAST),
    "red_cube_3": (APPROACH_WEST, APPROACH_EAST),
    "red_cube_4": (APPROACH_WEST, APPROACH_EAST),
    "red_cube_5": (APPROACH_WEST, APPROACH_EAST),
    "blue_cube_1": (APPROACH_WEST, APPROACH_EAST),
    "blue_cube_2": (APPROACH_WEST, APPROACH_EAST),
    "blue_cube_3": (APPROACH_WEST, APPROACH_EAST),
    "blue_cube_4": (APPROACH_WEST, APPROACH_EAST),
    "blue_cube_5": (APPROACH_WEST, APPROACH_EAST),
}

# The "inside" side faces the robot's working area (between the shelves):
# east for the red shelf, west for the blue shelf. It gets a modest bonus so
# a feasible inside approach wins over an outside detour of comparable
# length; the path term still dominates and a blocked inside side is simply
# not in CUBE_APPROACH_SIDES.
APPROACH_INSIDE_SIDE = {
    "red": APPROACH_EAST,
    "blue": APPROACH_WEST,
}
# The inside preference is intentionally small: in the current layout the
# "inside" side of the red shelf is the tight shelf corridor, so a fixed
# bonus can trap the base for the delivery leg. The path-length and
# delivery terms already make the base approach from whichever side it is
# on; the bonus only breaks exact ties.
APPROACH_INSIDE_BONUS = 0.10
# Approaching a cube through the tight corridor between the two shelves
# (e.g. the west side of the north blue cubes) makes the base turn between
# the pickup trays; any corner-cut overlaps a tray and the global planner
# then refuses the start ("Starting point in lethal space"). Penalize those
# entries so the planner prefers open-area (east/north) approaches.
CORRIDOR_APPROACH_X_MIN = -3.2
CORRIDOR_APPROACH_X_MAX = -0.8
CORRIDOR_APPROACH_Y_MIN = 0.0
CORRIDOR_APPROACH_Y_MAX = 2.5
CORRIDOR_APPROACH_PENALTY = 3.0


def in_corridor_approach(x: float, y: float) -> bool:
    """Return True when (x, y) threads the shelf corridor north of the rock."""
    values = (float(x), float(y))
    if not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Approach coordinates must be finite")
    return (
        CORRIDOR_APPROACH_X_MIN <= values[0] <= CORRIDOR_APPROACH_X_MAX
        and CORRIDOR_APPROACH_Y_MIN <= values[1] <= CORRIDOR_APPROACH_Y_MAX
    )
# Chassis yaw change at the entry, in path-metre equivalents.


APPROACH_YAW_WEIGHT = 0.30


@dataclass(frozen=True)
class ApproachCandidate:
    """One grasp approach: a Nav2 entry staging pose and a final dock."""

    side: str
    cube: str
    entry: ZonePose
    dock: ZonePose


def prefer_same_side_approaches(
    candidates: Sequence[ApproachCandidate],
    robot_x: float,
    cube_x: float,
) -> Sequence[ApproachCandidate]:
    """Keep approaches that do not require crossing the solid pickup tray."""
    values = (float(robot_x), float(cube_x))
    if not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Robot and cube coordinates must be finite")
    preferred_side = (
        APPROACH_EAST if values[0] >= values[1] else APPROACH_WEST
    )
    preferred = tuple(
        candidate
        for candidate in candidates
        if candidate.side == preferred_side
    )
    return preferred or candidates


def dynamic_gate_waypoints(
    start_xy: Sequence[float],
    target_xy: Sequence[float],
) -> Sequence[ZonePose]:
    """Route the initial leg to the universal origin staging area."""
    values = tuple(float(value) for point in (start_xy, target_xy)
                   for value in point[:2])
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Gate routing requires finite XY points")
    start_x, start_y, target_x, _target_y = values
    if (9.3 <= start_x < UNIVERSAL_SPAWN_MIN_X
            and -0.3 <= start_y <= 0.8 and target_x < 8.0):
        # A failed initial crossing is still east of lane 1. Re-enter the
        # waiting line outside the gate, then keep the remaining stages.
        return (
            ZonePose("universal_east_reentry", 10.10,
                     UNIVERSAL_EAST_WAIT_Y, math.pi),
            ZonePose("universal_mover_1_clear", UNIVERSAL_MOVER_1_CLEAR_X,
                     UNIVERSAL_EAST_WAIT_Y, math.pi),
            ZonePose("universal_mover_2_clear", 3.80,
                     UNIVERSAL_EAST_WAIT_Y, math.pi),
            ZonePose("universal_origin_staging", 0.00, 0.00, math.pi),
        )
    if not (
        start_x >= UNIVERSAL_SPAWN_MIN_X
        and UNIVERSAL_SPAWN_MIN_Y <= start_y <= UNIVERSAL_SPAWN_MAX_Y
        and target_x < UNIVERSAL_SPAWN_MIN_X
    ):
        return ()
    return (
        # This is a turn already present on the 15.83 m inflated-map A*
        # shortest path. Making it a stage prevents RPP interpolation from
        # cutting the wall tip near (12.0, -0.4); it is not a scripted
        # moving-obstacle dodge and adds no geometric detour.
        ZonePose(
            "universal_corridor_entry",
            UNIVERSAL_CORRIDOR_ENTRY_X,
            UNIVERSAL_CORRIDOR_ENTRY_Y,
            math.pi,
        ),
        # Move north while still east of the map wall tip, then hold clear
        # of the mover's x=8.5 lane. The straight line from the old entry
        # to this point cuts the wall inflation at 0.20 m; the intermediate
        # point keeps the static clearance above 0.60 m.
        ZonePose(
            "universal_wall_tip_clear",
            UNIVERSAL_WALL_TIP_CLEAR_X,
            UNIVERSAL_WALL_TIP_CLEAR_Y,
            math.pi,
        ),
        ZonePose(
            "universal_east_wait",
            UNIVERSAL_EAST_WAIT_X,
            # Use an actual 5cm grid-cell centre. A boundary goal was
            # quantized diagonally away from the physical 3cm handoff disk;
            # pursuing the last millimetres then turned toward that offset.
            # The straight crossing still validates its measured Y lane.
            UNIVERSAL_EAST_WAIT_ANCHOR_Y,
            math.pi,
        ),
        # Hold the same wall-clear Y lane across both original patrols.
        ZonePose("universal_mover_1_clear", UNIVERSAL_MOVER_1_CLEAR_X,
                 UNIVERSAL_EAST_WAIT_Y, math.pi),
        ZonePose("universal_mover_2_clear", 3.80,
                 UNIVERSAL_EAST_WAIT_Y, math.pi),
        ZonePose(
            "universal_origin_staging",
            UNIVERSAL_SPAWN_EXIT_X,
            UNIVERSAL_SPAWN_EXIT_Y,
            math.pi,
        ),
    )


def corridor_lane_release_clear(
    robot_y: float, mover_y: float, moving_away: bool,
    minimum: float = EAST_LANE_RELEASE_DISTANCE,
) -> bool:
    """Share the gate's separation rule with crossing admission."""
    if not all(math.isfinite(v) for v in (robot_y, mover_y, minimum)):
        return False
    return (minimum > 0.0 and moving_away
            and abs(mover_y - robot_y) >= minimum)


def tray_crossing_waypoints(
    cube_xy: Sequence[float],
    approach_side: str,
    target_xy: Sequence[float],
    *,
    north_wall_y: Optional[float] = None,
) -> Sequence[ZonePose]:
    """Route around a solid pickup tray when delivery is on its far side."""
    values = tuple(float(value) for point in (cube_xy, target_xy)
                   for value in point[:2])
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Tray routing requires finite XY points")
    if north_wall_y is not None and not math.isfinite(north_wall_y):
        raise NavigationValueError("Tray wall boundary must be finite")
    cube_x, cube_y, target_x, target_y = values
    if approach_side not in (APPROACH_WEST, APPROACH_EAST):
        return ()
    target_side = APPROACH_EAST if target_x >= cube_x else APPROACH_WEST
    if target_side == approach_side:
        return ()
    wall_constrained = (
        north_wall_y is not None
        and north_wall_y - (cube_y + TRAY_CROSSING_CLEARANCE)
        < TRAY_BASE_RADIUS + TRAY_TRACKING_RESERVE + TRAY_REQUIRED_CLEARANCE
    )
    clearance = (WALL_TRAY_CROSSING_CLEARANCE if wall_constrained
                 else TRAY_CROSSING_CLEARANCE)
    crossing_y = cube_y + (
        clearance
        if not wall_constrained and target_y >= cube_y
        else -clearance
    )
    west_x = cube_x - clearance
    east_x = cube_x + clearance
    first_x, second_x = (
        (east_x, west_x)
        if approach_side == APPROACH_EAST
        else (west_x, east_x)
    )
    if wall_constrained:
        # These are mandatory anchors, not optional through-poses hints.
        # Face along the crossing rather than requesting a 180-degree turn
        # back toward the shelf at each corner.
        yaw = math.pi if approach_side == APPROACH_EAST else 0.0
        return (
            ZonePose("tray_wall_clear_near", first_x, crossing_y, yaw),
            ZonePose("tray_wall_clear_far", second_x, crossing_y, yaw),
        )
    return (
        ZonePose("tray_near_corner", first_x, crossing_y, 0.0),
        ZonePose("tray_far_corner", second_x, crossing_y, 0.0),
    )


def angle_delta(target: float, current: float) -> float:
    """Return the signed shortest angular difference target - current."""
    values = (float(target), float(current))
    if not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Angles must be finite")
    return math.atan2(
        math.sin(values[0] - values[1]),
        math.cos(values[0] - values[1]),
    )


def approach_dock(
    cube_x: float,
    cube_y: float,
    side: str,
    standoff: float = CUBE_DOCK_STANDOFF,
) -> ZonePose:
    """Return the base dock pose for one cardinal approach side."""
    if side not in APPROACH_DIRECTIONS:
        raise NavigationValueError(f"Unsupported approach side {side!r}")
    (dx, dy), yaw = APPROACH_DIRECTIONS[side]
    return ZonePose(
        f"dock_{side}",
        cube_x - dx * standoff,
        cube_y - dy * standoff,
        yaw,
    )


def approach_entry(
    cube_x: float,
    cube_y: float,
    side: str,
    standoff: float = CUBE_DOCK_STANDOFF,
    entry_offset: float = CUBE_ENTRY_OFFSET,
) -> ZonePose:
    """Return the Nav2 staging pose for one cardinal approach side."""
    if side not in APPROACH_DIRECTIONS:
        raise NavigationValueError(f"Unsupported approach side {side!r}")
    (dx, dy), yaw = APPROACH_DIRECTIONS[side]
    return ZonePose(
        f"entry_{side}",
        cube_x - dx * (standoff + entry_offset),
        cube_y - dy * (standoff + entry_offset),
        yaw,
    )


def approach_score(
    entry: ZonePose,
    dock: ZonePose,
    robot_pose: Sequence[float],
    inside_bonus: float = 0.0,
    yaw_weight: float = APPROACH_YAW_WEIGHT,
) -> float:
    """Return the total cost of one approach (lower is better)."""
    values = tuple(
        float(value)
        for point in (robot_pose, (entry.x, entry.y, entry.yaw))
        for value in point[:3]
    )
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise NavigationValueError(
            "Approach scoring needs a finite robot pose and entry"
        )
    robot_x, robot_y, robot_yaw = values[:3]
    entry_x, entry_y, entry_yaw = values[3:]
    path_length = math.hypot(
        entry_x - robot_x, entry_y - robot_y
    ) + math.hypot(dock.x - entry_x, dock.y - entry_y)
    yaw_cost = yaw_weight * abs(angle_delta(entry_yaw, robot_yaw))
    return path_length + yaw_cost - inside_bonus


def approach_delivery_score(
    entry: ZonePose,
    dock: ZonePose,
    robot_pose: Sequence[float],
    delivery_xy: Sequence[float],
    inside_bonus: float = 0.0,
    yaw_weight: float = APPROACH_YAW_WEIGHT,
) -> float:
    """
    Return the total cost of one approach including the delivery leg.

    The side that leaves the base best positioned for the target delivery
    zone wins: robot -> entry + entry -> dock + dock -> delivery, plus a
    small chassis-yaw term and the inside preference. This prevents picking
    an "inside" approach that saves a few metres on pickup but strands the
    base on the wrong side of the shelves for the delivery leg.
    """
    values = tuple(
        float(value)
        for point in (robot_pose, delivery_xy)
        for value in point[:2]
    )
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise NavigationValueError(
            "Approach scoring needs finite robot and delivery points"
        )
    robot_x, robot_y, delivery_x, delivery_y = values
    corridor_penalty = (
        CORRIDOR_APPROACH_PENALTY
        if in_corridor_approach(entry.x, entry.y)
        else 0.0
    )
    return (
        math.hypot(entry.x - robot_x, entry.y - robot_y)
        + math.hypot(dock.x - entry.x, dock.y - entry.y)
        + delivery_leg_estimate((dock.x, dock.y), (delivery_x, delivery_y))
        + corridor_penalty
        + yaw_weight * abs(angle_delta(entry.yaw, float(robot_pose[2])))
        - inside_bonus
    )


def choose_approach_for_delivery(
    candidates: Sequence[ApproachCandidate],
    robot_pose: Sequence[float],
    delivery_xy: Sequence[float],
    inside_sides: Sequence[str] = (),
) -> ApproachCandidate:
    """Return the cheapest approach including the delivery leg."""
    if not candidates:
        raise NavigationValueError("No approach candidates were provided")
    best: ApproachCandidate = None
    best_score = float("inf")
    for candidate in candidates:
        bonus = (
            APPROACH_INSIDE_BONUS
            if candidate.side in inside_sides
            else 0.0
        )
        score = approach_delivery_score(
            candidate.entry,
            candidate.dock,
            robot_pose,
            delivery_xy,
            inside_bonus=bonus,
        )
        if score < best_score:
            best = candidate
            best_score = score
    return best


def choose_approach(
    candidates: Sequence[ApproachCandidate],
    robot_pose: Sequence[float],
    inside_sides: Sequence[str] = (),
) -> ApproachCandidate:
    """Return the cheapest feasible approach candidate for one cube."""
    if not candidates:
        raise NavigationValueError("No approach candidates were provided")
    best: ApproachCandidate = None
    best_score = float("inf")
    for candidate in candidates:
        bonus = (
            APPROACH_INSIDE_BONUS
            if candidate.side in inside_sides
            else 0.0
        )
        score = approach_score(
            candidate.entry,
            candidate.dock,
            robot_pose,
            inside_bonus=bonus,
        )
        if score < best_score:
            best = candidate
            best_score = score
    return best


def should_use_cube_aisle_entry(
    current_x: float,
    dock_x: float,
    aisle_entry_available: bool,
) -> bool:
    """Choose an east-side aisle entry when the robot is east of the dock."""
    values = (float(current_x), float(dock_x))
    if not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Cube entry coordinates must be finite")
    return bool(aisle_entry_available) and current_x > dock_x


BLUE_INNER_ENTRY_SWITCH_X = -3.0


def should_use_blue_inner_entry(current_x: float) -> bool:
    """Use the blue east-side dock unless the robot is far to the west."""
    value = float(current_x)
    if not math.isfinite(value):
        raise NavigationValueError("Blue entry coordinate must be finite")
    return value >= BLUE_INNER_ENTRY_SWITCH_X


# Shelf-corridor trap: the space between the two shelves south of the large
# rock. A dock inside it cannot start the delivery leg directly; the base
# must first exit the corridor to the north, which the Euclidean two-leg
# estimate misses (and which empirically stalls the local controller). The
# penalty steers selection and approach choice toward open-area docks.
CORRIDOR_TRAP_X_MIN = -3.2
CORRIDOR_TRAP_X_MAX = -0.8
CORRIDOR_TRAP_Y_MAX = -2.6
CORRIDOR_TRAP_PENALTY = 3.0


def in_shelf_corridor_trap(x: float, y: float) -> bool:
    """Return True when (x, y) lies inside the trapped shelf corridor."""
    values = (float(x), float(y))
    if not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Trap coordinates must be finite")
    return (
        CORRIDOR_TRAP_X_MIN <= values[0] <= CORRIDOR_TRAP_X_MAX
        and values[1] <= CORRIDOR_TRAP_Y_MAX
    )


# Open corridor waypoint north of the shelves, used to route deliveries
# whose pickup sits south of the large rock (the rock closes the corridor).
NORTH_CORRIDOR_WAYPOINT = (0.0, 1.0)


def delivery_leg_estimate(
    dock_xy: Sequence[float],
    delivery_xy: Sequence[float],
) -> float:
    """
    Estimate the real delivery path length from a pickup dock.

    A pickup south of the shelves cannot cross the rock-blocked corridor to
    reach a northern delivery zone directly; route it through the open north
    corridor instead. This keeps the two-leg selection honest for the long
    blue->C leg, where the Euclidean line crosses the rock.
    """
    values = tuple(
        float(value)
        for point in (dock_xy, delivery_xy)
        for value in point[:2]
    )
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise NavigationValueError(
            "Delivery estimate needs finite dock and delivery points"
        )
    dock_x, dock_y, delivery_x, delivery_y = values
    # A northern delivery from a dock at or east of the blue shelf, or from
    # inside the shelf corridor, must first go around the shelf top (the
    # blue shelf spans y=-6..2 at x=-1). Route via the open north portal so
    # the estimate matches the real path instead of a Euclidean line that
    # crosses the shelf.
    if delivery_y > -1.0:
        in_corridor = (
            CORRIDOR_TRAP_X_MIN <= dock_x <= CORRIDOR_TRAP_X_MAX
            and dock_y < 1.0
        )
        at_or_east_of_blue_shelf = dock_x >= -1.5 and dock_y < 1.0
        if in_corridor or at_or_east_of_blue_shelf:
            north_x, north_y = NORTH_CORRIDOR_WAYPOINT
            return math.hypot(north_x - dock_x, north_y - dock_y) + math.hypot(
                delivery_x - north_x,
                delivery_y - north_y,
            )
    return math.hypot(delivery_x - dock_x, delivery_y - dock_y)


# Each delivery room has a different safe entrance geometry. These are the
# first open poses on the primary route, not the final manipulation docks.
# B must approach from the east of obstacle_3; A and C retain direct entries
# because their floor bays open into unobstructed space.
ZONE_ROUTE_APPROACH = {
    # Zone A lives at (-2.41, 2.99). The old (-8.40,-7.00) entry was zone C
    # and made every A-delivery cube score look like a C haul, so the
    # dynamic picker chose the wrong trays.
    "A": (-2.41, 2.99),
    "B": (-0.40, -12.00),
    "C": (-9.40, -7.00),
}
ZONE_ROUTE_RISK_PENALTY = {"A": 0.35, "B": 1.25, "C": 0.55}
DELIVERY_TURN_WEIGHT = 0.22
DELIVERY_FAILURE_PENALTY = 8.0
SIDE_GRASP_DIFFICULTY = {
    APPROACH_WEST: 0.0,
    APPROACH_EAST: 0.0,
    APPROACH_NORTH: 0.35,
    APPROACH_SOUTH: 0.35,
}


def delivery_route_estimate(
    dock_xy: Sequence[float],
    target_zone: str,
    place_xy: Sequence[float],
) -> float:
    """Estimate the configured primary route from a pickup to one zone."""
    zone = normalize_zone(target_zone)
    approach = ZONE_ROUTE_APPROACH[zone]
    values = tuple(
        float(value)
        for point in (dock_xy, place_xy)
        for value in point[:2]
    )
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Delivery route estimate needs finite XY")
    dock_x, dock_y, place_x, place_y = values
    return delivery_leg_estimate((dock_x, dock_y), approach) + math.hypot(
        place_x - approach[0], place_y - approach[1]
    )


def cube_delivery_cost(
    robot_pose: Sequence[float],
    candidate: ApproachCandidate,
    target_zone: str,
    place_xy: Sequence[float],
    failure_count: int = 0,
    use_legacy_route_estimate: bool = True,
) -> float:
    """Score a cube by path, turns, zone risk, grasp difficulty, and history."""
    if failure_count < 0:
        raise NavigationValueError("Failure count cannot be negative")
    values = tuple(float(value) for value in robot_pose[:3])
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Cube delivery scoring needs a finite pose")
    robot_x, robot_y, robot_yaw = values
    zone = normalize_zone(target_zone)
    destination = tuple(float(value) for value in place_xy[:2])
    if len(destination) != 2 or not all(math.isfinite(v) for v in destination):
        raise NavigationValueError("Delivery scoring needs finite destination XY")
    entry_heading = math.atan2(
        candidate.entry.y - robot_y, candidate.entry.x - robot_x
    )
    route_end = (
        ZONE_ROUTE_APPROACH[zone] if use_legacy_route_estimate else destination
    )
    route_heading = math.atan2(
        route_end[1] - candidate.dock.y,
        route_end[0] - candidate.dock.x,
    )
    delivery_distance = (
        delivery_route_estimate(
            (candidate.dock.x, candidate.dock.y), zone, place_xy
        ) if use_legacy_route_estimate else math.hypot(
            destination[0] - candidate.dock.x,
            destination[1] - candidate.dock.y
        )
    )
    path = (
        math.hypot(candidate.entry.x - robot_x, candidate.entry.y - robot_y)
        + math.hypot(
            candidate.dock.x - candidate.entry.x,
            candidate.dock.y - candidate.entry.y,
        )
        + delivery_distance
    )
    turns = DELIVERY_TURN_WEIGHT * (
        abs(angle_delta(entry_heading, robot_yaw))
        + abs(angle_delta(route_heading, candidate.dock.yaw))
    )
    corridor_risk = (
        CORRIDOR_APPROACH_PENALTY
        if in_corridor_approach(candidate.entry.x, candidate.entry.y)
        else 0.0
    )
    return (
        path
        + turns
        + corridor_risk
        + ZONE_ROUTE_RISK_PENALTY[zone]
        + SIDE_GRASP_DIFFICULTY[candidate.side]
        + DELIVERY_FAILURE_PENALTY * int(failure_count)
    )


def staged_cube_delivery_cost(
    robot_pose: Sequence[float],
    candidate: ApproachCandidate,
    target_zone: str,
    place_xy: Sequence[float],
    use_legacy_route_estimate: bool = True,
) -> float:
    """Include mandatory spawn staging before estimating the pickup leg.

    Every initial candidate below the spawn threshold is visited after the
    same origin portal. Scoring it directly from spawn undercounts that
    transit and incorrectly favours trays near spawn instead of the portal.
    Other legs retain the existing route estimator and all safety geometry.
    """
    direct = cube_delivery_cost(
        robot_pose, candidate, target_zone, place_xy,
        use_legacy_route_estimate=use_legacy_route_estimate)
    guides = dynamic_gate_waypoints(
        robot_pose, (candidate.dock.x, candidate.dock.y))
    if not guides:
        return direct
    distance = 0.0
    previous = tuple(robot_pose)
    for guide in guides:
        distance += math.hypot(guide.x - previous[0], guide.y - previous[1])
        previous = (guide.x, guide.y, guide.yaw)
    return distance + cube_delivery_cost(
        previous, candidate, target_zone, place_xy,
        use_legacy_route_estimate=use_legacy_route_estimate)


def triangle_wave(elapsed_sec: float, period_sec: float) -> float:
    """Return a continuous -1..1..-1 triangle wave."""
    if not math.isfinite(elapsed_sec):
        raise NavigationValueError("Elapsed time must be finite")
    if not math.isfinite(period_sec) or period_sec <= 0.0:
        raise NavigationValueError("Period must be positive and finite")
    phase = (elapsed_sec % period_sec) / period_sec
    if phase < 0.5:
        return -1.0 + 4.0 * phase
    return 3.0 - 4.0 * phase


MOVER_HOLD_MIN_TRIGGER = 1.05

# One shared crossing model for admission, direct control and the final guard.
# The 0.90 m Y sweep traverses twice per cycle. Keep the physical geometry and
# safety separation unchanged. Faster patrols require a physically confirmed
# waiting pose: the ordinary 0.20m arrival error erased the safe window.
# The private 0.03m wait checker plus 0.04m odometry proof keep that window
# observable at 10Hz, including the original slow/delayed transit profiles.
CORRIDOR_PATROL_PERIOD = 7.2
CORRIDOR_PATROL_SPEED = 1.8 / CORRIDOR_PATROL_PERIOD
CORRIDOR_TRANSIT_SPEED = 1.10


# Lane the two corridor actors block when their crossing is triggered.
MOVER_CROSSING_LANE_Y = -0.10


def mover_crossing_blocked(
    dx: float,
    dy: float,
    cos_yaw: float,
    sin_yaw: float,
    speed: float,
    lateral_limit: float,
    standoff: float,
    decel: float,
    min_trigger: float = MOVER_HOLD_MIN_TRIGGER,
) -> bool:
    """Whether one corridor mover gates the base's current approach.

    The movers patrol Y[-0.8, 0.1] across the spawn corridor (robot lane
    y ~ 0.2), so passing beside a mover at its north swing leaves as little
    as 0.03-0.05 m of surface clearance even while the robot is already
    yielding (measured 2026-10-05: 0.038 and 0.053 m). The base must hold
    until the mover swings south far enough that the whole crossing keeps
    the 0.10 m margin: body radius 0.28 + mover radius 0.18 + margin 0.10
    gives the lateral limit. The trigger distance grows with the current
    speed so the base can stop before the standoff point.

    ``dx``/``dy`` are the mover-center-minus-base offsets in the map frame,
    ``cos_yaw``/``sin_yaw`` the base heading, ``speed`` the base's planar
    speed, ``decel`` the conservative deceleration used for the stopping
    distance, and ``standoff`` the minimum center distance wanted at rest.
    """
    if not all(math.isfinite(value) for value in (
        dx, dy, cos_yaw, sin_yaw, speed, lateral_limit, standoff, decel,
    )):
        return False
    if speed < 0.0 or lateral_limit <= 0.0 or decel <= 0.0:
        return False
    forward = dx * cos_yaw + dy * sin_yaw
    lateral = -dx * sin_yaw + dy * cos_yaw
    if forward <= 0.0:
        return False
    stop_distance = speed * speed / (2.0 * decel)
    trigger = max(min_trigger, stop_distance + standoff)
    return forward <= trigger and abs(lateral) < lateral_limit


def mover_lane_threat(
    lateral: float,
    toward_lane_speed: float,
    threat_lateral: float = 0.90,
    minimum_speed: float = 0.05,
) -> bool:
    """Whether a mover outside the lane window is still about to enter it.

    A mover swinging back toward the spawn corridor would re-enter the
    lane window while the base crosses, so a hold must persist until the
    mover is clear AND moving away. ``lateral`` is the signed base-frame
    lateral offset of the mover's center, ``toward_lane_speed`` its
    velocity component toward the base's lane (positive = closing).
    """
    if not all(math.isfinite(value) for value in (
        lateral, toward_lane_speed, threat_lateral, minimum_speed,
    )):
        return False
    if threat_lateral <= 0.0 or minimum_speed < 0.0:
        return False
    return (
        0.0 <= abs(lateral) < threat_lateral
        and toward_lane_speed > minimum_speed
    )


def corridor_crossing_clear(
    robot_x: float, robot_y: float, yaw: float, mover_x: float,
    mover_y: float, mover_vy: float, clearance: float = 0.60,
    crossing_speed: float = 0.8, dispatch_delay: float = 0.4,
    patrol_speed: float = 0.225,
) -> bool:
    """
    Estimate admission for the frozen original Y patrols.

    Predict a straight crossing at the supplied speed and latency, with
    reflection at Y=-0.8/0.1. This is a conservative admission test, not a
    replacement for live collision protection or physical validation.
    """
    values = (robot_x, robot_y, yaw, mover_x, mover_y, mover_vy, clearance,
              crossing_speed, dispatch_delay, patrol_speed)
    if not all(math.isfinite(v) for v in values):
        return False
    if crossing_speed <= 0.0 or dispatch_delay < 0.0 or patrol_speed <= 0.0:
        return False
    # Unknown direction, out-of-layout positions or non-crossing headings
    # cannot justify admitting the base into a live lane.
    if abs(mover_vy) < 0.02 or not -0.82 <= mover_y <= 0.12:
        return False
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    if abs(cos_yaw) < 0.4:
        return False
    forward = (mover_x - robot_x) * cos_yaw
    if forward < -clearance:
        return True
    duration = (abs(mover_x - robot_x) + clearance) / (
        crossing_speed * abs(cos_yaw)) + dispatch_delay
    if duration > 6.0:
        return False
    y_value = min(0.1, max(-0.8, mover_y))
    velocity = math.copysign(patrol_speed, mover_vy)
    steps = int(math.ceil(duration / 0.05))
    for step in range(steps + 1):
        elapsed = step * 0.05
        travel = max(0.0, elapsed - dispatch_delay) * crossing_speed
        bx = robot_x + cos_yaw * travel
        by = robot_y + sin_yaw * travel
        if math.hypot(mover_x - bx, y_value - by) < clearance:
            return False
        y_value += velocity * 0.05
        if y_value < -0.8:
            y_value = -1.6 - y_value
            velocity = abs(velocity)
        elif y_value > 0.1:
            y_value = 0.2 - y_value
            velocity = -abs(velocity)
    return True


def layout_a_sweep_waypoints(
    start_x: float, start_y: float,
) -> Sequence[ZonePose]:
    """Stage the V2 west-tip bottleneck between wall 1 and the red1 alcove."""
    if not all(math.isfinite(value) for value in (start_x, start_y)):
        return ()
    if not (-7.5 < start_x and 1.6 < start_y < 4.6):
        return ()
    # East-to-A delivery must clear the moving wall's east corner too.
    # The first guide keeps it above the swept rectangle before heading west.
    west = ZonePose("A_SWEEP_WEST_CLEAR", -7.60, 2.50, math.pi)
    entry = ZonePose("A_SWEEP_ENTRY", -5.55, 2.50, math.pi)
    if start_x >= -4.7:
        return (ZonePose("A_SWEEP_EAST_CLEAR", -3.20, 2.70, math.pi),
                entry, west)
    if start_x <= -6.1 and start_y <= 2.7:
        return (west,)
    return (entry, west)


# --- B bay -> blue4 pocket staging (2026-10-05 V2 map evidence) ------------
# blue_cube_4 sits in the bay whose north wall runs along map y=-3.62 from
# x=3.5 to x=12.5; its east end is the only opening on that side, and the
# west wall (x=3.5) ends at y=-6.0. A pickup started in the B bay therefore
# plans a ~24 m loop: out to the east, west along the corridor above that
# wall, then back south into the pocket. On the 2026-10-05 final2 run the
# single entry goal let NavFn put that corridor lane at y=-2.98 - only
# 0.01 m clear of the wall lethal row at y=-3.58 - while the base weaves
# about +-0.31 m at 1.65 m/s there (measured /cmd_vel_nav swings of
# +1.83/-1.50 rad/s inside 0.4 s). The footprint then covered lethal cells,
# RegulatedPurePursuitController held zero velocity for 28.9 s (244
# "collision ahead"), two identical retries from (9.07,-3.29) could not
# help because that cell is inflated (cost 71, below the 253 lethal guard),
# and only the bounded recovery escape to (9.18,-2.80) plus the replan from
# there finished the leg. These two anchors pin the corridor lane at
# y=-2.50, i.e. 1.08 m from the lethal row, which covers the measured
# weave plus the 0.28 m chassis radius. They apply only to a start inside
# that bay and a target inside that pocket, so no other leg inherits them.
BAY_POCKET_START_MIN_X = 8.0
BAY_POCKET_START_MIN_Y = -7.6
BAY_POCKET_START_MAX_Y = -3.6
BAY_POCKET_TARGET_MIN_X = 3.4
BAY_POCKET_TARGET_MAX_X = 6.2
BAY_POCKET_TARGET_MAX_Y = -4.5
BAY_POCKET_GATE_X = 12.40
BAY_POCKET_GATE_Y = -2.50
BAY_POCKET_CORRIDOR_X = 4.40
BAY_POCKET_CORRIDOR_Y = -2.50


def bay_pocket_waypoints(
    start_xy: Sequence[float],
    target_xy: Sequence[float],
) -> Sequence[ZonePose]:
    """Stage a B-bay to blue4-pocket leg clear of the bay north wall."""
    values = tuple(
        float(value) for point in (start_xy, target_xy)
        for value in point[:2]
    )
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise NavigationValueError("Bay pocket routing requires finite XY")
    start_x, start_y, target_x, target_y = values
    if not (
        start_x >= BAY_POCKET_START_MIN_X
        and BAY_POCKET_START_MIN_Y <= start_y <= BAY_POCKET_START_MAX_Y
    ):
        return ()
    if not (
        BAY_POCKET_TARGET_MIN_X <= target_x <= BAY_POCKET_TARGET_MAX_X
        and target_y <= BAY_POCKET_TARGET_MAX_Y
    ):
        return ()
    return (
        ZonePose("bay_pocket_gate", BAY_POCKET_GATE_X, BAY_POCKET_GATE_Y,
                 math.pi),
        ZonePose("bay_pocket_corridor", BAY_POCKET_CORRIDOR_X,
                 BAY_POCKET_CORRIDOR_Y, math.pi),
    )


def a_wall_tip_waypoints(
    start_xy: Sequence[float], target_xy: Sequence[float],
) -> Sequence[ZonePose]:
    """Clear the current A room's south wall before turning west.

    Wall_38 ends at (-2.8573, 0.8964) in the saved world state. A diagonal
    approach let RPP enter that corner at cruise speed on the five-blue GUI
    run. Stage east of the tip, then above it; the 0.757 m centre clearance
    includes the unchanged 0.28 m navigation radius and 0.10 m margin.
    Apply only to the current A dock, never a relocated or legacy zone.
    """
    values = tuple(float(v) for point in (start_xy, target_xy)
                   for v in point[:2])
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        raise NavigationValueError("A wall routing requires finite XY")
    sx, sy, tx, ty = values
    if not (-5.8 <= tx <= -4.2 and 3.0 <= ty <= 4.6 and sy < 1.65):
        return ()
    north = ZonePose("A_WALL_NORTH_CLEAR", -2.10, 1.85, math.pi / 2)
    if sx >= -2.70 and sy >= -0.30:
        return (north,)
    south = ZonePose("A_WALL_SOUTH_CLEAR", -2.10, -0.30, math.pi / 2)
    if sx < -3.60 and sy < -3.50:
        # A west pickup must first clear Wall_75's south end at Y=-6.3555.
        # The wrist camera hit that end before the chassis in the contact
        # trace; leave 0.794 m for the carried arm, not just the base circle.
        return (
            ZonePose("A_WALL_WEST_CLEAR", -4.25, -7.15, 0.0),
            ZonePose("A_WALL_SOUTH_TIP_CLEAR", -2.10, -7.15, math.pi / 2),
            south, north)
    return (south, north)


def wall57_pocket_waypoints(start_xy, target_xy):
    """Constrain only A/red1's double-wall S passage, in either direction."""
    sx, sy = start_xy
    tx, ty = target_xy
    if not all(math.isfinite(v) for v in (sx, sy, tx, ty)):
        return ()
    def north(x, y):
        return -5.3 <= x <= -3.7 and 3.2 <= y <= 4.2
    def pocket(x, y):
        return -6.8 <= x <= -6.4 and -3.0 <= y <= -2.6
    points = [(-2.1, 1.85), (-2.1, 0.), (-4.1, 0.), (-4.1, -2.8)]
    if north(sx, sy) and pocket(tx, ty):
        pass
    elif pocket(sx, sy) and north(tx, ty):
        points.reverse()
    else:
        return ()
    following = points[1:] + [(tx, ty)]
    return tuple(ZonePose(f'WALL57_POCKET_{i}', p[0], p[1],
                         math.atan2(q[1]-p[1], q[0]-p[0]))
                 for i, (p, q) in enumerate(zip(points, following)))


def lower_east_corridor_waypoints(
    start_xy: Sequence[float], target_xy: Sequence[float],
) -> Sequence[ZonePose]:
    """
    Use the lower corridor instead of an un-crossable patrol lane.

    A free global path at y=-0.45 intersects the original movers' entire
    Y patrol: even at their far end the lateral gap is below the 0.60 m
    crossing admission margin. Waiting there cannot create a safe window.
    The x=4.4 descent enters below the zigzag roof, west of both patrols;
    the y=-2.5 lane keeps 1.08 m from the bay north wall. Bay exits first
    clear that wall's east end before traversing the same lower lane.
    Only upper-left <-> lower-east transfers use these frozen-map guides.
    """
    values = tuple(
        float(v) for point in (start_xy, target_xy) for v in point[:2]
    )
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        raise NavigationValueError("Lower corridor requires finite XY")
    sx, sy, tx, ty = values
    if sx <= 2.0 and sy >= 1.5 and tx >= 8.0 and ty <= -1.8:
        guides = [ZonePose("lower_east_lane", 4.4, -2.5, 0.0)]
        if ty <= -3.6:
            guides.append(ZonePose("lower_east_bay_gate", 13.4, -2.5, 0.0))
        return tuple(guides)
    if sx >= 8.0 and sy <= -3.6 and tx <= 2.0 and ty >= 1.5:
        return (
            ZonePose("lower_east_bay_gate", 13.4, -2.5, math.pi),
            ZonePose("lower_east_lane", 4.4, -2.5, math.pi),
        )
    return ()


def c_southeast_transport_waypoints(start_xy, target_xy):
    """Clear the south wall tip from blue-5's west exit to the current C dock.

    The generic northward bay staging enters the diagonal shelf band here.
    Keep this exception inside the verified start/dock regions; Nav2 still
    plans both legs, with a mandatory south-of-wall anchor before turning.
    """
    sx, sy = start_xy
    tx, ty = target_xy
    if (not all(math.isfinite(v) for v in (sx, sy, tx, ty))
            or not (10.6 <= sx <= 11.0 and -13.2 <= sy <= -12.8
                    and 2.5 <= tx <= 3.3 and -12.4 <= ty <= -11.8)):
        return ()
    return (ZonePose("C_SOUTH_WALL_CLEAR", 6.0, -13.2,
                     math.atan2(ty + 13.2, tx - 6.0)),)
