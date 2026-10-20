"""Tests for competition navigation validation and motion helpers."""

import math

import pytest

from tools_demo.navigation_math import NavigationValueError
from tools_demo.navigation_math import APPROACH_EAST
from tools_demo.navigation_math import APPROACH_NORTH
from tools_demo.navigation_math import APPROACH_SOUTH
from tools_demo.navigation_math import APPROACH_WEST
from tools_demo.navigation_math import ApproachCandidate
from tools_demo.navigation_math import approach_dock
from tools_demo.navigation_math import approach_entry
from tools_demo.navigation_math import approach_score
from tools_demo.navigation_math import choose_approach
from tools_demo.navigation_math import choose_approach_for_delivery
from tools_demo.navigation_math import costmap_static_layer_ready
from tools_demo.navigation_math import delivery_leg_estimate
from tools_demo.navigation_math import mover_crossing_blocked
from tools_demo.navigation_math import delivery_route_estimate
from tools_demo.navigation_math import dynamic_gate_waypoints
from tools_demo.navigation_math import bay_pocket_waypoints
from tools_demo.navigation_math import cube_delivery_cost
from tools_demo.navigation_math import navigation_progress_stalled
from tools_demo.navigation_math import in_corridor_approach
from tools_demo.navigation_math import in_shelf_corridor_trap
from tools_demo.navigation_math import CUBE_APPROACH_SIDES
from tools_demo.navigation_math import navigation_handoff_reached
from tools_demo.navigation_math import bounded_dock_realign
from tools_demo.navigation_math import should_use_cube_aisle_entry
from tools_demo.navigation_math import should_use_blue_inner_entry
from tools_demo.navigation_math import should_use_zone_exit_waypoints
from tools_demo.navigation_math import normalize_location
from tools_demo.navigation_math import normalize_zone
from tools_demo.navigation_math import parse_location_pose
from tools_demo.navigation_math import parse_zone_pose
from tools_demo.navigation_math import prefer_same_side_approaches
from tools_demo.navigation_math import triangle_wave
from tools_demo.navigation_math import tray_crossing_waypoints
from tools_demo.navigation_math import ZonePose
import tools_demo.navigation_math as navigation_math


def test_bounded_pre_navigation_turn_only_for_large_heading_change():
    """Keep minor turns in Nav2 and pre-turn hard ones in open space."""
    align = navigation_math.needs_pre_navigation_alignment
    assert not align(2.0, 0.20)
    assert not align(0.8, 0.60)
    # Medium heading error only on longer legs (RPP absorbs short hops).
    # The 0.55 / 1.10 pair also trims the long A<->C rounds, where ~11 legs
    # each paid an in-place turn and two matrix cases stopped at 300.1/300.2 s
    # (20261014T064649Z and its 4rA_1bC twin).
    assert not align(1.3, 0.50)
    assert align(3.5, 0.50)
    assert align(0.7, 0.95)


def test_pickup_dock_turns_in_place_until_safe_forward_heading():
    """Never reverse or drive sideways into a pickup shelf."""
    command = navigation_math.pickup_dock_velocity(
        distance=0.4, yaw_error=-0.30, max_speed=0.4,
        deceleration=1.0, max_angular_speed=2.5,
    )
    assert command[0] == 0.0
    assert command[1] < 0.0
    aligned = navigation_math.pickup_dock_velocity(
        distance=0.4, yaw_error=0.05, max_speed=0.4,
        deceleration=1.0, max_angular_speed=2.5,
    )
    assert aligned[0] > 0.0
    assert 0.0 < aligned[1] <= 0.20


def test_pickup_dock_steers_cross_track_error_toward_the_dock():
    """A Nav2 lateral residual is removed before the tray, not carried in."""
    left = navigation_math.pickup_dock_velocity(
        distance=0.45, yaw_error=0.0, max_speed=0.22,
        deceleration=1.2, max_angular_speed=0.8,
        lateral_error=0.08, forward_distance=0.45,
    )
    right = navigation_math.pickup_dock_velocity(
        distance=0.45, yaw_error=0.0, max_speed=0.22,
        deceleration=1.2, max_angular_speed=0.8,
        lateral_error=-0.08, forward_distance=0.45,
    )
    assert left[0] == right[0] == 0.0
    assert left[1] > 0.0 > right[1]

    # Once the chassis has turned into the correction, forward motion resumes
    # and the requested final-yaw term counter-steers as the miss shrinks.
    tracking = navigation_math.pickup_dock_velocity(
        distance=0.40, yaw_error=-0.16, max_speed=0.22,
        deceleration=1.2, max_angular_speed=0.8,
        lateral_error=0.06, forward_distance=0.40,
    )
    assert tracking[0] > 0.0


@pytest.mark.parametrize(
    ("value", "expected"),
    (
        ("a", "A"),
        (" B ", "B"),
        ("zone_c", "C"),
        ("Zone A", "A"),
    ),
)
def test_zone_commands_are_normalized(value, expected):
    """Accept the supported short and prefixed zone forms."""
    assert normalize_zone(value) == expected


@pytest.mark.parametrize("value", ("", "D", "zone 4", "AA"))
def test_invalid_zone_commands_are_rejected(value):
    """Reject commands outside the configured A/B/C contract."""
    with pytest.raises(NavigationValueError):
        normalize_zone(value)


@pytest.mark.parametrize(
    ("value", "expected"),
    (
        ("red", "RED"),
        ("pickup_blue", "BLUE"),
        ("source red", "RED"),
        ("zone_b", "B"),
        ("blue_exit", "BLUE_EXIT"),
        ("red-exit", "RED_EXIT"),
    ),
)
def test_mission_locations_include_color_pickups(value, expected):
    """Normalize both pickup locations and destination zones."""
    assert normalize_location(value) == expected


def test_pickup_pose_requires_finite_coordinates():
    """Validate pickup poses through the general location parser."""
    pose = parse_location_pose("pickup_red", [-2.1, 1.0, 0.0])
    assert pose.zone == "RED"
    with pytest.raises(NavigationValueError):
        parse_location_pose("blue", [0.0, math.inf, 0.0])


def test_zone_pose_requires_three_finite_values():
    """Build a typed pose and reject malformed parameter values."""
    pose = parse_zone_pose("b", [2, 0.0, math.pi])
    assert pose.zone == "B"
    assert pose.x == 2.0
    assert pose.yaw == pytest.approx(math.pi)

    with pytest.raises(NavigationValueError):
        parse_zone_pose("A", [1.0, 2.0])
    with pytest.raises(NavigationValueError):
        parse_zone_pose("A", [1.0, math.nan, 0.0])


def test_navigation_handoff_uses_fresh_odom_distance():
    """Hand Nav2 to direct docking before a fast base passes its goal."""
    assert navigation_handoff_reached(-1.50, 0.80, -1.34, 1.40, 0.80)
    assert not navigation_handoff_reached(
        -1.75,
        0.05,
        -1.34,
        1.40,
        0.80,
    )


def test_costmap_readiness_uses_population_not_a_stale_probe_coordinate():
    """A scattered layout cannot depend on one historical tray centre."""
    assert not costmap_static_layer_ready([], 0, 0)
    assert not costmap_static_layer_ready([0] * 399, 20, 20)
    assert not costmap_static_layer_ready([0] * 400, 20, 20)
    populated = [100] * 100 + [0] * 300
    assert costmap_static_layer_ready(populated, 20, 20)


@pytest.mark.parametrize("distance", (0.0, -0.1, math.inf))
def test_navigation_handoff_rejects_unsafe_distance(distance):
    """Reject handoff radii that cannot define a bounded final dock."""
    with pytest.raises(NavigationValueError):
        navigation_handoff_reached(0.0, 0.0, 1.0, 1.0, distance)


def test_dock_realign_clamps_drift_without_moving_inside_tolerance():
    """Correct ordinary dock drift while keeping tray clearance bounded."""
    assert bounded_dock_realign(0.365, -0.009) == pytest.approx((0.0, 0.0))
    assert bounded_dock_realign(0.395, 0.034) == pytest.approx(
        (0.025, 0.034)
    )
    assert bounded_dock_realign(0.310, -0.060) == pytest.approx(
        (-0.040, -0.035)
    )


@pytest.mark.parametrize("slot_x", (math.nan, math.inf))
def test_dock_realign_rejects_nonfinite_slot_data(slot_x):
    """Do not convert unavailable odometry into a base motion."""
    with pytest.raises(NavigationValueError):
        bounded_dock_realign(slot_x, 0.0)


def test_target_behind_requires_pre_navigation_alignment():
    """Detect the red-cube exit to zone C rear-heading transition."""
    helper = getattr(navigation_math, "target_is_behind", None)
    assert helper is not None
    assert helper(-4.37, 3.61, 0.03, -9.40, -7.00)
    assert not helper(-4.37, 3.61, 0.03, 2.00, 3.61)
    assert not helper(-4.37, 3.61, 0.03, -4.37, 9.00)


def test_forward_dock_remaining_stops_at_dock_plane():
    """Ignore lateral millimetres once a forward-only dock reaches its X."""
    helper = getattr(navigation_math, "forward_dock_remaining", None)
    assert helper is not None
    assert helper(-4.85, 3.00, -4.36, 3.00, 0.0) == pytest.approx(0.49)
    assert helper(-4.35, 3.04, -4.36, 3.00, 0.0) <= 0.0
    assert helper(4.85, 3.00, 4.36, 3.00, math.pi) == pytest.approx(0.49)


def test_reverse_departure_detects_a_crossed_entry_plane():
    """A blue exit must end when fast backoff skips its 4 cm radial window."""
    helper = navigation_math.reverse_dock_errors
    assert helper(-9.636, -3.029, -9.24, -3.0, math.pi) == pytest.approx(
        (0.396, 0.029)
    )
    assert helper(-9.19, -3.07, -9.24, -3.0, math.pi) == pytest.approx(
        (-0.05, 0.07)
    )
    assert helper(-5.122, -1.35, -9.24, -3.0, math.pi) == pytest.approx(
        (-4.118, 1.65)
    )


def test_triangle_wave_hits_endpoints_and_repeats():
    """Generate a deterministic continuous obstacle trajectory."""
    assert triangle_wave(0.0, 8.0) == pytest.approx(-1.0)
    assert triangle_wave(2.0, 8.0) == pytest.approx(0.0)
    assert triangle_wave(4.0, 8.0) == pytest.approx(1.0)
    assert triangle_wave(6.0, 8.0) == pytest.approx(0.0)
    assert triangle_wave(8.0, 8.0) == pytest.approx(-1.0)


# ---------------------------------------------------------------------------
# Dynamic cube-approach planning
# ---------------------------------------------------------------------------

def test_approach_dock_and_entry_geometry():
    """Each cardinal side stages the cube dead ahead at the dock."""
    cube = (-3.0, 0.0)
    west_dock = approach_dock(*cube, APPROACH_WEST)
    assert (west_dock.x, west_dock.y, west_dock.yaw) == pytest.approx(
        (-3.36, 0.0, 0.0)
    )
    west_entry = approach_entry(*cube, APPROACH_WEST)
    assert (west_entry.x, west_entry.y) == pytest.approx((-3.90, 0.0))
    east_dock = approach_dock(*cube, APPROACH_EAST)
    assert (east_dock.x, east_dock.y, east_dock.yaw) == pytest.approx(
        (-2.64, 0.0, math.pi)
    )
    north_dock = approach_dock(*cube, APPROACH_NORTH)
    assert (north_dock.x, north_dock.y, north_dock.yaw) == pytest.approx(
        (-3.0, 0.36, -math.pi / 2.0)
    )
    south_entry = approach_entry(*cube, APPROACH_SOUTH)
    assert (south_entry.x, south_entry.y) == pytest.approx((-3.0, -0.90))


def test_approach_score_uses_robot_pose_and_inside_bonus():
    """Path length dominates; an inside bonus lowers a comparable side."""
    cube = (-3.0, 0.0)
    east = ApproachCandidate(
        APPROACH_EAST,
        "red_cube_3",
        approach_entry(*cube, APPROACH_EAST),
        approach_dock(*cube, APPROACH_EAST),
    )
    west = ApproachCandidate(
        APPROACH_WEST,
        "red_cube_3",
        approach_entry(*cube, APPROACH_WEST),
        approach_dock(*cube, APPROACH_WEST),
    )
    robot_east = (-1.0, 0.0, 0.0)
    assert approach_score(east.entry, east.dock, robot_east) < approach_score(
        west.entry, west.dock, robot_east
    )
    robot_west = (-5.0, 0.0, 0.0)
    assert approach_score(west.entry, west.dock, robot_west) < approach_score(
        east.entry, east.dock, robot_west
    )


def test_choose_approach_picks_inside_when_comparable():
    """A feasible inside side wins over an outside side of similar length."""
    cube = (-3.0, 0.0)
    candidates = [
        ApproachCandidate(
            APPROACH_EAST,
            "red_cube_3",
            approach_entry(*cube, APPROACH_EAST),
            approach_dock(*cube, APPROACH_EAST),
        ),
        ApproachCandidate(
            APPROACH_WEST,
            "red_cube_3",
            approach_entry(*cube, APPROACH_WEST),
            approach_dock(*cube, APPROACH_WEST),
        ),
    ]
    chosen = choose_approach(candidates, (-2.0, 0.0, 0.0), (APPROACH_EAST,))
    assert chosen.side == APPROACH_EAST
    chosen_no_bonus = choose_approach(candidates, (-2.0, 0.0, 0.0))
    assert chosen_no_bonus.side == APPROACH_EAST


def test_corridor_approach_penalty_marks_only_the_shelf_gap():
    """Only entries threading the shelf corridor (north blue west) pay."""
    assert in_corridor_approach(-1.76, 2.0)
    assert in_corridor_approach(-2.0, 0.5)
    assert not in_corridor_approach(0.0, 2.0)
    assert not in_corridor_approach(-3.76, 0.0)
    assert not in_corridor_approach(-1.36, -2.0)


def test_delivery_leg_estimate_routes_south_pickups_via_north_corridor():
    """A southern pickup to a northern zone cannot cross the rock corridor."""
    northern_place = (-2.41, 2.99)  # zone A floor bay, north of the rock
    straight = math.dist((-1.36, -4.0), northern_place)
    assert (
        delivery_leg_estimate((-1.36, -4.0), northern_place)
        > straight + 1.0
    )
    assert delivery_leg_estimate((-1.36, 2.0), northern_place) < 3.0
    # Southern deliveries stay direct: the moved C bay is south-west.
    assert delivery_leg_estimate((-1.36, -4.0), (-9.4, -7.0)) < 10.0


def test_in_shelf_corridor_trap_marks_the_rock_blocked_corridor():
    """The trap is the shelf corridor south of the large rock."""
    assert in_shelf_corridor_trap(-1.36, -4.0)
    assert in_shelf_corridor_trap(-2.5, -3.5)
    assert not in_shelf_corridor_trap(-1.36, 0.0)
    assert not in_shelf_corridor_trap(0.0, -4.0)


def test_choose_approach_for_delivery_favours_the_delivery_side():
    """The delivery leg decides the approach when pickup legs are similar."""
    # Blue cubes keep both the west (corridor) and east (open area) docks,
    # so they exercise the delivery-aware tie-break.
    cube = (-1.0, 0.0)
    candidates = [
        ApproachCandidate(
            APPROACH_EAST,
            "blue_cube_3",
            approach_entry(*cube, APPROACH_EAST),
            approach_dock(*cube, APPROACH_EAST),
        ),
        ApproachCandidate(
            APPROACH_WEST,
            "blue_cube_3",
            approach_entry(*cube, APPROACH_WEST),
            approach_dock(*cube, APPROACH_WEST),
        ),
    ]
    robot = (-2.0, 0.0, 0.0)
    # Delivering to zone A (south-west): the west dock leaves the base on
    # the west side, so the delivery leg is shorter.
    chosen = choose_approach_for_delivery(
        candidates, robot, (-8.4, -7.0), (APPROACH_WEST,)
    )
    assert chosen.side == APPROACH_WEST
    # Delivering to zone B (east) from beside the cube: the east dock wins.
    chosen_b = choose_approach_for_delivery(
        candidates, (-0.5, 0.0, 0.0), (4.0, -9.6), (APPROACH_WEST,)
    )
    assert chosen_b.side == APPROACH_EAST


def test_choose_approach_rejects_empty_candidates():
    """Refuse to plan without any approach candidate."""
    with pytest.raises(NavigationValueError):
        choose_approach([], (0.0, 0.0, 0.0))


def test_every_cube_keeps_the_west_approach():
    """The validated outside approach remains available for every cube."""
    for cube_key, sides in CUBE_APPROACH_SIDES.items():
        assert APPROACH_WEST in sides, cube_key
        assert cube_key in {f"{c}_cube_{n}"
                            for c in ("red", "blue")
                            for n in range(1, 6)}


def test_every_cube_has_an_approach_from_either_open_side():
    """A robot must not cross a solid tray to reach its only dock."""
    for sides in CUBE_APPROACH_SIDES.values():
        assert sides == (APPROACH_WEST, APPROACH_EAST)


def test_prefer_same_side_approaches_avoids_crossing_the_tray():
    """Select the entry on the robot side before scoring delivery distance."""
    cube = (12.0, -5.0)
    west = ApproachCandidate(
        APPROACH_WEST,
        "red_cube_4",
        approach_entry(*cube, APPROACH_WEST),
        approach_dock(*cube, APPROACH_WEST),
    )
    east = ApproachCandidate(
        APPROACH_EAST,
        "red_cube_4",
        approach_entry(*cube, APPROACH_EAST),
        approach_dock(*cube, APPROACH_EAST),
    )
    candidates = (west, east)
    assert prefer_same_side_approaches(candidates, 15.0, 12.0) == (east,)
    assert prefer_same_side_approaches(candidates, 9.0, 12.0) == (west,)


def test_prefer_same_side_approaches_falls_back_when_side_is_disabled():
    """Keep a valid route when the near-side dock is unavailable."""
    cube = (12.0, -5.0)
    west = ApproachCandidate(
        APPROACH_WEST,
        "red_cube_4",
        approach_entry(*cube, APPROACH_WEST),
        approach_dock(*cube, APPROACH_WEST),
    )
    assert prefer_same_side_approaches((west,), 15.0, 12.0) == (west,)


@pytest.mark.parametrize("target", ((12.9, -5.0), (5.12, 1.91)))
def test_dynamic_gate_waypoints_use_universal_origin_staging(target):
    """Every first pickup shares one origin-staging avoidance corridor."""
    route = dynamic_gate_waypoints((15.0, -2.0), target)
    assert len(route) == 6
    assert (route[0].x, route[0].y) == pytest.approx(
        (12.6, -0.6)
    )
    assert (route[1].x, route[1].y) == pytest.approx(
        (12.6, 0.45)
    )
    assert (route[2].x, route[2].y) == pytest.approx(
        (9.80, 0.55)
    )
    assert (route[-1].x, route[-1].y) == pytest.approx(
        (0.0, 0.0)
    )
    assert all(pose.yaw == pytest.approx(math.pi) for pose in route)


@pytest.mark.parametrize("target", ((12.9, -5.0), (5.12, 1.91)))
def test_dynamic_gate_waypoints_trigger_for_advanced_spawn(target):
    """The 2026-09 advanced spawn keeps the mandatory staging corridor."""
    route = dynamic_gate_waypoints((13.2, -0.95), target)
    assert len(route) == 6
    assert (route[0].x, route[0].y) == pytest.approx((12.6, -0.6))
    assert (route[1].x, route[1].y) == pytest.approx((12.6, 0.45))
    assert (route[2].x, route[2].y) == pytest.approx((9.80, 0.55))
    assert (route[-1].x, route[-1].y) == pytest.approx((0.0, 0.0))


def test_dynamic_gate_waypoints_omit_same_side_route():
    """Do not revisit the exhibition corridor after the initial exit."""
    assert dynamic_gate_waypoints((-2.0, 0.0), (-6.0, -2.5)) == ()


def test_tray_crossing_waypoints_go_around_the_target_facing_corner():
    """Red-4 east pickup reaches western A without crossing its tray."""
    route = tray_crossing_waypoints(
        (12.0, -5.0), APPROACH_EAST, (-2.41, 2.99)
    )
    assert [(round(pose.x, 2), round(pose.y, 2)) for pose in route] == [
        (12.65, -4.35),
        (11.35, -4.35),
    ]


def test_tray_crossing_waypoints_omit_same_side_delivery():
    """Red-1 east pickup can head directly to the east-side A dock."""
    assert tray_crossing_waypoints(
        (-4.0, 3.0), APPROACH_EAST, (-2.41, 2.99)
    ) == ()


def test_triangle_wave_rejects_nonpositive_period():
    """Reject an obstacle period that cannot define motion."""
    with pytest.raises(NavigationValueError):
        triangle_wave(0.0, 0.0)


@pytest.mark.parametrize(
    ("robot_x", "expected"),
    ((-2.0, True), (-3.36, False), (-8.0, False)),
)
def test_cube_entry_side_follows_the_robot_approach(robot_x, expected):
    """Use the reachable aisle entry east of a red dock only when needed."""
    assert should_use_cube_aisle_entry(robot_x, -3.36, True) is expected


def test_cube_entry_side_requires_a_configured_alternate():
    """Never select an aisle pose that is absent from the configuration."""
    assert not should_use_cube_aisle_entry(-2.0, -3.36, False)


@pytest.mark.parametrize(
    ("robot_x", "expected"),
    ((-2.0, True), (-3.0, True), (-8.0, False)),
)
def test_blue_entry_side_avoids_crossing_the_shelf(robot_x, expected):
    """Use the blue outer dock only when arriving from western zones."""
    assert should_use_blue_inner_entry(robot_x) is expected


def test_zone_route_estimates_are_independent():
    """A, B, and C use distinct configured delivery geometry."""
    pickup = (-1.36, -4.0)
    zone_a = delivery_route_estimate(pickup, "A", (-2.41, 2.99))
    zone_b = delivery_route_estimate(pickup, "B", (-0.40, -12.0))
    zone_c = delivery_route_estimate(pickup, "C", (-9.40, -7.0))
    # A/B/C must remain distinct real hauls, not collapsed onto one bay.
    assert abs(zone_a - zone_b) > 0.1
    assert abs(zone_a - zone_c) > 0.1
    assert abs(zone_b - zone_c) > 0.1
    assert zone_a > 5.0
    assert zone_b > 5.0
    assert zone_c > 5.0
    # A's approach anchor is zone A, not zone C.
    from tools_demo.navigation_math import ZONE_ROUTE_APPROACH

    assert ZONE_ROUTE_APPROACH["A"] != ZONE_ROUTE_APPROACH["C"]
    assert abs(ZONE_ROUTE_APPROACH["A"][1] - 2.99) < 0.05


def test_cube_delivery_cost_includes_failure_history():
    """A failed cube is deprioritized without changing safety limits."""
    candidate = ApproachCandidate(
        "west",
        "red_cube_1",
        ZonePose("entry", -3.76, -2.0, 0.0),
        ZonePose("dock", -3.36, -2.0, 0.0),
    )
    base = cube_delivery_cost(
        (-2.0, -2.0, 0.0), candidate, "A", (-8.4, -7.0)
    )
    failed = cube_delivery_cost(
        (-2.0, -2.0, 0.0), candidate, "A", (-8.4, -7.0),
        failure_count=1,
    )
    assert failed == pytest.approx(base + 8.0)


def test_navigation_progress_stalled_detects_slow_corner_crawl():
    """Low-speed wall-corner crawling must trigger bounded recovery."""
    assert navigation_progress_stalled(8.0, 0.10, 8.0, 0.50)
    assert not navigation_progress_stalled(8.0, 0.60, 8.0, 0.50)


def test_navigation_progress_stalled_rejects_invalid_measurements():
    """Progress watchdog inputs must be finite and non-negative."""
    with pytest.raises(NavigationValueError):
        navigation_progress_stalled(float("nan"), 0.10, 8.0, 0.20)
    with pytest.raises(NavigationValueError):
        navigation_progress_stalled(8.0, -1.0, 8.0, 0.20)


@pytest.mark.parametrize(
    ("zone", "yaw", "expected"),
    (
        ("A", 0.0, False),
        ("A", math.pi, True),
        ("B", 0.0, True),
        ("C", math.pi, False),
        ("C", 0.0, True),
    ),
)
def test_zone_exit_waypoint_depends_on_safe_a_dock_side(zone, yaw, expected):
    """Skip a waypoint when reverse motion already clears A or C."""
    assert should_use_zone_exit_waypoints(zone, yaw) is expected


def test_zone_exit_waypoint_rejects_nonfinite_yaw():
    """Reject invalid orientation instead of bypassing a safe exit route."""
    with pytest.raises(NavigationValueError):
        should_use_zone_exit_waypoints("A", float("nan"))


def test_mover_crossing_blocks_north_swing_in_lane():
    """A mover at the lane's north swing inside the trigger gates the base."""
    assert mover_crossing_blocked(
        dx=1.0, dy=-0.18, cos_yaw=1.0, sin_yaw=0.0,
        speed=1.0, lateral_limit=0.56, standoff=0.95, decel=1.5,
    )


def test_mover_crossing_clears_once_swing_opens_window():
    """A mover south of the 0.56 m lateral window does not gate the base."""
    assert not mover_crossing_blocked(
        dx=1.0, dy=-0.60, cos_yaw=1.0, sin_yaw=0.0,
        speed=1.0, lateral_limit=0.56, standoff=0.95, decel=1.5,
    )


def test_mover_crossing_trigger_grows_with_speed():
    """At higher speed the gate engages earlier than the minimum trigger."""
    slow = mover_crossing_blocked(
        dx=1.35, dy=0.0, cos_yaw=1.0, sin_yaw=0.0,
        speed=0.2, lateral_limit=0.56, standoff=0.95, decel=1.5,
    )
    fast = mover_crossing_blocked(
        dx=1.35, dy=0.0, cos_yaw=1.0, sin_yaw=0.0,
        speed=1.6, lateral_limit=0.56, standoff=0.95, decel=1.5,
    )
    assert slow is False
    assert fast is True


def test_mover_crossing_ignores_behind_and_invalid():
    """Movers behind the base and invalid inputs never gate."""
    assert not mover_crossing_blocked(
        dx=-1.0, dy=0.0, cos_yaw=1.0, sin_yaw=0.0,
        speed=1.0, lateral_limit=0.56, standoff=0.95, decel=1.5,
    )
    assert not mover_crossing_blocked(
        dx=float('nan'), dy=0.0, cos_yaw=1.0, sin_yaw=0.0,
        speed=1.0, lateral_limit=0.56, standoff=0.95, decel=1.5,
    )
    assert not mover_crossing_blocked(
        dx=1.0, dy=0.0, cos_yaw=1.0, sin_yaw=0.0,
        speed=1.0, lateral_limit=0.0, standoff=0.95, decel=1.5,
    )


def test_bay_pocket_waypoints_pin_the_bay_to_blue4_corridor():
    """A B-bay start toward the blue4 pocket keeps a wall-clear lane."""
    guides = bay_pocket_waypoints((10.30, -5.00), (5.20, -5.40))
    assert [pose.zone for pose in guides] == [
        "bay_pocket_gate", "bay_pocket_corridor"]
    assert guides[0].x == pytest.approx(12.40)
    assert guides[0].y == pytest.approx(-2.50)
    assert guides[1].x == pytest.approx(4.40)
    assert guides[1].y == pytest.approx(-2.50)
    # Both anchors keep more than the measured +-0.31 m corridor weave plus
    # the 0.28 m chassis radius clear of the bay wall lethal row (y=-3.58);
    # NavFn alone planned that lane at y=-2.98 and the base deadlocked.
    for pose in guides:
        assert pose.y - (-3.58) > 0.31 + 0.28


def test_bay_pocket_waypoints_ignore_other_legs():
    """Only the B-bay to blue4-pocket leg is staged."""
    # Start already north of the bay wall.
    assert bay_pocket_waypoints((10.30, -2.85), (5.20, -5.40)) == ()
    # Target is the B delivery bay itself.
    assert bay_pocket_waypoints((10.30, -5.00), (11.30, -5.00)) == ()
    # Target is another blue pickup outside the pocket (blue_cube_3).
    assert bay_pocket_waypoints((10.30, -5.00), (-1.10, -6.00)) == ()
    # Start outside the bay (already parked in the west pocket).
    assert bay_pocket_waypoints((5.00, -5.40), (5.20, -5.40)) == ()


def test_bay_pocket_waypoints_reject_non_finite_input():
    """Invalid geometry fails loudly instead of staging a bad pose."""
    with pytest.raises(NavigationValueError):
        bay_pocket_waypoints((float("nan"), -5.0), (5.20, -5.40))
