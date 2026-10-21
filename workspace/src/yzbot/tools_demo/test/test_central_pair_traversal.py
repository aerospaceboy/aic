"""Pickup routes must not manufacture response evidence by extra traversals."""

from types import SimpleNamespace

from tools_demo.task_navigator import (
    CENTRAL_MOVER_GATE_EAST,
    CENTRAL_MOVER_GATE_WEST,
    TaskNavigator,
    ZonePose,
)

WEST = CENTRAL_MOVER_GATE_WEST.zone
EAST = CENTRAL_MOVER_GATE_EAST.zone


def course(start_x, start_y, dock=(2.0, -5.0), responses=frozenset(),
           zone="BLUE_CUBE_3"):
    """Return the zone names of the assembled approach course."""
    node = SimpleNamespace(
        use_legacy_layout_routes=False,
        dock_pose=ZonePose(zone, dock[0], dock[1], 0.0),
        central_mission_active=bool(responses),
        central_mission_responses=set(responses),
        get_logger=lambda: SimpleNamespace(info=lambda *a, **k: None),
    )
    return [p.zone for p in TaskNavigator._cube_approach_waypoints(
        node, zone, start_x, start_y)]


def traversals(names):
    """Count how many times the course crosses the patrol band."""
    return sum(1 for name in names if name in (WEST, EAST))


def test_missing_responses_do_not_generate_an_out_and_back():
    """A south-side approach follows the task route without a demonstration."""
    names = course(0.0, -5.0)
    assert traversals(names) == 0, names


def test_the_already_verified_case_is_unchanged_and_omits_the_gates():
    """Both carts proven: no gate stages at all, as before."""
    names = course(0.0, 0.0, responses={"crossing_cart_1", "crossing_cart_2"})
    assert WEST not in names and EAST not in names, names


def test_partial_responses_do_not_generate_a_second_traversal():
    """Response telemetry does not add motion to the pickup route."""
    names = course(0.0, -5.0, responses={"crossing_cart_2"})
    assert traversals(names) == 0, names


def test_red_cube_2_keeps_its_single_consistent_pass():
    """The one-pass crossing is what red-2 was validated with; do not double it."""
    names = course(0.0, -5.0, dock=(1.9, -5.4), zone="RED_CUBE_2")
    # Exactly one WEST and one EAST, in that order, then red-2's north entry.
    assert traversals(names) == 2, names
    assert names.index(WEST) < names.index(EAST), names


def test_no_extra_gate_geometry_is_generated():
    """Neither crossing direction is added to a safe south-side route."""
    names = course(0.0, -5.0)
    crossings = [n for n in names if n in (WEST, EAST)]
    assert crossings == [], names
