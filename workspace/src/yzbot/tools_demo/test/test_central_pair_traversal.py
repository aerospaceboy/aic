"""The central pair must be crossed enough to actually meet both carts.

The two central carts sweep a 2.33 s band. One traversal samples that phase
once and can miss both of them: measured 2026-10-17, ``1rA_4bC`` logged exactly
one CENTRAL_MOVER_GATE_EAST and one CENTRAL_MOVER_GATE_WEST for the whole run,
never encountered either cart, completed 5/5 at 227.3 s and failed only the
central-response gate. The passing ``2rA_3bC`` course was
``[north_flank, south_flank, WEST, EAST, WEST]``.
"""

from types import SimpleNamespace

import pytest

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


def test_a_single_traversal_is_not_enough_when_nothing_is_proven_yet():
    """The regression: the entry condition is false, so only [EAST, WEST]."""
    names = course(0.0, -5.0)
    assert traversals(names) >= 3, names
    assert names.count(WEST) == 2, names
    assert names[-1] == WEST, names


def test_the_already_verified_case_is_unchanged_and_omits_the_gates():
    """Both carts proven: no gate stages at all, as before."""
    names = course(0.0, 0.0, responses={"crossing_cart_1", "crossing_cart_2"})
    assert WEST not in names and EAST not in names, names


def test_a_partial_response_set_still_earns_the_second_traversal():
    """One cart proven is not enough; the course must still cross twice."""
    names = course(0.0, -5.0, responses={"crossing_cart_2"})
    assert traversals(names) >= 3, names


def test_red_cube_2_keeps_its_single_consistent_pass():
    """The one-pass crossing is what red-2 was validated with; do not double it."""
    names = course(0.0, -5.0, dock=(1.9, -5.4), zone="RED_CUBE_2")
    # Exactly one WEST and one EAST, in that order, then red-2's north entry.
    assert traversals(names) == 2, names
    assert names.index(WEST) < names.index(EAST), names


def test_the_extra_traversal_reuses_only_validated_gate_poses():
    """No new geometry: every crossing stage is one of the two gate poses."""
    names = course(0.0, -5.0)
    crossings = [n for n in names if n in (WEST, EAST)]
    assert crossings == [WEST, EAST, WEST], names
