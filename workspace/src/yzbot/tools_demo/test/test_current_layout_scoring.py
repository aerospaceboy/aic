"""Check current-zone scoring and independent side selection."""
import math
from types import SimpleNamespace

import pytest

from tools_demo.mission_coordinator import MissionCoordinator
from tools_demo.navigation_math import (
    ApproachCandidate, NavigationValueError, ZonePose, cube_delivery_cost,
)


def candidate(x, y):
    """A west dock with a staging entry 0.54 m behind it."""
    return ApproachCandidate("west", "blue_cube_1",
                             ZonePose("entry", x - .54, y, 0.0),
                             ZonePose("dock", x, y, 0.0))


def test_current_b_location_reverses_obsolete_south_route_preference():
    """A nearby B candidate wins even though the historic B route is south."""
    near = candidate(11.44, -2.5)
    south = candidate(-1.64, -6.0)
    start, destination = (-4.5, 3.8, math.pi), (11.7, -5.0)

    def current(c):
        return cube_delivery_cost(
            start, c, "B", destination, use_legacy_route_estimate=False)
    assert current(near) < current(south)


def test_current_destination_changes_the_score():
    """Relocating the zone affects scoring without editing named anchors."""
    c = candidate(0.0, 3.8)
    near = cube_delivery_cost(
        (0.0, 3.8, 0.0), c, "A", (-5.4, 3.8),
        use_legacy_route_estimate=False)
    far = cube_delivery_cost(
        (0.0, 3.8, 0.0), c, "A", (-13.0, 3.8),
        use_legacy_route_estimate=False)
    assert near < far


def test_current_scoring_rejects_invalid_destination():
    """A malformed zone cannot silently produce a NaN rank."""
    with pytest.raises(NavigationValueError):
        cube_delivery_cost((0., 0., 0.), candidate(1., 1.), "A",
                           (math.nan, 1.), use_legacy_route_estimate=False)


def test_approach_choice_uses_current_scoring_when_legacy_disabled():
    """The coordinator must apply the same mode to approach-side selection."""
    near, south = candidate(11.44, -2.5), candidate(-1.64, -6.0)
    node = SimpleNamespace(use_legacy_selection_costs=False,
                           zone_centers={"B": (11.7, -5.)})
    # Distinct valid entities bypass the same-cube side preference filter.
    near = ApproachCandidate(near.side, "candidate", near.entry, near.dock)
    south = ApproachCandidate(south.side, "candidate", south.entry, south.dock)
    assert MissionCoordinator._choose_delivery_candidate(
        node, [near, south], (-4.5, 3.8, math.pi), "B") is near
