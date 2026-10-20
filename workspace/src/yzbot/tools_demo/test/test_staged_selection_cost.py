"""Reject spawn-nearest choices that ignore the compulsory origin portal."""

import math

import pytest
from tools_demo.navigation_math import ApproachCandidate
from tools_demo.navigation_math import NavigationValueError
from tools_demo.navigation_math import ZonePose
from tools_demo.navigation_math import cube_delivery_cost
from tools_demo.navigation_math import staged_cube_delivery_cost


def candidate(side):
    """Use the current blue1's two permitted approach poses."""
    east = side == "east"
    return ApproachCandidate(
        side, "blue_cube_1",
        ZonePose("entry", 8.3 if east else 6.5, 3.6,
                 math.pi if east else 0.0),
        ZonePose("dock", 7.76 if east else 7.04, 3.6,
                 math.pi if east else 0.0))


def test_side_choice_uses_origin_portal_instead_of_nearest_spawn_tray():
    pose = (13.2, -0.95, math.pi)
    place = (11.7, -5.0)
    costs = [cube_delivery_cost(pose, candidate(s), "B", place,
                                use_legacy_route_estimate=False)
             for s in ("west", "east")]
    staged = [staged_cube_delivery_cost(pose, candidate(s), "B", place,
                                        use_legacy_route_estimate=False)
              for s in ("west", "east")]
    assert costs[1] < costs[0]
    assert staged[0] < staged[1]
    assert min(staged) > min(costs)


def test_after_spawn_the_same_delivery_cost_is_preserved():
    pose = (10.33, -5.0, 0.0)
    approach = candidate("east")
    assert staged_cube_delivery_cost(pose, approach, "B", (11.7, -5.0),
                                     False) == cube_delivery_cost(
        pose, approach, "B", (11.7, -5.0),
        use_legacy_route_estimate=False)


@pytest.mark.parametrize(
    "pose", [(13.2, -0.95), (13.2, -0.95, float("nan"))])
def test_staging_cannot_hide_invalid_original_pose(pose):
    with pytest.raises(NavigationValueError):
        staged_cube_delivery_cost(pose, candidate("east"), "B",
                                  (11.7, -5.0), False)
