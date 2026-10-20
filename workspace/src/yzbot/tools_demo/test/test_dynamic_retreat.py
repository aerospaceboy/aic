"""Central emergency recovery never reuses the spawn-lane admission."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools_demo.dynamic_retreat import distance_to_patrol, retreat_admitted
from tools_demo.task_navigator import TaskNavigator

PATROLS = [[0., -3.85, 0., -3.27], [.4, -3.85, .4, -3.27]]
ACTORS = [(0., -3.5, .14, .10), (.4, -4.0, .14, .10)]


def test_safe_early_retreat_finishes_outside_both_patrols():
    assert retreat_admitted((0.0, -7.8, math.pi/2), .5, .25, .20, ACTORS, PATROLS)


def test_early_admission_accounts_for_existing_forward_velocity():
    assert retreat_admitted((0.0, -7.8, math.pi/2), .5, .25, 1.0, ACTORS, PATROLS)
    assert not retreat_admitted((0.0, -7.8, math.pi/2), .5, .25, 1.01, ACTORS, PATROLS)


def test_rear_caution_slowdown_cannot_invalidate_the_timed_proof():
    actors = [ACTORS[0], (0., -9.7, .12, .1)]
    assert not retreat_admitted((0., -7.8, math.pi/2), .5, .25, .2, actors, PATROLS)


def test_recorded_late_head_on_retreat_is_not_admitted():
    # Isolated R1 at sim 444.05: body clearance was 0.0737 < 0.10 m.
    old = [[-1.05, -4.3, 1.1, -6.42, 3.1, -6.42],
           [-1.6, -4.3, .4, -6.85, 3.1, -6.85]]
    actors = [(-.089, -6.226, .14, .10), (1.68, -6.42, .14, .10)]
    assert not retreat_admitted((-.317, -6.664, 1.58), .5, .25, 0., actors, old)


@pytest.mark.parametrize('age', [-.01, .751, float('nan')])
def test_stale_or_future_source_pose_cannot_authorize_retreat(age):
    actors = [(0., -6.6, .12, age), ACTORS[1]]
    assert not retreat_admitted((0., -7.8, math.pi/2), .5, .25, .2, actors, PATROLS)


def test_fresh_instantaneous_clearance_is_not_future_clearance():
    # Actor may enter from behind during the full manoeuvre.
    rear = [(0., -8.5, .12, .1), ACTORS[0]]
    assert not retreat_admitted((0., -7.8, math.pi/2), .5, .25, .2, rear, PATROLS)


def test_waiting_endpoint_inside_either_complete_sweep_is_unsafe():
    assert not retreat_admitted((0., -3.2, math.pi/2), .5, .25, 0., ACTORS, PATROLS)


@pytest.mark.parametrize('points', [[0., 0.], [0., 0., 0., 0.], [0., 0., 1., float('nan')]])
def test_malformed_patrol_is_not_an_admission(points):
    assert not retreat_admitted((0., -7.8, math.pi/2), .5, .25, .2,
                                ACTORS, [points, PATROLS[1]])


def test_point_distance_uses_full_polyline_not_endpoint_only():
    assert distance_to_patrol((.10, -3.75), PATROLS[0]) == pytest.approx(.10)


def test_original_topic_identity_is_exact_not_numeric_suffix():
    node = SimpleNamespace(obstacle_reverse_distance_1=.2,
                           obstacle_reverse_distance_2=.5,
                           obstacle_reverse_distance=.15)
    assert TaskNavigator._obstacle_reverse_distance_for(node, '/competition/obstacles/obstacle_1') == .2
    assert TaskNavigator._obstacle_reverse_distance_for(node, '/competition/obstacles/crossing_cart_1') == .5
    assert TaskNavigator._obstacle_reverse_distance_for(node, '/elsewhere/item_1') == .15


def test_rejected_central_retreat_never_cancels_nav2():
    handle = Mock()
    node = SimpleNamespace(_obstacle_reverse_distance_for=Mock(return_value=.5),
                           _central_retreat_clear=Mock(return_value=False),
                           goal_handle=handle)
    TaskNavigator._begin_obstacle_reverse(node, '/competition/obstacles/crossing_cart_1')
    handle.cancel_goal_async.assert_not_called()
