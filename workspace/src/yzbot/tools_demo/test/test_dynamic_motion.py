"""A service-delayed patrol may pause, but must never catch up by jumping."""
import math

import pytest
from tools_demo.dynamic_motion import (
    PolylinePatrol,
    admit_proximity_guarded_step,
)


def test_arclength_interpolation_corners_and_reverse_are_continuous():
    path = PolylinePatrol([0,0,1,0,1,2],12)
    assert path.maximum_speed == .5
    assert path.position(0) == (0,0)
    assert path.position(2) == (1,0)
    assert path.position(6) == (1,2)
    assert path.position(10) == pytest.approx((1,0), abs=1e-12)
    assert path.position(12) == (0,0)
    for i in range(1200):
        assert math.dist(path.position(i*.01),path.position((i+1)*.01)) <= .005+1e-12


@pytest.mark.parametrize('points,period', [([0,0,0,0],8),([0,0,1],8),
                                           ([0,0,1,float('nan')],8),([0,0,1,1],0)])
def test_invalid_patrol_cannot_be_loaded(points,period):
    with pytest.raises(ValueError):
        PolylinePatrol(points,period)


def test_proximity_guard_stops_before_actor_can_press_into_robot():
    admitted, paused = admit_proximity_guarded_step(
        (0.0, 0.80), (0.0, 0.70), (0.0, 0.0), 0.75, 0.95, False,
    )
    assert (admitted, paused) == (False, True)


def test_proximity_guard_holds_phase_until_robot_clears_hysteresis():
    assert admit_proximity_guarded_step(
        (0.0, 0.80), (0.0, 0.90), (0.0, 0.0), 0.75, 0.95, True,
    ) == (False, True)
    assert admit_proximity_guarded_step(
        (0.0, 0.80), (0.0, 0.90), (0.0, -0.20), 0.75, 0.95, True,
    ) == (True, False)


def test_disabled_proximity_guard_preserves_original_movers():
    assert admit_proximity_guarded_step(
        (0.0, 0.1), (0.0, 0.0), (0.0, 0.0), 0.0, 0.0, False,
    ) == (True, False)


def test_proximity_guard_rejects_inverted_hysteresis():
    with pytest.raises(ValueError):
        admit_proximity_guarded_step(
            (0.0, 1.0), (0.0, 0.5), (0.0, 0.0), 0.75, 0.50, False,
        )


def test_frozen_actor_escapes_only_after_the_timeout():
    """A frozen actor must never deadlock on a base that cannot clear.

    Evidence 20261013T211009: the base sat at the central gate for ~50 s while
    the frozen mover held its position, then the leg failed. After the timeout
    the actor may step only AWAY from the robot, which never presses in.
    """
    args = ((0.0, 0.80), (0.0, 0.90), (0.0, 0.0), 0.75, 0.95, True)

    # Before the timeout: still frozen, even when the step moves away.
    assert admit_proximity_guarded_step(
        *args, paused_sec=4.9, freeze_timeout_sec=5.0) == (False, True)

    # After the timeout: moving away is admitted.
    assert admit_proximity_guarded_step(
        *args, paused_sec=5.1, freeze_timeout_sec=5.0) == (True, True)

    # After the timeout: a step toward the robot is still refused.
    assert admit_proximity_guarded_step(
        (0.0, 0.80), (0.0, 0.70), (0.0, 0.0), 0.75, 0.95, True,
        paused_sec=5.1, freeze_timeout_sec=5.0) == (False, True)

    # A disabled timeout keeps the historical freeze behaviour.
    assert admit_proximity_guarded_step(
        *args, paused_sec=99.0, freeze_timeout_sec=0.0) == (False, True)
