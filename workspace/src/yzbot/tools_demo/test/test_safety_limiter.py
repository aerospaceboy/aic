"""Tests for the moving-obstacle velocity safety limiter math."""

import math
import inspect
from types import SimpleNamespace
from unittest.mock import Mock

from tools_demo.safety_math import compute_scale
from tools_demo.safety_math import safe_crossing_phase
from tools_demo.safety_limiter import SafetyLimiter


def test_final_guard_subscribes_to_both_new_carts_without_new_lane_bypass():
    source = inspect.getsource(SafetyLimiter.__init__)
    for name in ['obstacle_1','obstacle_2','crossing_cart_1','crossing_cart_2']:
        assert '/competition/obstacles/'+name in source
    transit = inspect.getsource(SafetyLimiter._swept_band_transit)
    assert '"moving_obstacle" not in model' in transit
    assert 'crossing_cart' not in transit


def test_no_obstacles_does_not_scale():
    """With no obstacles the command passes through unchanged."""
    assert compute_scale((0.0, 0.0, 0.0), {}, 0.7, 1.6) == 1.0


def test_no_robot_does_not_scale():
    """Without a robot pose nothing can be evaluated."""
    assert compute_scale(None, {"o": (0.5, 0.0)}, 0.7, 1.6) == 1.0


def test_distant_obstacle_does_not_scale():
    """An obstacle beyond the caution radius is ignored."""
    obstacles = {"o": (5.0, 5.0)}
    assert compute_scale((0.0, 0.0, 0.0), obstacles, 0.7, 1.6) == 1.0


def test_obstacle_inside_danger_stops():
    """An obstacle inside the danger radius stops the robot."""
    obstacles = {"o": (0.5, 0.0)}
    assert compute_scale((0.0, 0.0, 0.0), obstacles, 0.7, 1.6) == 0.0


def test_obstacle_in_caution_slows_proportionally():
    """An obstacle in the caution band scales the command down."""
    obstacles = {"o": (1.0, 0.0)}
    scale = compute_scale((0.0, 0.0, 0.0), obstacles, 0.7, 1.6)
    assert abs(scale - 0.333) < 0.01


def test_behind_obstacle_ignored_by_narrow_cone():
    """An obstacle behind the robot does not slow a forward cone."""
    obstacles = {"o": (-0.5, 0.0)}
    scale = compute_scale(
        (0.0, 0.0, 0.0), obstacles, 0.85, 1.6, cone_half_angle=0.5
    )
    assert scale == 1.0


def test_reverse_command_checks_the_rear_cone():
    """Reverse motion must react behind the robot, not in front of it."""
    obstacles = {"front": (0.5, 0.0), "rear": (-0.6, 0.0)}
    scale = compute_scale(
        (0.0, 0.0, 0.0),
        obstacles,
        0.7,
        1.6,
        cone_half_angle=0.5,
        motion_sign=-1.0,
    )
    assert scale == 0.0


def test_rotation_in_place_is_not_overridden_by_truth_guard():
    """The laser collision monitor owns rotation safety."""
    obstacles = {"o": (0.4, 0.0)}
    scale = compute_scale(
        (0.0, 0.0, 0.0),
        obstacles,
        0.7,
        1.6,
        cone_half_angle=0.5,
        motion_sign=0.0,
    )
    assert scale == 1.0


def test_east_gate_waits_at_safe_position_for_southbound_mover():
    """A crossing waits east of the lane, then opens on fresh retreat data."""
    harness = SimpleNamespace(
        robot=(9.6, 0.15, math.pi),
        obstacles={"competition_moving_obstacle_1": (8.5, -0.50, 1_000_000_000)},
        obstacle_previous_y={"competition_moving_obstacle_1": -0.45},
        obstacle_timeout=0.75,
        east_gate_open=False,
        get_clock=Mock(return_value=SimpleNamespace(
            now=Mock(return_value=SimpleNamespace(nanoseconds=1_000_000_000))
        )),
        get_logger=Mock(return_value=Mock()),
    )
    assert SafetyLimiter.east_gate_blocked(harness, 1.0)
    # The release floor is 1.00 m of separation since 2026-10-06: a release at
    # 0.81 m let the actor pass the crossing base at 0.0892 m surface
    # clearance (case 2rB_3bA), so the actor must retreat further before the
    # gate opens. Base y=0.15 needs the actor south of -0.85.
    harness.obstacles["competition_moving_obstacle_1"] = (
        8.5, -0.75, 1_000_000_000
    )
    harness.obstacle_previous_y["competition_moving_obstacle_1"] = -0.71
    assert safe_crossing_phase(-0.75, -0.71)
    assert SafetyLimiter.east_gate_blocked(harness, 1.0)
    harness.obstacles["competition_moving_obstacle_1"] = (
        8.5, -0.90, 1_000_000_000
    )
    harness.obstacle_previous_y["competition_moving_obstacle_1"] = -0.86
    assert not SafetyLimiter.east_gate_blocked(harness, 1.0)
    assert harness.east_gate_open
    harness.robot = (8.58, 0.235, math.pi)
    harness.obstacles["competition_moving_obstacle_1"] = (
        8.5, -0.08, 1_000_000_000
    )
    assert not SafetyLimiter.east_gate_blocked(harness, 1.0)


def test_east_gate_fails_closed_without_fresh_actor_truth():
    """A missing actor sample cannot release the safe wait position."""
    harness = SimpleNamespace(
        robot=(9.6, 0.15, math.pi), obstacles={},
        obstacle_previous_y={}, obstacle_timeout=0.75,
        east_gate_open=False,
        get_clock=Mock(return_value=SimpleNamespace(
            now=Mock(return_value=SimpleNamespace(nanoseconds=1_000_000))
        )),
    )
    assert SafetyLimiter.east_gate_blocked(harness, 1.0)
