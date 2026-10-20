"""Gate waiting must not park inside the moving actor's scored sweep."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools_demo.safety_limiter import SafetyLimiter


def make_gate_harness(x, y):
    """Use the live gate callback with a fresh non-releasing actor phase."""
    return SimpleNamespace(
        robot=(x, y, math.pi),
        obstacles={"competition_moving_obstacle_1": (8.5, -0.50, 10**9)},
        obstacle_previous_y={"competition_moving_obstacle_1": -0.45},
        obstacle_timeout=0.75,
        east_gate_open=False,
        get_clock=Mock(return_value=SimpleNamespace(
            now=Mock(return_value=SimpleNamespace(nanoseconds=10**9)),
        )),
        get_logger=Mock(return_value=Mock()),
    )


@pytest.mark.parametrize("x", [9.0, 9.02, 9.04])
def test_gate_never_admits_a_wait_inside_scored_actor_sweep(x):
    """A stopped gate must retain 0.10 m clearance throughout the patrol."""
    harness = make_gate_harness(x, 0.10)
    blocked = SafetyLimiter.east_gate_blocked(harness, 1.0)
    sweep_distance = math.hypot(x - 8.5, 0.0)
    assert not blocked or sweep_distance - 0.28 - 0.18 >= 0.10


@pytest.mark.parametrize("x", [9.30, 9.60, 9.75])
def test_gate_preserves_fail_closed_wait_at_safe_east_positions(x):
    """Fresh unsafe phase and missing truth both hold the safe wait area."""
    harness = make_gate_harness(x, 0.10)
    assert SafetyLimiter.east_gate_blocked(harness, 1.0)
    harness.obstacles.clear()
    assert SafetyLimiter.east_gate_blocked(harness, 1.0)


def test_gate_does_not_relock_after_committing_past_safe_wait_boundary():
    """Re-locking a released gate may not trap the base near the sweep."""
    harness = make_gate_harness(9.04, 0.10)
    harness.east_gate_open = True
    assert not SafetyLimiter.east_gate_blocked(harness, 1.0)
    assert harness.east_gate_open
