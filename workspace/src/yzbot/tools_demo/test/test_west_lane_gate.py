"""Gate the second corridor lane the way the first one already is."""

# Matrix case 3rB_2bA collided with competition_moving_obstacle_2 twice: the
# corridor route passes ~0.4 m south of the actor's southern extreme, so the
# eastern corridor's release phase (actor retreating south) hands the base the
# worst case - the actor turns around 0.3 m from the crossing path. Release
# instead on separation from the base's own travel line, which is both safer
# and available for ~5 s of every 8 s patrol instead of ~1 s.

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools_demo.safety_limiter import SafetyLimiter

LANE_X = 5.0
ACTOR = "competition_moving_obstacle_2"


def make_gate_harness(x, y, yaw, actor_y=-0.10, prev_y=None,
                      open_latch=False, now_ns=10**9, stamp=None):
    """Build the live gate state with a fresh, non-releasing actor phase."""
    if stamp is None:
        stamp = now_ns
    return SimpleNamespace(
        robot=(x, y, yaw),
        obstacles={ACTOR: (LANE_X, actor_y, stamp)},
        obstacle_previous_y={ACTOR: actor_y if prev_y is None else prev_y},
        obstacle_timeout=0.75,
        west_lane_open=open_latch,
        get_clock=Mock(return_value=SimpleNamespace(
            now=Mock(return_value=SimpleNamespace(nanoseconds=now_ns)),
        )),
        get_logger=Mock(return_value=Mock()),
    )


@pytest.mark.parametrize("x", [5.80, 6.00, 6.55])
def test_westbound_approach_holds_before_the_second_lane(x):
    """A westbound base waits >=0.80 m from lane 2, not inside its sweep."""
    harness = make_gate_harness(x, -0.60, math.pi)
    assert SafetyLimiter.west_lane_blocked(harness, 1.0)
    assert x - LANE_X - 0.28 - 0.18 >= 0.33


@pytest.mark.parametrize("x", [3.45, 3.80, 4.20])
def test_eastbound_approach_holds_before_the_second_lane(x):
    """The west-side approach is gated by the same rule."""
    harness = make_gate_harness(x, -0.60, 0.0)
    assert SafetyLimiter.west_lane_blocked(harness, 1.0)


def test_missing_truth_fails_closed_at_the_gate():
    """Without fresh actor truth the lane phase is unknown, so hold."""
    harness = make_gate_harness(6.00, -0.60, math.pi)
    harness.obstacles.clear()
    assert SafetyLimiter.west_lane_blocked(harness, 1.0)


def test_colliding_phase_keeps_the_base_out_of_the_lane():
    """The exact 3rB_2bA geometry must stay blocked for a south-side route."""
    harness = make_gate_harness(6.20, -1.10, math.pi,
                                actor_y=-0.75, prev_y=-0.60)
    assert SafetyLimiter.west_lane_blocked(harness, 1.0)
    assert not harness.west_lane_open


def test_actor_approaching_the_travel_line_stays_blocked():
    """Separation alone is not enough while the actor closes on the path."""
    harness = make_gate_harness(6.20, -1.10, math.pi,
                                actor_y=-0.45, prev_y=-0.35)
    assert SafetyLimiter.west_lane_blocked(harness, 1.0)


def test_south_side_base_crosses_once_the_actor_moves_away():
    """A separated, retreating actor releases the lane and latches it open."""
    harness = make_gate_harness(6.20, -1.10, math.pi,
                                actor_y=-0.50, prev_y=-0.60)
    assert not SafetyLimiter.west_lane_blocked(harness, 1.0)
    assert harness.west_lane_open
    # The latch survives the actor turning while it stays clear of the path.
    harness.obstacle_previous_y[ACTOR] = 0.05
    harness.obstacles[ACTOR] = (LANE_X, 0.00, 10**9)
    assert not SafetyLimiter.west_lane_blocked(harness, 1.0)


def test_latched_gate_relocks_when_the_actor_returns_to_the_path():
    """An actor back inside 0.40 m of the travel line re-arms the gate."""
    harness = make_gate_harness(6.20, -1.10, math.pi,
                                actor_y=-0.75, prev_y=-0.60, open_latch=True)
    assert SafetyLimiter.west_lane_blocked(harness, 1.0)
    assert not harness.west_lane_open


def test_north_side_route_releases_on_the_retreating_actor():
    """A route north of the patrol releases as the actor runs south."""
    harness = make_gate_harness(6.20, 0.40, math.pi,
                                actor_y=-0.20, prev_y=-0.10)
    assert not SafetyLimiter.west_lane_blocked(harness, 1.0)
    assert harness.west_lane_open


def test_north_side_route_holds_while_the_actor_closes():
    """The same route waits while the actor is rising toward its lane."""
    harness = make_gate_harness(6.20, 0.40, math.pi,
                                actor_y=-0.20, prev_y=-0.30)
    assert SafetyLimiter.west_lane_blocked(harness, 1.0)
    assert not harness.west_lane_open


def test_reverse_motion_is_not_gated():
    """The gate governs forward travel only; reverse keeps Nav2 authority."""
    harness = make_gate_harness(6.00, -0.60, math.pi)
    assert not SafetyLimiter.west_lane_blocked(harness, -1.0)


def test_positions_outside_the_lane_reach_are_not_gated():
    """Routes that cannot reach lane 2 are untouched."""
    harness = make_gate_harness(6.00, 2.40, math.pi)
    assert not SafetyLimiter.west_lane_blocked(harness, 1.0)


def test_callback_holds_the_westbound_command_at_the_gate():
    """The wired callback must publish zero translation, not drive through."""
    harness = make_gate_harness(6.00, -0.60, math.pi)
    harness.base_hold = False
    published = []
    harness.publisher = SimpleNamespace(publish=published.append)
    harness.east_gate_blocked = lambda _sign: False
    harness.west_lane_blocked = lambda _sign: True
    message = SimpleNamespace(linear=SimpleNamespace(x=1.20, y=0.0),
                              angular=SimpleNamespace(z=0.30))
    SafetyLimiter.cmd_vel_callback(harness, message)
    assert len(published) == 1
    assert published[0].linear.x == 0.0
    assert published[0].angular.z == pytest.approx(0.30)
