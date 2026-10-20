"""Regression tests for the swept-band transit commitment.

Campaign policy_20261001T233650 (GUI, 48 rounds) produced 4 collisions and 2
unsafe clearances that all shared one geometry: the base was inside the
collision band of moving obstacle_1 (lane x=8.5, base x 8.13..8.48) and the
truth guard had zeroed the commanded speed first. Stopping inside a swept
band leaves a stationary target for the returning mover, so the limiter must
transit the band instead.
"""

from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist

from tools_demo.safety_limiter import SafetyLimiter


class _Clock:
    def __init__(self, nanoseconds):
        self._nanoseconds = nanoseconds

    def now(self):
        return SimpleNamespace(nanoseconds=self._nanoseconds)


class _Publisher:
    def __init__(self):
        self.published = []

    def publish(self, message):
        self.published.append(message)


class _Logger:
    def info(self, _message, **_kwargs):
        pass

    def warn(self, _message, **_kwargs):
        pass


def _harness(robot, obstacle, since=None, now_ns=1_000_000_000_000):
    harness = SimpleNamespace(
        robot=robot,
        obstacles={} if obstacle is None else {"competition_moving_obstacle_1": obstacle},
        obstacle_timeout=0.75,
        caution=1.6,
        swept_band_half=0.60,
        swept_band_speed=0.80,
        swept_band_commit=6.0,
        swept_band_since_ns=since,
        publisher=_Publisher(),
        get_clock=lambda: _Clock(now_ns),
        get_logger=lambda: _Logger(),
    )
    # The node calls self._swept_band_transit(...); the stub must resolve it
    # to the real implementation under test.
    harness._swept_band_transit = (
        lambda message: SafetyLimiter._swept_band_transit(harness, message)
    )
    return harness


def _twist(linear_x, angular_z=0.0):
    message = Twist()
    message.linear.x = linear_x
    message.angular.z = angular_z
    return message


def test_transit_replaces_stop_inside_lane_band():
    """A base stopped inside the lane must transit instead of parking."""
    now = 1_000_000_000_000
    harness = _harness((8.30, 0.28, 3.14159), (8.5, 0.10, now), now_ns=now)
    out = SafetyLimiter._swept_band_transit(harness, _twist(1.65, -0.55))
    assert out is not None
    assert out.linear.x == pytest.approx(0.80)
    assert out.angular.z == pytest.approx(-0.55)
    # The commit clock starts on the first transit sample.
    assert harness.swept_band_since_ns == now


def test_no_transit_outside_the_lane_band():
    """Well clear of the lane the ordinary guard keeps its authority."""
    now = 1_000_000_000_000
    harness = _harness((7.00, 0.28, 3.14159), (8.5, 0.10, now), now_ns=now)
    assert SafetyLimiter._swept_band_transit(harness, _twist(1.65)) is None
    assert harness.swept_band_since_ns is None


def test_transit_expires_so_a_wedged_base_cannot_drive_forever():
    """Past the commit window the guard takes over again."""
    now = 1_000_000_000_000
    stale_start = now - 7_000_000_000
    harness = _harness(
        (8.30, 0.28, 3.14159), (8.5, 0.10, now), since=stale_start, now_ns=now
    )
    assert SafetyLimiter._swept_band_transit(harness, _twist(1.65)) is None
    assert harness.swept_band_since_ns is None


def test_stale_obstacle_truth_does_not_commit_transit():
    """Without fresh truth the band is unknown, so no commitment."""
    now = 1_000_000_000_000
    harness = _harness(
        (8.30, 0.28, 3.14159), (8.5, 0.10, now - 2_000_000_000), now_ns=now
    )
    assert SafetyLimiter._swept_band_transit(harness, _twist(1.65)) is None


def test_transit_ignores_a_command_that_is_already_stopped():
    """A base commanded to hold is not pushed into motion."""
    now = 1_000_000_000_000
    harness = _harness((8.30, 0.28, 3.14159), (8.5, 0.10, now), now_ns=now)
    assert SafetyLimiter._swept_band_transit(harness, _twist(0.01)) is None


def test_transit_speed_follows_a_slower_upstream_command():
    """Transit never exceeds the upstream command."""
    now = 1_000_000_000_000
    harness = _harness((8.30, 0.28, 3.14159), (8.5, 0.10, now), now_ns=now)
    out = SafetyLimiter._swept_band_transit(harness, _twist(0.40))
    assert out is not None
    assert out.linear.x == pytest.approx(0.40)


def test_callback_publishes_transit_instead_of_a_stop():
    """The wired callback must not zero the base inside the band."""
    now = 1_000_000_000_000
    harness = _harness((8.30, 0.28, 3.14159), (8.5, 0.10, now), now_ns=now)
    harness.base_hold = False
    harness.east_gate_open = False
    harness.east_gate_blocked = lambda _sign: False
    harness.west_lane_blocked = lambda _sign: False
    harness.scale_for_obstacles = lambda _sign, _command=None: 0.0
    SafetyLimiter.cmd_vel_callback(harness, _twist(1.65, -0.55))
    assert len(harness.publisher.published) == 1
    published = harness.publisher.published[0]
    assert published.linear.x == pytest.approx(0.80)


def test_callback_still_stops_outside_the_band():
    """Outside the band the original stop behaviour is preserved."""
    now = 1_000_000_000_000
    harness = _harness((7.00, 0.28, 3.14159), (8.5, 0.10, now), now_ns=now)
    harness.base_hold = False
    harness.east_gate_open = False
    harness.east_gate_blocked = lambda _sign: False
    harness.west_lane_blocked = lambda _sign: False
    harness.scale_for_obstacles = lambda _sign, _command=None: 0.0
    SafetyLimiter.cmd_vel_callback(harness, _twist(1.65, -0.55))
    assert len(harness.publisher.published) == 1
    published = harness.publisher.published[0]
    assert published.linear.x == pytest.approx(0.0)
