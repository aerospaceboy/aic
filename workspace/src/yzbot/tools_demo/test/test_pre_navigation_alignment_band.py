"""Pre-navigation turn: a bounded stop band that is not the docking band.

The in-place turn before a hard leg exists so Nav2 does not have to start the
leg with a spin/recovery stall. It is not a docking pose, so it does not need
the 0.04 rad docking band -- and the proportional law ``w = 3.0 * error``
spends most of its time in the last few degrees. On the 2026-10-17 4rA_1bB
round the navigator pre-turned 31 times for 46.1 s in total, each one a
near-constant ~1.5 s that barely varied with the turn size (1.80 s at 0.67 rad,
2.03 s at 3.13 rad), which is the tail of that law rather than physics.

These tests pin the separation: the pre-navigation branch stops at the loose
band, docking keeps the tight band, and the loose band is provably no worse
than a leg that never pre-turned at all.
"""

import inspect
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools_demo.navigation_math import needs_pre_navigation_alignment
from tools_demo.task_navigator import (
    TaskNavigator,
    resolve_pre_navigation_tolerance,
)

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "bot_navigation/config/competition_zones.yaml"
NAV_PARAM = REPO / "bot_navigation/param/originbot_nav2.yaml"

DOCKING_BAND = 0.04
PRE_NAV_BAND = 0.25
SHIM_THRESHOLD = 0.45


class AlignmentHarness:
    """Minimal stand-in for the direct-velocity alignment callback."""

    def __init__(self, error, pre_navigation=True):
        self.pre_navigation_alignment = pre_navigation
        self.alignment_target_yaw = 0.0
        self.latest_yaw = -float(error)
        self.latest_x = 0.0
        self.latest_y = 0.0
        self.alignment_deadline_ns = 10 ** 18
        # The zone-egress branch precedes the pre-navigation branch in the
        # callback and owns its own velocity law; this harness exercises the
        # pre-navigation one.
        self.zone_egress_target = None
        self.goal_start_ns = 0
        self.alignment_yaw_tolerance = DOCKING_BAND
        self.pre_navigation_alignment_yaw_tolerance = PRE_NAV_BAND
        self.alignment_max_angular_speed = 2.00
        self.dock_pose = None
        self.commands = []
        self.states = []
        self.dispatched = []
        self._logger = Mock()

    def get_clock(self):
        now = SimpleNamespace(nanoseconds=1_000_000_000)
        return SimpleNamespace(now=lambda: now)

    def get_logger(self):
        return self._logger

    def publish_state(self, state, **fields):
        self.states.append((state, fields.get("phase")))

    def stop_alignment(self):
        self.commands.append(("stop", None))

    def _publish_direct_velocity(self, command):
        self.commands.append(("twist", command.angular.z))

    def _send_next_goal(self):
        self.dispatched.append(True)


@pytest.mark.parametrize("error", [0.05, 0.10, 0.15, 0.20, 0.24])
def test_pre_navigation_turn_completes_anywhere_inside_the_loose_band(error):
    """Up to 0.25 rad the turn is finished and the leg is dispatched."""
    harness = AlignmentHarness(error)
    TaskNavigator.alignment_callback(harness)
    assert harness.dispatched == [True]
    assert harness.pre_navigation_alignment is False
    assert ("sending", "pre_navigation_complete") in harness.states


@pytest.mark.parametrize("error", [0.26, 0.30, 0.45, 1.00, math.pi])
def test_pre_navigation_turn_keeps_rotating_above_the_loose_band(error):
    """Beyond 0.25 rad the turn continues and nothing is dispatched."""
    harness = AlignmentHarness(error)
    TaskNavigator.alignment_callback(harness)
    assert harness.dispatched == []
    assert harness.pre_navigation_alignment is True
    twists = [value for kind, value in harness.commands if kind == "twist"]
    assert twists and all(value != 0.0 for value in twists)


def test_the_loose_band_never_completes_before_the_old_band_would_have():
    """The change can only ever shorten the turn, never lengthen it."""
    for error in (0.05, 0.12, 0.25):
        harness = AlignmentHarness(error)
        TaskNavigator.alignment_callback(harness)
        assert harness.dispatched == [True]
    harness = AlignmentHarness(0.03)
    TaskNavigator.alignment_callback(harness)
    assert harness.dispatched == [True]


def test_docking_alignment_still_uses_the_tight_band():
    """The docking branch must not inherit the loose pre-navigation band."""
    source = inspect.getsource(TaskNavigator.alignment_callback)
    assert source.count("pre_navigation_alignment_yaw_tolerance") == 1
    assert "abs(error) <= self.alignment_yaw_tolerance" in source
    assert "abs(error) <= self.pre_navigation_alignment_yaw_tolerance" in source


def test_the_gate_would_not_pre_turn_at_the_loose_band():
    """Non-regression proof: 0.25 rad is inside the 'do not align' region.

    ``needs_pre_navigation_alignment`` already refuses to pre-turn at or below
    0.35 rad, so finishing there cannot be worse than a leg that never
    pre-turned.
    """
    for distance in (0.5, 1.0, 3.0, 8.0, 15.0):
        assert needs_pre_navigation_alignment(distance, PRE_NAV_BAND) is False
        assert needs_pre_navigation_alignment(distance, 0.35) is False
    # And the gate does still fire for the hard turns it was written for.
    assert needs_pre_navigation_alignment(8.0, 0.36) is True
    assert needs_pre_navigation_alignment(8.0, 1.20) is True


def test_the_loose_band_stays_below_the_rotation_shim_threshold():
    """Nav2 must still have nothing to rotate once the turn completes."""
    parameters = NAV_PARAM.read_text()
    assert "angular_dist_threshold: 0.45" in parameters
    assert "rotate_to_heading_min_angle: 0.45" in parameters
    assert PRE_NAV_BAND < SHIM_THRESHOLD
    assert PRE_NAV_BAND <= 0.35


def test_config_declares_both_bands_without_touching_the_docking_one():
    """The shipped values are explicit and the docking band is unchanged."""
    config = CONFIG.read_text()
    assert "alignment_yaw_tolerance: 0.04" in config
    assert f"pre_navigation_alignment_yaw_tolerance: {PRE_NAV_BAND}" in config


def test_a_non_positive_configuration_inherits_the_docking_band():
    """Unconfigured deployments keep the previous behaviour exactly."""
    assert resolve_pre_navigation_tolerance(0.0, 0.04) == 0.04
    assert resolve_pre_navigation_tolerance(-1.0, 0.04) == 0.04
    assert resolve_pre_navigation_tolerance(0.25, 0.04) == 0.25


def test_the_tail_saving_is_the_measured_magnitude():
    """The change removes the fixed exponential tail from every pre-turn.

    Time in the proportional tail from the cap breakpoint w/gain down to the
    stop band is ln((w/gain)/band)/gain. Moving 0.04 -> 0.25 removes
    ln(0.25/0.04)/3.0 = 0.61 s per turn; 31 turns in the measured round.
    """
    gain, cap = 3.0, 2.00
    breakpoint = cap / gain
    saved = (
        math.log(breakpoint / DOCKING_BAND) / gain
        - math.log(breakpoint / PRE_NAV_BAND) / gain
    )
    assert saved == pytest.approx(0.61, abs=0.02)
    assert saved * 31 == pytest.approx(18.9, abs=0.6)


def test_the_resolved_bands_are_logged_for_every_run():
    """An acceptance run must carry the deployed values, not the yaml."""
    source = inspect.getsource(TaskNavigator.__init__)
    assert "alignment_bands " in source
    assert "pre_navigation=" in source
