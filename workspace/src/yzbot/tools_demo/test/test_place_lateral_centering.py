"""Recover placement refusals caused by docking geometry error."""

import math
from types import SimpleNamespace

import pytest

from tools_demo.mission_coordinator import (
    MissionCoordinator,
    PLACE_ARM_FORWARD_TARGET,
    PLACE_ARM_MAX_FORWARD,
    PLACE_ARM_MIN_FORWARD,
    PLACE_LATERAL_ALLOW,
    place_lateral_centering_needed,
    place_reposition_correction,
    place_target_inside_arm_window,
)


@pytest.mark.parametrize("local_y,expected", [
    (0.0, False), (0.019, False), (0.05, False), (0.08, False),
    (-0.08, False), (0.081, True), (-0.122, True), (0.16, True),
    (None, False), (float("nan"), False), (float("inf"), False),
])
def test_only_a_meaningful_lateral_offset_triggers_centring(
        local_y, expected):
    assert place_lateral_centering_needed(local_y) is expected


@pytest.mark.parametrize("local,expected", [
    ((0.33, 0.0), None),
    ((0.34, 0.01), None),
    ((0.33, -0.122), (0.0, -0.122)),
    ((0.203, -0.132), (-0.127, -0.132)),
    ((0.50, 0.30), (0.15, 0.15)),
    ((None, None), None),
    ((0.33,), None),
    ((float("nan"), 0.0), None),
])
def test_reposition_correction_targets_the_arm_window(local, expected):
    correction = place_reposition_correction(local)
    if expected is None:
        assert correction is None
    else:
        assert correction == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize("local,expected", [
    ((0.33, 0.0), True),
    ((0.25, -0.119), True),
    ((0.38, 0.119), True),
    ((0.203, -0.132), False),
    ((0.40, 0.0), False),
    ((0.33, 0.13), False),
    (None, False),
])
def test_arm_window_is_the_final_authority(local, expected):
    assert place_target_inside_arm_window(local) is expected


def test_arm_window_constants_match_the_checked_placement_workspace():
    assert PLACE_ARM_MIN_FORWARD == 0.25
    assert PLACE_ARM_MAX_FORWARD == 0.38
    assert PLACE_ARM_FORWARD_TARGET == 0.33
    assert PLACE_LATERAL_ALLOW < 0.12


def test_place_slot_local_uses_world_slot_and_current_pose():
    node = SimpleNamespace(
        _odom_pose=lambda: (11.362, -4.943, 0.034),
        _delivery_slot_world=lambda job: (11.692, -5.053),
    )
    local = MissionCoordinator._place_slot_local(node, object())
    assert local is not None
    assert local[0] > 0.2
    assert local[1] < -0.10
    assert abs(local[1]) > 0.08


def test_place_slot_local_is_none_without_odometry():
    node = SimpleNamespace(_odom_pose=lambda: None,
                           _delivery_slot_world=lambda job: (0.0, 0.0))
    assert MissionCoordinator._place_slot_local(node, object()) is None
