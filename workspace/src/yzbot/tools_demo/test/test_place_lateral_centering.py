"""Recover placement refusals caused by lateral docking error."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools_demo.mission_coordinator import (
    MissionCoordinator,
    PLACE_LATERAL_ALLOW,
    place_lateral_centering_needed,
)


@pytest.mark.parametrize("local_y,expected", [
    (0.0, False), (0.019, False), (0.05, False), (0.08, False),
    (-0.08, False), (0.081, True), (-0.122, True), (0.16, True),
    (None, False), (float("nan"), False), (float("inf"), False),
])
def test_only_a_meaningful_lateral_offset_triggers_centring(
        local_y, expected):
    assert place_lateral_centering_needed(local_y) is expected


def test_arm_window_remains_the_hard_limit():
    assert PLACE_LATERAL_ALLOW < 0.12
    assert PLACE_LATERAL_ALLOW > 0.08


def test_place_slot_local_uses_world_slot_and_current_pose():
    node = SimpleNamespace(
        _odom_pose=lambda: (11.362, -4.943, 0.034),
        _delivery_slot_world=lambda job: (11.692, -5.053),
    )
    local = MissionCoordinator._place_slot_local(node, object())
    assert local is not None
    assert local[0] > 0.2
    assert local[1] < -0.10          # the 0.12-arm-window refusal case
    assert abs(local[1]) > 0.08


def test_place_slot_local_is_none_without_odometry():
    node = SimpleNamespace(_odom_pose=lambda: None,
                           _delivery_slot_world=lambda job: (0.0, 0.0))
    assert MissionCoordinator._place_slot_local(node, object()) is None
