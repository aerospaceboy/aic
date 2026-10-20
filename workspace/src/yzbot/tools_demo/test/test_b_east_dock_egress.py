"""B egress after navigation has cleared the completed dock goal."""
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from tools_demo.task_navigator import TaskNavigator, ZonePose


class EgressHarness:
    start_zone_egress = TaskNavigator.start_zone_egress

    def __init__(self, yaw):
        self.latest_x = 0.40 if math.cos(yaw) < 0 else -0.40
        self.latest_y = -12.00
        self.latest_yaw = yaw
        self.active_zone = "B_EXIT"
        self.zone_exit_gateway = ZonePose("B_retreat", -1.49, -12.0, 0.0)
        # _clear_goal() clears this before the later B_EXIT command.
        self.dock_pose = None
        self.refresh_map_pose = Mock()
        self._nearest_delivery_zone = Mock(return_value="B")
        self._b_egress_target = Mock(return_value=None)
        self.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_000_000))
        self.alignment_timeout_ns = 1_000_000
        self.publish_state = Mock()


@pytest.mark.parametrize("yaw", [math.pi, -math.pi, math.pi - 0.05])
def test_east_dock_backs_away_after_completed_goal_is_cleared(yaw):
    harness = EgressHarness(yaw)
    harness.start_zone_egress()
    assert harness.zone_egress_target == pytest.approx((
        harness.latest_x - 0.45 * math.cos(yaw),
        harness.latest_y - 0.45 * math.sin(yaw)))
    assert harness.zone_egress_target[0] > harness.latest_x


def test_west_dock_keeps_configured_gateway():
    harness = EgressHarness(0.0)
    harness.start_zone_egress()
    assert harness.zone_egress_target == (-1.49, -12.0)
