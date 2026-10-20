"""Replay the impossible crossing route and published grid boundaries."""
from types import SimpleNamespace

import pytest
from nav_msgs.msg import OccupancyGrid

from tools_demo.navigation_math import lower_east_corridor_waypoints
from tools_demo.task_navigator import TaskNavigator


def test_north_to_blue1_descends_west_of_the_patrol_lines():
    """The failed y=-0.45 eastbound lane cannot offer a safe window."""
    route = lower_east_corridor_waypoints((-4.6, 3.8), (10.9, -2.5))
    assert [(p.x, p.y) for p in route] == [(4.4, -2.5)]


def test_north_to_bay_clears_the_north_wall_east_end():
    """Deliveries descend outside the bay wall instead of crossing it."""
    route = lower_east_corridor_waypoints((-2.1, 3.8), (11.3, -5.0))
    assert [(p.x, p.y) for p in route] == [(4.4, -2.5), (13.4, -2.5)]


def test_bay_to_upper_left_uses_the_same_safe_lane_in_reverse():
    """Returning to A must not wait in the movers' swept Y band."""
    route = lower_east_corridor_waypoints((10.3, -5.0), (-5.0, 3.8))
    assert [(p.x, p.y) for p in route] == [(13.4, -2.5), (4.4, -2.5)]


@pytest.mark.parametrize("start,goal", [
    ((13.2, -.95), (-2.1, 3.8)),
    ((10.3, -5.), (12.9, -6.6)),
    ((-10., -5.), (2.9, -12.1)),
])
def test_corridor_guides_do_not_intercept_unrelated_legs(start, goal):
    """Birth staging, local bay trips and C routes retain their own plans."""
    assert not lower_east_corridor_waypoints(start, goal)


def grid_node():
    """Create a real int8 occupancy message with translated Nav2 values."""
    grid = OccupancyGrid()
    grid.info.width = grid.info.height = 2
    grid.info.resolution = 0.1
    grid.data = [100, 99, 71, -1]
    node = SimpleNamespace(global_costmap=grid, latest_x=.05, latest_y=.05)
    node._global_cost_at = lambda x, y: TaskNavigator._global_cost_at(node, x, y)
    return node


def test_real_grid_lethal_start_is_detected_without_rejecting_inflation():
    """A raw-byte comparison cannot pass this serialized int8 grid test."""
    node = grid_node()
    assert TaskNavigator._lethal_start_cost(node) == 100
    node.latest_x = .15
    assert TaskNavigator._lethal_start_cost(node) is None
    node.latest_x, node.latest_y = .05, .15
    assert TaskNavigator._lethal_start_cost(node) is None


@pytest.mark.parametrize("x,y", [(-.01, .05), (.05, -.01), (.2, .05)])
def test_cost_probe_does_not_alias_a_point_outside_the_grid(x, y):
    """Truncating a negative coordinate used to read cell zero by mistake."""
    assert TaskNavigator._global_cost_at(grid_node(), x, y) is None
