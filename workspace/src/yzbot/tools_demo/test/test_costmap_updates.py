"""Replay Nav2 incremental maps at the navigator's cache boundary."""

from types import SimpleNamespace
from unittest.mock import Mock

from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
import pytest

from tools_demo.task_navigator import TaskNavigator


def harness():
    """Start with a full map whose pickup cell is inflated."""
    node = SimpleNamespace(global_costmap=None,
                           _retry_pending_command=Mock(),
                           get_logger=lambda: SimpleNamespace(warn=Mock()))
    full = OccupancyGrid()
    full.header.frame_id = 'map'
    full.info.width = 4
    full.info.height = 3
    full.info.resolution = 0.05
    full.data = [0] * 12
    full.data[5] = 83
    TaskNavigator._global_costmap_callback(node, full)
    return node


def apply_update(node, update):
    """Old navigator ignores the incremental topic entirely."""
    callback = getattr(TaskNavigator, '_global_costmap_update_callback', None)
    if callback is not None:
        callback(node, update)


@pytest.mark.parametrize('new_cost', [0, 99, 100])
def test_incremental_cost_replaces_stale_pickup_cell(new_cost):
    """Both clearing and new lethal occupancy must reach cost queries."""
    node = harness()
    update = OccupancyGridUpdate()
    update.header.frame_id = 'map'
    update.x, update.y, update.width, update.height = 1, 1, 1, 1
    update.data = [new_cost]
    apply_update(node, update)
    assert TaskNavigator._global_cost_at(node, 0.075, 0.075) == new_cost


def test_two_row_update_keeps_other_cells_intact():
    """Patch rows using full-map stride rather than patch width."""
    node = harness()
    update = OccupancyGridUpdate()
    update.header.frame_id = 'map'
    update.x, update.y, update.width, update.height = 1, 1, 2, 2
    update.data = [10, 20, 30, 40]
    apply_update(node, update)
    assert list(node.global_costmap.data) == [
        0, 0, 0, 0, 0, 10, 20, 0, 0, 30, 40, 0,
    ]


@pytest.mark.parametrize('frame,x,width,data', [
    ('odom', 1, 1, [0]), ('map', -1, 1, [0]),
    ('map', 3, 2, [0, 0]), ('map', 1, 2, [0]),
])
def test_invalid_incremental_update_cannot_corrupt_cache(
    frame, x, width, data,
):
    """Reject frame, bounds and payload errors atomically."""
    node = harness()
    before = list(node.global_costmap.data)
    update = OccupancyGridUpdate()
    update.header.frame_id = frame
    update.x, update.y, update.width, update.height = x, 1, width, 1
    update.data = data
    apply_update(node, update)
    assert list(node.global_costmap.data) == before
