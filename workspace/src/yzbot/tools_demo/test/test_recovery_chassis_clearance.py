"""Tests for the recovery chassis-clearance gate (4bC_1rB review finding).

The review reproduced a bounded-recovery pose whose centre cell was free while
the rotated 0.32 m chassis corner intruded 8 mm into a 1.0 m world box. The
recovery gate must therefore check the circumscribed chassis circle plus a
margin, not one cell.
"""

import math
from types import SimpleNamespace

from tools_demo.task_navigator import TaskNavigator


def costmap_with_box(x_min, x_max, y_min, y_max,
                     resolution=0.05, width=200, height=200):
    """Occupancy-grid stub with one lethal world-space box."""
    origin_x, origin_y = -5.0, -10.0
    data = [0] * (width * height)
    for py in range(height):
        y = origin_y + (py + 0.5) * resolution
        if not (y_min <= y <= y_max):
            continue
        for px in range(width):
            x = origin_x + (px + 0.5) * resolution
            if x_min <= x <= x_max:
                data[py * width + px] = 255
    info = SimpleNamespace(width=width, height=height, resolution=resolution,
                           origin=SimpleNamespace(
                               position=SimpleNamespace(x=origin_x,
                                                        y=origin_y)))
    return SimpleNamespace(info=info, data=data)


def node_with(costmap):
    node = SimpleNamespace(
        global_costmap=costmap,
        RECOVERY_CHASSIS_HALF_DIAGONAL=
        TaskNavigator.RECOVERY_CHASSIS_HALF_DIAGONAL,
        RECOVERY_POSE_MARGIN=TaskNavigator.RECOVERY_POSE_MARGIN,
    )
    node._global_cost_at = TaskNavigator._global_cost_at.__get__(node)
    node._costmap_cell_free = TaskNavigator._costmap_cell_free.__get__(node)
    return node


def check(node, x, y):
    return TaskNavigator._recovery_pose_clearance(node, x, y)


def test_berth_pose_with_center_free_but_corner_inside_box_is_rejected():
    # The exact reproduction: robot centre 0.87,-7.50 near a 1 m box spanning
    # x in [1.07, 2.07]; the centre cell is free but the chassis corner is not.
    box = costmap_with_box(1.07, 2.07, -8.23, -7.23)
    node = node_with(box)
    ok, margin = check(node, 0.87, -7.50)
    assert ok is False
    assert margin == 0.0


def test_recovery_pose_with_full_chassis_margin_is_accepted():
    box = costmap_with_box(1.07, 2.07, -8.23, -7.23)
    node = node_with(box)
    # 0.55 m from the box edge leaves the 0.226 m chassis half-diagonal and the
    # 0.04 m gate margin free in every direction.
    ok, margin = check(node, 0.55, -7.50)
    assert ok is True
    assert margin > 0.2


def test_missing_costmap_keeps_historical_behaviour_but_reports_it():
    node = node_with(None)
    ok, margin = check(node, 0.87, -7.50)
    assert ok is True
    assert math.isnan(margin)


def test_box_outside_chassis_ring_does_not_reject_pose():
    box = costmap_with_box(1.07, 2.07, -8.23, -7.23)
    node = node_with(box)
    # 0.30 m from the box edge in +x: ring radius 0.266 stays outside it.
    ok, _ = check(node, 0.80, -6.80)
    assert ok is True
