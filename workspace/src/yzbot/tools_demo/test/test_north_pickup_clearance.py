"""A -> blue1 must clear the real loaded northern tray and wall tips."""
import inspect
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest

from tools_demo.navigation_math import ZonePose
from tools_demo.task_navigator import TaskNavigator
from test_navigation_contract import _compose_planar, _planar_pose, _point_to_oriented_box

REPO = Path(__file__).resolve().parents[2]


def loaded_boxes():
    root = ET.parse(REPO/'mybot_description/worlds/offic_room.world').getroot()
    result = []
    for model in root.findall('./world/model'):
        state = root.find(f'./world/state/model[@name="{model.attrib["name"]}"]')
        model_pose = _planar_pose(state if state is not None else model)
        for link in model.findall('link'):
            saved = state.find(f'link[@name="{link.attrib["name"]}"]') if state is not None else None
            # Saved link poses are WORLD poses, not model-relative offsets.
            link_pose = _planar_pose(saved) if saved is not None else _compose_planar(model_pose, _planar_pose(link))
            for collision in link.findall('collision'):
                size = collision.findtext('geometry/box/size')
                if not size:
                    continue
                sx, sy, sz = map(float, size.split())
                p = _compose_planar(link_pose, _planar_pose(collision))
                if p[2]-sz/2 > .5 or p[2]+sz/2 <= 0:
                    continue
                result.append((model.attrib['name']+'/'+link.attrib['name'],
                               (p[0], p[1], sx/2, sy/2, p[5])))
    return result


@pytest.mark.parametrize('start', [(-4.55, 3.77), (-5.2, 3.4), (-5.2, 4.1),
                                  (-3.8, 3.4), (-3.8, 4.1)])
def test_north_pickup_lane_clears_loaded_world_with_arrival_error(start):
    node = SimpleNamespace(use_legacy_layout_routes=False, get_logger=lambda: Mock(),
                           dock_pose=ZonePose('BLUE_CUBE_1_ENTRY', 6.5, 3.6, 0.))
    guides = TaskNavigator._cube_approach_waypoints(node, 'BLUE_CUBE_1', *start)
    assert [g.zone for g in guides] == ['north_pickup_tray_west_clear', 'north_pickup_tray_east_clear']
    chain = [start]+[(g.x, g.y) for g in guides]+[(6.5, 3.6)]
    boxes = loaded_boxes()
    for a, b in zip(chain, chain[1:]):
        for i in range(201):
            x, y = a[0]+(b[0]-a[0])*i/200, a[1]+(b[1]-a[1])*i/200
            gap, name = min((_point_to_oriented_box(x, y, box)-.28-.20, name)
                            for name, box in boxes)
            assert gap >= .10, (start, x, y, name, gap)


def test_clear_lane_does_not_change_other_starts_or_pickups():
    node = SimpleNamespace(use_legacy_layout_routes=False, get_logger=lambda: Mock(),
                           dock_pose=ZonePose('BLUE_CUBE_1_ENTRY', 6.5, 3.6, 0.))
    assert not TaskNavigator._cube_approach_waypoints(node, 'BLUE_CUBE_1', 2., 3.7)
    assert not TaskNavigator._cube_approach_waypoints(node, 'BLUE_CUBE_1', -4.5, -2.)
    node.dock_pose = ZonePose('entry_east', 8.3, 3.6, math.pi)
    assert not TaskNavigator._cube_approach_waypoints(node, 'BLUE_CUBE_1', -4.5, 3.7)
    assert '"north_pickup_tray_"' in inspect.getsource(TaskNavigator._send_next_goal)


@pytest.mark.parametrize('start', [(6.5, 3.6), (6.3, 3.4), (6.3, 3.8),
                                  (6.7, 3.4), (6.7, 3.8)])
def test_loaded_return_uses_the_same_clear_lane_in_reverse(start):
    place = ZonePose('A', -5., 3.8, math.pi)
    node = SimpleNamespace(
        _route_pose=lambda zone, key: place, _delivery_dock_pose=lambda zone, p: p,
        _route_waypoints=lambda zone, key: [], zones={'A': place},
        dock_arrival_zone=.04, dock_speed_zone=.25, use_legacy_layout_routes=False,
        latest_x=start[0], latest_y=start[1], get_logger=lambda: Mock())
    TaskNavigator._configure_delivery_route(node, 'A')
    assert [p.zone for p in node.pending_poses] == [
        'north_delivery_tray_east_clear', 'north_delivery_tray_west_clear', 'A']
    assert node.a_corridor_route_active
    chain = [start]+[(p.x, p.y) for p in node.pending_poses]
    boxes = loaded_boxes()
    for a, b in zip(chain, chain[1:]):
        for i in range(201):
            x, y = a[0]+(b[0]-a[0])*i/200, a[1]+(b[1]-a[1])*i/200
            gap, name = min((_point_to_oriented_box(x, y, box)-.28-.20, name)
                            for name, box in boxes)
            assert gap >= .10-1e-9, (start, x, y, name, gap)
    assert '"north_delivery_tray_"' in inspect.getsource(TaskNavigator._send_next_goal)
