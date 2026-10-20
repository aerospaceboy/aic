"""Keep loaded geometry safe without a new global planner or controller."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from tools_demo.navigation_math import wall57_pocket_waypoints
from test_north_pickup_clearance import loaded_boxes
from test_navigation_contract import _point_to_oriented_box

REPO = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('reverse', [False, True])
def test_s_passage_whole_segments_clear_payload_and_arrival_reserve(reverse):
    start, target = (-4.55, 3.77), (-6.6, -2.8)
    if reverse:
        start, target = target, start
    route = wall57_pocket_waypoints(start, target)
    assert len(route) == 4
    chain = [start] + [(p.x, p.y) for p in route] + [target]
    boxes = loaded_boxes()
    for a, b in zip(chain, chain[1:]):
        for i in range(201):
            point = (a[0]+(b[0]-a[0])*i/200, a[1]+(b[1]-a[1])*i/200)
            assert min(_point_to_oriented_box(*point, box)-.35-.20
                       for _, box in boxes) >= .10


def test_s_passage_keeps_frozen_tree_except_tighter_arrival_checker():
    normal = ET.parse(REPO/'bot_navigation/config/navigate_to_pose_recovery.xml').getroot()
    corner = ET.parse(REPO/'tools_demo/config/navigate_clearance_to_pose.xml').getroot()
    follow = corner.find('.//FollowPath')
    assert follow.attrib['goal_checker_id'] == 'central_goal_checker'
    follow.attrib['goal_checker_id'] = 'general_goal_checker'
    assert ET.tostring(corner) == ET.tostring(normal)


def test_other_layout_and_west_dock_cannot_enter_east_s_passage():
    assert not wall57_pocket_waypoints((-4.55, 3.77), (-8.4, -2.8))
    assert not wall57_pocket_waypoints((-4.55, 3.77), (-3.8, -12.4))
    assert not wall57_pocket_waypoints((float('nan'), 3.77), (-6.6, -2.8))
