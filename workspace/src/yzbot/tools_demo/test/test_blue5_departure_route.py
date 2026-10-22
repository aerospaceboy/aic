"""Do not pull a loaded blue5 return into the real diagonal wall."""

import pytest

from tools_demo.navigation_math import ZonePose
from tools_demo.transport_routes import current_delivery_route
from test_navigation_contract import _point_to_oriented_box
from test_north_pickup_clearance import loaded_boxes


def test_old_translated_blue5_staging_cannot_fit_the_real_chassis():
    boxes = loaded_boxes()
    distance, name = min(
        (_point_to_oriented_box(10.9306, -10.9997, box), name)
        for name, box in boxes)
    assert name == 'officeroom/Wall_132'
    assert distance < .28


@pytest.mark.parametrize('start', [(10.90, -13.0), (10.93, -13.0),
                                  (11.0, -13.1), (12.7, -13.0)])
@pytest.mark.parametrize('backup', [False, True])
def test_blue5_to_b_plans_around_the_wall_without_an_unsafe_translated_anchor(start, backup):
    place = ZonePose('B_place_pose_slot', 11.3, -4.84, 0.)
    poses, _ = current_delivery_route('B', *start, place, backup=backup)
    assert poses == [place]


def test_other_required_bay_clearance_anchors_are_not_removed():
    place = ZonePose('B', 11.3, -4.84, 0.)
    poses, dedicated = current_delivery_route('B', 12.3, -9.8, place)
    assert dedicated
    assert [p.zone for p in poses[:-1]] == [
        'bay_pocket_north_1', 'bay_pocket_north_2',
        'bay_pocket_north_3', 'bay_pocket_north_4']
    assert poses[-1] == place
