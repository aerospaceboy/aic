"""Delivery regression: swept-volume clearance and truthful slowdown events."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from tools_demo.b_delivery_route import b_south_waypoints
from tools_demo.navigation_math import ZonePose
from tools_demo.continuous_response import ContinuousResponse
from test_navigation_contract import _world_boxes, _point_to_oriented_box


def test_south_corridor_clears_static_geometry_and_entire_patrol_sweeps():
    repo = Path(__file__).resolve().parents[2]
    boxes = _world_boxes(ET.parse(repo / 'mybot_description/worlds/offic_room.world').getroot())
    # Synthetic east dock beyond the helper's dock.x > 9.0 gate: the
    # corridor itself is legacy geometry, not the moved B bay at (0, -12).
    route = b_south_waypoints(
        11.17, -4.99, ZonePose('B_legacy_east', 12.0, 0.0, math.pi)
    )
    assert len(route) == 2
    a, b = route
    for i in range(301):
        x = a.x + (b.x-a.x)*i/300
        y = a.y + (b.y-a.y)*i/300
        assert min(_point_to_oriented_box(x, y, box) for _, box in boxes) > 0.50
        for ox in (5.0, 8.5):
            oy = min(-0.1, max(-1.7, y))
            assert math.hypot(x-ox, y-oy) - 0.28 - 0.18 > 0.20


def test_b_corridor_does_not_change_north_west_or_unknown_starts():
    dock = ZonePose('B_legacy_east', 12.0, 0.0, math.pi)
    for x, y in [(11.15, 3.0), (-3.1, 3.0), (None, -5.0), (11.0, None)]:
        assert not b_south_waypoints(x, y, dock)
    # The moved B bay docks west of the helper's gate, so it stays inert.
    assert not b_south_waypoints(
        11.17, -4.99, ZonePose('B', -0.40, -12.0, 0.0)
    )


def test_continuous_response_needs_observed_deceleration():
    observer = ContinuousResponse(2.6, 1.4, 0.85)
    assert not observer.observe(1.1, 0.0, 1.0)
    assert not observer.observe(1.1, 0.4, 1.2)
    assert not observer.observe(2.0, 1.3, 1.4)
    assert observer.observe(1.2, 0.6, 1.6)
    assert observer.evidence['peak_speed'] == 1.3


def test_continuous_response_rejects_stale_and_outside_encounters():
    observer = ContinuousResponse(2.6, 1.4, 0.85)
    observer.observe(2.0, 1.3, 1.0)
    assert not observer.observe(1.1, 0.3, 1.0)
    assert not observer.observe(1.1, 0.3, 3.0)
    observer.observe(2.0, 1.3, 3.2)
    observer.observe(3.0, 1.3, 3.4)
    assert not observer.observe(1.1, 0.3, 3.6)
