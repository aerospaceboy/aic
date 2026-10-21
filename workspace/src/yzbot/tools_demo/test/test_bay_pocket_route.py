"""The south-east bay exit must be proven against the real occupancy map.

AB8-3 and FX1 lost a round because a zone-B delivery from the bay sent one
direct goal whose line crosses the bay wall (map clearance 0.00 m); RPP then
reported collision ahead every cycle until the deadline.
"""

import math
from pathlib import Path

import pytest

from tools_demo.navigation_math import ZonePose
from tools_demo.transport_routes import (
    BAY_POCKET_NORTH_ANCHORS,
    bay_pocket_north_route,
)

REPO = Path(__file__).resolve().parents[2]
MAP = REPO / "bot_navigation" / "maps" / "mapn3"


def _load_map():
    raw = MAP.with_suffix(".pgm").read_bytes()
    # Minimal P5 parser: magic, optional comments, width, height, max value.
    tokens, index = [], 0
    while len(tokens) < 4:
        while index < len(raw) and raw[index:index + 1].isspace():
            index += 1
        if raw[index:index + 1] == b"#":
            while index < len(raw) and raw[index:index + 1] != b"\n":
                index += 1
            continue
        start = index
        while index < len(raw) and not raw[index:index + 1].isspace():
            index += 1
        tokens.append(raw[start:index])
    index += 1
    width, height = int(tokens[1]), int(tokens[2])
    pixels = raw[index:index + width * height]
    return pixels, width, height


def _clearance(samples):
    """Minimum distance from each sample to an occupied map cell."""
    pixels, width, height = _load_map()
    resolution, origin_x, origin_y = 0.05, -15.6, -33.4
    occupied = [
        (col, row)
        for row in range(height)
        for col in range(width)
        if pixels[row * width + col] < 90
    ]
    worst = float("inf")
    for x, y in samples:
        col = int(round((x - origin_x) / resolution))
        row = height - 1 - int(round((y - origin_y) / resolution))
        best = min(
            math.hypot(col - oc, row - orow) * resolution
            for oc, orow in occupied
            if abs(oc - col) <= 40 and abs(orow - row) <= 40
        )
        worst = min(worst, best)
    return worst


def _sampled_leg(start, end, count=40):
    return [
        (start[0] + (end[0] - start[0]) * i / count,
         start[1] + (end[1] - start[1]) * i / count)
        for i in range(count + 1)
    ]


def test_direct_bay_to_zone_b_line_crosses_the_wall():
    """Document the defect: the old single goal passes through the wall."""
    clearance = _clearance(_sampled_leg((12.3, -9.8), (11.3, -4.84)))
    assert clearance < 0.10


def test_pocket_start_gets_the_exit_anchors():
    route = bay_pocket_north_route((12.3, -9.8), ZonePose("B", 11.3, -4.84, 0.0))
    assert len(route) == len(BAY_POCKET_NORTH_ANCHORS)
    assert [pose.zone for pose in route] == [
        "bay_pocket_north_1", "bay_pocket_north_2",
        "bay_pocket_north_3", "bay_pocket_north_4",
    ]
    assert (route[1].x, route[1].y) == (9.90, -8.30)
    assert (route[2].x, route[2].y) == (9.75, -7.70)


@pytest.mark.parametrize("start", [
    (12.3, -9.8), (11.0, -8.2), (10.0, -7.9),
])
def test_exit_route_keeps_the_map_clearance(start):
    route = bay_pocket_north_route(start, ZonePose("B", 11.3, -4.84, 0.0))
    assert route
    points = [(start[0], start[1])] + [(p.x, p.y) for p in route] + [(11.3, -4.84)]
    for a, b in zip(points, points[1:]):
        clearance = _clearance(_sampled_leg(a, b))
        assert clearance >= 0.40, (a, b, clearance)


def test_other_starts_keep_their_existing_route():
    assert not bay_pocket_north_route((5.0, -5.0), ZonePose("B", 11.3, -4.84, 0.0))
    assert not bay_pocket_north_route((12.3, -6.0), ZonePose("B", 11.3, -4.84, 0.0))
    assert not bay_pocket_north_route(
        (12.0, -9.0), ZonePose("A", -5.0, 3.8, 0.0), "A")
