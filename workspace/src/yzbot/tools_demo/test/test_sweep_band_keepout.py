"""Contract tests for the sweep-band keepout publisher."""

import math

from tools_demo.sweep_band_keepout import (
    band_points,
    keepout_message,
    parse_bands,
)

BAND_SPECS = [
    "preview_moving_partition_1:-7.00:1.84:-3.80:1.96",
    "preview_moving_partition_2:6.44:-6.30:7.96:-4.70",
]


def test_parse_bands_round_trip():
    """Well-formed band specifications parse into numeric rectangles."""
    bands = parse_bands(BAND_SPECS)
    assert [band[0] for band in bands] == [
        "preview_moving_partition_1",
        "preview_moving_partition_2",
    ]
    assert bands[1][1:] == (6.44, -6.30, 7.96, -4.70)


def test_parse_bands_rejects_malformed():
    """Malformed or degenerate specifications are skipped, not raised."""
    assert parse_bands(["", "name:1:2:3", "name:a:b:c:d",
                        "name:5:0:1:1", "name:0:5:1:1"]) == []


def test_band_points_cover_inflated_rectangle():
    """The grid covers the rectangle plus the margin at the resolution."""
    margin = 0.38
    resolution = 0.05
    points = band_points(6.44, -6.30, 7.96, -4.70, margin, resolution)
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    assert min(xs) <= 6.44 - margin + 1e-9
    assert max(xs) >= 7.96 + margin - resolution - 1e-9
    assert min(ys) <= -6.30 - margin + 1e-9
    assert max(ys) >= -4.70 + margin - resolution - 1e-9
    # every point stays within one resolution step of the inflated rectangle
    for x, y, _ in points:
        assert 6.44 - margin - 1e-9 <= x <= 7.96 + margin + resolution
        assert -6.30 - margin - 1e-9 <= y <= -4.70 + margin + resolution
    # grid spacing honours the resolution
    unique_x = sorted(set(round(x, 6) for x in xs))
    steps = {round(b - a, 6) for a, b in zip(unique_x, unique_x[1:])}
    assert steps == {round(resolution, 6)}


def test_keepout_message_layout():
    """The cloud is one dense row of xyz float32 points."""
    points = [(0.0, 0.0, 0.0), (0.05, 0.0, 0.0)]
    message = keepout_message(points, "map", 12.5)
    assert message.header.frame_id == "map"
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == \
        (12, 500000000)
    assert (message.height, message.width) == (1, 2)
    assert message.point_step == 12
    assert len(message.data) == 24
    assert [field.name for field in message.fields] == ["x", "y", "z"]


def test_wall_two_band_covers_the_squeeze_site():
    """The 2026-10-05 squeeze site (7.3, -6.59) falls inside the keepout."""
    points = band_points(6.44, -6.30, 7.96, -4.70, 0.38, 0.05)
    assert any(
        math.hypot(x - 7.3, y + 6.59) < 0.2 for x, y, _ in points
    )
