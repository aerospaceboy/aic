"""Contract tests for the carried-payload scan mask."""

import math

from sensor_msgs.msg import LaserScan

from tools_demo.payload_scan_mask import (
    in_payload_box,
    masked_scan,
    should_mask,
)

HOLDING_STATES = frozenset(
    {
        "picking",
        "navigating_departure",
        "navigating_destination",
        "placing",
        "verifying",
        "releasing",
    }
)


def test_payload_return_is_masked():
    """The measured payload return (~0.36 m forward, +/-0.17 m side) masks."""
    assert in_payload_box(0.40, 0.5, 0.30, 0.48, 0.24)
    assert in_payload_box(0.38, -0.5, 0.30, 0.48, 0.24)


def test_dock_tray_return_stays_visible():
    """A tray face ~0.24 m ahead of the chassis stays outside the box."""
    assert not in_payload_box(0.24, 0.0, 0.30, 0.48, 0.24)


def test_close_and_far_returns_stay_visible():
    """Returns nearer than the box and real obstacles beyond it pass."""
    assert not in_payload_box(0.42, 1.2, 0.30, 0.48, 0.24)
    assert not in_payload_box(1.5, 0.0, 0.30, 0.48, 0.24)


def test_mask_gates_on_holding_states_only():
    """Only cube-holding mission states activate the box."""
    for state in (
        "picking",
        "navigating_departure",
        "navigating_destination",
        "placing",
        "verifying",
        "releasing",
    ):
        assert should_mask(state, HOLDING_STATES)
    for state in (
        "idle",
        "navigating_pickup",
        "waiting_perception",
        "navigating_zone_exit",
        "completed",
        "failed",
    ):
        assert not should_mask(state, HOLDING_STATES)


def test_mask_keeps_level_state_without_repeated_publishes():
    """The mission state is level, not a heartbeat: a docked picking phase
    longer than any publish gap must keep the mask active."""
    assert should_mask("navigating_destination", HOLDING_STATES)
    assert should_mask("picking", HOLDING_STATES)


def test_mask_fails_open_before_first_state_message():
    """Without any state message the node must not mask the scan."""
    assert not should_mask(None, HOLDING_STATES)


def _scan_with(ranges_and_angles):
    scan = LaserScan()
    scan.header.frame_id = "laser_link"
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    count = len(ranges_and_angles)
    scan.angle_increment = 2.0 * math.pi / count
    scan.range_min = 0.38
    scan.range_max = 12.0
    scan.ranges = [value for value, _ in ranges_and_angles]
    return scan


def test_masked_scan_removes_only_payload_returns():
    """The republished scan keeps frame, geometry, and other returns."""
    count = 360
    scan = _scan_with([(0.0, 0.0)] * count)
    for index in range(count):
        angle = scan.angle_min + index * scan.angle_increment
        if index % 7 == 0:
            scan.ranges[index] = 1.0 + 0.01 * index
        elif 0.30 <= 0.40 * math.cos(angle) <= 0.48 and abs(
                0.40 * math.sin(angle)) <= 0.24:
            scan.ranges[index] = 0.40
    masked = masked_scan(scan, 0.30, 0.48, 0.24)
    assert masked.header.frame_id == "laser_link"
    assert len(masked.ranges) == count
    removed = 0
    for index, (original, value) in enumerate(
            zip(scan.ranges, masked.ranges)):
        angle = scan.angle_min + index * scan.angle_increment
        if in_payload_box(original, angle, 0.30, 0.48, 0.24):
            assert math.isnan(value)
            removed += 1
        else:
            assert value == original
    assert removed > 0


def test_masked_scan_without_payload_is_passthrough_values():
    """A scan without payload-box returns is republished unchanged."""
    count = 90
    scan = _scan_with([(1.0 + 0.01 * index, 0.0) for index in range(count)])
    masked = masked_scan(scan, 0.30, 0.48, 0.24)
    assert masked.ranges == scan.ranges
    assert masked.angle_min == scan.angle_min
    assert masked.range_max == scan.range_max
