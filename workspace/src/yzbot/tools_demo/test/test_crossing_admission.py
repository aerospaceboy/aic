"""Behavioral regression tests for repeat-crossing safety and payload input."""

import math
from types import SimpleNamespace
from unittest.mock import Mock, patch

from std_msgs.msg import String
from tools_demo.navigation_math import corridor_crossing_clear
from tools_demo.payload_scan_mask import PayloadScanMask
from tools_demo.task_navigator import TaskNavigator


def test_returning_mover_is_not_admitted_at_collision_phase():
    """Reject the physical phase from the previous contact."""
    assert not corridor_crossing_clear(6.1, 0.264, math.pi, 5.0, -0.125, 0.225)


def test_retreating_mover_has_a_clear_crossing_window():
    """Accept a clear retreat phase with time to finish crossing."""
    assert corridor_crossing_clear(6.1, 0.264, math.pi, 5.0, -0.5, -0.225)


def test_unknown_velocity_and_invalid_truth_cannot_admit():
    """Unknown motion and invalid coordinates cannot justify entry."""
    assert not corridor_crossing_clear(6.1, 0.264, math.pi, 5.0, -0.5, 0.0)
    assert not corridor_crossing_clear(
        6.1, 0.264, math.pi, 5.0, math.nan, 0.225)


def test_admission_is_independent_of_previous_response():
    """Prior response scoring never disables another safety hold."""
    topic = '/competition/obstacles/obstacle_2'
    harness = SimpleNamespace(
        mover_response_seen={topic}, mover_admitted=set(),
        mover_hold_lateral=0.56, mover_hold_decel=1.5,
        mover_hold_standoff=1.05, mover_hold_entry_lateral=1.2,
        _corridor_mover_topics=lambda: (topic,),
        _mover_base_frame=lambda t: (1.0, 0.0, 1.0, 0.3, 1.0, 0.0, 0.8),
        _mover_hold_clear=lambda t: False,
    )
    assert TaskNavigator._mover_yield_topic(harness) == topic


def test_admission_latches_only_one_crossing_and_rearms_after_leaving():
    """Prevent hold oscillation without exempting later crossings."""
    topic = '/competition/obstacles/obstacle_2'
    harness = SimpleNamespace(
        mover_response_seen={topic}, mover_admitted={topic},
        mover_hold_lateral=0.56, mover_hold_decel=1.5,
        mover_hold_standoff=1.05, mover_hold_entry_lateral=1.2,
        _corridor_mover_topics=lambda: (topic,),
        _mover_base_frame=lambda t: (0.3, 0.3, 0.3, 0.3, 1, 0, 0.8),
        _mover_hold_clear=lambda t: False,
    )
    assert TaskNavigator._mover_yield_topic(harness) is None
    harness._mover_base_frame = lambda t: (-0.7, 0.3, -0.7, 0.3, 1, 0, 0.8)
    assert TaskNavigator._mover_yield_topic(harness) is None
    assert topic not in harness.mover_admitted
    harness._mover_base_frame = lambda t: (1.0, 0.3, 1.0, 0.3, 1, 0, 0.8)
    assert TaskNavigator._mover_yield_topic(harness) == topic


def test_velocity_uses_distinct_samples():
    """Use distinct position samples to retain movement direction."""
    harness = SimpleNamespace(mover_tracks={}, mover_velocities={})
    with patch('tools_demo.task_navigator.time.monotonic',
               side_effect=[1.0, 1.2]):
        TaskNavigator._update_mover_track(harness, 'mover', -0.50)
        TaskNavigator._update_mover_track(harness, 'mover', -0.545)
    assert math.isclose(
        TaskNavigator._mover_world_vy(harness, 'mover'), -0.225)


def test_stale_truth_never_releases_a_hold():
    """A stale position cannot release the stopped base."""
    harness = SimpleNamespace(mover_tracks={'mover': (-0.5, 1.0)})
    with patch('tools_demo.task_navigator.time.monotonic', return_value=2.0):
        assert not TaskNavigator._mover_hold_clear(harness, 'mover')


def test_unsafe_timeout_stops_and_fails_instead_of_resuming():
    """An unsafe timeout fails the leg and keeps commanding zero."""
    harness = SimpleNamespace(
        mover_hold_blocked=False, mover_hold_topic='mover',
        mover_hold_started_ns=0, mover_hold_min_ns=1,
        obstacle_reverse_wait_deadline_ns=5,
        _publish_obstacle_velocity=Mock(), _mover_hold_clear=lambda t: False,
        _finish_obstacle_reverse=Mock(), _clear_goal=Mock(),
        publish_error=Mock(), publish_state=Mock(),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10)),
    )
    TaskNavigator._mover_hold_step(harness)
    harness._finish_obstacle_reverse.assert_not_called()
    harness._publish_obstacle_velocity.assert_called_with(0.0)
    harness.publish_state.assert_called_with(
        'failed', reason='mover_crossing_unsafe_timeout')
    assert harness.mover_hold_blocked
    TaskNavigator._mover_hold_step(harness)
    harness._finish_obstacle_reverse.assert_not_called()


def test_malformed_mission_state_preserves_previous_level():
    """Malformed JSON cannot kill masking or overwrite held state."""
    harness = SimpleNamespace(mission_state='navigating_destination')
    for data in ('[]', 'null', 'true', '{"state":[]}', '{broken'):
        message = String(data=data)
        PayloadScanMask.state_callback(harness, message)
        assert harness.mission_state == 'navigating_destination'
