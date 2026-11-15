import math
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock

import numpy as np
import pytest
import yaml
from PIL import Image

from tools_demo.navigation_math import ZonePose
from tools_demo.task_navigator import TaskNavigator, static_footprint_line_clear
from tools_demo.terminal_motion import (
    delivery_entry, terminal_command, ground_cubes_clear, command_sweep,
)


@pytest.fixture(scope='module')
def official_map():
    path = Path(__file__).resolve().parents[2] / 'bot_navigation/maps/mapn3.yaml'
    metadata = yaml.safe_load(path.read_text())
    pixels = np.flipud(np.array(Image.open(path.parent / metadata['image'])))
    cells = np.where(pixels < 89.25, 100, np.where(pixels > 191.25, 0, -1))
    return N(info=N(width=cells.shape[1], height=cells.shape[0], resolution=.05,
                    origin=N(position=N(x=metadata['origin'][0], y=metadata['origin'][1]),
                             orientation=N(x=0., y=0., z=0., w=1.))),
             data=cells.ravel().tolist())


@pytest.mark.parametrize('zone,x,y,yaw', [
    ('A', -5., 3.8, math.pi), ('B', 11.3, -5., 0.), ('C', 2.9, -12.1, 0.)])
@pytest.mark.parametrize('offset', [-.16, -.053333, .053333, .16])
def test_every_delivery_slot_has_an_external_static_clear_entry(
        official_map, zone, x, y, yaw, offset):
    dock = ZonePose(zone, x, y + offset, yaw)
    node = N(dense_fast_transit=True, active_zone=zone, dock_pose=dock,
             pending_poses=[dock], static_map=official_map,
             get_logger=lambda: Mock(), publish_error=Mock(),
             publish_state=Mock(), _clear_goal=Mock())
    assert TaskNavigator._stage_delivery_terminal(node)
    entry = node.pending_poses[-1]
    assert entry.zone.endswith('_delivery_entry')
    assert .449 < math.hypot(entry.x - dock.x, entry.y - dock.y) < .751
    assert static_footprint_line_clear(
        official_map, (entry.x, entry.y), (dock.x, dock.y), .38)
    assert TaskNavigator._stage_delivery_terminal(node)
    assert node.pending_poses[-1] == entry


def test_fourth_c_slot_finishes_without_curving_into_prior_deliveries(official_map):
    cubes = [(3.1976, -12.260), (3.2701, -12.1533), (3.1875, -12.0467)]
    dock = ZonePose('C', 2.9, -11.940, 0.)
    entry = delivery_entry(dock, .45)
    pose = (entry.x, entry.y - .04, 1.07)
    for _ in range(180):
        values, phase = terminal_command(pose, dock)
        sweep = command_sweep(pose, values, .45)
        assert all(ground_cubes_clear(p, cubes) for p in sweep)
        assert static_footprint_line_clear(
            official_map, pose[:2], sweep[-1][:2], .38)
        if phase == 'arrived':
            break
        pose = command_sweep(pose, values)[2]  # 0.05 s exact integration.
    else:
        pytest.fail('Terminal entry failed to converge')
    assert math.hypot(pose[0] - dock.x, pose[1] - dock.y) <= .025
    assert abs(pose[2]) <= .045


def test_heading_and_lateral_trim_do_not_advance_toward_occupied_pad():
    dock = ZonePose('B', 11.3, -5., 0.)
    assert terminal_command((10.65, -5.1, 1.), dock)[0][:2] == (0., 0.)
    command, _ = terminal_command((10.65, -5.1, 0.), dock)
    assert command[0] == 0. and command[1] > 0.


def test_ground_cube_guard_checks_rotated_wheels_and_entire_command():
    assert ground_cubes_clear((0., 0., 0.), [(.30, 0.)])
    assert not ground_cubes_clear((0., 0., math.pi / 4), [(.30, 0.)])
    points = command_sweep((0., 0., 0.), (.4, 0., 0.), .45)
    assert not all(ground_cubes_clear(p, [(.40, 0.)]) for p in points)
    assert not ground_cubes_clear((float('nan'), 0., 0.), [])


@pytest.mark.parametrize('actor_x', [5., 8.5])
@pytest.mark.parametrize('direction', [-1., 1.])
def test_admission_keeps_original_direction_after_braking_across_actor_x(
        actor_x, direction):
    topic = '/competition/obstacles/obstacle_2'
    now = 2_000_000_000
    node = N(
        dense_fast_transit=False, mover_hold_topic=topic,
        mover_hold_direction=direction,
        latest_x=actor_x + direction * .28, latest_y=2.26,
        latest_yaw=0. if direction > 0 else math.pi,
        latest_vx=0., latest_vy=0., latest_odom_received_ns=now,
        mover_hold_stationary_since_ns=now - 600_000_000,
        mover_hold_started_ns=0, mover_hold_min_ns=1_500_000_000,
        mover_hold_blocked=False,
        obstacle_positions={topic: (actor_x, 2.)},
        obstacle_reverse_wait_deadline_ns=now + 10_000_000_000,
        mover_admitted=set(), nav_goal_pose=None,
        _mover_hold_clear=lambda t: True,
        _publish_obstacle_velocity=Mock(), _publish_direct_velocity=Mock(),
        get_clock=lambda: N(now=lambda: N(nanoseconds=now)),
        get_logger=lambda: Mock())
    TaskNavigator._mover_hold_step(node)
    assert node.obstacle_reverse_state == 'crossing'
    assert node.mover_crossing_exit_x == actor_x + direction * .7
    assert math.cos(node.mover_crossing_yaw) * direction > .99
    node._publish_direct_velocity.assert_not_called()


def test_stopped_h1_state_cannot_be_sent_east_through_wall(official_map):
    topic = 'mover'
    node = N(dense_fast_transit=True, latest_x=5.305, latest_y=2.167,
             latest_yaw=0., obstacle_positions={topic: (5., 1.935)},
             mover_hold_topic=topic, mover_hold_direction=1., static_map=official_map)
    assert not TaskNavigator._corridor_direct_lane_clear(node, topic)
    # The measured post-braking pose must finish WEST, without reversing.
    node.latest_x, node.latest_y, node.mover_hold_direction = 4.72, 2.26, -1.
    assert TaskNavigator._corridor_direct_lane_clear(node, topic)


@pytest.mark.parametrize('missing', ['odom', 'cloud'])
def test_missing_fresh_terminal_evidence_holds_complete_zero(official_map, missing):
    now = 2_000_000_000
    node = N(latest_x=2.45, latest_y=-11.94, latest_yaw=0.,
             latest_odom_received_ns=now if missing != 'odom' else None,
             terminal_cubes_stamp=now if missing != 'cloud' else None,
             terminal_cubes=[], static_map=official_map, active_zone='C',
             dock_pose=ZonePose('C', 2.9, -11.94, 0.),
             _publish_direct_velocity=Mock(), get_logger=lambda: Mock())
    TaskNavigator._delivery_terminal_step(node, now)
    cmd = node._publish_direct_velocity.call_args.args[0]
    assert cmd.linear.x == cmd.linear.y == cmd.angular.z == 0.
