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
    assert .299 < math.hypot(entry.x - dock.x, entry.y - dock.y) < .751
    assert static_footprint_line_clear(
        official_map, (entry.x, entry.y), (entry.x, entry.y), .48)
    assert static_footprint_line_clear(
        official_map, (entry.x, entry.y), (dock.x, dock.y), .38)
    assert TaskNavigator._stage_delivery_terminal(node)
    assert node.pending_poses[-1] == entry


@pytest.mark.parametrize('start', [(2.45, -11.98, 1.07),
                                 (2.511, -11.979, -1.796),
                                 (2.5257, -11.872, -1.426)])
def test_fourth_c_slot_finishes_without_curving_into_prior_deliveries(official_map, start):
    cubes = [(3.1976, -12.260), (3.2701, -12.1533), (3.1875, -12.0467)]
    dock = ZonePose('C', 2.9, -11.940, 0.)
    pose = start
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
    topic = '/competition/obstacles/obstacle_1' if actor_x == 8.5 else '/competition/obstacles/obstacle_2'
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


def test_wait_anchor_small_overshoot_has_a_safe_signed_return(official_map):
    anchor = ZonePose('universal_east_wait', 9.825, .525, math.pi)
    node = N(latest_x=9.74, latest_y=.50, static_map=official_map,
             _costmap_segment_clear=lambda *a, **kw: True)
    assert TaskNavigator._spawn_wait_path_clear(node, anchor)
    speeds, _ = terminal_command((9.74, .50, math.pi), anchor, .20,
                                 heading_tolerance=.025, arrival_tolerance=.015)
    assert speeds[0] < 0.  # Reverse east while remaining west-facing.
    for point in command_sweep((9.74, .50, math.pi), speeds, .45):
        assert point[0] - 8.5 >= .76
    node.latest_x = 9.29
    assert not TaskNavigator._spawn_wait_path_clear(node, anchor)


def test_local_clearance_alone_cannot_admit_globally_occupied_c_entry(official_map):
    # The first failed repair selected x=2.35: locally free but too close
    # to the west wall for the padded .38m global footprint.
    dock = ZonePose('C', 2.9, -12.1, 0.)
    node = N(dense_fast_transit=True, active_zone='C', dock_pose=dock,
             pending_poses=[dock], static_map=official_map,
             get_logger=lambda: Mock(), publish_error=Mock(),
             publish_state=Mock(), _clear_goal=Mock(),
             _global_cost_at=lambda x, y: 100 if x < 2.5 else 0)
    assert TaskNavigator._stage_delivery_terminal(node)
    assert node.pending_poses[-1].x >= 2.5


def test_external_entry_handoff_does_not_require_final_slot_precision():
    from tools_demo.navigation_math import navigation_handoff_reached
    node = N(active_route_zone='C', active_return_zone=None,
             nav_goal_pose=ZonePose('C_delivery_entry', 2.45, -12.1, 0.),
             nav_handoff_distance=.20, pending_poses=[], route_profiles={})
    radius = TaskNavigator._route_handoff_distance(node)
    assert navigation_handoff_reached(2.511, -11.979, 2.45, -12.1, radius)


def capture_node(grid):
    return N(dense_fast_transit=True, active_zone='C',
             nav_goal_pose=ZonePose('C_delivery_entry', 2.45, -12.1, 0.),
             dock_pose=ZonePose('C', 2.9, -12.1, 0.), pending_poses=[],
             latest_x=2.5257, latest_y=-11.872, latest_yaw=-1.426,
             latest_vx=0., latest_vy=0., latest_odom_received_ns=2_000_000_000,
             terminal_cubes_stamp=2_000_000_000, terminal_cubes=[], static_map=grid,
             get_clock=lambda: N(now=lambda: N(nanoseconds=2_000_000_000)))


def test_real_stopped_c_pose_is_a_safe_precision_capture_region(official_map):
    node = capture_node(official_map)
    assert TaskNavigator._delivery_capture_allowed(node)
    node.terminal_cubes = [(3.1976, -12.26), (3.2701, -12.1533), (3.1875, -12.0467)]
    assert TaskNavigator._delivery_capture_allowed(node)


@pytest.mark.parametrize('failure', ['stale', 'cube', 'wall', 'fast', 'missing_map'])
def test_capture_region_never_substitutes_for_sweep_or_freshness_proof(official_map, failure):
    node = capture_node(official_map)
    if failure == 'stale':
        node.terminal_cubes_stamp = 0
    elif failure == 'cube':
        node.terminal_cubes = [(node.latest_x, node.latest_y)]
    elif failure == 'wall':
        node.dock_pose = ZonePose('C', 1.8, -12.1, 0.)
    elif failure == 'fast':
        node.latest_vx = .3
    else:
        node.static_map = None
    assert not TaskNavigator._delivery_capture_allowed(node)


def test_precision_capture_requests_cancel_instead_of_claiming_arrival(official_map):
    node = capture_node(official_map)
    node.goal_handle = Mock()
    node.nav_handoff_requested = node.goal_timed_out = False
    node.nav_batch_poses = []
    node.nav_handoff_radius_since_ns = None
    node.nav_handoff_max_speed = .45
    node.nav_handoff_speed_grace_ns = 500_000_000
    node._route_handoff_distance = lambda: .20
    node.get_logger = lambda: Mock()
    node.publish_state = Mock()
    node._handoff_cancel_callback = Mock()
    TaskNavigator._request_nav_handoff_if_reached(node)
    assert node.nav_handoff_requested
    node.goal_handle.cancel_goal_async.assert_called_once()
    assert node.publish_state.call_args.kwargs['reason'] == 'checked_delivery_capture'


def test_terminal_ground_cube_qos_matches_the_real_publisher():
    from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                          qos_check_compatible, QoSCompatibility)
    from tools_demo.task_navigator import ground_cube_qos
    publisher = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    # Humble may warn about system-default liveliness, which is resolved
    # only by the RMW. The reliability mismatch itself is a definite ERROR.
    assert ground_cube_qos().reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos_check_compatible(publisher, ground_cube_qos())[0] != QoSCompatibility.ERROR
    incompatible = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    assert qos_check_compatible(publisher, incompatible)[0] == QoSCompatibility.ERROR


def test_published_ground_cube_format_produces_fresh_terminal_geometry():
    import struct
    from sensor_msgs.msg import PointCloud2, PointField
    message = PointCloud2()
    message.header.frame_id = 'map'
    message.header.stamp.sec = 2
    message.height, message.width = 1, 1
    message.point_step = message.row_step = 12
    message.fields = [PointField(name=k, offset=i * 4,
                                 datatype=PointField.FLOAT32, count=1)
                      for i, k in enumerate(('x', 'y', 'z'))]
    message.data = struct.pack('<fff', 3.2, -12.1, .035)
    node = N()
    TaskNavigator._terminal_cubes_callback(node, message)
    assert node.terminal_cubes_stamp == 2_000_000_000
    assert node.terminal_cubes[0] == pytest.approx((3.2, -12.1))
    message.point_step = 4
    TaskNavigator._terminal_cubes_callback(node, message)
    assert node.terminal_cubes_stamp is None
