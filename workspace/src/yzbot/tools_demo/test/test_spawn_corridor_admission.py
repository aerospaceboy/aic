"""Shared admission, protected continuous handoff, and no duplicate turn."""

import math
import json
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from action_msgs.msg import GoalStatus
import pytest
import yaml

from tools_demo.corridor_admission import spawn_corridor_admission, wait_anchor_velocity
from tools_demo.navigation_math import ZonePose
from tools_demo.safety_limiter import SafetyLimiter
from tools_demo.safety_math import compute_scale
from tools_demo.task_navigator import TaskNavigator


def test_precision_wait_tracking_keeps_collision_and_arrival_proofs():
    config = Path(__file__).resolve().parents[2] / 'bot_navigation/param/originbot_nav2.yaml'
    control = yaml.safe_load(config.read_text())['controller_server']['ros__parameters']
    ordinary, precise = control['CentralTransit'], control['CorridorApproach']
    tuning = {'desired_linear_vel', 'lookahead_dist', 'min_lookahead_dist',
              'max_lookahead_dist', 'lookahead_time', 'min_approach_linear_velocity'}
    assert {k: v for k, v in precise.items() if k not in tuning} == {
        k: v for k, v in ordinary.items() if k not in tuning}
    assert all(0 < precise[k] < ordinary[k] for k in tuning)
    assert control['corridor_wait_goal_checker']['xy_goal_tolerance'] == .003
    assert precise['use_collision_detection'] is True
    assert precise['max_allowed_time_to_collision_up_to_carrot'] == .65


def test_wait_anchor_matches_map_cells_without_changing_arrival_radius():
    from tools_demo.navigation_math import (
        UNIVERSAL_EAST_WAIT_X, UNIVERSAL_EAST_WAIT_ANCHOR_Y,
        UNIVERSAL_EAST_WAIT_Y,
    )
    root = Path(__file__).resolve().parents[2] / 'bot_navigation/maps'
    for path in root.glob('*.yaml'):
        grid = yaml.safe_load(path.read_text())
        for value, origin in zip((UNIVERSAL_EAST_WAIT_X, UNIVERSAL_EAST_WAIT_ANCHOR_Y),
                                 grid['origin'][:2]):
            index = (value - origin) / grid['resolution']
            assert abs(index - math.floor(index) - .5) < 1e-9
    assert abs(UNIVERSAL_EAST_WAIT_ANCHOR_Y - UNIVERSAL_EAST_WAIT_Y) < .07
    node = SimpleNamespace(
        nav_goal_pose=ZonePose('universal_east_wait', UNIVERSAL_EAST_WAIT_X,
                               UNIVERSAL_EAST_WAIT_ANCHOR_Y, math.pi),
        latest_x=UNIVERSAL_EAST_WAIT_X + .05, latest_y=UNIVERSAL_EAST_WAIT_ANCHOR_Y)
    assert not TaskNavigator._central_anchor_arrived(node)


def clock():
    return SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=100_000_000_000))


def participants(x, ox, oy, velocity):
    model = ('competition_moving_obstacle_1' if ox == 8.5
             else 'competition_moving_obstacle_2')
    topic = '/competition/obstacles/obstacle_' + ('1' if ox == 8.5 else '2')
    nav = SimpleNamespace(
        mover_tracks={topic: (oy, time.monotonic())},
        _mover_base_frame=lambda _: (0.,) * 7,
        latest_x=x, latest_y=.55, latest_yaw=math.pi,
        obstacle_positions={topic: (ox, oy)},
        obstacle_source_stamps={topic: 100.},
        get_clock=clock, _mover_world_vy=lambda _: velocity)
    guard = SimpleNamespace(
        robot=(x, .55, math.pi), robot_stamp_ns=100_000_000_000,
        robot_speed=0., obstacle_timeout=.75,
        obstacles={model: (ox, oy, 100_000_000_000)},
        obstacle_source_stamps={model: 100.},
        obstacle_previous_source_stamps={model: 99.9},
        obstacle_previous_y={model: oy - velocity * .1},
        obstacle_motion_samples={model: (oy, 100.)},
        obstacle_motion_velocities={model: (velocity, 100.)},
        get_clock=clock, get_logger=lambda: Mock(),
        east_gate_open=False, west_lane_open=False)
    return nav, guard, topic, model


@pytest.mark.parametrize('x,ox,rolling', [
    (9.8, 8.5, False), (6.65, 5., False), (7.1, 5., True),
])
@pytest.mark.parametrize('oy', [-.75, -.60, -.45, -.30, -.10, .05])
@pytest.mark.parametrize('velocity', [-.25, .25])
def test_navigator_and_guard_make_the_same_window_decision(
    x, ox, rolling, oy, velocity,
):
    nav, guard, topic, model = participants(x, ox, oy, velocity)
    expected = spawn_corridor_admission(
        (x, .55, math.pi), (ox, oy), velocity, rolling)
    assert TaskNavigator._mover_hold_clear(nav, topic, rolling) == expected
    decision = SafetyLimiter._spawn_admission_clear(guard, model, rolling)
    assert decision == expected


def test_stale_truth_cannot_release_either_owner():
    nav, guard, topic, model = participants(9.8, 8.5, -.6, -.25)
    nav.obstacle_source_stamps[topic] = 98.
    guard.obstacle_source_stamps[model] = 98.
    assert not TaskNavigator._mover_hold_clear(nav, topic)
    assert not SafetyLimiter._spawn_admission_clear(guard, model)


def test_duplicate_truth_packets_do_not_destroy_a_proven_direction():
    _, guard, _, model = participants(9.8, 8.5, -.5, -.25)
    assert SafetyLimiter._spawn_admission_clear(guard, model)
    message = String(data=json.dumps({
        'success': True, 'model': model, 'x': 8.5, 'y': -.5,
        'stamp_sec': 100.}))
    for _ in range(3):
        SafetyLimiter.obstacle_callback(guard, message)
    assert SafetyLimiter._spawn_admission_clear(guard, model)
    assert guard.obstacle_motion_velocities[model] == (-.25, 100.)


def test_stationary_truth_cannot_refresh_old_velocity_indefinitely():
    _, guard, _, model = participants(9.8, 8.5, -.6, -.25)
    guard.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=101_000_000_000))
    guard.robot_stamp_ns = 101_000_000_000
    message = String(data=json.dumps({
        'success': True, 'model': model, 'x': 8.5, 'y': -.6,
        'stamp_sec': 101.}))
    SafetyLimiter.obstacle_callback(guard, message)
    assert model not in guard.obstacle_motion_velocities
    assert not SafetyLimiter._spawn_admission_clear(guard, model)


@pytest.mark.parametrize('history,release', [
    ([(99_990_000_000, (9.8, .55, math.pi))], True),
    ([], False), ([(99_000_000_000, (9.8, .55, math.pi))], False),
])
def test_clock_lag_uses_confirmed_odometry_without_admitting_future_data(history, release):
    _, guard, _, model = participants(9.8, 8.5, -.5, -.25)
    guard.robot_stamp_ns = 100_012_000_000
    guard.odom_history = history
    assert SafetyLimiter._spawn_admission_clear(guard, model) is release


def capture_node():
    grid = OccupancyGrid()
    grid.info.width, grid.info.height, grid.info.resolution = 400, 200, .05
    grid.info.origin.position.y = -5.
    grid.info.origin.orientation.w = 1.
    grid.data = [0] * 80000
    return SimpleNamespace(
        nav_goal_pose=ZonePose('universal_east_wait', 9.825, .525, math.pi),
        pending_poses=[ZonePose('universal_mover_1_clear', 6.65, .55, math.pi)],
        use_legacy_layout_routes=False, goal_handle=Mock(),
        latest_x=11., latest_y=.45, latest_yaw=math.pi,
        latest_vx=.75, latest_vy=0., latest_odom_received_ns=100_000_000_000,
        static_map=grid, _costmap_segment_clear=Mock(return_value=True),
        get_clock=clock, get_logger=lambda: Mock(),
        _handoff_cancel_callback=Mock(), _continue_staged_navigation=Mock(),
        _publish_obstacle_velocity=Mock(), _publish_direct_velocity=Mock(),
        _send_pose=Mock())


def test_wait_capture_cancels_without_issuing_motion_before_old_result():
    node = capture_node()
    TaskNavigator._request_spawn_wait_capture(node)
    assert node.spawn_wait_capture_goal == node.nav_goal_pose
    node.goal_handle.cancel_goal_async.assert_called_once()
    node._publish_direct_velocity.assert_not_called()
    node._continue_staged_navigation.assert_not_called()
    TaskNavigator._request_spawn_wait_capture(node)
    node.goal_handle.cancel_goal_async.assert_called_once()


@pytest.mark.parametrize('key,value', [
    ('latest_odom_received_ns', 99_000_000_000), ('latest_vx', 1.1),
    ('latest_yaw', 2.), ('static_map', None), ('latest_y', -.1),
    ('use_legacy_layout_routes', True), ('pending_poses', []),
])
def test_unsafe_wait_capture_keeps_nav2_and_the_original_anchor(key, value):
    node = capture_node()
    setattr(node, key, value)
    TaskNavigator._request_spawn_wait_capture(node)
    node.goal_handle.cancel_goal_async.assert_not_called()


def start_capture(node):
    node.spawn_wait_capture_goal = node.nav_goal_pose
    node.nav_handoff_requested = True
    TaskNavigator._complete_nav_handoff(node)
    assert node.spawn_wait_capture_active
    assert node.goal_handle is None
    node._continue_staged_navigation.assert_not_called()


def test_wait_capture_still_requires_the_original_physical_precision_and_rest():
    node = capture_node()
    start_capture(node)
    node.latest_x, node.latest_y = 9.875, .525
    TaskNavigator._spawn_wait_capture_step(node)
    node._continue_staged_navigation.assert_not_called()
    node.latest_x = 9.845
    TaskNavigator._spawn_wait_capture_step(node)
    node._continue_staged_navigation.assert_not_called()  # Still moving.
    node.latest_vx = 0.
    TaskNavigator._spawn_wait_capture_step(node)
    node._continue_staged_navigation.assert_called_once()


def test_transient_map_change_holds_the_same_owner_without_restarting_pursuit():
    node = capture_node()
    start_capture(node)
    node.static_map = None
    TaskNavigator._spawn_wait_capture_step(node)
    node._publish_obstacle_velocity.assert_called_once_with(0.)
    node._send_pose.assert_not_called()
    assert node.spawn_wait_capture_active
    node._continue_staged_navigation.assert_not_called()


def test_persistent_block_timeout_falls_back_to_the_same_anchor():
    node = capture_node()
    start_capture(node)
    node.spawn_wait_capture_deadline_ns = 99_000_000_000
    TaskNavigator._spawn_wait_capture_step(node)
    node._publish_obstacle_velocity.assert_called_once_with(0.)
    node._send_pose.assert_called_once_with(node.nav_goal_pose)
    assert node.spawn_wait_capture_disabled


def test_wall_tip_rest_starts_one_checked_lane_owner_without_a_nav_goal():
    node = capture_node()
    node.goal_handle = None
    node.latest_x, node.latest_y = 12.6, .45
    node.latest_vx = 0.
    node.publish_state = Mock()
    assert TaskNavigator._start_spawn_wait_from_rest(node, node.nav_goal_pose)
    assert node.spawn_wait_capture_active
    assert node.spawn_wait_capture_goal == node.nav_goal_pose
    node._send_pose.assert_not_called()
    node._publish_direct_velocity.assert_not_called()  # The checked timer owns motion.


@pytest.mark.parametrize('key,value', [
    ('latest_vx', .2), ('latest_y', -.1), ('latest_odom_received_ns', 99_000_000_000),
    ('static_map', None), ('latest_yaw', 1.8),
])
def test_starting_lane_from_rest_requires_every_safety_proof(key, value):
    node = capture_node()
    node.goal_handle = None
    node.latest_x, node.latest_y, node.latest_vx = 12.6, .45, 0.
    setattr(node, key, value)
    assert not TaskNavigator._start_spawn_wait_from_rest(node, node.nav_goal_pose)


def test_wait_capture_needs_a_terminal_old_action_result():
    node = capture_node()
    node.spawn_wait_capture_goal = node.nav_goal_pose
    node.nav_handoff_requested = True
    node.goal_timed_out = False
    node._complete_nav_handoff = Mock()
    future = Mock()
    future.result.return_value.status = GoalStatus.STATUS_EXECUTING
    TaskNavigator.result_callback(node, future)
    node._complete_nav_handoff.assert_not_called()
    future.result.return_value.status = GoalStatus.STATUS_CANCELED
    TaskNavigator.result_callback(node, future)
    node._complete_nav_handoff.assert_called_once()


def test_user_cancel_stops_capture_without_advancing():
    node = capture_node()
    node.spawn_wait_capture_active = True
    node.publish_state, node._clear_goal = Mock(), Mock()
    TaskNavigator.cancel_callback(node, SimpleNamespace(data=True))
    node._publish_obstacle_velocity.assert_called_once_with(0.)
    node.publish_state.assert_called_once_with('canceled')
    node._clear_goal.assert_called_once()
    node._continue_staged_navigation.assert_not_called()


def test_wait_capture_can_use_prior_confirmed_pose_but_never_future_only_data():
    node = capture_node()
    values = (node.latest_x, node.latest_y, node.latest_yaw, node.latest_vx, node.latest_vy)
    node.latest_odom_received_ns = 100_012_000_000
    assert TaskNavigator._spawn_wait_observation(node) is None
    node.spawn_odom_history = [(99_990_000_000, values)]
    assert TaskNavigator._spawn_wait_observation(node) == values


@pytest.mark.parametrize('lateral', [-.05, .0, .15, .25, .30])
@pytest.mark.parametrize('start_x', [11.24, 12.6])
@pytest.mark.parametrize('heading', [-.4, -.25, .0, .25, .4])
@pytest.mark.parametrize('latency', [0., .10])
def test_wait_lane_merge_finishes_aligned_without_a_parked_turn(
    lateral, start_x, heading, latency,
):
    x, y, yaw = start_x, .525 - lateral, math.pi + heading
    vx, wz = 0., 0.
    delayed = []
    dt = .02
    for step in range(800):
        if math.hypot(x - 9.825, y - .525) <= .03 and abs(
                math.remainder(yaw - math.pi, 2 * math.pi)) <= .04:
            break
        v, w = wait_anchor_velocity((x, y, yaw), (9.825, .525))
        assert 0. < v <= .35  # Turning correction is carried while moving.
        assert abs(w) <= .60
        delayed.append((v, w))
        index = max(0, len(delayed) - 1 - round(latency / dt))
        v, w = delayed[index]
        vx += max(-5. * dt, min(5. * dt, v - vx))
        wz += max(-5. * dt, min(5. * dt, w - wz))
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        yaw += wz * dt
        assert 9.75 <= x <= 12.85 and .20 <= y <= .60
    else:
        pytest.fail('Did not meet the original position and lane-heading proofs')


def test_stopped_guard_prearms_without_creating_a_motion_command():
    _, guard, _, _ = participants(9.8, 8.5, -.5, -.25)
    guard.base_hold = False
    guard.publisher = Mock()
    guard.east_gate_blocked = lambda _: False
    guard.west_lane_blocked = lambda _: False
    guard.scale_for_obstacles = lambda _: 1.
    assert not guard.east_gate_open
    SafetyLimiter.cmd_vel_callback(guard, Twist())
    assert guard.east_gate_open
    sent = guard.publisher.publish.call_args.args[0]
    assert sent.linear.x == sent.linear.y == sent.angular.z == 0.


@pytest.mark.parametrize('source,release', [(100., True), (98., False)])
def test_idle_timer_proves_the_window_without_any_velocity_message(source, release):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from unittest.mock import patch

    context = Context()
    # Use a private context so this timer test does not disturb other nodes.
    rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    node_init = Node.__init__
    with patch('tools_demo.safety_limiter.Node.__init__',
               lambda self, name: node_init(self, name, context=context)):
        node = SafetyLimiter()
    try:
        _, guard, _, model = participants(9.8, 8.5, -.5, -.25)
        for name, value in vars(guard).items():
            if name not in {'get_logger', 'get_clock'}:
                setattr(node, name, value)
        node.get_clock = clock
        node.obstacle_source_stamps[model] = source
        node.publisher = Mock()
        deadline = time.monotonic() + .25
        while time.monotonic() < deadline and not node.east_gate_open:
            rclpy.spin_once(node, executor=executor, timeout_sec=.06)
        assert node.east_gate_open is release
        node.publisher.publish.assert_not_called()
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown(context=context)


@pytest.mark.parametrize('oy,known,release', [
    (-.3, True, True), (.05, True, False), (-.3, False, False),
])
def test_moving_guard_proves_the_second_window_before_its_gate(oy, known, release):
    _, guard, _, _ = participants(7.1, 5., oy, -.25)
    guard.robot_speed = .9
    if not known:
        guard.obstacle_motion_velocities.clear()
    SafetyLimiter._prewarm_spawn_gates(guard, rolling=True)
    assert guard.west_lane_open is release


def rolling_node(clear=True):
    first = ZonePose('universal_mover_1_clear', 6.65, .55, math.pi)
    second = ZonePose('universal_mover_2_clear', 3.8, .55, math.pi)
    return SimpleNamespace(
        nav_goal_pose=first, pending_poses=[second], goal_handle=None,
        latest_x=7.1, latest_y=.55, latest_yaw=math.pi,
        latest_vx=.9, latest_vy=0., latest_odom_received_ns=100_000_000_000,
        obstacle_source_stamps={
            '/competition/obstacles/obstacle_1': 100.,
            '/competition/obstacles/obstacle_2': 100.},
        obstacle_positions={'/competition/obstacles/obstacle_1': (8.5, -.6),
                            '/competition/obstacles/obstacle_2': (5., -.3)},
        get_clock=clock, get_logger=lambda: Mock(),
        _mover_hold_clear=Mock(return_value=clear), mover_admitted=set(),
        stage_index=4, mover_crossing_exit_x=6.65,
        publish_state=Mock())


def test_second_proven_window_extends_the_same_straight_crossing():
    node = rolling_node()
    assert TaskNavigator._try_rolling_corridor_handoff(node)
    assert node.nav_goal_pose.zone == 'universal_mover_2_clear'
    assert node.mover_crossing_exit_x == 3.8
    assert not node.pending_poses
    node._mover_hold_clear.assert_called_once_with(
        '/competition/obstacles/obstacle_2', rolling=True)


@pytest.mark.parametrize('key,value', [
    ('latest_vx', .2), ('latest_x', 8.0), ('latest_y', .1),
    ('latest_odom_received_ns', 99_000_000_000),
    ('pending_poses', [ZonePose('mandatory_wall', 6., .55, math.pi)]),
    ('obstacle_source_stamps', {'/competition/obstacles/obstacle_1': 100.}),
])
def test_rolling_handoff_cannot_skip_safety_or_advance_on_unknown_motion(
    key, value,
):
    node = rolling_node()
    setattr(node, key, value)
    assert not TaskNavigator._try_rolling_corridor_handoff(node)
    assert node.nav_goal_pose.zone == 'universal_mover_1_clear'


def test_unsafe_second_window_retains_the_first_safe_stop():
    node = rolling_node(False)
    assert not TaskNavigator._try_rolling_corridor_handoff(node)
    assert node.mover_crossing_exit_x == 6.65
    assert node.pending_poses[0].zone == 'universal_mover_2_clear'


def test_the_admission_owner_replaces_duplicate_generic_pre_alignment():
    target = ZonePose('universal_mover_1_clear', 6.65, .55, math.pi)
    node = SimpleNamespace(
        latest_x=9.8, latest_y=.55, latest_yaw=2.46,
        dock_pose=ZonePose('entry', 0., 0., math.pi),
        pending_poses=[target],
        get_logger=lambda: Mock(), _send_next_goal=Mock(),
        get_clock=clock, publish_state=Mock())
    TaskNavigator.start_pre_navigation_alignment(node)
    assert not node.pre_navigation_alignment
    node._send_next_goal.assert_called_once()


@pytest.mark.parametrize('rolling,x,ox', [(False, 9.8, 8.5), (True, 7.1, 5.)])
def test_admitted_phases_preserve_physical_margin_at_intermediate_speeds(
    rolling, x, ox,
):
    admitted = 0
    for oy in (-.7, -.6, -.45, -.3, -.1):
        velocity = -.25
        if not spawn_corridor_admission((x, .55, math.pi), (ox, oy),
                                        velocity, rolling):
            continue
        admitted += 1
        for speed in (.56, .65, .70, .8, .85, .95, 1.1):
            for delay in ((0., .2, .4) if rolling else (0., .4, .8)):
                duration = (x - ox + .6) / speed + delay
                for i in range(math.ceil(duration / .01) + 1):
                    t = i * .01
                    phase = (oy + .8 + velocity * t) % 2.8
                    actor_y = -.8 + (phase if phase <= 1.4 else 2.8 - phase)
                    robot_x = x - speed * max(0., t - delay)
                    assert math.hypot(robot_x - ox, .55 - actor_y) >= .56
    assert admitted > 0


@pytest.mark.parametrize('x,ox,rolling', [
    (9.8, 8.5, False), (6.65, 5., False), (7.1, 5., True),
])
def test_admitted_windows_remain_safe_with_the_live_guard_slowdown(
    x, ox, rolling,
):
    admitted = 0
    for phase_index in range(37):
        oy = -.8 + 1.4 * phase_index / 36
        for velocity in (-.25, .25):
            if not spawn_corridor_admission((x, .55, math.pi), (ox, oy),
                                            velocity, rolling):
                continue
            admitted += 1
            for delay in ((0., .2, .4) if rolling else (0., .4, .8)):
                robot_x = x
                cleared = False
                for index in range(241):
                    t = index * .025
                    phase = (oy + .8 + velocity * t) % 2.8
                    ay = -.8 + (phase if phase <= 1.4 else 2.8 - phase)
                    assert math.hypot(robot_x - ox, .55 - ay) >= .56
                    if robot_x <= ox - .6:
                        cleared = True
                        break
                    if t >= delay:
                        actor = ('competition_moving_obstacle_1' if ox == 8.5
                                 else 'competition_moving_obstacle_2')
                        actors = {actor: (ox, ay)}
                        scale = compute_scale(
                            (robot_x, .55, math.pi), actors,
                            .75, 1.60, math.radians(65), 1.)
                        robot_x -= 1.1 * scale * .025
                assert cleared
    assert admitted > 0
