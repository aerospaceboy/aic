"""Reject reversed IK branches and keep the pickup approach in one plane."""

import importlib.util
import ast
import math
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


SPEC = importlib.util.spec_from_file_location(
    "pick_approach_module", Path(__file__).parents[1] / "pick_place_moveit.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
APPROACH = MODULE.MoveItPickPlace.move_pick_approach


def arm_for_red3():
    yaw = math.atan2(-.01558, .36012)
    solution = dict(zip(MODULE.ARM_JOINT_NAMES,
                        (yaw, .778, 1.151, 0., -1.213, yaw)))
    positions = dict.fromkeys(MODULE.ARM_JOINT_NAMES, 0.)
    arm = SimpleNamespace(
        cube_name='red_cube_3',
        pick_safe_pose=(.36012, -.01558, .1216),
        tool_orientation=(0., 1., 0., 0.), positions=positions,
        tool_pose=MODULE.pose_from_values((0., 0., .66), (0., 0., 0., 1.)),
        get_logger=lambda: Mock(), compute_ik=Mock(return_value=solution))
    arm.arm_positions = lambda: dict(arm.positions)
    arm.pick_tool_pose = lambda: arm.tool_pose
    arm.recover_pick_plane_start = MethodType(
        MODULE.MoveItPickPlace.recover_pick_plane_start, arm)

    def lift(xyz, label, **kwargs):
        arm.tool_pose.position.z = xyz[2]
        return True

    arm.cartesian_move = Mock(side_effect=lift)

    def move(target, label, **kwargs):
        arm.positions.update(target)
        if label == 'pick_plane_recovery_upright':
            arm.tool_pose.position.z = .66
        return True

    arm.move_to_joint_target = Mock(side_effect=move)
    return arm, solution, yaw


def test_red3_reversed_177_degree_solution_is_never_executed():
    arm, solution, yaw = arm_for_red3()
    reversed_solution = dict(solution)
    reversed_solution.update(joint1=yaw + math.pi, joint6=yaw + math.pi)
    arm.compute_ik = Mock(side_effect=[reversed_solution, solution])
    assert APPROACH(arm)
    calls = arm.move_to_joint_target.call_args_list
    assert [c.args[1] for c in calls] == [
        'pick_plane_align_high', 'pick_approach']
    assert calls[-1].args[0] == solution
    constraints = calls[-1].kwargs['path_constraints'].joint_constraints
    assert {c.joint_name for c in constraints} == {
        'joint1', 'joint4', 'joint6'}
    assert all(c.tolerance_above == .020 for c in constraints)
    assert calls[0].args[0]['joint2'] == 0.
    assert calls[0].args[0]['joint1'] == yaw
    high = calls[0].kwargs['path_constraints'].joint_constraints
    pitches = [j for j in high
               if j.joint_name in {'joint2', 'joint3', 'joint5'}]
    assert all(j.position == 0. and j.tolerance_above == .020 for j in pitches)
    assert all(c.kwargs['seed_positions']['joint1'] == yaw
               for c in arm.compute_ik.call_args_list)


def test_no_safe_ik_branch_cannot_trigger_any_motion():
    arm, solution, yaw = arm_for_red3()
    solution.update(joint1=yaw + math.pi, joint6=yaw + math.pi)
    assert not APPROACH(arm)
    arm.move_to_joint_target.assert_not_called()


@pytest.mark.parametrize('bad', [math.nan, math.inf, 3.14])
def test_invalid_or_near_limit_ik_is_rejected(bad):
    arm, solution, _ = arm_for_red3()
    solution['joint3'] = bad
    assert not APPROACH(arm)
    arm.move_to_joint_target.assert_not_called()


def test_already_aligned_start_avoids_redundant_alignment():
    arm, _, yaw = arm_for_red3()
    arm.positions.update(MODULE.pick_plane_axes(yaw))
    assert APPROACH(arm)
    assert arm.move_to_joint_target.call_count == 1
    assert arm.move_to_joint_target.call_args.args[1] == 'pick_approach'


@pytest.mark.parametrize('height,shoulder', [(.18, 0.), (.66, 2.8)])
def test_low_or_reversed_scan_start_recovers_before_plane_approach(height, shoulder):
    arm, _, _ = arm_for_red3()
    arm.positions['joint1'] = shoulder
    arm.tool_pose.position.z = height
    assert APPROACH(arm)
    labels = [call.args[1] for call in arm.move_to_joint_target.call_args_list]
    assert labels[0] == 'pick_plane_recovery_upright'
    assert labels[-1] == 'pick_approach'
    if height < MODULE.PICK_PLANE_ALIGN_HEIGHT:
        lift = arm.cartesian_move.call_args
        assert lift.args[0][:2] == (0., 0.)
        assert lift.args[0][2] > MODULE.PICK_PLANE_ALIGN_HEIGHT
        assert lift.kwargs['orientation'] == (0., 0., 0., 1.)
        assert lift.kwargs['avoid_collisions'] is True
    else:
        arm.cartesian_move.assert_not_called()
    constraints = arm.move_to_joint_target.call_args_list[0].kwargs['path_constraints']
    assert len(constraints.joint_constraints) == 6
    assert not constraints.position_constraints
    for joint in constraints.joint_constraints:
        assert joint.position - joint.tolerance_below <= 0.
        assert joint.position + joint.tolerance_above >= 0.


@pytest.mark.parametrize('stage', ['lift', 'reset', 'unverified'])
def test_failed_or_unverified_recovery_cannot_start_the_pick(stage):
    arm, _, _ = arm_for_red3()
    arm.positions['joint1'] = 2.8
    if stage == 'lift':
        arm.tool_pose.position.z = .18
        arm.cartesian_move = Mock(return_value=False)
    else:
        arm.move_to_joint_target = Mock(return_value=stage == 'unverified')
    assert not APPROACH(arm)
    arm.compute_ik.assert_not_called()
    assert all(call.args[1] != 'pick_approach'
               for call in arm.move_to_joint_target.call_args_list)


def test_lift_reporting_success_below_the_floor_cannot_start_joint_recovery():
    arm, _, _ = arm_for_red3()
    arm.tool_pose.position.z = .18
    arm.cartesian_move = Mock(return_value=True)
    assert not APPROACH(arm)
    arm.move_to_joint_target.assert_not_called()


@pytest.mark.parametrize('scanned', [True, False])
def test_scan_uses_the_forward_grasp_branch_before_recognition(scanned):
    arm = SimpleNamespace(
        wait_until_ready=lambda: True, update_stand_collision_objects=lambda *a: True,
        publish_state=Mock(), enforce_arm_joint_limits=lambda: True,
        move_pick_approach=Mock(return_value=scanned), pick_safe_pose=(.3, -.03, .32))
    assert MODULE.MoveItPickPlace.run_scan_only(arm) is scanned
    arm.move_pick_approach.assert_called_once_with(label='scan_pose')


def visual_arm():
    arm, _, _ = arm_for_red3()
    arm.operation, arm.target_source = 'pick', 'perception'
    arm.tool_pose = MODULE.pose_from_values((.30, -.03, .32), (0., 1., 0., 0.))
    arm.positions.update(dict(zip(MODULE.ARM_JOINT_NAMES,
                                  (-.10, .20, 1.15, 0., -1.79, -.10))))
    arm.pick_pose = (.36012, -.01558, .0516)
    arm.pick_lift_duration, arm.pick_descend_duration, arm.transport_x = 1.1, .6, .28
    arm.move_visual_pick_approach = MethodType(
        MODULE.MoveItPickPlace.move_visual_pick_approach, arm)
    arm.descend_pick_vertically = MethodType(MODULE.MoveItPickPlace.descend_pick_vertically, arm)

    def cartesian(xyz, label, **kwargs):
        arm.tool_pose = MODULE.pose_from_values(xyz, arm.tool_orientation)
        return True

    arm.cartesian_move = Mock(side_effect=cartesian)
    return arm


def test_recognized_target_aligns_high_then_descends_without_homing_or_rising():
    arm = visual_arm()
    assert APPROACH(arm)
    labels = [c.args[1] for c in arm.move_to_joint_target.call_args_list]
    assert labels == ['pick_visual_align_plane_high']
    assert all(c.args[0]['joint3'] == 1.15 for c in arm.move_to_joint_target.call_args_list)
    paths = arm.cartesian_move.call_args_list
    assert [c.args[1] for c in paths] == [
        'pick_visual_align_xy_high', 'pick_visual_descend_to_safe']
    assert all(c.args[0][2] <= .32 for c in paths)
    assert paths[0].args[0][:2] == paths[1].args[0][:2]
    assert paths[1].kwargs['avoid_collisions'] is True
    assert arm.pick_pose[2] == .0516
    arm.compute_ik.assert_not_called()


@pytest.mark.parametrize('height,shoulder', [(.18, -.10), (.32, 2.8)])
def test_unready_visual_pose_cannot_home_after_recognition(height, shoulder):
    arm = visual_arm()
    arm.tool_pose.position.z = height
    arm.positions['joint1'] = shoulder
    assert not APPROACH(arm)
    arm.move_to_joint_target.assert_not_called()
    arm.cartesian_move.assert_not_called()


def test_visual_descent_keeps_the_original_xy_gate():
    arm = visual_arm()
    move = arm.cartesian_move.side_effect

    def drift(xyz, label, **kwargs):
        move(xyz, label, **kwargs)
        arm.tool_pose.position.x += .01
        return True

    arm.cartesian_move.side_effect = drift
    assert not APPROACH(arm)
    assert all(c.args[1] != 'pick_visual_descend_to_safe'
               for c in arm.cartesian_move.call_args_list)


def test_uncertified_folded_configuration_cannot_start_a_joint_reset():
    arm, _, _ = arm_for_red3()
    arm.positions.update(joint2=1.2, joint3=1.5)
    assert MODULE.pick_recovery_height_floor(arm.positions) < .25
    assert not MODULE.MoveItPickPlace.recover_pick_plane_start(arm, arm.tool_pose, 0.)
    arm.move_to_joint_target.assert_not_called()


@pytest.mark.parametrize('sign', [-1., 1.])
def test_entire_recovery_joint_box_keeps_the_original_physical_height_floor(sign):
    import random
    import numpy as np

    # Independently compose the shipped URDF chain, including every wrist axis.
    def rotation(axis, angle):
        c, s = math.cos(angle), math.sin(angle)
        return (np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) if axis == 'y'
                else np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]))

    positions = dict(zip(MODULE.ARM_JOINT_NAMES,
                         (sign * .12, sign * .35, sign * .72, .12, 2.1, -.55)))
    constraints = MODULE.pick_recovery_constraints(positions)
    assert constraints is not None
    floor = MODULE.pick_recovery_height_floor(positions)
    assert floor >= MODULE.PICK_PLANE_ALIGN_HEIGHT
    rng = random.Random(42)
    for _ in range(500):
        q = [rng.uniform(j.position - j.tolerance_below,
                         j.position + j.tolerance_above)
             for j in constraints.joint_constraints]
        r = rotation('z', q[0]) @ rotation('y', q[1])
        xyz = np.array([0., 0., .18]) + r @ [0., 0., .14]
        r = r @ rotation('y', q[2])
        xyz += r @ [0., 0., .22]
        r = r @ rotation('z', q[3])
        xyz += r @ [0., 0., .06]
        r = r @ rotation('y', -q[4])
        xyz += r @ [0., 0., .06]
        assert xyz[2] >= floor


def test_recovery_bound_matches_the_shipped_arm_dimensions():
    import re

    urdf = Path(__file__).parents[2] / 'mybot_description/urdf'
    arm = (urdf / 'arm.xacro').read_text()
    values = dict(re.findall(r'name="(\w+)" value="([0-9.]+)"', arm))
    assert float(values['link2_len']) == .14
    assert float(values['link3_len']) + float(values['link4_len']) == .28
    assert float(values['link5_len']) == .06
    base = (urdf / 'large_base.xacro').read_text()
    height = float(re.search(r'name="base_height" value="([0-9.]+)"', base)[1])
    pedestal = height / 2 + .01 + float(values['base_h']) / 2 + float(values['link1_len'])
    assert pedestal == pytest.approx(.18)


def test_timed_out_recovery_drains_its_exact_action_before_returning():
    future = object()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: future)
    arm = SimpleNamespace(
        velocity_scaling=1., acceleration_scaling=1., latest_joint_state=None,
        move_group_client=Mock(), publish_state=Mock(), get_logger=lambda: Mock(),
        _wait_for_future=Mock(side_effect=[handle, None]),
        _cancel_and_drain_execution=Mock(return_value=True), wait_for_joint_state=Mock())
    assert not MODULE.MoveItPickPlace.move_to_joint_target(
        arm, dict.fromkeys(MODULE.ARM_JOINT_NAMES, 0.),
        'pick_plane_recovery_upright', result_timeout_sec=10.)
    arm._cancel_and_drain_execution.assert_called_once_with(
        handle, future, 'pick_plane_recovery_upright')
    arm.wait_for_joint_state.assert_not_called()


@pytest.mark.parametrize('outcome', ['failed', 'unverified'])
def test_alignment_failure_cannot_start_the_low_approach(outcome):
    arm, _, _ = arm_for_red3()
    arm.move_to_joint_target = Mock(return_value=outcome != 'failed')
    assert not APPROACH(arm)
    assert arm.move_to_joint_target.call_count == 1
    assert (arm.move_to_joint_target.call_args.args[1]
            == 'pick_plane_align_high')


def test_physical_side_axis_error_after_success_is_rejected():
    arm, _, _ = arm_for_red3()
    move = arm.move_to_joint_target.side_effect

    def drift(target, label, **kwargs):
        move(target, label, **kwargs)
        if label == 'pick_approach':
            arm.positions['joint4'] += .10
        return True

    arm.move_to_joint_target.side_effect = drift
    assert not APPROACH(arm)


def test_selector_prefers_the_shorter_safe_pitch_configuration():
    arm, solution, _ = arm_for_red3()
    distant = dict(solution, joint2=1.4, joint3=1.8, joint5=-2.0)
    arm.compute_ik = Mock(side_effect=[distant, solution])
    assert APPROACH(arm)
    assert arm.move_to_joint_target.call_args.args[0] == solution


def test_plane_failure_cannot_descend_close_or_attach():
    arm = SimpleNamespace(
        wait_until_ready=lambda: True, stabilize_base=lambda: True,
        update_stand_collision_objects=lambda *args: True,
        prepare_scene_enabled=False, stabilize_pick_cube=lambda: True,
        publish_state=Mock(), cube_name='red_cube_3',
        command_gripper=Mock(return_value=True),
        move_pick_approach=Mock(return_value=False),
        descend_pick_vertically=Mock(), attach_cube=Mock(),
        attached=False, base_stabilized=True, release_base=Mock())
    assert not MODULE.MoveItPickPlace.run_pick_only(arm)
    arm.descend_pick_vertically.assert_not_called()
    arm.attach_cube.assert_not_called()
    assert all(c.args == (0., 'open')
               for c in arm.command_gripper.call_args_list)
    arm.release_base.assert_called_once()


def test_plane_error_rejects_unknown_axes_and_distinguishes_half_turns():
    assert math.isinf(MODULE.pick_plane_error({}, 0.))
    assert MODULE.pick_plane_error(
        {'joint1': math.pi, 'joint4': 0., 'joint6': math.pi}, 0.) > 3.


@pytest.mark.parametrize('bad', [math.nan, math.inf])
def test_invalid_initial_pitch_feedback_never_reaches_planning(bad):
    arm, _, _ = arm_for_red3()
    arm.positions['joint2'] = bad
    assert not APPROACH(arm)
    arm.compute_ik.assert_not_called()
    arm.move_to_joint_target.assert_not_called()


def inventory_cases():
    """Use the shipped inventory and enabled sides, including future slots."""
    root = Path(__file__).parents[2]
    tree = ast.parse((root / 'tools_demo/tools_demo/mission_coordinator.py')
                     .read_text())
    slots = next(ast.literal_eval(n.value) for n in tree.body
                 if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == 'CUBE_WORLD_SLOTS'
                         for t in n.targets))
    config = yaml.safe_load(
        (root / 'tools_demo/config/mission.yaml').read_text())
    sides = config['competition_mission_coordinator']['ros__parameters'][
        'cube_approach_allowlist']
    return [(entry, slots[entry.split(':')[0]]) for entry in sides]


@pytest.mark.parametrize('entry,world_xy', inventory_cases(),
                         ids=lambda value: str(value))
@pytest.mark.parametrize('lateral_error', [-.025, .0, .025])
def test_every_enabled_slot_selects_forward_plane(
    entry, world_xy, lateral_error,
):
    name, side = entry.split(':')
    heading = math.pi if side == 'east' else 0.
    dock = (world_xy[0] - .36 * math.cos(heading),
            world_xy[1] - .36 * math.sin(heading), 0.)
    quaternion = (0., 0., math.sin(heading / 2), math.cos(heading / 2))
    local = MODULE.world_to_local_position(
        dock, quaternion, (*world_xy, .13))
    arm, solution, _ = arm_for_red3()
    arm.cube_name = name
    arm.pick_safe_pose = (local[0] + .010, local[1] + lateral_error + .001,
                          .1216)
    yaw = math.atan2(arm.pick_safe_pose[1], arm.pick_safe_pose[0])
    solution.update(MODULE.pick_plane_axes(yaw))
    reverse = dict(solution, joint1=yaw + math.pi, joint6=yaw + math.pi)
    arm.compute_ik = Mock(side_effect=[reverse, solution])
    assert APPROACH(arm)
    assert arm.move_to_joint_target.call_args.args[0] == solution
    constraints = arm.move_to_joint_target.call_args.kwargs['path_constraints']
    axes = {c.joint_name: c.position for c in constraints.joint_constraints}
    assert axes == MODULE.pick_plane_axes(yaw)


def trajectory_for_plane(yaw):
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(MODULE.ARM_JOINT_NAMES)
    for pitch in (.78, .80, .82):
        point = JointTrajectoryPoint()
        point.positions = [yaw, pitch, 1.15, 0., -1.21, yaw]
        trajectory.joint_trajectory.points.append(point)
    return trajectory


@pytest.mark.parametrize('axis', [0, 3, 5])
@pytest.mark.parametrize('turn', [math.pi, 2 * math.pi])
def test_descent_rejects_an_intermediate_flip_even_with_valid_endpoints(
    axis, turn,
):
    trajectory = trajectory_for_plane(-.04)
    assert MODULE.pick_trajectory_in_plane(trajectory, -.04)
    trajectory.joint_trajectory.points[1].positions[axis] += turn
    assert not MODULE.pick_trajectory_in_plane(trajectory, -.04)


@pytest.mark.parametrize('bad', ['empty', 'missing_axis', 'short', 'nan'])
def test_unknown_descent_trajectory_is_not_plane_evidence(bad):
    trajectory = trajectory_for_plane(.03)
    if bad == 'empty':
        trajectory.joint_trajectory.points = []
    elif bad == 'missing_axis':
        trajectory.joint_trajectory.joint_names[0] = 'other'
    elif bad == 'short':
        trajectory.joint_trajectory.points[1].positions = [0.]
    else:
        trajectory.joint_trajectory.points[1].positions[1] = math.nan
    assert not MODULE.pick_trajectory_in_plane(trajectory, .03)


def test_cartesian_plane_violation_never_reaches_execution():
    trajectory = trajectory_for_plane(.03)
    trajectory.joint_trajectory.points[1].positions[0] += math.pi
    response = SimpleNamespace(error_code=SimpleNamespace(val=1), fraction=1.,
                               solution=trajectory)
    arm = SimpleNamespace(
        latest_joint_state=JointState(), tool_orientation=(0., 1., 0., 0.),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: MODULE.RobotState().joint_state.header.stamp)),
        get_logger=lambda: Mock(), publish_state=Mock(),
        cartesian_client=Mock(), execute_client=Mock(),
        minimum_cartesian_fraction=.99,
        _wait_for_future=Mock(return_value=response))
    assert not MODULE.MoveItPickPlace.cartesian_move(
        arm, (.37, .011, .087), 'pick_descend', plane_yaw=.03)
    arm.execute_client.send_goal_async.assert_not_called()
    request = arm.cartesian_client.call_async.call_args.args[0]
    assert len(request.path_constraints.joint_constraints) == 3


def test_recovery_lift_keeps_the_scan_orientation_and_collision_checking():
    response = SimpleNamespace(error_code=SimpleNamespace(val=-1))
    arm = SimpleNamespace(
        latest_joint_state=JointState(), tool_orientation=(0., 1., 0., 0.),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: MODULE.RobotState().joint_state.header.stamp)),
        get_logger=lambda: Mock(), publish_state=Mock(),
        cartesian_client=Mock(), execute_client=Mock(),
        _wait_for_future=Mock(return_value=response))
    scan_orientation = (0., 0., 0., 1.)
    assert not MODULE.MoveItPickPlace.cartesian_move(
        arm, (.3, -.03, .30), 'pick_plane_recovery_lift',
        orientation=scan_orientation)
    request = arm.cartesian_client.call_async.call_args.args[0]
    q = request.waypoints[0].orientation
    assert (q.x, q.y, q.z, q.w) == scan_orientation
    assert request.avoid_collisions is True
    assert arm.tool_orientation == (0., 1., 0., 0.)
    arm.execute_client.send_goal_async.assert_not_called()
