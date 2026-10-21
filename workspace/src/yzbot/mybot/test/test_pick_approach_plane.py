"""Reject reversed IK branches and keep the pickup approach in one plane."""

import importlib.util
import ast
import math
from pathlib import Path
from types import SimpleNamespace
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
        pick_tool_pose=lambda: MODULE.pose_from_values(
            (0., 0., .66), (0., 0., 0., 1.)),
        get_logger=lambda: Mock(), compute_ik=Mock(return_value=solution))
    arm.arm_positions = lambda: dict(arm.positions)

    def move(target, label, **kwargs):
        arm.positions.update(target)
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
def test_low_or_reversed_start_is_refused_before_planning(height, shoulder):
    arm, _, _ = arm_for_red3()
    arm.positions['joint1'] = shoulder
    arm.pick_tool_pose = lambda: MODULE.pose_from_values(
        (0., 0., height), (0., 0., 0., 1.))
    assert not APPROACH(arm)
    arm.compute_ik.assert_not_called()
    arm.move_to_joint_target.assert_not_called()


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
