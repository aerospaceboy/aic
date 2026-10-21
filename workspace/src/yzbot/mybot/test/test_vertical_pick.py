"""Exercise physical-pose guards and recovery for the final pickup column."""

import copy
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import Pose


SPEC = importlib.util.spec_from_file_location(
    "vertical_pick_module", Path(__file__).parents[1] / "pick_place_moveit.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
DESCEND = MODULE.MoveItPickPlace.descend_pick_vertically


def raw_pose(xyz, quaternion):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = quaternion
    return pose


def arm_at_safe_height():
    pose = MODULE.pose_from_values((.401, .002, .15), (0., 1., 0., 0.))
    arm = SimpleNamespace(
        pick_pose=(.4, 0., .08), pick_safe_pose=(.4, 0., .15),
        transport_x=.28, tool_orientation=(0., 1., 0., 0.),
        pick_descend_duration=.6, pose=pose, get_logger=lambda: Mock(),
        move_to_pose=Mock(), command_gripper=Mock(),
        wait_for_joint_state=Mock(return_value=True))
    arm.pick_tool_pose = lambda: copy.deepcopy(arm.pose)

    def move(target, label, **kwargs):
        arm.pose = MODULE.pose_from_values(target, arm.tool_orientation)
        return True

    arm.cartesian_move = Mock(side_effect=move)
    return arm


def test_descent_freezes_measured_xy_and_keeps_the_lift_in_that_column():
    arm = arm_at_safe_height()
    assert DESCEND(arm)
    target = arm.cartesian_move.call_args.args[0]
    assert target == (.401, .002, .08)
    assert arm.cartesian_move.call_args.kwargs['plane_yaw'] == pytest.approx(
        math.atan2(.002, .401))
    assert arm.pick_pose == target
    assert arm.pick_safe_pose == (.401, .002, .15)
    assert arm.transport_pose[1] == .002
    arm.move_to_pose.assert_not_called()


@pytest.mark.parametrize('bad_pose', [
    MODULE.pose_from_values((.42, 0., .15), (0., 1., 0., 0.)),
    MODULE.pose_from_values((.4, 0., .15), (0., 1., 0., .1)),
    MODULE.pose_from_values((.4, 0., .15), (.1, 1., 0., 0.)),
    raw_pose((.4, 0., .15), (0., 0., 0., 0.)),
    MODULE.pose_from_values((float('nan'), 0., .15), (0., 1., 0., 0.)),
    None,
])
def test_misaligned_or_unknown_physical_pose_never_starts_descent(bad_pose):
    arm = arm_at_safe_height()
    arm.pick_tool_pose = lambda: bad_pose
    assert not DESCEND(arm)
    arm.cartesian_move.assert_not_called()


def test_twice_failed_cartesian_descent_cannot_fall_back_to_joint_motion():
    arm = arm_at_safe_height()
    arm.cartesian_move = Mock(return_value=False)
    assert not DESCEND(arm)
    assert arm.cartesian_move.call_count == 2
    assert [c.args[1] for c in arm.cartesian_move.call_args_list] == [
        "pick_descend", "pick_descend_retry"]
    assert all(c.kwargs['plane_yaw'] == pytest.approx(math.atan2(.002, .401))
               for c in arm.cartesian_move.call_args_list)
    arm.move_to_pose.assert_not_called()
    arm.command_gripper.assert_not_called()


@pytest.mark.parametrize('bad_end', [
    MODULE.pose_from_values((.41, .002, .08), (0., 1., 0., 0.)),
    MODULE.pose_from_values((.401, .002, .09), (0., 1., 0., 0.)),
    MODULE.pose_from_values((.401, .002, .08), (0., 1., 0., .1)),
    MODULE.pose_from_values((.401, .002, .08), (0., float('nan'), 0., 0.)),
    None,
])
def test_controller_success_still_requires_physical_endpoint(bad_end):
    arm = arm_at_safe_height()
    initial = arm.pick_tool_pose()
    arm.pick_tool_pose = Mock(side_effect=[initial, bad_end])
    assert not DESCEND(arm)


def test_quaternion_error_treats_opposite_signs_as_the_same_orientation():
    assert MODULE.quaternion_angular_error(
        (0., -1., 0., 0.), (0., 1., 0., 0.)) == 0.
    assert math.isinf(MODULE.quaternion_angular_error(
        (0., 0., 0., 0.), (0., 1., 0., 0.)))


def test_gazebo_tcp_conversion_uses_the_existing_footprint_frame():
    pose = MODULE.pose_from_values((.4, .02, .15 + MODULE.BASE_LINK_HEIGHT),
                                   (0., 1., 0., 0.))
    arm = SimpleNamespace(get_entity_pose=Mock(return_value=pose))
    result = MODULE.MoveItPickPlace.pick_tool_pose(arm)
    arm.get_entity_pose.assert_called_once_with(
        "six_arm::link6", "six_arm::base_footprint")
    assert result.position.z == pytest.approx(.15)


@pytest.mark.parametrize('failure', ['planning', 'endpoint'])
def test_pick_entry_never_closes_after_a_failed_vertical_guard(failure):
    arm = arm_at_safe_height()
    for name in ('wait_until_ready', 'stabilize_base',
                 'update_stand_collision_objects', 'stabilize_pick_cube'):
        setattr(arm, name, Mock(return_value=True))
    arm.prepare_scene_enabled = False
    arm.publish_state = Mock()
    arm.cube_name = 'red_cube_2'
    arm.attached = False
    arm.base_stabilized = True
    arm.release_base = Mock(return_value=True)
    arm.move_to_pose = Mock(return_value=True)
    arm.move_pick_approach = Mock(return_value=True)
    arm.command_gripper = Mock(return_value=True)
    arm.calibrate_pick_height_at_safe_pose = Mock(return_value=True)
    arm.refine_pick_target_before_descend = Mock(return_value=True)
    arm.descend_pick_vertically = lambda: DESCEND(arm)
    if failure == 'planning':
        arm.cartesian_move = Mock(return_value=False)
    else:
        original = arm.pose
        bad = raw_pose((.401, .002, .08), (0., 1., 0., .1))
        arm.pick_tool_pose = Mock(side_effect=[original, bad])
    assert not MODULE.MoveItPickPlace.run_pick_only(arm)
    assert all(call.args == (0., "open")
               for call in arm.command_gripper.call_args_list)
    arm.release_base.assert_called_once()


@pytest.mark.parametrize('cube_z', [.0106, .0266])
def test_live_height_keeps_fingers_above_tray_and_overlapping_cube(cube_z):
    arm = arm_at_safe_height()
    arm.cube_name = 'red_cube_2'
    arm.pick_pose = (.4, 0., .0516)  # old command penetrated the tray
    arm.relative_cube_position = lambda: (.01, -.001, .15 - cube_z)
    assert MODULE.MoveItPickPlace.calibrate_pick_height_at_safe_pose(arm)
    assert arm.pick_pose[2] == pytest.approx(cube_z + .075)
    finger_bottom = arm.pick_pose[2] - MODULE.PICK_FINGER_DOWN_REACH
    support_top = cube_z - MODULE.CUBE_HALF_SIZE
    assert finger_bottom - support_top == pytest.approx(.008)
    assert finger_bottom < cube_z + MODULE.CUBE_HALF_SIZE
    assert MODULE.GRASP_Z_LIMITS[0] < arm.pick_pose[2] - cube_z
    assert arm.pick_pose[2] - cube_z < MODULE.GRASP_Z_LIMITS[1]
    assert arm.pick_safe_pose == (.4, 0., .15)


@pytest.mark.parametrize('relative', [None, (.01, 0., .5), (.01, 0., .01),
                                      (.01, 0., float('nan'))])
def test_bad_live_height_cannot_retarget_the_pick(relative):
    arm = arm_at_safe_height()
    arm.relative_cube_position = lambda: relative
    original = arm.pick_pose
    assert not MODULE.MoveItPickPlace.calibrate_pick_height_at_safe_pose(arm)
    assert arm.pick_pose == original
    arm.cartesian_move.assert_not_called()
