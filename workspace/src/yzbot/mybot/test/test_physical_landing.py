"""Competition placement must be proved by physics, not written coordinates."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import Pose

spec=importlib.util.spec_from_file_location('physical_pick_place',
    Path(__file__).parents[1]/'pick_place_moveit.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def pose(x=.4,y=0.,z=.035):
    p=Pose();p.position.x=x;p.position.y=y;p.position.z=z;p.orientation.w=1.
    return p


def worker(monkeypatch,poses):
    clock=[0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(module.time,'sleep',lambda dt:clock.__setitem__(0,clock[0]+dt))
    values=iter(poses)
    n=SimpleNamespace(cube_name='red_cube_1',release_world_pose=(.4,0.,.035),
        get_logger=lambda:Mock(),place_failure_reason=None,set_entity_pose=Mock(),
        get_entity_pose=lambda *args:next(values,poses[-1]))
    return n


def test_free_fall_then_settled_actual_pose_is_accepted_without_writes(monkeypatch):
    n=worker(monkeypatch,[pose(z=.09),pose(z=.06),pose(x=.415),pose(x=.415)])
    assert module.MoveItPickPlace.wait_for_physical_landing(n)
    n.set_entity_pose.assert_not_called()


@pytest.mark.parametrize('actual',[pose(x=.6),pose(z=.10),pose(z=-.1),pose(x=float('nan'))])
def test_wrong_slot_airborne_underfloor_and_invalid_cubes_are_not_deliveries(monkeypatch,actual):
    n=worker(monkeypatch,[actual])
    assert not module.MoveItPickPlace.wait_for_physical_landing(n,timeout_sec=.4)
    assert n.place_failure_reason == 'physical_landing_unproved'
    n.set_entity_pose.assert_not_called()


def test_cube_still_sliding_cannot_be_declared_settled(monkeypatch):
    n=worker(monkeypatch,[pose(x=.4+i*.004) for i in range(8)])
    assert not module.MoveItPickPlace.wait_for_physical_landing(n,timeout_sec=.5)


def test_competition_pose_writer_is_fail_closed_even_if_a_future_caller_uses_it():
    n=SimpleNamespace(prepare_scene_enabled=False,get_logger=lambda:Mock(),
                      set_entity_client=Mock())
    assert not module.MoveItPickPlace.set_entity_pose(n,'red_cube_1',pose())
    n.set_entity_client.call_async.assert_not_called()


def test_detach_never_applies_the_requested_world_slot(monkeypatch):
    monkeypatch.setattr(module.time,'sleep',lambda _:None)
    response=SimpleNamespace(success=True)
    n=SimpleNamespace(cube_name='red_cube_1',release_world_pose=(99.,99.,.035),
        attached=True,detach_client=Mock(),_wait_for_future=lambda *a,**k:response,
        get_entity_pose=Mock(),set_entity_pose=Mock(),publish_gripper_state=Mock())
    assert module.MoveItPickPlace.detach_cube(n)
    assert n.attached is False
    n.set_entity_pose.assert_not_called()
    n.get_entity_pose.assert_not_called()


def test_bad_grasp_retry_requires_a_new_valid_window_and_never_resets_cube(monkeypatch):
    monkeypatch.setattr(module.time,'sleep',lambda _:None)
    n=SimpleNamespace(cube_name='red_cube_1',attached=True,validate_grasp_window=Mock(side_effect=[(.01,0.,.04),None]),
        attach_cube=Mock(return_value=True),confirm_attachment_constraint=Mock(return_value=None),
        detach_cube=Mock(return_value=True),set_entity_pose=Mock(),get_logger=lambda:Mock())
    assert module.MoveItPickPlace.attach_cube_with_stability_retry(n) is None
    n.attach_cube.assert_called_once()
    n.set_entity_pose.assert_not_called()
