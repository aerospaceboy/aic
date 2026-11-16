"""A visible paint box is not a physical table: use the actual support."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from tools_demo.mission_state_machine import MissionStateMachine
from tools_demo.task_protocol import TaskItem, TaskPlan


def test_all_three_painted_zones_use_the_floor_as_the_delivery_support():
    root=Path(__file__).parents[2]
    world=ET.parse(root/'mybot_description/worlds/offic_room.world').getroot()
    params=yaml.safe_load((root/'tools_demo/config/mission.yaml').read_text())[
        'competition_mission_coordinator']['ros__parameters']
    for name in ('zone_a','zone_b','zone_c'):
        model=world.find(f".//model[@name='{name}']")
        assert model is not None
        assert not model.findall('.//collision')
        assert model.findtext('link/visual/geometry/box/size') == '1 0.5 0.02'
    assert params['delivery_surface_z'] == 0.
    assert params['delivery_cube_half_height'] == .015


def test_unconfirmed_release_is_not_a_second_place_or_an_assumed_delivery():
    machine=MissionStateMachine()
    machine.start(TaskPlan('physical-test','rules',(TaskItem('red',1,'A'),)))
    machine.state='placing'
    action=machine.placement_released_unconfirmed()
    assert action.kind=='verify' and machine.state=='verifying'
    assert machine.completed_count==machine.skipped_count==0
    machine.verification_finished(False)
    assert machine.completed_count==0
