"""Experimental straight speed must not raise turn, gate or docking limits."""
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from tools_demo.task_navigator import TaskNavigator, ZonePose


@pytest.mark.parametrize('enabled', [False, True])
def test_dense_straight_speed_preserves_every_other_stage_limit(enabled):
    published=[]
    node=SimpleNamespace(dense_fast_transit=enabled,
        stage_speed_limit_publisher=SimpleNamespace(publish=published.append),
        get_logger=lambda:Mock())
    straight=('A_WALL_SOUTH_CLEAR', 'A_WALL_WEST_CLEAR',
              'north_pickup_tray_west_clear', 'north_pickup_tray_east_clear',
              'north_delivery_tray_west_clear', 'north_delivery_tray_east_clear')
    for stage in straight:
        TaskNavigator._publish_stage_speed_limit(node,ZonePose(stage,0.,0.,0.))
        assert published[-1].speed_limit == 1.5
        assert published[-1].speed_limit < 1.78
    for stage,speed in [('A_WALL_NORTH_CLEAR',.75),('FLOW_A_turn',.75),
                        ('FLOW_E_turn',1.2),('CENTRAL_MOVER_GATE_WEST',1.),
                        ('bay_pocket_north_exit',.6),('BLUE_CUBE_5_ENTRY',0.)]:
        TaskNavigator._publish_stage_speed_limit(node,ZonePose(stage,0.,0.,0.))
        assert published[-1].speed_limit == speed


@pytest.mark.parametrize('zone', ['A', 'B', 'C'])
def test_external_delivery_goal_does_not_limit_the_whole_transit_to_dock_speed(zone):
    published = []
    node = SimpleNamespace(dense_fast_transit=True,
        stage_speed_limit_publisher=SimpleNamespace(publish=published.append),
        get_logger=lambda: Mock())
    TaskNavigator._publish_stage_speed_limit(node, ZonePose('FLOW_A_turn', 0., 0., 0.))
    assert published[-1].speed_limit == .75
    # The final action can begin 15m away. Release the previous stage cap;
    # RPP's configured near-goal scaling and the precision owner slow the end.
    TaskNavigator._publish_stage_speed_limit(
        node, ZonePose(zone + '_delivery_entry', 15., 0., 0.))
    assert published[-1].speed_limit == 0.
