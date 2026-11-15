import math
from tools_demo.navigation_math import ZonePose
from tools_demo.transit_anchor import resolve_transit_anchor


def live_cost(x, y):
    return 100 if math.hypot(x, y) < .35 else 0


def test_dynamic_occupancy_relocates_virtual_goal_and_checks_complete_tube():
    checked = []
    def clear(a, b):
        checked.append((a, b))
        return True
    original = ZonePose('universal_origin_staging', 0., 0., math.pi)
    target = resolve_transit_anchor(original, (2., 0.), (-2., 0.), live_cost, clear)
    assert target != original
    assert target.zone == original.zone and target.yaw == original.yaw
    assert 0 < math.hypot(target.x, target.y) <= 1.
    assert live_cost(target.x, target.y) == 0
    assert ((2., 0.), (target.x, target.y)) in checked
    assert ((target.x, target.y), (-2., 0.)) in checked


def test_protected_targets_and_admission_gates_never_move():
    for name in ('A', 'entry', 'dock', 'RED_CUBE_2', 'universal_mover_1_clear',
                 'CENTRAL_MOVER_GATE_WEST', 'FLOW_A_ARC_1'):
        pose = ZonePose(name, 0., 0., 0.)
        assert resolve_transit_anchor(pose, (2., 0.), (-2., 0.), live_cost,
                                      lambda a, b: True) == pose


def test_static_wall_or_unknown_never_gets_bypassed():
    pose = ZonePose('A_WALL_NORTH_CLEAR', 0., 0., 0.)
    assert resolve_transit_anchor(pose, (2., 0.), (-2., 0.), live_cost,
                                  lambda a, b: False) == pose
    assert resolve_transit_anchor(pose, (2., 0.), (-2., 0.), lambda x, y: None,
                                  lambda a, b: True) == pose


def test_relocation_cannot_cut_a_wall_between_neighbouring_stages():
    pose = ZonePose('A_WALL_NORTH_CLEAR', 0., 0., 0.)
    assert resolve_transit_anchor(pose, (2., 0.), (-2., 0.), live_cost,
                                  lambda a, b: a == b or a == (0., 0.)) == pose


def test_unoccupied_or_final_virtual_goal_is_preserved():
    pose = ZonePose('universal_origin_staging', 0., 0., 0.)
    assert resolve_transit_anchor(pose, (2., 0.), (-2., 0.), lambda x, y: 0,
                                  lambda a, b: True) == pose
    assert resolve_transit_anchor(pose, (2., 0.), None, live_cost,
                                  lambda a, b: True) == pose
