"""Reject plausible-looking results that lack order/safety/timing evidence."""
import json

import pytest
from tools_demo.evidence_audit import audit_run

QUESTION = "Move 1 blue cube to zone B and 4 red cubes to zone A."


def fixture():
    mission = {"state": "completed", "completed": 5, "skipped": 0,
               "elapsed_sec": 250.0}
    avoidance = {"safe": True, "collision_detected": False,
                 "obstacles_seen": 4, "sample_count": 100,
                 "avoidance_demonstrated": True,
                 "avoidance_events": ["spawn_avoidance_1", "spawn_avoidance_2"],
                 "encountered_obstacles": [
                     "competition_moving_obstacle_1",
                     "competition_moving_obstacle_2"],
                 "response_obstacles": [
                     "competition_moving_obstacle_1",
                     "competition_moving_obstacle_2"],
                 "minimum_clearance": 0.13,
                 "response_kinds": {f"competition_moving_obstacle_{i}": "yield"
                                    for i in (1, 2)}}
    entities = ["blue_cube_1"] + [f"red_cube_{i}" for i in (2, 5, 3, 1)]
    launch = "\n".join(
        f"manip_start operation=pick cube={cube}\n"
        f"delivery_truth cube={cube} zone={'B' if i == 0 else 'A'} "
        "x=0 y=0 z=0 valid=True slot_error=0.0000 precise=True"
        for i, cube in enumerate(entities))
    return mission, avoidance, launch


def report(mission, avoidance, launch):
    result = "MISSION_TERMINAL=" + json.dumps(mission) + "\n"
    result += "AVOIDANCE_EVIDENCE=" + json.dumps(avoidance) + "\n"
    result += "CASE_ACCEPTED=true collision_faults=[]\n"
    return audit_run(QUESTION, result, launch)


def test_ordered_motion_pass_is_not_full_competition_acceptance():
    result = report(*fixture())
    assert result["motion_under_300"]
    assert result["full_competition_acceptance"] == "not_tested"


@pytest.mark.parametrize("elapsed", [300.0, 392.0, float("nan"), None])
def test_functional_completion_is_separate_from_300_seconds(elapsed):
    mission, avoidance, launch = fixture()
    mission["elapsed_sec"] = elapsed
    result = report(mission, avoidance, launch)
    assert result["motion_functional_pass"]
    assert not result["motion_under_300"]


@pytest.mark.parametrize("damage", [
    "order", "missing_truth", "retry", "imprecise", "yield", "clearance"])
def test_incomplete_or_unsafe_evidence_cannot_pass(damage):
    mission, avoidance, launch = fixture()
    if damage == "order":
        launch = launch.replace("cube=blue_cube_1", "cube=red_cube_4")
    if damage == "missing_truth":
        launch = launch.replace("delivery_truth cube=blue_cube_1", "missing")
    if damage == "retry":
        launch += "\nmanip_start operation=pick cube=red_cube_1"
    if damage == "imprecise":
        launch = launch.replace("precise=True", "precise=False", 1)
    if damage == "yield":
        avoidance["response_kinds"].pop("competition_moving_obstacle_2")
    if damage == "clearance":
        avoidance["minimum_clearance"] = 0.09
    assert not report(mission, avoidance, launch)["motion_functional_pass"]


def test_two_effective_movers_need_not_be_the_original_pair():
    """The formal rules count distinct effective moving models."""
    mission, avoidance, launch = fixture()
    name = 'preview_moving_partition_1'
    avoidance['response_kinds'].pop('competition_moving_obstacle_2')
    avoidance['response_kinds'][name] = 'detour'
    avoidance['encountered_obstacles'] = [
        'competition_moving_obstacle_1', name]
    avoidance['response_obstacles'] = [
        'competition_moving_obstacle_1', name]
    avoidance['avoidance_events'] = [
        'spawn_avoidance_1', 'dynamic_avoidance_' + name]
    result = report(mission, avoidance, launch)
    assert result['two_dynamic_responses']
    assert result['motion_functional_pass']
    assert not result['dual_yield']


def test_a_response_without_an_encounter_is_not_effective():
    """An unrelated moving wall response cannot supply a second event."""
    mission, avoidance, launch = fixture()
    avoidance['encountered_obstacles'] = ['competition_moving_obstacle_1']
    assert not report(mission, avoidance, launch)['motion_functional_pass']


def test_reordered_complete_goods_pass_new_rules_but_not_old_rules():
    mission, avoidance, launch = fixture()
    blocks = launch.splitlines()
    launch = "\n".join(blocks[2:] + blocks[:2])
    result = report(mission, avoidance, launch)
    assert not result["order_verified"]
    assert result["requirements_verified"] and result["goods_score"] == 20
    assert result["motion_under_290"]
    text = "MISSION_TERMINAL=" + json.dumps(mission) + "\n"
    text += "AVOIDANCE_EVIDENCE=" + json.dumps(avoidance) + "\n"
    text += "CASE_ACCEPTED=true collision_faults=[]\n"
    strict = audit_run(QUESTION, text, launch, True)
    assert not strict["motion_functional_pass"]


def test_wrong_zone_and_duplicate_cube_cannot_gain_full_goods_score():
    mission, avoidance, launch = fixture()
    wrong = report(mission, avoidance, launch.replace("zone=B", "zone=C"))
    assert not wrong["requirements_verified"] and wrong["goods_score"] == 16
    duplicate = report(mission, avoidance,
                       launch.replace("red_cube_5", "red_cube_2"))
    assert not duplicate["requirements_verified"]
    assert duplicate["goods_score"] == 16


@pytest.mark.parametrize(
    "elapsed, passed", [(289.9, True), (290, False), (294.2, False)])
def test_290_target_is_separate_from_300_rule(elapsed, passed):
    mission, avoidance, launch = fixture()
    mission["elapsed_sec"] = elapsed
    result = report(mission, avoidance, launch)
    assert result["motion_under_300"]
    assert result["motion_under_290"] is passed
