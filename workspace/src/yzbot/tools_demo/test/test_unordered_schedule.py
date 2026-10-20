"""Exercise order freedom without weakening inventory or safety evidence."""


import pytest
from tools_demo.mission_state_machine import MissionJob
from tools_demo.unordered_schedule import optimize_schedule


def test_route_can_interleave_colors_and_never_repeat_a_cube():
    jobs = tuple(MissionJob(i + 1, 3, c, c + "_cube_1", z)
                 for i, (c, z) in enumerate([("red", "A"), ("red", "A"),
                                             ("blue", "B")]))
    options = {"red": [("red_cube_1", "east", 1),
                       ("red_cube_2", "east", 2)],
               "blue": [("blue_cube_1", "west", 3)]}
    edges = {(0, 3): 1, (10, 1): 1, (10, 2): 2,
             (20, 1): 1, (20, 2): 1, (0, 1): 9, (0, 2): 9,
             (10, 3): 20}
    result, entries, estimate = optimize_schedule(
        jobs, options, (0, 0, 0), {"A": (10, 0, 0), "B": (20, 0, 0)},
        lambda pose, cube, zone: edges.get((pose[0], cube), 20))
    assert [j.color for j in result] == ["blue", "red", "red"]
    assert len(set(entries)) == 3 and estimate == 3
    assert [j.index for j in result] == [1, 2, 3]
    assert jobs[0].color == "red"


def test_no_inventory_cannot_produce_a_partial_schedule():
    jobs = [MissionJob(1, 1, "red", "red_cube_1", "A")]
    with pytest.raises(ValueError, match="complete legal"):
        optimize_schedule(jobs, {}, (0, 0, 0), {"A": (0, 0, 0)},
                          lambda *args: 1)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -1])
def test_invalid_costs_are_not_used(bad):
    jobs = [MissionJob(1, 1, "red", "red_cube_1", "A")]
    with pytest.raises(ValueError):
        optimize_schedule(jobs, {"red": [("red_cube_1", "east", 1)]},
                          (0, 0, 0), {"A": (0, 0, 0)},
                          lambda *args: bad)
