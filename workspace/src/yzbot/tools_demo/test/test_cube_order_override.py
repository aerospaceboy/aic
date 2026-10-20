"""Task order, policy isolation, and failed-pick retry regressions."""
import json
import re
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from tools_demo.mission_coordinator import MissionCoordinator
from tools_demo.pick_order_policy import ordered_task_key
from tools_demo.pick_order_policy import validate_pick_entries

POLICY = json.loads((Path(__file__).resolve().parents[1] / "config"
                    / "cube_order_policy.json").read_text())["tasks"]


def jobs_for(key):
    jobs = []
    for part in key.split("_"):
        count, color, zone = re.fullmatch(r"(\d+)([br])([ABC])", part).groups()
        jobs += [SimpleNamespace(color={"b": "blue", "r": "red"}[color],
                                 target_zone=zone) for _ in range(int(count))]
    return jobs


class Harness:
    _consume_cube_order_override = (
        MissionCoordinator._consume_cube_order_override)
    _cube_order_override_entries = (
        MissionCoordinator._cube_order_override_entries)
    _policy_entries_for_jobs = MissionCoordinator._policy_entries_for_jobs
    _seed_cube_order_override = MissionCoordinator._seed_cube_order_override
    _load_cube_order_policy = MissionCoordinator._load_cube_order_policy

    def __init__(self, policy=None, override=()):
        self._cube_order_policy = POLICY if policy is None else policy
        self._param_value = list(override)
        self._task_order_entries = None
        self._override_index = 0
        self._override_pending_side = None
        self._cube_done = set()
        self._cube_excluded = set()
        self.get_logger = Mock(return_value=Mock())
        self.set_parameters = Mock()
        self.machine = SimpleNamespace(jobs=[])

    def get_parameter(self, name):
        return SimpleNamespace(value=(list(self._param_value)
                               if name == "cube_order_override" else ""))

    def seed(self, key):
        self.machine.jobs = jobs_for(key)
        self._seed_cube_order_override()
        return list(self._task_order_entries)


@pytest.mark.parametrize("key", sorted(POLICY))
def test_all_48_policies_preserve_actual_instruction_order(key):
    harness = Harness()
    jobs = jobs_for(key)
    assert ordered_task_key(jobs) == key
    entries = harness.seed(key)
    assert len(entries) == len(jobs) == 5
    assert len({entry.split(":")[0] for entry in entries}) == 5
    for job, entry in zip(jobs, entries):
        assert harness._consume_cube_order_override(job) == entry.split(":")[0]
    harness.set_parameters.assert_not_called()


def test_second_mission_reseeds_without_persisting_automatic_override():
    harness = Harness()
    first = harness.seed("1bB_4rA")
    second = harness.seed("4rB_1bA")
    assert first != second
    assert second[0].startswith("red_")
    assert harness._override_index == 0
    assert harness._param_value == []
    harness.set_parameters.assert_not_called()


def test_failed_pick_does_not_discard_future_color():
    harness = Harness()
    harness.seed("1bB_4rA")
    blue, red = harness.machine.jobs[:2]
    harness._consume_cube_order_override(blue)
    assert harness._consume_cube_order_override(blue) is None
    assert harness._override_index == 1
    assert harness._consume_cube_order_override(red).startswith("red_")


def test_runtime_override_is_snapshotted_until_next_task():
    entries = Harness().seed("1bB_4rA")
    harness = Harness(override=entries)
    assert harness.seed("1bB_4rA") == entries
    harness._param_value = []
    selected = harness._consume_cube_order_override(harness.machine.jobs[0])
    assert selected == entries[0].split(":")[0]
    assert harness.seed("4rB_1bA")[0].startswith("red_")


@pytest.mark.parametrize("damage", [
    "short", "duplicate", "wrong_color", "unknown", "side"])
def test_invalid_policy_is_rejected_as_a_whole(damage):
    jobs = jobs_for("1bB_4rA")
    entries = Harness().seed("1bB_4rA")
    if damage == "short":
        entries.pop()
    if damage == "duplicate":
        entries[2] = entries[1]
    if damage == "wrong_color":
        entries[0] = "red_cube_5:west"
    if damage == "unknown":
        entries[0] = "blue_cube_999:west"
    if damage == "side":
        entries[0] = "blue_cube_1:south"
    assert validate_pick_entries(entries, jobs) == []
    harness = Harness(override=entries)
    assert harness.seed("1bB_4rA") == []


@pytest.mark.parametrize("row", [
    None, [], {"order": None}, {"order": [None]}, {"order": [{}]}])
def test_malformed_policy_row_falls_back_without_exception(row):
    assert Harness(policy={"1bB_4rA": row}).seed("1bB_4rA") == []


def test_unknown_task_keeps_same_color_greedy_fallback():
    assert Harness().seed("2bA_2rB") == []


def test_excluded_and_done_entities_are_skipped_without_side_leak():
    harness = Harness()
    entries = harness.seed("4rB_1bA")
    harness._cube_excluded.add(entries[0].split(":")[0])
    harness._cube_done.add(entries[1].split(":")[0])
    selected = harness._consume_cube_order_override(harness.machine.jobs[0])
    assert selected == entries[2].split(":")[0]
