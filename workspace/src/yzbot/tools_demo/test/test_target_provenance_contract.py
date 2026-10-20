"""Contracts for pick-target provenance and carry-window accounting.

These lock the two properties the 2026-09-18 baseline needed:
1. before a target is used for a grasp the coordinator records which path
   produced it and how old the observation is, and a refused descend is not
   charged to the cube's grasp-failure budget (that budget is what exhausted
   the inventory in 1bA_4rC);
2. the campaign metrics keep instantaneous carry_window valid=False samples
   separate from a real carry abort.
"""

import ast
import pathlib
import re
import tempfile

TOOLS_DEMO = pathlib.Path(__file__).resolve().parents[1]
COORDINATOR = TOOLS_DEMO / "tools_demo" / "mission_coordinator.py"
CAMPAIGN = TOOLS_DEMO / "scripts" / "run_zone9_campaign.py"


def coordinator_source():
    return COORDINATOR.read_text(encoding="utf-8")


def campaign_source():
    return CAMPAIGN.read_text(encoding="utf-8")


def test_every_target_path_records_its_provenance():
    """Fresh perception, cached slot and slot fallback all name themselves."""
    source = coordinator_source()
    assert 'self.target_source = "perception"' in source
    assert 'self.target_source = "cached_slot"' in source
    assert 'self.target_source = "slot_fallback"' in source
    assert source.count("self.target_observed_at = time.monotonic()") == 3


def test_manipulation_request_carries_source_and_age():
    source = coordinator_source()
    assert '"target_source": self.target_source' in source
    assert '"target_age_sec": (' in source


def test_slot_fallback_is_marked_unverified_by_name():
    """The fallback target comes from the nominal slot, not an observation."""
    source = coordinator_source()
    fallback = source.split("def _attempt_slot_fallback", 1)[1]
    assert 'self.target_source = "slot_fallback"' in fallback.split(
        "def ", 1
    )[0]


def test_refused_descend_does_not_consume_the_grasp_failure_budget():
    source = coordinator_source()
    assert "APPROACH_REJECTION_LIMIT = 2" in source
    assert "self._approach_rejections = {}" in source
    marker = 'and rejection_detail.startswith("approach_rejected")'
    branch = source.split(marker, 1)[1]
    branch = branch.split("if cube is not None:", 1)[0]
    assert "self._approach_rejections[cube] = rejections" in branch
    assert "APPROACH_REJECTION_LIMIT" in branch
    assert "_cube_failures[" not in branch, (
        "a refused descend must not count as a grasp failure"
    )
    assert "_cube_excluded.add" not in branch
    # The bounded branch is checked before the unconditional failure counter.
    handler = source.split('if operation == "pick" and not success:', 1)[1]
    assert handler.index("approach_rejected") < handler.index(
        "failures = self._cube_failures.get(cube, 0) + 1"
    )


def log_metrics_function():
    """Load only log_metrics so importing the runner cannot create run dirs."""
    tree = ast.parse(campaign_source())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "log_metrics":
            module = ast.Module(body=[node], type_ignores=[])
            namespace = {"re": re, "pathlib": pathlib}
            exec(compile(module, str(CAMPAIGN), "exec"), namespace)
            return namespace["log_metrics"]
    raise AssertionError("log_metrics not found in the campaign runner")


SYNTHETIC_LOG = "\n".join(
    [
        "[pick_place_moveit-31] carry_window x=0.01941 y=-0.00067 "
        "z=0.06569 valid=True",
        "[pick_place_moveit-31] carry_window x=0.01941 y=0.06610 "
        "z=0.06569 valid=False",
        "[pick_place_moveit-31] carry_window x=0.01941 y=0.06603 "
        "z=0.06569 valid=False",
        "[pick_place_moveit-31] persistent_result id=3 success=False detail=pick failed",
        "[task_navigator-20] nav_retry attempt=1",
        "[task_navigator-20] nav_result state=timed_out elapsed=30.0s",
        '{"collision_detected": true}',
        "[competition_task_navigator] collision_fault raised",
        "[rtabmap] Starting point in lethal space",
        "[mission_coordinator-30] task_id=abc source=llm order=1:redx1->A",
    ]
)


def test_instantaneous_carry_window_invalid_is_not_a_carry_abort():
    """Only a real abort may be reported as a carry failure."""
    metrics = log_metrics_function()
    with tempfile.TemporaryDirectory() as directory:
        log = pathlib.Path(directory) / "launch.log"
        log.write_text(SYNTHETIC_LOG, encoding="utf-8")
        result = metrics(str(log))
    assert result["carry_invalid"] == 2, "instantaneous samples are diagnostic"
    assert result["carry_abort"] == 0, "no abort line was logged"
    assert result["pick_failed"] == 1
    assert result["nav_retry"] == 1
    assert result["nav_timeout"] == 1
    assert result["collision_detected"] == 1
    assert result["collision_fault"] == 1
    assert result["lethal_start"] == 1
    assert result["plan_source"] == "llm"


def test_carry_abort_is_counted_only_from_the_abort_line():
    metrics = log_metrics_function()
    text = SYNTHETIC_LOG + "\n" + (
        "Attached cube left the safe carry window during transport"
    )
    with tempfile.TemporaryDirectory() as directory:
        log = pathlib.Path(directory) / "launch.log"
        log.write_text(text, encoding="utf-8")
        result = metrics(str(log))
    assert result["carry_abort"] == 1


def test_runner_never_reports_carry_invalid_as_a_failure_reason():
    """The runner's gate stays on aborts; samples stay diagnostics."""
    source = campaign_source()
    for line in source.splitlines():
        if "reasons.append" in line:
            assert "carry_invalid" not in line
    assert 'record["metrics"]["carry_abort"]' in source


def test_runner_still_gates_completion_timeout_collision_and_order():
    """The acceptance rules themselves must be unchanged."""
    source = campaign_source()
    assert 'if mission.get("state") != "completed":' in source
    assert 'if mission.get("completed") != 5:' in source
    assert 'if mission.get("skipped") not in (0, None):' in source
    assert "elif elapsed > 290.0:" in source
    assert 'if avoidance.get("collision_detected") is True:' in source
    assert "if not validated_avoidance(avoidance):" in source
    assert 'record["metrics"]["plan_source"] != "llm"' in source
    assert "record[\"metrics\"][\"plan_order\"] != record[\"expected_order\"]" in source
