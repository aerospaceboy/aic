"""Tests for ordered mission orchestration and failure recovery."""

import json

import pytest

from tools_demo.mission_state_machine import expand_jobs
from tools_demo.mission_state_machine import MissionStateMachine
from tools_demo.mission_state_machine import MissionValueError
from tools_demo.mission_state_machine import parse_task_plan_json
from tools_demo.mission_state_machine import select_reachable_target
from tools_demo.mission_state_machine import selected_cube_snapshot
from tools_demo.task_protocol import TaskItem
from tools_demo.task_protocol import TaskPlan


def sample_plan():
    """Return a two-color plan with an observable execution order."""
    return TaskPlan(
        task_id="test-task",
        source="rules",
        items=(
            TaskItem("red", 2, "A"),
            TaskItem("blue", 1, "C"),
        ),
    )


def complete_job(machine):
    """Drive the current job through all successful events."""
    assert machine.navigation_finished(True).kind == "perceive"
    target = {
        "color": machine.current_job.color,
        "position": {"x": 0.4, "y": 0.0, "z": 0.03},
    }
    assert machine.perception_finished(target).kind == "pick"
    departure = machine.manipulation_finished(True)
    assert departure.kind == "navigate"
    assert departure.location == f"{machine.current_job.color.upper()}_EXIT"
    destination = machine.navigation_finished(True)
    assert destination.kind == "navigate"
    assert destination.location == machine.current_job.target_zone
    assert machine.navigation_finished(True).kind == "place"
    assert machine.manipulation_finished(True).kind == "verify"
    final_job = machine.current_job.index == machine.current_job.total
    zone_exit = machine.verification_finished(True)
    if final_job:
        assert zone_exit.kind == "done"
        assert machine.state == "completed"
        return zone_exit
    assert zone_exit.kind == "navigate"
    assert zone_exit.location == f"{machine.current_job.target_zone}_EXIT"
    assert machine.state == "navigating_zone_exit"
    return machine.navigation_finished(True)


def test_public_plan_is_revalidated_before_execution():
    """Accept parser output and reject inconsistent derived fields."""
    plan = sample_plan()
    parsed = parse_task_plan_json(plan.to_json())
    assert parsed == plan

    payload = plan.to_dict()
    payload["counts"]["red"] = 9
    with pytest.raises(MissionValueError, match="counts disagree"):
        parse_task_plan_json(json.dumps(payload))


def test_jobs_expand_quantities_in_exact_model_order():
    """Map the PDF task order to deterministic Gazebo entity names."""
    jobs = expand_jobs(sample_plan())
    assert [
        (job.color, job.cube_name, job.target_zone)
        for job in jobs
    ] == [
        ("red", "red_cube_1", "A"),
        ("red", "red_cube_2", "A"),
        ("blue", "blue_cube_1", "C"),
    ]


def test_jobs_keep_blue_group_before_red_group_when_plan_says_so():
    """Shortest-path selection must not reorder the rule-defined groups."""
    plan = TaskPlan(
        task_id="blue-first",
        source="rules",
        items=(
            TaskItem("blue", 1, "C"),
            TaskItem("red", 2, "A"),
        ),
    )
    jobs = expand_jobs(plan)
    assert [(job.color, job.target_zone) for job in jobs] == [
        ("blue", "C"),
        ("red", "A"),
        ("red", "A"),
    ]


def test_selected_cube_snapshot_reports_runtime_choice_without_mutation():
    """Dashboard state names the chosen cube while preserving job order."""
    original = {"cube_name": "blue_cube_1", "color": "blue"}
    snapshot = selected_cube_snapshot(original, "blue_cube_3")
    assert snapshot["cube_name"] == "blue_cube_3"
    assert original["cube_name"] == "blue_cube_1"


def test_normal_flow_counts_only_verified_deliveries():
    """Complete all ordered stages and count only zone-verified cubes."""
    machine = MissionStateMachine(maximum_retries=1)
    first = machine.start(sample_plan())
    assert first.kind == "navigate"
    assert first.location == "RED"

    second_job = complete_job(machine)
    assert second_job.kind == "navigate"
    assert second_job.job.cube_name == "red_cube_2"
    complete_job(machine)
    final_job = complete_job(machine)

    assert final_job.kind == "done"
    assert machine.state == "completed"
    assert machine.completed_count == 3
    assert machine.skipped_count == 0
    assert machine.snapshot()["current_index"] == 3


def test_missing_target_retries_then_skips_without_reordering():
    """Retry perception, skip an unavailable cube, then keep plan order."""
    machine = MissionStateMachine(maximum_retries=1)
    machine.start(sample_plan())
    machine.navigation_finished(True)

    retry = machine.perception_finished(None)
    assert retry.kind == "perceive"
    next_job = machine.perception_finished(None)
    # A non-carrying skip first backs out to the tray-clear entry.
    assert next_job.kind == "navigate"
    assert next_job.location == "RED_EXIT"
    assert machine.skipped_count == 0
    advance = machine.navigation_finished(True)
    assert advance.job.cube_name == "red_cube_2"
    assert machine.skipped_count == 1


def test_failure_while_carrying_requires_release_before_advance():
    """Never advance to another cube while the prior cube may be attached."""
    machine = MissionStateMachine(maximum_retries=0)
    machine.start(sample_plan())
    machine.navigation_finished(True)
    target = {
        "color": "red",
        "position": {"x": 0.4, "y": 0.0, "z": 0.03},
    }
    machine.perception_finished(target)
    machine.manipulation_finished(True)

    release = machine.navigation_finished(False)
    assert release.kind == "release"
    assert machine.state == "releasing"
    next_job = machine.manipulation_finished(True)
    assert next_job.kind == "navigate"
    assert next_job.job.cube_name == "red_cube_2"
    assert machine.skipped_count == 1


def test_successful_pick_requires_safe_departure_before_destination():
    """Never route directly through the low pickup shelf after a grasp."""
    machine = MissionStateMachine(maximum_retries=1)
    machine.start(sample_plan())
    machine.navigation_finished(True)
    target = {
        "color": "red",
        "position": {"x": 0.4, "y": 0.0, "z": 0.03},
    }
    machine.perception_finished(target)

    departure = machine.manipulation_finished(True)
    assert departure.location == "RED_EXIT"
    assert machine.state == "navigating_departure"
    destination = machine.navigation_finished(True)
    assert destination.location == "A"
    assert machine.state == "navigating_destination"


def test_verified_delivery_exits_zone_before_next_pickup():
    """Never start the next route through the cube just placed ahead."""
    machine = MissionStateMachine(maximum_retries=1)
    machine.start(sample_plan())
    machine.navigation_finished(True)
    target = {
        "color": "red",
        "position": {"x": 0.4, "y": 0.0, "z": 0.03},
    }
    machine.perception_finished(target)
    machine.manipulation_finished(True)
    machine.navigation_finished(True)
    machine.navigation_finished(True)
    machine.manipulation_finished(True)

    zone_exit = machine.verification_finished(True)

    assert zone_exit.kind == "navigate"
    assert zone_exit.location == "A_EXIT"
    assert machine.state == "navigating_zone_exit"
    assert machine.completed_count == 0
    next_pickup = machine.navigation_finished(True)
    assert next_pickup.location == "RED"
    assert machine.completed_count == 1


def test_final_verified_delivery_completes_without_unused_zone_exit():
    """Do not navigate away after the last delivery is already verified."""
    plan = TaskPlan(
        task_id="final-delivery",
        source="rules",
        items=(TaskItem("blue", 1, "C"),),
    )
    machine = MissionStateMachine(maximum_retries=1)
    machine.start(plan)
    machine.navigation_finished(True)
    target = {
        "color": "blue",
        "position": {"x": 0.4, "y": 0.0, "z": 0.03},
    }
    machine.perception_finished(target)
    machine.manipulation_finished(True)
    machine.navigation_finished(True)
    machine.navigation_finished(True)
    machine.manipulation_finished(True)

    result = machine.verification_finished(True)

    assert result.kind == "done"
    assert machine.state == "completed"
    assert machine.completed_count == 1
    assert machine.skipped_count == 0
    assert machine.snapshot()["detail"] == "completed=1,skipped=0"


def test_all_skipped_jobs_end_as_failed():
    """Do not report mission success when no requested cube completed."""
    plan = TaskPlan(
        task_id="none",
        source="rules",
        items=(TaskItem("blue", 1, "B"),),
    )
    machine = MissionStateMachine(maximum_retries=0)
    machine.start(plan)
    machine.navigation_finished(True)
    result = machine.perception_finished(None)
    assert result.kind == "navigate"
    assert result.location == "BLUE_EXIT"
    result = machine.navigation_finished(True)
    assert result.kind == "done"
    assert machine.state == "failed"


def test_partial_completion_is_not_reported_as_completed():
    """Fail a round when any requested cube was skipped."""
    plan = TaskPlan(
        task_id="partial",
        source="rules",
        items=(TaskItem("blue", 2, "B"),),
    )
    machine = MissionStateMachine(maximum_retries=0)
    machine.start(plan)
    complete_job(machine)
    machine.navigation_finished(True)
    result = machine.perception_finished(None)
    assert result.kind == "navigate"
    assert result.location == "BLUE_EXIT"
    result = machine.navigation_finished(True)

    assert result.kind == "done"
    assert machine.completed_count == 1
    assert machine.skipped_count == 1
    assert machine.state == "failed"


def test_failed_pick_rescans_instead_of_repeating_stale_pose():
    """A grasp retry must use a new RGB-D observation."""
    machine = MissionStateMachine(maximum_retries=1)
    machine.start(sample_plan())
    machine.navigation_finished(True)
    target = {
        "color": "red",
        "position": {"x": 0.32, "y": 0.0, "z": 0.03},
    }
    machine.perception_finished(target)

    retry = machine.manipulation_finished(False)

    assert retry.kind == "perceive"
    assert machine.state == "waiting_perception"
    assert machine.selected_target is None
    assert machine.detail == "rescanning_after_pick_failure"


def test_pick_failure_switches_cube_after_safe_departure_without_skip():
    """Keep the ordered item while leaving its failed cube dock safely."""
    machine = MissionStateMachine(maximum_retries=1)
    machine.start(sample_plan())
    machine.navigation_finished(True)
    target = {
        "color": "red",
        "position": {"x": 0.32, "y": 0.0, "z": 0.03},
    }
    machine.perception_finished(target)

    departure = machine.pick_failed_switch_cube()
    assert departure.kind == "navigate"
    assert departure.location == "RED_EXIT"
    assert machine.skipped_count == 0
    assert machine.current_job.cube_name == "red_cube_1"

    retry_same_item = machine.navigation_finished(True)
    assert retry_same_item.kind == "navigate"
    assert retry_same_item.location == "RED"
    assert retry_same_item.job.cube_name == "red_cube_1"
    assert machine.skipped_count == 0


def test_perception_can_have_a_larger_recovery_budget_than_motion():
    """A transient empty RGB-D frame must not consume a physical retry."""
    machine = MissionStateMachine(
        maximum_retries=1,
        maximum_perception_retries=2,
    )
    machine.start(sample_plan())
    machine.navigation_finished(True)

    first_retry = machine.perception_finished(None)
    second_retry = machine.perception_finished(None)
    next_job = machine.perception_finished(None)

    assert first_retry.kind == "perceive"
    assert second_retry.kind == "perceive"
    assert next_job.kind == "navigate"
    assert next_job.location == "RED_EXIT"
    assert machine.skipped_count == 0
    advance = machine.navigation_finished(True)
    assert advance.job.cube_name == "red_cube_2"
    assert machine.skipped_count == 1


def test_cancel_during_arm_motion_requests_safety_release():
    """Require a release before cancel is terminal while the arm moves."""
    machine = MissionStateMachine(maximum_retries=0)
    machine.start(sample_plan())
    machine.navigation_finished(True)
    target = {
        "color": "red",
        "position": {"x": 0.4, "y": 0.0, "z": 0.03},
    }
    machine.perception_finished(target)
    # perception_finished leaves the machine in "picking": the arm is inside a
    # motion and may be in a pose that is unsafe to hold.
    assert machine.state == "picking"
    action = machine.cancel()
    assert action.kind == "release"
    assert machine.state == "releasing"
    terminal = machine.manipulation_finished(True)
    assert terminal.kind == "done"
    assert machine.state == "canceled"


def test_cancel_in_transit_keeps_holding_the_carried_cube():
    """
    A deadline cancel mid-transport must not drop the cube on the floor.

    Measured: round 4 of a five-cube run grasped blue_cube_5, set off for zone
    C, hit the five-minute deadline, and released the cube in transit. Grasping
    scores 4 points per item independently of placement, so a cube held in the
    validated carry pose is evidence of a successful grasp while the same cube
    dropped at an arbitrary floor position is evidence of nothing.
    """
    machine = MissionStateMachine(maximum_retries=0)
    machine.start(sample_plan())
    machine.navigation_finished(True)
    target = {
        "color": "red",
        "position": {"x": 0.4, "y": 0.0, "z": 0.03},
    }
    machine.perception_finished(target)
    machine.manipulation_finished(True)
    # The cube is now secured in the transport pose on a navigation leg.
    assert machine.state == "navigating_departure"
    action = machine.cancel()
    assert action.kind == "done"
    assert action.reason == "mission_canceled_holding_cube"
    assert machine.state == "canceled"
    assert machine.detail == "mission_canceled_holding_cube"


def test_cancel_before_any_grasp_is_immediately_terminal():
    """With nothing carried there is no release to wait for."""
    machine = MissionStateMachine(maximum_retries=0)
    machine.start(sample_plan())
    assert machine.state == "navigating_pickup"
    action = machine.cancel()
    assert action.kind == "done"
    assert action.reason == "mission_canceled"
    assert machine.state == "canceled"


def test_exhausted_pickup_inventory_skips_without_departure_or_reuse():
    """No remaining shelf slot advances the item without revisiting a cube."""
    machine = MissionStateMachine(maximum_retries=1)
    machine.start(sample_plan())

    action = machine.pickup_inventory_exhausted()

    assert action.kind == "navigate"
    assert action.location == "RED"
    assert machine.skipped_count == 1
    assert machine.job_offset == 1
    assert machine.state == "navigating_pickup"
    assert machine.detail == "going_to_red_pickup"


def test_exhausted_pickup_inventory_requires_pickup_state():
    """Inventory exhaustion cannot silently advance a different phase."""
    machine = MissionStateMachine(maximum_retries=1)
    machine.start(sample_plan())
    machine.navigation_finished(True)

    with pytest.raises(MissionValueError, match="state=waiting_perception"):
        machine.pickup_inventory_exhausted()


def test_target_selection_is_reachable_color_filtered_and_center_first():
    """Choose the reachable target nearest the calibrated shelf centerline."""
    targets = [
        {"color": "red", "position": {"x": 0.4, "y": 0.08, "z": 0.03}},
        {"color": "blue", "position": {"x": 0.4, "y": -0.12, "z": 0.03}},
        {"color": "red", "position": {"x": 0.41, "y": -0.06, "z": 0.03}},
        {"color": "red", "position": {"x": 0.8, "y": -0.20, "z": 0.03}},
    ]
    selected = select_reachable_target(
        targets,
        color="red",
        minimum_x=0.25,
        maximum_x=0.55,
        maximum_abs_y=0.20,
        preferred_y=0.04,
    )
    assert selected["position"]["y"] == pytest.approx(0.08)


def test_target_selection_uses_expected_cube_slot_and_rejects_below_floor():
    """Pick the detection matching the known slot, never a neighbour cube."""
    targets = [
        {"color": "red", "position": {"x": 0.36, "y": -0.11, "z": 0.026}},
        {"color": "red", "position": {"x": 0.36, "y": 0.24, "z": 0.026}},
        {"color": "red", "position": {"x": 0.36, "y": 0.02, "z": -0.02}},
    ]
    selected = select_reachable_target(
        targets,
        color="red",
        minimum_x=0.25,
        maximum_x=0.55,
        maximum_abs_y=0.30,
        expected_position=(0.36, -0.12),
        expected_y_tolerance=0.08,
    )
    assert selected["position"]["y"] == pytest.approx(-0.11)
    assert selected["position"]["z"] >= 0.005


def test_target_selection_rejects_far_from_expected_slot():
    """No detection near the expected slot yields no target (forces rescan)."""
    targets = [
        {"color": "red", "position": {"x": 0.36, "y": 0.24, "z": 0.026}},
    ]
    selected = select_reachable_target(
        targets,
        color="red",
        minimum_x=0.25,
        maximum_x=0.55,
        maximum_abs_y=0.30,
        expected_position=(0.36, -0.12),
        expected_y_tolerance=0.08,
    )
    assert selected is None


def test_target_selection_accepts_v2_floor_cube_with_explicit_minimum_z():
    """The v2 floor inventory is below the mobile base_link origin."""
    targets = [
        {"color": "red", "position": {"x": 0.32, "y": 0.02, "z": -0.075}},
    ]
    selected = select_reachable_target(
        targets,
        color="red",
        minimum_x=0.18,
        maximum_x=0.34,
        maximum_abs_y=0.30,
        minimum_z=-0.12,
        expected_position=(0.32, 0.02),
    )
    assert selected is targets[0]


def test_target_selection_can_follow_inventory_lateral_slot_order():
    """Use the known alternating shelf slots after the center cube."""
    targets = [
        {"color": "red", "position": {"x": 0.4, "y": -0.12, "z": 0.03}},
        {"color": "red", "position": {"x": 0.4, "y": 0.12, "z": 0.03}},
    ]
    selected = select_reachable_target(
        targets,
        color="red",
        minimum_x=0.25,
        maximum_x=0.55,
        maximum_abs_y=0.20,
        preferred_y=0.0,
        preferred_lateral_sign=1,
    )
    assert selected["position"]["y"] == pytest.approx(0.12)
