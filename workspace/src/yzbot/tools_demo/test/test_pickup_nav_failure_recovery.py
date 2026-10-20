"""Fault injection: a failed pickup approach must not run the tray departure.

The departure (RED_EXIT) assumes the base is parked at the per-cube dock
inside the tray's costmap inflation. When the pickup NAVIGATION itself
failed, the base never reached the dock; running the departure from a
mid-route position drives the direct controller toward the tray-clear
entry - i.e. toward the tray - and stalls until timeout (observed in the
1bA_4rB followup failure: aborted inbound at (-4.52,-1.26), then RED_EXIT
32 s timeouts twice, then mission failure).
"""

from tools_demo.mission_state_machine import MissionStateMachine
from tools_demo.task_protocol import TaskItem
from tools_demo.task_protocol import TaskPlan


def make_machine():
    machine = MissionStateMachine()
    machine.start(
        TaskPlan(
            task_id="navfail",
            source="rules",
            items=(
                TaskItem("red", 2, "A"),
                TaskItem("blue", 1, "C"),
            ),
        )
    )
    return machine


def test_single_pickup_nav_failure_retries_the_approach():
    machine = make_machine()
    retry = machine.navigation_finished(False)
    assert retry.kind == "navigate"
    assert retry.location == "RED"


def test_double_pickup_nav_failure_advances_without_departure():
    machine = make_machine()
    assert machine.navigation_finished(False).kind == "navigate"
    recovery = machine.navigation_finished(False)
    assert recovery.kind == "navigate"
    # The base never docked: the next leg is the next job's pickup
    # approach, never the tray departure.
    assert recovery.location != "RED_EXIT"
    assert machine.current_job.color == "red"
    assert machine.job_offset == 1
    assert machine.state == "navigating_pickup"
