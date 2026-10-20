"""Unit tests for the competition dashboard state model."""

import json

from tools_demo.dashboard_state import DashboardState


def test_dashboard_preserves_last_valid_json_snapshot():
    state = DashboardState()

    assert state.update_json("mission", '{"state":"picking"}')
    assert not state.update_json("mission", "{invalid")
    assert state.snapshot()["mission"] == {"state": "picking"}


def test_dashboard_path_downsampling_preserves_endpoints():
    state = DashboardState(maximum_path_points=4)

    state.update_path((index, index * 2) for index in range(10))
    path = state.snapshot()["path"]

    assert len(path) == 4
    assert path[0] == [0.0, 0.0]
    assert path[-1] == [9.0, 18.0]


def test_dashboard_snapshot_is_isolated_from_callers():
    state = DashboardState()
    state.update_json("plan", '{"counts":{"red":2,"blue":1}}')

    snapshot = state.snapshot()
    snapshot["plan"]["counts"]["red"] = 99

    assert state.snapshot()["plan"]["counts"]["red"] == 2


def test_dashboard_robot_pose_includes_planar_yaw():
    state = DashboardState()

    state.update_robot(1.25, -0.5, (0.0, 0.0, 0.0, 1.0))

    assert state.snapshot()["robot"] == {
        "x": 1.25,
        "y": -0.5,
        "yaw": 0.0,
    }


def test_dashboard_path_starts_at_latest_robot_pose():
    state = DashboardState()
    state.update_path(
        [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0)]
    )

    state.update_robot(0.8, 0.1, (0.0, 0.0, 0.0, 1.0))
    path = state.snapshot()["path"]

    assert path[0] == [0.8, 0.1]
    assert path[1] == [1.0, 0.0]
    assert path[-1] == [1.5, 0.0]


def test_dashboard_never_stores_robot_history():
    state = DashboardState()
    state.update_robot(0.0, 0.0, (0.0, 0.0, 0.0, 1.0))
    state.update_robot(0.6, 0.0, (0.0, 0.0, 0.0, 1.0))

    assert "trail" not in state.snapshot()


def test_dashboard_resets_routes_when_a_new_task_starts():
    state = DashboardState()
    state.update_path([(0.0, 0.0), (1.0, 1.0)])
    state.update_path([(0.0, 0.0), (0.2, 0.1)], key="local_path")
    state.update_robot(0.5, 0.5, (0.0, 0.0, 0.0, 1.0))

    assert state.update_json("mission", '{"task_id":"new-task"}')
    snapshot = state.snapshot()

    assert snapshot["path"] == []
    assert snapshot["local_path"] == []
    assert "trail" not in snapshot


def test_dashboard_records_local_path_freshness():
    state = DashboardState(maximum_path_points=3)

    state.update_path(
        [(0.0, 0.0), (0.5, 0.2), (1.0, 0.3)],
        key="local_path",
    )
    snapshot = state.snapshot()

    assert snapshot["local_path"][-1] == [1.0, 0.3]
    assert snapshot["local_path_updated_at"] > 0.0


def test_dashboard_anchors_local_path_at_latest_robot_pose():
    state = DashboardState()
    state.update_path([(0.0, 0.0), (1.0, 0.0)], key="local_path")

    state.update_robot(0.4, 0.1, (0.0, 0.0, 0.0, 1.0))

    assert state.snapshot()["local_path"][0] == [0.4, 0.1]


def test_dashboard_rejects_late_path_after_navigation_finishes():
    state = DashboardState()
    state.update_json("navigation", '{"state":"active"}')
    state.update_path([(0.0, 0.0), (1.0, 0.0)])
    assert state.snapshot()["path"]

    state.update_json("navigation", '{"state":"succeeded"}')
    state.update_path([(0.0, 0.0), (1.0, 0.0)])

    assert state.snapshot()["path"] == []


def test_dashboard_stores_only_latest_direct_obstacle_truth():
    state = DashboardState()
    assert state.update_obstacle(
        '{"success":true,"model":"obstacle_1","x":1.0,"y":2.0}'
    )
    assert state.update_obstacle(
        '{"success":true,"model":"obstacle_1","x":1.2,"y":2.3}'
    )

    snapshot = state.snapshot()
    assert snapshot["obstacles"] == {"obstacle_1": [1.2, 2.3]}
    assert snapshot["obstacles_updated_at"] > 0.0


def test_dashboard_clears_stale_routes_between_navigation_legs():
    state = DashboardState()
    state.update_path([(0.0, 0.0), (1.0, 0.0)])
    state.update_path([(0.0, 0.0), (0.5, 0.0)], key="local_path")

    assert state.update_json("navigation", '{"state":"sending"}')
    assert state.snapshot()["path"] == []
    assert state.snapshot()["local_path"] == []


def _mission(index, completed=0, skipped=0, state="picking", **extra):
    """Build one mission payload the way the coordinator publishes it."""
    payload = {
        "schema_version": "1.0",
        "task_id": "t1",
        "current_index": index,
        "completed": completed,
        "skipped": skipped,
        "state": state,
        "color": "red",
        "cube_name": f"red_cube_{index}",
        "target_zone": "A",
        "detail": "picking",
        "elapsed_sec": 10.0 * index,
    }
    payload.update(extra)
    return json.dumps(payload)


def test_rounds_record_one_entry_per_mission_round():
    """The 2026 panel must show each round's result, not just totals."""
    state = DashboardState()
    state.update_json("mission", _mission(1))
    rounds = state.snapshot()["rounds"]
    assert len(rounds) == 1
    assert rounds[0]["index"] == 1
    assert rounds[0]["result"] == "active"
    assert rounds[0]["cube_name"] == "red_cube_1"

    # Round 2 starting means round 1 finished; completed advanced, so it
    # completed.
    state.update_json("mission", _mission(2, completed=1))
    rounds = state.snapshot()["rounds"]
    assert len(rounds) == 2
    assert rounds[0]["result"] == "completed"
    assert rounds[1]["result"] == "active"


def test_rounds_distinguish_skipped_from_completed():
    """A skipped round must not be reported as completed."""
    state = DashboardState()
    state.update_json("mission", _mission(1))
    state.update_json("mission", _mission(2, skipped=1))
    rounds = state.snapshot()["rounds"]
    assert rounds[0]["result"] == "skipped"

    state.update_json("mission", _mission(3, completed=1, skipped=1))
    rounds = state.snapshot()["rounds"]
    assert rounds[1]["result"] == "completed"


def test_rounds_close_the_final_round_on_a_terminal_state():
    """The last round has no successor to close it, so state must."""
    state = DashboardState()
    state.update_json("mission", _mission(1))
    state.update_json(
        "mission", _mission(1, completed=1, state="completed")
    )
    rounds = state.snapshot()["rounds"]
    assert len(rounds) == 1
    assert rounds[0]["result"] == "completed"


def test_rounds_mark_a_cancelled_final_round_as_cancelled():
    """A round cut off by the five-minute deadline is not a completion."""
    state = DashboardState()
    state.update_json("mission", _mission(1, completed=0))
    state.update_json("mission", _mission(2, completed=1))
    state.update_json(
        "mission", _mission(2, completed=1, state="canceled")
    )
    rounds = state.snapshot()["rounds"]
    assert rounds[0]["result"] == "completed"
    assert rounds[1]["result"] == "canceled"


def test_rounds_reset_when_a_new_task_starts():
    """A new task must not inherit the previous task's round log."""
    state = DashboardState()
    state.update_json("mission", _mission(1, completed=0))
    state.update_json("mission", _mission(2, completed=1))
    assert len(state.snapshot()["rounds"]) == 2

    fresh = json.loads(_mission(1))
    fresh["task_id"] = "t2"
    state.update_json("mission", json.dumps(fresh))
    rounds = state.snapshot()["rounds"]
    assert len(rounds) == 1
    assert rounds[0]["index"] == 1
    assert rounds[0]["result"] == "active"


def test_rounds_keep_live_fields_fresh_within_one_round():
    """Progress within a round must be visible before it closes."""
    state = DashboardState()
    state.update_json("mission", _mission(1, detail="recognizing_red"))
    state.update_json("mission", _mission(1, detail="going_to_zone_A"))
    rounds = state.snapshot()["rounds"]
    assert len(rounds) == 1
    assert rounds[0]["detail"] == "going_to_zone_A"
    assert rounds[0]["result"] == "active"
