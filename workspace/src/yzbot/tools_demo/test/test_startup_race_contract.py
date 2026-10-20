"""Contracts for cold-start mission timing and speed profiling."""

from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]


def test_mission_waits_for_first_odom_before_selecting_jobs():
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "mission_coordinator.py"
    ).read_text(encoding="utf-8")
    assert "self.odom_pose = None" in source
    assert "self.pending_plan_message = message" in source
    assert "mission_start_waiting_for_odom" in source
    assert "mission_start_odom_ready" in source
    assert "self._odom_pose() is None" in source


def test_speed_profile_uses_live_state_and_matching_terminal_task():
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "speed_profile_recorder.py"
    ).read_text(encoding="utf-8")
    subscription = source.split('"/competition/mission/state"', 1)[1].split(
        ")", 1
    )[0]
    assert "live_qos(" in subscription
    assert "state not in TERMINAL_MISSION_STATES" in source


def test_manual_costudio_mode_defaults_to_gazebo_and_rviz():
    """Operator one-key start opens Gazebo GUI and RViz by default."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "scripts"
        / "start_manual_problem_mode.sh"
    ).read_text(encoding="utf-8")
    assert 'headless:="${COSTUDIO_HEADLESS:-False}"' in source
    assert 'debug_gui:="${COSTUDIO_DEBUG_GUI:-True}"' in source
    assert 'dashboard:="${COSTUDIO_DASHBOARD:-False}"' in source
    assert 'mybot/pick_place_moveit.*competition_pick_place' in source
    assert 'flock -n 9' in source
    # Every background launch must release the start lock (coStudio bridge,
    # competition system, early RViz, and optional fallback RViz).
    assert source.count('9>&-') >= 3  # bridge, launch, early/fallback rviz
    # Gazebo client and RViz both start by default (demo/record path).
    assert 'start_rviz="${COSTUDIO_RVIZ:-True}"' in source
    # RViz must start with the stack, not after the readiness gate.
    assert "提前启动导航视图" in source
    campaign = (
        REPOSITORY / "tools_demo" / "scripts" / "run_zone9_campaign.py"
    ).read_text(encoding="utf-8")
    # Scoring stays headless even though the one-key default is GUI+RViz.
    assert 'env.setdefault("COSTUDIO_HEADLESS", "True")' in campaign
    assert 'env.setdefault("COSTUDIO_RVIZ", "False")' in campaign
