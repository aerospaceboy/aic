"""Verify V2 corridor guides and canceled-control lifetime."""

from types import SimpleNamespace
from unittest.mock import Mock

from tools_demo.navigation_math import dynamic_gate_waypoints
from tools_demo.navigation_math import layout_a_sweep_waypoints
from tools_demo.task_navigator import TaskNavigator


def test_fresh_command_releases_the_old_failed_hold():
    """A new command cannot stay stopped by a previous timeout's latch."""
    node = SimpleNamespace(_direct_control=True, _release_direct_control=Mock())
    TaskNavigator._reset_mover_control_for_command(node)
    assert not node.mover_hold_blocked and node.mover_hold_topic is None
    assert node.obstacle_reverse_state == "none"
    assert not node.obstacle_reverse_active
    node._release_direct_control.assert_called_once()


def test_canceled_leg_remains_stopped_until_new_command():
    """The fresh-command reset belongs to the validated command boundary."""
    import inspect
    source = inspect.getsource(TaskNavigator.command_callback)
    assert source.index("if self.active_zone is not None") < source.index(
        "self._reset_mover_control_for_command()")
    assert source.index("if not self._global_costmap_ready()") < source.index(
        "self._reset_mover_control_for_command()")


def test_both_patrol_anchors_keep_the_wider_wall_clear_lane():
    """The second patrol must not drift back to the unsafe y=0.1 line."""
    route = dynamic_gate_waypoints((13.2, -.95), (-6.0, 3.4))
    assert [(p.x, p.y) for p in route[2:5]] == [
        (9.825, .525), (6.65, .55), (3.8, .55)]


def test_a_sweep_guides_avoid_the_alcove_and_wall_band():
    """The original east pickup receives a channel entry and west exit."""
    route = layout_a_sweep_waypoints(-5.6, 3.4)
    assert [(p.x, p.y) for p in route] == [(-5.55, 2.5), (-7.6, 2.5)]


def test_rescue_near_west_tip_does_not_backtrack_to_east_entry():
    """A stalled vehicle near the west end heads out of the bottleneck."""
    route = layout_a_sweep_waypoints(-7.0, 2.4)
    assert [(p.x, p.y) for p in route] == [(-7.6, 2.5)]


def test_a_guides_do_not_intercept_unrelated_rooms():
    """Other regions retain global navigation without stale-layout guides."""
    assert not layout_a_sweep_waypoints(6.4, -3.8)
    assert not layout_a_sweep_waypoints(-11.0, -5.6)


def test_same_side_waiting_goal_cannot_start_a_crossing():
    """The crossing controller must never overshoot its own wait anchor."""
    node = SimpleNamespace(
        _corridor_mover_topics=lambda: ["mover"],
        _mover_base_frame=lambda topic: (-1.7, -.4, 1.7, .4, -1, 0, 1.5),
        nav_goal_pose=SimpleNamespace(x=9.8), latest_x=10.2,
        obstacle_positions={"mover": (8.5, -.35)},
        mover_admitted=set())
    assert TaskNavigator._mover_yield_topic(node) is None


def test_progress_watchdog_does_not_cancel_controlled_wait():
    """An intentional wait uses its own deadline, not the Nav2 watchdog."""
    node = SimpleNamespace(
        pending_start_command=None, obstacle_reverse_active=True,
        goal_handle=Mock(), goal_start_ns=0)
    TaskNavigator.timeout_callback(node)
    node.goal_handle.cancel_goal_async.assert_not_called()


def test_failed_base_cleanup_never_starts_departure():
    """Replay the failed release result observed in candidate2-A."""
    from tools_demo.mission_coordinator import MissionCoordinator
    node = SimpleNamespace(
        publish_error=Mock(), machine=SimpleNamespace(state="picking"),
        handle_action=Mock())
    assert MissionCoordinator._reject_unsafe_manipulation_departure(
        node, {"success": False, "base_released": False}, "pick")
    assert node.machine.state == "failed"
    assert node.handle_action.call_args.args[0].kind == "done"


def test_failed_attached_pick_cannot_switch_to_another_cube():
    """A failed detach leaves a load and cannot be treated as empty."""
    from tools_demo.mission_coordinator import MissionCoordinator
    node = SimpleNamespace(
        publish_error=Mock(), machine=SimpleNamespace(state="picking"),
        handle_action=Mock())
    assert MissionCoordinator._reject_unsafe_manipulation_departure(
        node, {"success": False, "base_released": True,
               "cube_attached": True}, "pick")


def test_successful_loaded_pick_can_start_transport():
    """An attached cube is expected after a successful pick."""
    from tools_demo.mission_coordinator import MissionCoordinator
    node = SimpleNamespace(publish_error=Mock(), handle_action=Mock())
    assert not MissionCoordinator._reject_unsafe_manipulation_departure(
        node, {"success": True, "base_released": True,
               "cube_attached": True}, "pick")


def test_east_delivery_clears_the_partition_corner_before_heading_west():
    """The red2-to-A route must not skim wall1's moving east corner."""
    route = layout_a_sweep_waypoints(6.9, 3.8)
    assert [(p.x, p.y) for p in route] == [
        (-3.2, 2.7), (-5.55, 2.5), (-7.6, 2.5)]


def test_red1_departure_above_channel_must_descend_before_west_tip():
    """A pickup near x=-6.1 above the alcove cannot cut diagonally west."""
    route = layout_a_sweep_waypoints(-6.2, 3.4)
    assert route[0].zone == "A_SWEEP_ENTRY"
