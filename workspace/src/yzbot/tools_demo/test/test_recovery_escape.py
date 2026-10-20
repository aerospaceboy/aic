"""Regression checks for bounded-recovery escape selection.

Baseline evidence (all campaign logs, 2026-09-17..19): 13 bounded recoveries
were rejected and only 2 started; 10 of the 13 rejections happened while the
robot's own cell read cost >= 50 (99 x5, 83, 71 x2, 68 x2) because the segment
probe skipped only 6 cm of the robot's own inflation. Two rounds (1rC_4bA,
2bA_3rC in policy_20260919T023140) aborted their pickup legs from such a cell.
"""

import importlib.util
import math
import pathlib
from types import MethodType, SimpleNamespace

MODULE_PATH = (
    pathlib.Path(__file__).resolve().parents[1]
    / "tools_demo"
    / "task_navigator.py"
)
SPEC = importlib.util.spec_from_file_location("task_navigator", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeCostmap:
    """100x100 cells at 5 cm, origin (-5, -5), all free unless blocked."""

    def __init__(self, blocked):
        self.info = SimpleNamespace(
            width=200,
            height=200,
            resolution=0.05,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-5.0, y=-5.0)
            ),
        )
        self.data = [0] * (self.info.width * self.info.height)
        for x_value, y_value, cost in blocked:
            px = int((x_value + 5.0) / self.info.resolution)
            py = int((y_value + 5.0) / self.info.resolution)
            if 0 <= px < self.info.width and 0 <= py < self.info.height:
                self.data[py * self.info.width + px] = cost


class FakeNavigator:
    def __init__(self, blocked, x=0.0, y=0.0, yaw=0.0):
        self.global_costmap = FakeCostmap(blocked)
        self.latest_x = x
        self.latest_y = y
        self.latest_yaw = yaw
        self.recovery_attempts = 0
        self.max_recovery_attempts = 2
        self.reverse_max_distance = 0.5
        self.reverse_speed = 0.15
        self.risk_zones = {}
        self.nav_batch_poses = []
        self.nav_goal_pose = MODULE.ZonePose("ZONE", 1.0, 0.0, 0.0)
        self.pending_poses = []
        self.dock_pose = None
        self.next_dock_pose = None
        self.final_yaw = None
        self.alignment_target_yaw = None
        self.alignment_timeout_ns = 1
        self.dock_arrival_tolerance = 0.05
        self.dock_max_speed = 0.2
        self.recovery_target = None
        self.recovery_pending_poses = []
        self.recovery_dock_pose = None
        self.recovery_next_dock_pose = None
        self.recovery_final_yaw = None
        self.events = []
        self.clock = SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=0)
        )
        self.get_clock = lambda: self.clock
        self._global_cost_at = MethodType(
            MODULE.TaskNavigator._global_cost_at, self
        )
        self._costmap_cell_free = MethodType(
            MODULE.TaskNavigator._costmap_cell_free, self
        )
        self._costmap_segment_clear = MethodType(
            MODULE.TaskNavigator._costmap_segment_clear, self
        )
        self._hazard_zone_for = MethodType(
            MODULE.TaskNavigator._hazard_zone_for, self
        )
        self._escape_waypoint = MethodType(
            MODULE.TaskNavigator._escape_waypoint, self
        )
        # Bound so the fake exercises the real mover-lane escape guard.
        self._recovery_segment_clear_of_movers = MethodType(
            MODULE.TaskNavigator._recovery_segment_clear_of_movers, self
        )
        self._try_bounded_recovery = MethodType(
            MODULE.TaskNavigator._try_bounded_recovery, self
        )
        self._inflation_escape_candidate = MethodType(
            MODULE.TaskNavigator._inflation_escape_candidate, self
        )

    def get_logger(self):
        return SimpleNamespace(
            info=lambda message: self.events.append(("info", message)),
            warn=lambda message: self.events.append(("warn", message)),
            error=lambda message: self.events.append(("error", message)),
        )

    def publish_state(self, state, **kwargs):
        self.events.append(("state", state))

    def messages(self, marker):
        return [text for _level, text in self.events if marker in str(text)]


def block_disc(cx, cy, count, cost=99):
    """Return (x, y, cost) triples for a small lethal disc."""
    cells = []
    step = 0.05
    span = int(count ** 0.5) + 1
    for ix in range(-span, span + 1):
        for iy in range(-span, span + 1):
            x_value = cx + ix * step
            y_value = cy + iy * step
            if math.hypot(x_value - cx, y_value - cy) <= count * step:
                cells.append((x_value, y_value, cost))
    return cells


def block_annulus(cx, cy, r_inner, r_outer, cost=99):
    """Return a filled lethal annulus, sampled on the 5 cm costmap grid."""
    cells = []
    step = 0.025
    span = int(r_outer / step) + 1
    for ix in range(-span, span + 1):
        for iy in range(-span, span + 1):
            x_value = cx + ix * step
            y_value = cy + iy * step
            radius = math.hypot(x_value - cx, y_value - cy)
            if r_inner <= radius <= r_outer:
                cells.append((x_value, y_value, cost))
    return cells


def test_segment_check_skips_only_the_robot_own_cells():
    """A lethal patch under the base must not veto the escape segment."""
    nav = FakeNavigator(block_disc(0.0, 0.0, 5))
    target = MODULE.ZonePose("ESCAPE", -0.5, 0.0, 0.0)
    strict = MODULE.TaskNavigator._costmap_segment_clear(
        nav, 0.0, 0.0, target
    )
    relaxed = MODULE.TaskNavigator._costmap_segment_clear(
        nav, 0.0, 0.0, target, MODULE.ESCAPE_SELF_CLEARANCE
    )
    assert strict is False, "the robot's own cell still blocks a strict check"
    assert relaxed is True, "cells inside the footprint must be skipped"


def test_segment_check_still_requires_free_space_beyond_the_footprint():
    """Skipping the own cells must not accept a blocked far segment."""
    nav = FakeNavigator(block_disc(-0.45, 0.0, 2))
    target = MODULE.ZonePose("ESCAPE", -0.5, 0.0, 0.0)
    assert MODULE.TaskNavigator._costmap_segment_clear(
        nav, 0.0, 0.0, target, MODULE.ESCAPE_SELF_CLEARANCE
    ) is False


def test_recovery_uses_a_bounded_alternative_heading():
    """A blocked reverse corridor must not abort the recovery outright."""
    blocked = block_disc(0.0, 0.0, 5) + block_disc(0.0, 0.30, 4)
    nav = FakeNavigator(blocked)
    assert nav._try_bounded_recovery() is True
    escape = nav.recovery_target
    distance = math.hypot(escape.x - nav.latest_x, escape.y - nav.latest_y)
    assert 0.0 < distance <= nav.reverse_max_distance + 1e-6
    assert nav._costmap_cell_free(escape.x, escape.y) is True
    assert nav.messages("recovery_candidate_heading")


def test_recovery_refuses_when_no_bounded_escape_is_clear():
    """A fully enclosed robot must be reported, not guessed at."""
    # Everything beyond the robot's own footprint is lethal, so no bounded
    # escape exists and the recovery must refuse rather than guess.
    nav = FakeNavigator(block_annulus(0.0, 0.0, 0.20, 0.60))
    assert nav._try_bounded_recovery() is False
    assert nav.messages("Bounded recovery rejected")
    assert nav.recovery_attempts == 0


def test_recovery_distance_never_exceeds_the_existing_limit():
    """Every candidate stays inside reverse_max_distance."""
    blocked = block_disc(0.0, 0.0, 5) + block_disc(0.0, 0.25, 3)
    nav = FakeNavigator(blocked)
    nav.reverse_max_distance = 0.35
    assert nav._try_bounded_recovery() is True
    distance = math.hypot(
        nav.recovery_target.x - nav.latest_x,
        nav.recovery_target.y - nav.latest_y,
    )
    assert distance <= nav.reverse_max_distance + 1e-6


def test_recovery_ignores_a_high_cost_endpoint():
    """A candidate whose endpoint is inflated must not be chosen."""
    # The robot's own cell is lethal and so is every reachable endpoint.
    nav = FakeNavigator(
        block_disc(0.0, 0.0, 5) + block_annulus(0.0, 0.0, 0.20, 0.60)
    )
    assert nav._try_bounded_recovery() is False


def test_recovery_stays_inside_the_historical_distance_cap():
    """A larger reverse_max_distance must not widen the escape budget.

    Phase-7 round 4bB_1rA logged recovery_candidate_heading distance=1.50m
    when the raw parameter was used instead of the historical
    min(0.5, reverse_max_distance) cap; that batch was discarded.
    """
    blocked = block_disc(0.0, 0.0, 5) + block_disc(0.0, 0.25, 3)
    nav = FakeNavigator(blocked)
    nav.reverse_max_distance = 1.5
    assert nav._try_bounded_recovery() is True
    distance = math.hypot(
        nav.recovery_target.x - nav.latest_x,
        nav.recovery_target.y - nav.latest_y,
    )
    assert distance <= MODULE.ESCAPE_MAX_DISTANCE + 1e-6
    assert MODULE.ESCAPE_MAX_DISTANCE == 0.5


def _mover_yield_harness(now_ns, involved_ns, used=0, retries=3):
    events = []
    nav = SimpleNamespace(
        mover_involved_ns=involved_ns,
        nav_goal_pose=object(),
        mover_involved_horizon_ns=8_000_000_000,
        mover_yield_retries=retries,
        mover_yield_retries_used=used,
        mover_yield_wait_ns=8_000_000_000,
        mover_hold_topic="/competition/obstacles/obstacle_2",
        obstacle_reverse_active=False,
        obstacle_reverse_state="none",
        obstacle_reverse_wait_deadline_ns=None,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=now_ns)),
        get_logger=lambda: SimpleNamespace(
            warn=lambda message: events.append(("warn", message)),
            info=lambda message: events.append(("info", message))),
        _publish_obstacle_velocity=lambda value: events.append(
            ("velocity", value)),
        _lethal_start_cost=lambda: None,
    )
    return nav, events


def test_mover_yield_retry_holds_the_leg_instead_of_escaping():
    """A crossing mover must be waited out, not escaped away from the goal.

    Evidence 20261013T174353: bounded escapes #1/#2 consumed inside 7 s while
    obstacle_2 still held the lane, then pickup_navigation_failed.
    """
    now = 3_000_000_000
    nav, events = _mover_yield_harness(now, involved_ns=1_000_000_000)

    assert MODULE.TaskNavigator._try_mover_yield_retry(nav) is True

    assert nav.obstacle_reverse_state == "waiting"
    assert nav.obstacle_reverse_active is True
    assert nav.mover_hold_topic is None  # wait step, not the hold step
    assert nav.obstacle_reverse_wait_deadline_ns == now + 8_000_000_000
    assert nav.mover_yield_retries_used == 1
    assert ("velocity", 0.0) in events
    assert any("mover_yield_retry 1/3" in str(text) for _lvl, text in events)


def test_mover_yield_retry_gates_on_recency_and_budget():
    now = 20_000_000_000

    stale, _ = _mover_yield_harness(now, involved_ns=1_000_000_000)
    assert MODULE.TaskNavigator._try_mover_yield_retry(stale) is False

    none_seen, _ = _mover_yield_harness(now, involved_ns=None)
    assert MODULE.TaskNavigator._try_mover_yield_retry(none_seen) is False

    exhausted, _ = _mover_yield_harness(
        now, involved_ns=now - 1_000_000_000, used=3)
    assert MODULE.TaskNavigator._try_mover_yield_retry(exhausted) is False

    no_goal, _ = _mover_yield_harness(now, involved_ns=now - 1_000_000_000)
    no_goal.nav_goal_pose = None
    assert MODULE.TaskNavigator._try_mover_yield_retry(no_goal) is False


def test_mover_yield_retry_also_covers_a_mover_pinning_the_base():
    """A mover overlapping the base makes the robot's own cell lethal.

    Evidence 20261013T181749: planner_server refused to plan from the robot's
    cell ("Starting point in lethal space", cost=100), the nav retry was
    blocked, and the bounded escape found no exit.
    """
    now = 20_000_000_000
    nav, events = _mover_yield_harness(now, involved_ns=None)
    nav._lethal_start_cost = lambda: 100

    assert MODULE.TaskNavigator._try_mover_yield_retry(nav) is True
    assert nav.obstacle_reverse_state == "waiting"
    assert ("velocity", 0.0) in events


def test_wait_step_holds_until_the_pinned_base_cell_clears():
    """Waiting on the lane ahead is not enough when the base itself is lethal.

    Evidence 20261013T182522: the mover-yield armed the wait, the lane ahead
    read clear within the same second, the goal was re-sent, and the planner
    refused it again with "Navigation start cell is lethal" (cost=100).
    """
    from unittest.mock import Mock

    now = 1_000_000_000
    nav = SimpleNamespace(
        obstacle_wait_clear_distance=0.85,
        obstacle_wait_requires_clear_cell=True,
        obstacle_reverse_wait_deadline_ns=now + 15_000_000_000,
        _obstacle_ahead=lambda _distance: None,
        _lethal_start_cost=lambda: 100,
        _finish_obstacle_reverse=Mock(),
        _publish_obstacle_velocity=Mock(),
        publish_error=Mock(),
        publish_state=Mock(),
        _clear_goal=Mock(),
        _try_bounded_recovery=Mock(),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=now)),
        get_logger=lambda: Mock(),
    )

    MODULE.TaskNavigator._obstacle_wait_step(nav)
    nav._finish_obstacle_reverse.assert_not_called()

    nav._lethal_start_cost = lambda: None
    MODULE.TaskNavigator._obstacle_wait_step(nav)
    nav._finish_obstacle_reverse.assert_called_once()


def _inflation_nav(cost_of, start_cost=77):
    return SimpleNamespace(
        latest_x=0.0,
        latest_y=0.0,
        latest_yaw=math.pi / 2,          # facing +y, so offset 0 is behind
        _global_cost_at=lambda x, y: cost_of(x, y),
        _costmap_segment_clear=lambda *args, **kwargs: True,
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )


def test_inflation_escape_accepts_a_cheaper_non_lethal_cell():
    """Sitting in a >=50 blob must not make every escape candidate invalid.

    Evidence 20261013T112844Z: base cost 77 while carrying a cube to zone A,
    "Bounded recovery rejected: no clear self-clearance escape within 1.50m",
    then destination_navigation_failed:holding_cube.
    """
    nav = _inflation_nav(lambda x, y: 40 if y < -0.1 else 77)
    candidate = MODULE.TaskNavigator._inflation_escape_candidate(nav, 0.5)
    assert candidate is not None
    assert candidate.y < 0.0
    assert nav._global_cost_at(candidate.x, candidate.y) == 40


def test_inflation_escape_gates_on_cost_and_lethality():
    # Free cell: the strict tier owns this case, so this tier stays out.
    free = _inflation_nav(lambda x, y: 40 if y < -0.1 else 40, start_cost=40)
    assert MODULE.TaskNavigator._inflation_escape_candidate(free, 0.5) is None

    # Every neighbour lethal: nothing may be accepted.
    lethal = _inflation_nav(lambda x, y: 100 if y < -0.1 else 77)
    assert MODULE.TaskNavigator._inflation_escape_candidate(lethal, 0.5) is None

    # Neighbour is non-lethal but not cheaper: not an escape.
    pricier = _inflation_nav(lambda x, y: 90 if y < -0.1 else 77)
    assert MODULE.TaskNavigator._inflation_escape_candidate(pricier, 0.5) is None


def test_frozen_mover_is_treated_as_a_static_obstacle():
    """A mover that stopped must not hold the leg waiting for a window.

    Evidence 20261013T211009: the base spent about 50 s at
    CENTRAL_MOVER_GATE_WEST with abort/yield/resume cycles while the frozen
    mover held its position, then pickup_navigation_failed.
    """
    now = 10_000_000_000
    nav = SimpleNamespace(
        mover_stall_ns=3_000_000_000,
        mover_last_move_ns={"/c1": now - 4_000_000_000},
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=now)),
    )
    assert MODULE.mover_is_stalled(nav, "/c1") is True

    nav.mover_last_move_ns["/c1"] = now - 1_000_000_000
    assert MODULE.mover_is_stalled(nav, "/c1") is False

    # Unknown topics are not assumed stalled.
    assert MODULE.mover_is_stalled(nav, "/other") is False

    # Objects that do not track movers are unaffected.
    assert MODULE.mover_is_stalled(SimpleNamespace(), "/c1") is False


def test_mover_tracking_state_exists_before_any_command():
    """The obstacle callback can fire before the first command arrives.

    Observed 20261013T132230Z: AttributeError in _obstacle_callback killed the
    navigator process (exit 1), the launch log stopped at nav_start, and the
    mission hung silently for minutes.
    """
    source = (pathlib.Path(__file__).resolve().parents[1]
              / "tools_demo" / "task_navigator.py").read_text(encoding="utf-8")
    init = source.split("def __init__", 1)[1]
    assert "self.mover_last_seen_positions: Dict[str, tuple] = {}" in init
    assert "self.mover_last_move_ns: Dict[str, int] = {}" in init


def test_pickup_dock_requires_the_lateral_band():
    """Forward-only arrival accepted a 0.25 m cross-track miss (20261013T213956).

    The mover stopped the approach early, the base was declared docked, the
    cube sat outside the wrist camera's view and the round ended as
    target_unavailable.
    """
    from tools_demo.navigation_math import pickup_dock_drive_distance

    # Inside the forward band but far off laterally: still owed a dock.
    assert pickup_dock_drive_distance(0.018, 0.25, 0.02, 0.06) == 0.25
    # Inside both bands: arrived.
    assert pickup_dock_drive_distance(0.01, 0.03, 0.02, 0.06) == 0.0
    # A forward miss still governs on its own.
    assert pickup_dock_drive_distance(0.15, 0.01, 0.02, 0.06) == 0.15
    # Sign of the lateral miss does not matter, and negatives clamp to zero.
    assert pickup_dock_drive_distance(0.0, -0.2, 0.02, 0.06) == 0.2
    assert pickup_dock_drive_distance(-0.3, 0.0, 0.02, 0.06) == 0.0
