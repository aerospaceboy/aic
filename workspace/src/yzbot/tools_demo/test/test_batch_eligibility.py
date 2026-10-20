"""Batch eligibility: refuse an unmergeable route before the base moves.

The screen is a *geometry* test, not a stage-name test. A through-path
follower rounds every interior anchor; an anchor keeps its corridor function
only while the direct chord between its neighbours is too narrow to drive.
Once that chord is reachable the base is sent straight past the anchor and the
staged route is lost.

The headline case is the real BLUE_CUBE_5 dispatch recorded on 2026-10-16
20:44 in ``diagnosis-dsh-v16-slot-extreme``:
``nav_goal_batch zone=BLUE_CUBE_5 stages=2 first=(2.450,-11.000)
final=(10.900,-13.000)`` from ``(2.42,-12.08)``. The base drove 6.15 m east,
stalled 2.0 s on "collision ahead", and only then fell back after 13.98 s,
before driving 6.2 m back to the anchor.
"""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools_demo.task_navigator import (
    BATCH_SHORTCUT_MAX_COST,
    BATCH_SHORTCUT_TURN_LIMIT,
    TaskNavigator,
    ZonePose,
    batch_eligible,
    line_clear,
)

MAP_RESOLUTION = 0.05
MAP_ORIGIN = (-15.6, -33.4)
MAP_WIDTH = 658
MAP_HEIGHT = 822

# Real recorded geometry.
BC5_START = (2.42, -12.08)
BC5_ANCHOR = ZonePose("C_return_waypoint", 2.45, -11.00, 0.0)
BC5_GOAL = ZonePose("BLUE_CUBE_5", 10.90, -13.00, 0.0)
# The west sibling: identical corner, but its chord is blocked in the real map
# (measured clearance 0.000 m) and it never fell back in 72 batches.
RC5_GOAL = ZonePose("RED_CUBE_5", -3.80, -12.40, 0.0)
# The north sibling: a gentle corner, 107 batches with no fallback.
BC4_START = (2.42, -12.24)
BC4_GOAL = ZonePose("BLUE_CUBE_4", 0.40, -8.80, 0.0)


class FakeCostmap:
    """Minimal OccupancyGrid stand-in, free except where told otherwise."""

    def __init__(self, blocked=()):
        self.info = SimpleNamespace(
            width=MAP_WIDTH,
            height=MAP_HEIGHT,
            resolution=MAP_RESOLUTION,
            origin=SimpleNamespace(position=SimpleNamespace(
                x=MAP_ORIGIN[0], y=MAP_ORIGIN[1])),
        )
        self.data = [0] * (MAP_WIDTH * MAP_HEIGHT)
        for x_value, y_value in blocked:
            self.block(x_value, y_value)

    def block(self, x_value, y_value, cost=100):
        """Mark one world cell with an occupied/inscribed cost."""
        px = int(math.floor((x_value - MAP_ORIGIN[0]) / MAP_RESOLUTION))
        py = int(math.floor((y_value - MAP_ORIGIN[1]) / MAP_RESOLUTION))
        if 0 <= px < MAP_WIDTH and 0 <= py < MAP_HEIGHT:
            self.data[py * MAP_WIDTH + px] = cost


class BatchHarness:
    """Dispatch a staged command without a ROS executor."""

    _send_next_goal = TaskNavigator._send_next_goal
    _fallback_batch_to_serial = TaskNavigator._fallback_batch_to_serial

    def __init__(self, poses, start, costmap=None, zone="BLUE_CUBE_5"):
        self.pending_poses = list(poses)
        self.latest_x, self.latest_y = start
        self.latest_yaw = 0.0
        self.active_zone = zone
        self.global_costmap = costmap
        self.nav_batch_poses = []
        self.nav_batch_attempted = False
        self.nav_goal_pose = None
        self.goal_handle = None
        self.nav_handoff_requested = False
        self.stage_index = 0
        self.stage_total = len(self.pending_poses)
        self.sent_batches = []
        self.sent_serial = []
        self.publish_state = Mock()
        self.publish_error = Mock()
        self._logger = Mock()

    def _wait_for_action_server(self, **kwargs):
        return True

    def get_logger(self):
        return self._logger

    def batch_eligible(self, poses):
        """Exercise the shipped module-level screen against this harness."""
        return batch_eligible(self, poses)

    def _send_pose_batch(self, poses):
        self.sent_batches.append(list(poses))

    def _send_pose(self, pose):
        self.sent_serial.append(pose)


def turn_at(start, anchor, goal):
    """Return the real polyline turn of the recorded three-point geometry."""
    incoming = math.atan2(anchor[1] - start[1], anchor[0] - start[0])
    outgoing = math.atan2(goal[1] - anchor[1], goal[0] - anchor[0])
    return abs((outgoing - incoming + math.pi) % (2.0 * math.pi) - math.pi)


def test_the_real_blue5_counterexample_selects_serial_before_dispatch():
    """The recorded dispatch must never become a batch action again."""
    harness = BatchHarness(
        [BC5_ANCHOR, BC5_GOAL],
        BC5_START,
        costmap=FakeCostmap(),
    )
    turn = turn_at(BC5_START, (BC5_ANCHOR.x, BC5_ANCHOR.y),
                   (BC5_GOAL.x, BC5_GOAL.y))
    assert turn > BATCH_SHORTCUT_TURN_LIMIT, (
        "the recorded blue5 corner must exceed the derived turn limit")
    harness._send_next_goal()
    assert harness.sent_batches == [], (
        "no batch action may be sent for the recorded blue5 route")
    assert harness.sent_serial == [BC5_ANCHOR]
    assert harness.pending_poses == [BC5_GOAL]
    assert harness.nav_batch_poses == []
    assert harness.nav_batch_attempted is False


def test_the_screen_logs_why_it_refused():
    """A refused batch must be auditable from the run log."""
    harness = BatchHarness([BC5_ANCHOR, BC5_GOAL], BC5_START,
                           costmap=FakeCostmap())
    harness._send_next_goal()
    messages = [call.args[0] for call in harness._logger.warn.call_args_list]
    assert any("nav_batch_screen_serial" in text for text in messages)
    assert any("reason=shortcut_open" in text for text in messages)


def test_a_blocked_shortcut_still_batches():
    """Same corner, blocked shortcut: the planner must pass the anchor.

    This is the real RED_CUBE_5 geometry, clean in 72 of 72 recorded batches.
    """
    blocked = []
    for step in range(0, 41):
        ratio = step / 40.0
        blocked.append((
            BC5_START[0] + ratio * (RC5_GOAL.x - BC5_START[0]),
            BC5_START[1] + ratio * (RC5_GOAL.y - BC5_START[1]),
        ))
    harness = BatchHarness(
        [BC5_ANCHOR, RC5_GOAL],
        BC5_START,
        costmap=FakeCostmap(blocked=blocked),
        zone="RED_CUBE_5",
    )
    harness._send_next_goal()
    assert harness.sent_batches == [[BC5_ANCHOR, RC5_GOAL]]
    assert harness.sent_serial == []


def test_a_gentle_corner_still_batches():
    """The BLUE_CUBE_4 pickup: 107 recorded batches, no fallback."""
    harness = BatchHarness(
        [BC5_ANCHOR, BC4_GOAL],
        BC4_START,
        costmap=FakeCostmap(),
        zone="BLUE_CUBE_4",
    )
    turn = turn_at(BC4_START, (BC5_ANCHOR.x, BC5_ANCHOR.y),
                   (BC4_GOAL.x, BC4_GOAL.y))
    assert turn < BATCH_SHORTCUT_TURN_LIMIT
    harness._send_next_goal()
    assert harness.sent_batches == [[BC5_ANCHOR, BC4_GOAL]]
    assert harness.sent_serial == []


def test_a_straight_route_is_untouched():
    """The screen must not degrade ordinary collinear staging."""
    poses = [ZonePose("mid", 2.0, 0.0, 0.0), ZonePose("end", 6.0, 0.0, 0.0)]
    harness = BatchHarness(poses, (0.0, 0.0), costmap=FakeCostmap(),
                           zone="B")
    harness._send_next_goal()
    assert harness.sent_batches == [poses]


def test_missing_start_pose_abstains_instead_of_refusing():
    """Without a measured pose the screen keeps the historical behaviour."""
    harness = BatchHarness([BC5_ANCHOR, BC5_GOAL], BC5_START,
                           costmap=FakeCostmap())
    harness.latest_x = None
    harness.latest_y = None
    harness._send_next_goal()
    assert harness.sent_batches == [[BC5_ANCHOR, BC5_GOAL]]


def test_missing_costmap_abstains_instead_of_refusing():
    """A cold costmap must not be read as a reachable shortcut."""
    harness = BatchHarness([BC5_ANCHOR, BC5_GOAL], BC5_START, costmap=None)
    harness._send_next_goal()
    assert harness.sent_batches == [[BC5_ANCHOR, BC5_GOAL]]


@pytest.mark.parametrize(
    "second,third",
    [
        ((2.45, -11.00), (10.90, -13.00)),   # duplicate anchor
        ((10.90, -13.00), (10.90, -13.00)),  # duplicate goal
        ((2.45, -11.00), (2.45, -11.00)),    # zero-length tail
    ],
)
def test_duplicate_and_zero_length_stages_do_not_crash(second, third):
    """Repeated or zero-length stages carry no turn and must stay safe."""
    poses = [
        ZonePose("anchor", second[0], second[1], 0.0),
        ZonePose("goal", third[0], third[1], 0.0),
    ]
    harness = BatchHarness(poses, BC5_START, costmap=FakeCostmap())
    harness._send_next_goal()
    # Exactly one dispatch happens: never both a batch and a serial goal.
    assert len(harness.sent_batches) + len(harness.sent_serial) == 1


def test_a_refused_batch_issues_exactly_one_action():
    """Refusal must not double-dispatch or leave a batch handle behind."""
    harness = BatchHarness([BC5_ANCHOR, BC5_GOAL], BC5_START,
                           costmap=FakeCostmap())
    harness._send_next_goal()
    assert len(harness.sent_serial) == 1
    assert harness.sent_batches == []
    assert harness.nav_batch_poses == []
    assert harness.goal_handle is None
    # A later failure cannot fall back a batch that was never sent.
    assert harness._fallback_batch_to_serial("late_result") is False


@pytest.mark.parametrize(
    "anchor_zone",
    [
        "A_WALL_NORTH_CLEAR",
        "A_WALL_SOUTH_TIP_CLEAR",
        "tray_wall_clear_near",
        "C_SOUTH_WALL_CLEAR",
        "WALL57_POCKET_1",
        "central_patrol_north_flank",
        "north_pickup_tray_west_clear",
        "north_delivery_tray_east_clear",
    ],
)
def test_forced_serial_anchors_never_re_enter_a_batch(anchor_zone):
    """Every already-proven serial anchor keeps its own confirmation."""
    poses = [
        ZonePose(anchor_zone, 2.45, -11.00, 0.0),
        ZonePose("goal", 10.90, -13.00, 0.0),
    ]
    harness = BatchHarness(poses, BC5_START, costmap=FakeCostmap())
    harness._send_next_goal()
    assert harness.sent_batches == []
    assert harness.sent_serial == [poses[0]]


@pytest.mark.parametrize(
    "zone,anchor_zone",
    [
        ("BLUE_CUBE_2", "obstacle_2_south_gate"),
        ("BLUE_CUBE_3", "CENTRAL_MOVER_GATE_WEST"),
    ],
)
def test_blue2_and_blue3_anchor_exclusions_are_preserved(zone, anchor_zone):
    """The two route-specific exclusions must stay in force."""
    poses = [
        ZonePose(anchor_zone, 2.45, -11.00, 0.0),
        ZonePose("goal", 10.90, -13.00, 0.0),
    ]
    harness = BatchHarness(poses, BC5_START, costmap=FakeCostmap(), zone=zone)
    harness._send_next_goal()
    assert harness.sent_batches == []


def test_a_central_gate_interior_anchor_still_forces_serial():
    """A through-path may not open on a non-gate stage and round a gate."""
    poses = [
        ZonePose("shelf_entry", 2.45, -11.00, 0.0),
        ZonePose("CENTRAL_MOVER_GATE_WEST", 10.90, -13.00, 0.0),
    ]
    harness = BatchHarness(poses, BC5_START, costmap=FakeCostmap(),
                           zone="RED_CUBE_2")
    harness._send_next_goal()
    assert harness.sent_batches == []


def test_shortcut_cost_bound_matches_the_nav2_occupancy_encoding():
    """99 is inscribed and 100 lethal, so 98 is the last free value."""
    assert BATCH_SHORTCUT_MAX_COST == 98
    assert BATCH_SHORTCUT_MAX_COST < 99


def test_inscribed_cells_make_a_shortcut_unreachable_but_free_ones_do_not():
    """The cost bound is applied per sampled cell along the chord."""
    harness = BatchHarness([BC5_ANCHOR, BC5_GOAL], BC5_START,
                           costmap=FakeCostmap())
    assert line_clear(
        harness.global_costmap, BC5_START[0], BC5_START[1],
        BC5_GOAL.x, BC5_GOAL.y,
        maximum_cost=BATCH_SHORTCUT_MAX_COST) is True
    harness.global_costmap.block(
        (BC5_START[0] + BC5_GOAL.x) / 2.0,
        (BC5_START[1] + BC5_GOAL.y) / 2.0, cost=99)
    assert line_clear(
        harness.global_costmap, BC5_START[0], BC5_START[1],
        BC5_GOAL.x, BC5_GOAL.y,
        maximum_cost=BATCH_SHORTCUT_MAX_COST) is False


def test_turn_limit_is_derived_from_the_configured_controller():
    """The limit must follow the goal tolerance and worst-case lookahead.

    A corner is effectively skipped once the follower's cut
    ``L * (1 - cos(turn / 2))`` exceeds the tolerance that was supposed to
    prove the anchor was reached.
    """
    tolerance = 0.20     # general_goal_checker.xy_goal_tolerance
    lookahead = 1.20     # max_lookahead_dist
    expected = 2.0 * math.acos(1.0 - tolerance / lookahead)
    assert BATCH_SHORTCUT_TURN_LIMIT == pytest.approx(expected, abs=1e-3)
    # And the cut at the limit really is the tolerance, not more. The
    # constant is stored to three decimals for readable logs, so allow that
    # rounding: the cut stays a fraction of a millimetre under the tolerance.
    cut = lookahead * (1.0 - math.cos(BATCH_SHORTCUT_TURN_LIMIT / 2.0))
    assert cut == pytest.approx(tolerance, abs=5e-4)
    assert cut <= tolerance


def test_navigation_parameters_still_match_the_derived_limit():
    """Guard the config the limit was derived from, not just the constant."""
    from pathlib import Path
    text = Path(__file__).resolve().parents[2] / (
        "bot_navigation/param/originbot_nav2.yaml")
    parameters = text.read_text()
    assert "xy_goal_tolerance: 0.20" in parameters
    assert "lookahead_dist: 0.70" in parameters
    assert "max_lookahead_dist: 1.20" in parameters
    assert "lookahead_time: 0.80" in parameters
    assert "use_velocity_scaled_lookahead_dist: true" in parameters


def test_the_screen_reports_the_live_chord_cost_when_it_abstains():
    """The 2026-10-17 diagnosis abstained silently on its target route.

    The live chromatic cost must be observable, so a turn above the limit
    always logs the measured worst chord cost and the verdict, whether the
    screen fires or not.
    """
    harness = BatchHarness([BC5_ANCHOR, BC5_GOAL], BC5_START,
                           costmap=FakeCostmap())
    harness._send_next_goal()
    probes = [call.args[0] for call in harness._logger.info.call_args_list]
    assert any("nav_batch_screen_probe" in text for text in probes)
    assert any("verdict=open" in text for text in probes)
    assert any("chord_worst_cost=" in text for text in probes)


def test_a_blocked_chord_logs_a_blocked_verdict_and_still_batches():
    """Abstaining must be visible in the log, not silent."""
    blocked = []
    for step in range(0, 41):
        ratio = step / 40.0
        blocked.append((
            BC5_START[0] + ratio * (BC5_GOAL.x - BC5_START[0]),
            BC5_START[1] + ratio * (BC5_GOAL.y - BC5_START[1]),
        ))
    harness = BatchHarness([BC5_ANCHOR, BC5_GOAL], BC5_START,
                           costmap=FakeCostmap(blocked=blocked))
    harness._send_next_goal()
    probes = [call.args[0] for call in harness._logger.info.call_args_list]
    assert any("verdict=blocked" in text for text in probes)
    assert harness.sent_batches == [[BC5_ANCHOR, BC5_GOAL]]
    assert not any("nav_batch_screen_serial" in call.args[0]
                   for call in harness._logger.warn.call_args_list)


def test_line_worst_cost_reports_the_maximum_and_flags_unknown_cells():
    """One sampling loop backs both the measurement and the predicate."""
    from tools_demo.task_navigator import line_worst_cost
    costmap = FakeCostmap()
    worst, unknown = line_worst_cost(
        costmap, BC5_START[0], BC5_START[1], BC5_GOAL.x, BC5_GOAL.y)
    assert worst == 0 and unknown is False
    costmap.block((BC5_START[0] + BC5_GOAL.x) / 2.0,
                  (BC5_START[1] + BC5_GOAL.y) / 2.0, cost=77)
    worst, unknown = line_worst_cost(
        costmap, BC5_START[0], BC5_START[1], BC5_GOAL.x, BC5_GOAL.y)
    assert worst == 77 and unknown is False
    # An unknown cell counts as not clear, exactly as before this refactor.
    costmap.block((BC5_START[0] + BC5_GOAL.x) / 2.0,
                  (BC5_START[1] + BC5_GOAL.y) / 2.0, cost=255)
    costmap.data[costmap.data.index(255)] = -1
    assert line_clear(costmap, BC5_START[0], BC5_START[1],
                      BC5_GOAL.x, BC5_GOAL.y) is False
