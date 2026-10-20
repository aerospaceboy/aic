"""Cross-module contract: world delivery slots vs the real arm workspace.

This is the regression for the v15 failure. It drives the REAL producer and
the REAL consumer instead of testing either side in isolation:

  * producer  ``MissionCoordinator._delivery_slot_offset`` /
    ``_delivery_slot_world`` -- the fixed world slot of a job,
  * consumer  ``mybot/pick_place_moveit.py: loaded_slot_tool_pose`` plus the
    ``Dock is too far`` guard that follows it -- the checks the arm enforces
    before it opens the gripper.

2026-10-16 measured failure (v15, case 4rA_1bB-r1): A had four jobs spread
over +/-0.160 m of world Y, the dock kept the zone-centre lateral coordinate,
and the fourth job produced tool y = -0.18454 m against a +/-0.12 m window.
The arm refused twice with identical geometry and the mission ended
``place_failed:holding_cube`` at 291.2 s with 4/5 delivered.
"""

import importlib.util
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from test_navigation_contract import _point_to_oriented_box, _world_boxes
from tools_demo.mission_coordinator import DETERMINISTIC_PLACE_FAILURES
from tools_demo.mission_coordinator import MissionCoordinator
from tools_demo.navigation_math import (
    A_NORTH_WALL_INNER_Y,
    DELIVERY_DOCK_MAX_FORWARD,
    DELIVERY_DOCK_MAX_LATERAL,
    DELIVERY_DOCK_MIN_FORWARD,
    NavigationValueError,
    ZonePose,
    a_wall_tip_waypoints,
    delivery_slot_dock,
    lower_east_corridor_waypoints,
    tray_crossing_waypoints,
    wall57_pocket_waypoints,
)
from tools_demo.task_navigator import TaskNavigator


REPOSITORY = Path(__file__).resolve().parents[2]

_SPEC = importlib.util.spec_from_file_location(
    "pick_place_moveit_contract",
    REPOSITORY / "mybot" / "pick_place_moveit.py",
)
ARM = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(ARM)

# competition_zones.yaml ``zone_*`` docks: a 0.40 m standoff from the painted
# zone centre, A facing west, B and C facing east.
ZONE_CENTERS = {"A": (-5.4, 3.8), "B": (11.7, -5.0), "C": (3.3, -12.1)}
ZONE_DOCKS = {"A": (-5.0, 3.8, math.pi), "B": (11.3, -5.0, 0.0),
              "C": (2.9, -12.1, 0.0)}
SLOT_HEIGHT = 0.035
TOOL_QUATERNION = (0.0, 1.0, 0.0, 0.0)
MEASURED_ATTACH = (0.020, -0.008, 0.042)
END_SLOTS = (-0.160, 0.160)
ARRIVAL_STEPS = (-0.06, -0.03, 0.0, 0.03, 0.06)
DOCK_CORRECTION_X_LIMIT = 0.18
DOCK_CORRECTION_Y_LIMIT = 0.030

# The four real zone-A delivery legs of the v15 run: start pose, the pickup
# cube the robot had just taken, and that pickup's approach side. Copied from
# the launch log so the route check below runs on real geometry.
REAL_A_LEGS = [
    (1.873, -5.398, (1.00, -5.40), "east"),
    (-6.633, -2.800, (-7.50, -2.80), "east"),
    (12.265, -9.801, (11.40, -9.80), "east"),
    (0.865, 3.798, (7.40, 3.60), "west"),
]


def _slot_jobs(count, zone="A"):
    return [
        SimpleNamespace(
            target_zone=zone, index=index,
            cube_name=f"red_cube_{index + 1}",
        )
        for index in range(count)
    ]


def _slot_producer(count, zone="A"):
    """Drive the real ``_delivery_slot_offset``, one offset per job."""
    jobs = _slot_jobs(count, zone)
    self = SimpleNamespace(
        machine=SimpleNamespace(jobs=jobs),
        place_slot_spacing=0.080,
        place_maximum_abs_y=0.160,
    )
    return [
        MissionCoordinator._delivery_slot_offset(self, job)[0]
        for job in jobs
    ]


def _tool_point(robot_xy, yaw, slot_xy, attach=MEASURED_ATTACH):
    quaternion = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    return ARM.loaded_slot_tool_pose(
        (robot_xy[0], robot_xy[1], 0.0),
        quaternion,
        (slot_xy[0], slot_xy[1], SLOT_HEIGHT),
        attach,
        TOOL_QUATERNION,
    )


def _arm_guard(robot_xy, yaw, slot_xy, attach=MEASURED_ATTACH):
    """Reproduce the arm's whole placement guard chain for one pose.

    Returns ``((tool, corr_x, corr_y), None)`` on acceptance and
    ``(None, reason)`` on refusal, using the real arm functions.
    """
    quaternion = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    try:
        tool = _tool_point(robot_xy, yaw, slot_xy, attach)
    except ValueError as exception:
        return None, str(exception)
    offset = ARM.rotate_vector(TOOL_QUATERNION, attach)
    cube = ARM.local_to_world_position(
        (robot_xy[0], robot_xy[1], 0.0), quaternion,
        tuple(tool[i] + offset[i] for i in range(3)),
    )
    corr_x = abs(cube[0] - slot_xy[0])
    corr_y = abs(cube[1] - slot_xy[1])
    if corr_x > DOCK_CORRECTION_X_LIMIT or corr_y > DOCK_CORRECTION_Y_LIMIT:
        return None, (
            "Dock is too far from the physical delivery area "
            f"corr=({corr_x:.4f},{corr_y:.4f})"
        )
    return (tool, corr_x, corr_y), None


def _slot_xy(zone, offset):
    center_x, center_y = ZONE_CENTERS[zone]
    return (center_x, center_y + offset)


# ---------------------------------------------------------------------------
# 1. The real producer against the real arm guard chain.
# ---------------------------------------------------------------------------


def test_real_slot_spread_rejects_end_slots_under_the_legacy_zone_dock():
    """The v15 defect, reproduced from the real functions.

    Regression guard: if this ever stops failing, the sideways dock is no
    longer what makes the end slots reachable and the change must be
    re-derived rather than kept.
    """
    rejected = []
    total = 0
    for zone in ZONE_CENTERS:
        dock_x, dock_y, dock_yaw = ZONE_DOCKS[zone]
        for count in range(2, 6):
            for offset in _slot_producer(count, zone):
                total += 1
                accepted, error = _arm_guard(
                    (dock_x, dock_y), dock_yaw, _slot_xy(zone, offset)
                )
                if accepted is None:
                    rejected.append((zone, count, offset, error))
    assert total == (2 + 3 + 4 + 5) * 3
    assert len(rejected) == 4 * 2 * 3
    assert [(entry[1], entry[2]) for entry in rejected[:2]] == [
        (2, -0.160), (2, 0.160)
    ]
    for _zone, _count, offset, error in rejected:
        assert abs(abs(offset) - 0.160) < 1e-9
        assert "outside checked placement workspace" in error


def test_sideways_dock_reaches_every_zone_count_and_slot():
    """All A/B/C x 1..5 x both-ends slots pass the arm's full guard chain."""
    checked = 0
    worst = None
    for zone in ZONE_CENTERS:
        dock_x, dock_y, dock_yaw = ZONE_DOCKS[zone]
        for count in range(1, 6):
            for offset in _slot_producer(count, zone):
                slot = _slot_xy(zone, offset)
                dock, reason = delivery_slot_dock(
                    ZonePose(zone, dock_x, dock_y, dock_yaw), slot
                )
                assert reason == "aligned"
                assert dock.yaw == dock_yaw
                assert dock.x == dock_x
                assert dock.y == pytest.approx(slot[1])
                accepted, error = _arm_guard(
                    (dock.x, dock.y), dock.yaw, slot
                )
                assert accepted is not None, (zone, count, offset, error)
                tool, corr_x, corr_y = accepted
                assert abs(tool[1]) <= 0.12
                margin = min(0.12 - abs(tool[1]),
                             DOCK_CORRECTION_X_LIMIT - corr_x,
                             DOCK_CORRECTION_Y_LIMIT - corr_y)
                worst = margin if worst is None else min(worst, margin)
                checked += 1
    assert checked == (1 + 2 + 3 + 4 + 5) * 3
    # Worst residual is the lateral docking offset; the world-Y error stays
    # at the measurement noise level because the heading is unchanged.
    assert worst > 0.02


@pytest.mark.parametrize("attach", [
    (0.015, -0.005, 0.038),
    (0.020, -0.008, 0.042),
    (0.025, -0.012, 0.046),
    (0.020, 0.000, 0.042),
])
def test_sideways_dock_survives_attach_offset_and_dock_error(attach):
    """Normal attach offsets and the full 0.06 m zone arrival tolerance."""
    for zone in ZONE_CENTERS:
        dock_x, dock_y, dock_yaw = ZONE_DOCKS[zone]
        for count in range(1, 6):
            for offset in _slot_producer(count, zone):
                slot = _slot_xy(zone, offset)
                dock, reason = delivery_slot_dock(
                    ZonePose(zone, dock_x, dock_y, dock_yaw), slot
                )
                assert reason == "aligned"
                for error_x in ARRIVAL_STEPS:
                    for error_y in ARRIVAL_STEPS:
                        robot = (dock.x + error_x, dock.y + error_y)
                        accepted, error = _arm_guard(
                            robot, dock_yaw, slot, attach
                        )
                        assert accepted is not None, (
                            zone, count, offset, error_x, error_y, error
                        )
                        tool, _corr_x, _corr_y = accepted
                        assert 0.25 <= tool[0] <= 0.38
                        assert abs(tool[1]) <= 0.12


def test_turning_the_base_is_not_enough_for_the_arm_guard_chain():
    """Why the dock moves sideways instead of turning to face the slot.

    Turning to face the slot does centre the tool's lateral coordinate, but
    the arm's 0.030 m world-Y dock-distance guard then fails: at the real v15
    dock pose the forward shortfall projects onto world Y. A sideways dock
    keeps the pad-axis heading, so that term stays at the measurement noise.
    """
    slot = (-5.4, 3.96)
    turned = math.atan2(slot[1] - 3.757, slot[0] + 4.947)
    accepted, error = _arm_guard((-4.947, 3.757), turned, slot)
    assert accepted is None
    assert "Dock is too far" in error
    # The sideways dock at the same pose keeps the whole chain satisfied.
    accepted, error = _arm_guard((-4.947, 3.96), math.pi, slot)
    assert accepted is not None, error
    _tool, corr_x, corr_y = accepted
    assert corr_y == pytest.approx(0.0, abs=1e-9)
    assert corr_x <= DOCK_CORRECTION_X_LIMIT


def test_delivery_slot_dock_refuses_out_of_band_geometry():
    """Out-of-band slots keep the configured dock and say why."""
    place = ZonePose("A", -5.0, 3.8, math.pi)
    assert delivery_slot_dock(place, (-5.4, 3.8)) == (place, "aligned")
    far = ZonePose("A", -5.0, 3.8 + DELIVERY_DOCK_MAX_LATERAL + 0.01, math.pi)
    assert delivery_slot_dock(place, (far.x, far.y))[0] is place
    assert delivery_slot_dock(place, (far.x, far.y))[1] == "fallback"
    for forward in (DELIVERY_DOCK_MIN_FORWARD - 0.01,
                    DELIVERY_DOCK_MAX_FORWARD + 0.01):
        assert delivery_slot_dock(
            place, (place.x - forward, place.y)
        )[1] == "fallback"
    with pytest.raises(NavigationValueError):
        delivery_slot_dock(place, (math.nan, 3.8))
    with pytest.raises(NavigationValueError):
        delivery_slot_dock(place, (-5.4, 3.8), max_lateral=-1.0)


def test_slot_dock_only_changes_the_lateral_coordinate():
    place = ZonePose("B", 11.3, -5.0, 0.0)
    dock, reason = delivery_slot_dock(place, (11.7, -5.16))
    assert reason == "aligned"
    assert (dock.x, dock.yaw) == (place.x, place.yaw)
    assert dock.y == pytest.approx(-5.16)
    assert dock.zone != place.zone


# ---------------------------------------------------------------------------
# 2. The shifted dock and its sweep stay clear of real world geometry.
# ---------------------------------------------------------------------------


def test_dock_and_sweep_clear_world_geometry_and_placed_cubes():
    root = ET.parse(
        REPOSITORY / "mybot_description" / "worlds" / "offic_room.world"
    ).getroot()
    boxes = _world_boxes(root)
    half_diagonal = 0.226
    for zone in ZONE_CENTERS:
        dock_x, dock_y, dock_yaw = ZONE_DOCKS[zone]
        for offset in END_SLOTS:
            slot = _slot_xy(zone, offset)
            dock, reason = delivery_slot_dock(
                ZonePose(zone, dock_x, dock_y, dock_yaw), slot
            )
            assert reason == "aligned"
            clearance = min(
                _point_to_oriented_box(dock.x, dock.y, box)
                for _name, box in boxes
            )
            assert clearance - half_diagonal > 0.50, (zone, offset)
            # The slot column itself stays outside the chassis sweep.
            distance = math.hypot(slot[0] - dock.x, slot[1] - dock.y)
            assert distance - half_diagonal - 0.015 > 0.15


def test_route_helpers_keep_their_positions_for_the_real_delivery_legs():
    """Other routes must not move: only staging yaw may shift.

    Runs the real A-zone leg helpers for the four real v15 legs, across every
    slot offset the producer can emit, and requires identical waypoint
    positions plus an unchanged A-north tray gate.
    """
    dock_x, dock_y, _dock_yaw = ZONE_DOCKS["A"]
    offsets = list(_slot_producer(4, "A")) + list(END_SLOTS)
    for start_x, start_y, cube_xy, side in REAL_A_LEGS:
        for offset in offsets:
            legacy = (dock_x, dock_y)
            shifted = (dock_x, dock_y + offset)
            for helper in (wall57_pocket_waypoints,
                           a_wall_tip_waypoints,
                           lower_east_corridor_waypoints):
                old = helper((start_x, start_y), legacy)
                new = helper((start_x, start_y), shifted)
                assert [(p.x, p.y, p.zone) for p in old] == [
                    (p.x, p.y, p.zone) for p in new
                ], (helper.__name__, start_x, start_y, offset)
            old_tray = tray_crossing_waypoints(
                cube_xy, side, legacy, north_wall_y=A_NORTH_WALL_INNER_Y)
            new_tray = tray_crossing_waypoints(
                cube_xy, side, shifted, north_wall_y=A_NORTH_WALL_INNER_Y)
            assert [(p.x, p.y) for p in old_tray] == [
                (p.x, p.y) for p in new_tray
            ]
            # The A north delivery-tray gate in _configure_delivery_route.
            assert 3.6 <= legacy[1] <= 4.0
            assert 3.6 <= shifted[1] <= 4.0


# ---------------------------------------------------------------------------
# 3. The navigator applies the slot to the delivery dock only.
# ---------------------------------------------------------------------------


def _navigator_harness(**overrides):
    state = dict(
        final_yaw=math.pi,
        delivery_slot_world=None,
        active_zone="A",
        latest_x=-5.0,
        latest_y=3.8,
        latest_yaw=math.pi,
        warnings=[],
        infos=[],
    )
    state.update(overrides)
    harness = SimpleNamespace(**state)
    harness.get_logger = lambda: SimpleNamespace(
        warn=lambda text: harness.warnings.append(text),
        info=lambda text: harness.infos.append(text),
    )
    return harness


def test_delivery_dock_follows_the_job_slot_for_zone_deliveries():
    harness = _navigator_harness(delivery_slot_world=(-5.4, 3.96))
    place = ZonePose("A", -5.0, 3.8, math.pi)
    dock = TaskNavigator._slot_aligned_delivery_dock(harness, "A", place)
    assert dock.y == pytest.approx(3.96)
    assert dock.x == place.x and dock.yaw == place.yaw
    assert any("delivery_slot_dock" in text for text in harness.infos)
    accepted, error = _arm_guard((dock.x, dock.y), dock.yaw, (-5.4, 3.96))
    assert accepted is not None, error


def test_delivery_dock_keeps_the_configured_pose_elsewhere():
    place = ZonePose("A", -5.0, 3.8, math.pi)
    # An accepted pickup or zone-exit command clears the slot, which is what
    # keeps the frozen dock on those legs.
    assert TaskNavigator._slot_aligned_delivery_dock(
        _navigator_harness(delivery_slot_world=None), "A", place) is place
    # A stale slot must never move a non-delivery location.
    for zone in ("A_EXIT", "RED_CUBE_1", "RED"):
        assert TaskNavigator._slot_aligned_delivery_dock(
            _navigator_harness(delivery_slot_world=(-5.4, 3.96)),
            zone, place) is place
    # No dock pose yet: nothing to move.
    assert TaskNavigator._slot_aligned_delivery_dock(
        _navigator_harness(delivery_slot_world=(-5.4, 3.96)), "A",
        None) is None


def test_out_of_band_slot_keeps_the_configured_dock_and_is_logged():
    harness = _navigator_harness(delivery_slot_world=(-5.4, 4.60))
    place = ZonePose("A", -5.0, 3.8, math.pi)
    dock = TaskNavigator._slot_aligned_delivery_dock(harness, "A", place)
    assert dock is place
    assert any("delivery_slot_dock_refused" in text
               for text in harness.warnings)


def test_slot_parser_accepts_only_two_finite_floats():
    assert TaskNavigator._parse_delivery_slot(" -5.400000 , 3.960000 ") == (
        -5.4, 3.96
    )
    for text in ("1.0", "a,b", "inf,0", "nan,0", "0,0,0", ""):
        assert TaskNavigator._parse_delivery_slot(text) is None


class _ClockNow:
    nanoseconds = 100


def _command_harness(active_zone=None, ready=False):
    harness = _navigator_harness(active_zone=active_zone)
    harness.publish_error = harness.warnings.append
    harness.publish_state = lambda *args, **kwargs: None
    harness.action_client = SimpleNamespace(server_is_ready=lambda: ready)
    harness.pending_start_command = None
    harness.pending_action_retry_ticks = 0
    harness._arm_pending_start_deadline = lambda: None
    harness._parse_delivery_slot = TaskNavigator._parse_delivery_slot
    harness._global_costmap_ready = lambda: ready
    harness._reset_mover_control_for_command = lambda: None
    harness.nav_retries = 0
    harness.departure_routes = {}
    harness.pickup_approaches = {}
    harness.cube_docks = {}
    harness.pending_poses = []
    harness.last_pickup_cube_xy = None
    harness.last_pickup_side = None
    harness.dock_pose = ZonePose("A", -5.0, 3.8, math.pi)
    harness.logs = []

    def configure(zone, backup=False):
        harness.pending_poses = []
        harness.dock_pose = TaskNavigator._slot_aligned_delivery_dock(
            harness, zone, ZonePose(zone, -5.0, 3.8, math.pi)
        )

    harness._configure_delivery_route = configure
    harness.get_clock = lambda: SimpleNamespace(now=lambda: _ClockNow())
    harness.start_alignment = lambda: harness.logs.append("started")
    harness.get_logger = lambda: SimpleNamespace(
        warn=lambda text: harness.warnings.append(text),
        info=lambda text: harness.logs.append(text),
    )
    return harness


def test_zone_command_sets_the_slot_and_a_later_command_clears_it():
    harness = _command_harness(ready=True)
    TaskNavigator.command_callback(
        harness, String(data="A:slot=-5.400000,3.960000")
    )
    assert harness.active_zone == "A"
    assert harness.delivery_slot_world == (-5.4, 3.96)
    assert harness.dock_pose.y == pytest.approx(3.96)
    assert "started" in harness.logs
    # The pickup/exit form has no slot and must clear the previous one.
    harness.active_zone = None
    TaskNavigator.command_callback(harness, String(data="A"))
    assert harness.delivery_slot_world is None
    assert harness.dock_pose.y == pytest.approx(3.8)


def test_command_callback_refuses_a_malformed_slot_instead_of_ignoring_it():
    harness = _command_harness(ready=True)
    TaskNavigator.command_callback(harness, String(data="A:slot=1.0"))
    assert harness.warnings, "a malformed slot must be refused loudly"
    assert harness.active_zone is None
    assert harness.delivery_slot_world is None


def test_deferred_command_cannot_clobber_the_inflight_slot():
    """DDS discovery defers a command; the wire string must survive intact."""
    harness = _command_harness(ready=False)
    harness.delivery_slot_world = (-5.4, 3.96)
    TaskNavigator.command_callback(harness, String(data="B:slot=11.7,-5.16"))
    assert harness.active_zone is None
    assert harness.delivery_slot_world == (-5.4, 3.96)
    assert harness.pending_start_command.data == "B:slot=11.7,-5.16"
    # The retry path re-parses the original message once prerequisites are up.
    harness.action_client = SimpleNamespace(server_is_ready=lambda: True)
    harness._global_costmap_ready = lambda: True
    TaskNavigator.command_callback(harness, harness.pending_start_command)
    assert harness.active_zone == "B"
    assert harness.delivery_slot_world == (11.7, -5.16)


def test_rejected_command_cannot_clobber_the_inflight_slot():
    harness = _command_harness(active_zone="A", ready=True)
    harness.delivery_slot_world = (-5.4, 3.96)
    TaskNavigator.command_callback(harness, String(data="B:slot=11.7,-5.16"))
    assert harness.warnings
    assert harness.active_zone == "A"
    assert harness.delivery_slot_world == (-5.4, 3.96)


def test_alignment_stays_cancellable_and_bounded():
    """The dock change must not create a new unbounded wait."""
    stopped = []
    cleared = []
    states = []
    harness = _navigator_harness()
    harness.alignment_target_yaw = 2.78
    harness.stop_alignment = lambda: stopped.append(True)
    harness._clear_goal = lambda: cleared.append(True)
    harness.publish_state = lambda state, **kwargs: states.append(state)
    TaskNavigator.cancel_callback(harness, SimpleNamespace(data=True))
    assert stopped == [True]
    assert cleared == [True]
    assert states == ["canceled"]
    harness.alignment_target_yaw = 2.78
    TaskNavigator.cancel_callback(harness, SimpleNamespace(data=False))
    assert stopped == [True]


def test_cancel_before_alignment_still_cancels_the_goal():
    cancelled = []
    harness = _navigator_harness()
    harness.alignment_target_yaw = None
    harness.goal_handle = SimpleNamespace(
        cancel_goal_async=lambda: cancelled.append(True)
    )
    harness.publish_state = lambda state, **kwargs: None
    harness.publish_error = lambda text: None
    TaskNavigator.cancel_callback(harness, SimpleNamespace(data=True))
    assert cancelled == [True]


# ---------------------------------------------------------------------------
# 4. Producer side: the slot the navigator is told is the slot the arm gets.
# ---------------------------------------------------------------------------


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


def _coordinator_state(**overrides):
    state = dict(
        machine=SimpleNamespace(jobs=_slot_jobs(4, "A")),
        place_slot_spacing=0.080,
        place_maximum_abs_y=0.160,
        zone_centers=ZONE_CENTERS,
        place_pose=(0.350, 0.000, 0.160),
        place_safe_pose=(0.350, 0.000, 0.200),
        place_minimum_x=0.290,
        place_maximum_x=0.350,
        delivery_surface_z=0.020,
        delivery_cube_half_height=0.015,
        pick_tool_orientation=[0.0, 1.0, 0.0, 0.0],
        manipulation_pick_transit_height=0.200,
        manipulation_pick_lift_duration=1.1,
        manipulation_transport_x=0.280,
        manipulation_transport_duration=1.1,
        manipulation_velocity_scaling=1.0,
        manipulation_acceleration_scaling=1.0,
        manipulation_cartesian_duration=0.6,
        manipulation_gripper_duration=0.35,
        manipulation_request_id=0,
        manipulation_pending_id=None,
        manipulation_operation=None,
        manipulation_job=None,
        manipulation_active=False,
        manipulation_timed_out=False,
        manipulation_result=None,
        manipulation_deadline=None,
        manipulation_timeout=150.0,
        target_source="test",
        target_observed_at=None,
        manipulation_request_publisher=_FakePublisher(),
        odom_pose=(-4.947, 3.757, 3.109),
        _refused_place_geometry=None,
        logs=[],
        errors=[],
        published=0,
    )
    state.update(overrides)
    harness = SimpleNamespace(**state)
    harness._odom_pose = lambda: harness.odom_pose
    harness.get_logger = lambda: SimpleNamespace(
        info=lambda text: harness.logs.append(text),
        warn=lambda text: harness.logs.append(text),
    )
    harness.publish_error = harness.errors.append
    harness._active_job = lambda: harness.machine.jobs[-1]
    harness._delivery_slot_world = (
        lambda job: MissionCoordinator._delivery_slot_world(harness, job)
    )
    harness._delivery_slot_offset = (
        lambda job: MissionCoordinator._delivery_slot_offset(harness, job)
    )
    harness._place_slot_poses = (
        lambda job: MissionCoordinator._place_slot_poses(harness, job)
    )
    return harness


def test_arm_release_pose_is_built_from_the_shared_world_slot():
    """One computation feeds the arm's release pose and the truth check."""
    harness = _coordinator_state()
    job = harness.machine.jobs[3]
    MissionCoordinator._send_manipulation_request(
        harness, "place", job, None
    )
    request = json.loads(
        harness.manipulation_request_publisher.messages[0]
    )
    expected = MissionCoordinator._delivery_slot_world(harness, job)
    assert expected == (-5.4, 3.96)
    assert request["release_world_pose"] == [
        expected[0], expected[1], 0.035
    ]
    # The arm refuses this slot from the legacy zone-centre dock...
    accepted, error = _arm_guard((-4.947, 3.757), 3.109, expected)
    assert accepted is None
    assert "outside checked placement workspace" in error
    # ...and accepts it from the sideways dock the navigator now plans.
    dock, reason = delivery_slot_dock(
        ZonePose("A", -5.0, 3.8, math.pi), expected
    )
    assert reason == "aligned"
    accepted, error = _arm_guard((dock.x, dock.y), dock.yaw, expected)
    assert accepted is not None, error


# ---------------------------------------------------------------------------
# 5. A refused placement geometry is never repeated unchanged.
# ---------------------------------------------------------------------------


def _placing_coordinator(**overrides):
    harness = _coordinator_state(**overrides)
    harness.machine.state = "placing"
    harness.machine.detail = "placing"
    harness.done = []
    harness.handle_action = (
        lambda action: harness.done.append((action.kind, action.reason))
    )
    harness._send_manipulation_request = (
        lambda *args, **kwargs: setattr(
            harness, "published", harness.published + 1
        )
    )
    return harness


def test_deterministic_refusal_records_the_slot_and_pose():
    harness = _coordinator_state()
    result = {"success": False, "failure_reason": "slot_unreachable"}
    MissionCoordinator._note_place_outcome(harness, result, "place", False)
    assert harness._refused_place_geometry == {
        "slot": (-5.4, 3.96), "pose": (-4.947, 3.757, 3.109),
        "reason": "slot_unreachable",
    }
    assert any("place_geometry_refused" in text for text in harness.logs)
    assert "slot_unreachable" in DETERMINISTIC_PLACE_FAILURES


def test_non_deterministic_and_successful_outcomes_clear_the_guard():
    for result, success in (
        ({"success": False, "failure_reason": "loaded_extension_failed"},
         False),
        ({"success": True}, True),
        ({"success": False}, False),
        ({"success": False, "failure_reason": "placement_proof_failed"},
         False),
    ):
        harness = _coordinator_state()
        harness._refused_place_geometry = {
            "slot": (-5.4, 3.96), "pose": (-4.947, 3.757, 3.109),
            "reason": "slot_unreachable",
        }
        MissionCoordinator._note_place_outcome(
            harness, result, "place", success
        )
        assert harness._refused_place_geometry is None
    # A non-place result must not touch the guard.
    harness = _coordinator_state()
    harness._refused_place_geometry = {"slot": (0.0, 0.0),
                                       "pose": (0.0, 0.0, 0.0),
                                       "reason": "slot_unreachable"}
    MissionCoordinator._note_place_outcome(
        harness, {"success": False}, "pick", False
    )
    assert harness._refused_place_geometry is not None


def test_unchanged_geometry_is_refused_and_changed_geometry_is_allowed():
    harness = _placing_coordinator()
    harness._refused_place_geometry = {
        "slot": (-5.4, 3.96), "pose": (-4.947, 3.757, 3.109),
        "reason": "slot_unreachable",
    }
    job = SimpleNamespace(target_zone="A", index=3, cube_name="red_cube_3")
    assert MissionCoordinator._place_repeat_refusal(harness, job) is not None
    # Odometry jitter inside the tolerance is still the same geometry.
    harness.odom_pose = (-4.951, 3.760, 3.115)
    assert MissionCoordinator._place_repeat_refusal(harness, job) is not None
    # A genuine re-dock moves the base and re-opens the retry.
    harness.odom_pose = (-4.90, 3.90, 2.95)
    assert MissionCoordinator._place_repeat_refusal(harness, job) is None
    # A different job's slot is different geometry even from the same pose.
    harness.odom_pose = (-4.947, 3.757, 3.109)
    other = SimpleNamespace(target_zone="A", index=1, cube_name="red_cube_1")
    assert MissionCoordinator._place_repeat_refusal(harness, other) is None


def test_start_manipulation_fails_closed_instead_of_repeating():
    harness = _placing_coordinator()
    harness._refused_place_geometry = {
        "slot": (-5.4, 3.96), "pose": (-4.947, 3.757, 3.109),
        "reason": "slot_unreachable",
    }
    harness._place_repeat_refusal = (
        lambda job: MissionCoordinator._place_repeat_refusal(harness, job)
    )
    action = SimpleNamespace(kind="place", job=None, target=None)
    MissionCoordinator.start_manipulation(harness, action)
    assert harness.published == 0, "the identical request must not be sent"
    assert harness.machine.state == "failed"
    assert harness.machine.detail == (
        "place_failed:slot_unreachable:holding_cube"
    )
    assert harness.done == [
        ("done", "place_failed:slot_unreachable:holding_cube")
    ]
    assert any("slot_unreachable_repeat" in text for text in harness.errors)


def test_start_manipulation_still_sends_once_the_geometry_changed():
    harness = _placing_coordinator()
    harness._refused_place_geometry = {
        "slot": (-5.4, 3.96), "pose": (-4.947, 3.757, 3.109),
        "reason": "slot_unreachable",
    }
    harness.odom_pose = (-4.90, 3.90, 2.95)
    harness._place_repeat_refusal = (
        lambda job: MissionCoordinator._place_repeat_refusal(harness, job)
    )
    action = SimpleNamespace(kind="place", job=None, target=None)
    MissionCoordinator.start_manipulation(harness, action)
    assert harness.published == 1
