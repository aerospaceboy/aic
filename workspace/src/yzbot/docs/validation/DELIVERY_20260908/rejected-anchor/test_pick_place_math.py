"""Unit tests for deterministic pick-and-place geometry helpers."""

import importlib.util
import math
from pathlib import Path

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


MODULE_PATH = Path(__file__).parents[1] / "pick_place_moveit.py"
SPEC = importlib.util.spec_from_file_location("pick_place_moveit", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_tool_offset_matches_downward_grasp_geometry():
    """The configured tool pose should put the cube on the pick stand."""
    result = MODULE.offset_from_tool(
        (0.400, 0.0, 0.080),
        (0.0, 0.9939, 0.0, -0.1100),
        0.045,
    )

    assert math.isclose(result[0], 0.39016, abs_tol=0.0001)
    assert math.isclose(result[1], 0.0, abs_tol=0.0001)
    assert math.isclose(result[2], 0.03608, abs_tol=0.0001)


def test_place_tool_pose_releases_cube_on_delivery_surface():
    """The placement tool pose should release the cube on the 0.10 m table."""
    result = MODULE.offset_from_tool(
        (0.350, 0.000, 0.160),
        (0.0, 0.9939, 0.0, -0.1100),
        0.045,
    )

    assert math.isclose(result[0], 0.34016, abs_tol=0.0001)
    assert math.isclose(result[1], 0.0, abs_tol=0.0001)
    assert math.isclose(result[2], 0.11608, abs_tol=0.0001)


def test_planar_local_world_round_trip():
    """Planar base coordinate conversions must be inverse operations."""
    model_xyz = (1.0, -2.0, 0.0)
    yaw = math.pi / 2.0
    quaternion = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    local = (0.4, 0.1, 0.03)

    world = MODULE.local_to_world_position(model_xyz, quaternion, local)
    recovered = MODULE.world_to_local_position(
        model_xyz,
        quaternion,
        world,
    )

    for actual, expected in zip(recovered, local):
        assert math.isclose(actual, expected, abs_tol=1.0e-9)


def test_roll_pitch_extraction_detects_an_unstable_chassis():
    """Transport validation must reject a base tipped beyond its limit."""
    roll, pitch = MODULE.roll_pitch_from_quaternion((0.0, 0.0, 0.0, 1.0))
    assert math.isclose(roll, 0.0, abs_tol=1.0e-9)
    assert math.isclose(pitch, 0.0, abs_tol=1.0e-9)

    angle = 0.20
    roll, pitch = MODULE.roll_pitch_from_quaternion(
        (math.sin(angle / 2.0), 0.0, 0.0, math.cos(angle / 2.0))
    )
    assert abs(roll) > MODULE.TRANSPORT_BASE_TILT_LIMIT
    assert math.isclose(pitch, 0.0, abs_tol=1.0e-9)


def test_retime_joint_trajectory_is_strictly_increasing():
    """Retiming must produce controller-safe monotonic timestamps."""
    trajectory = RobotTrajectory()
    for position in (0.0, 0.1, 0.2, 0.3):
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [1.0]
        trajectory.joint_trajectory.points.append(point)

    MODULE.retime_joint_trajectory(trajectory, duration_sec=4.0)

    timestamps = [
        point.time_from_start.sec
        + point.time_from_start.nanosec / 1_000_000_000
        for point in trajectory.joint_trajectory.points
    ]
    assert timestamps == [1.0, 2.0, 3.0, 4.0]
    assert all(
        not point.velocities
        for point in trajectory.joint_trajectory.points
    )


def test_zero_quaternion_is_rejected():
    """Invalid orientations must fail before a motion request is sent."""
    try:
        MODULE.normalize_quaternion((0.0, 0.0, 0.0, 0.0))
    except ValueError as exception:
        assert "zero" in str(exception).lower()
    else:
        raise AssertionError("Expected zero quaternion to be rejected")


def test_pick_transit_pose_preserves_xy_and_raises_z():
    """Inventory clearance must be a vertical move above the selected cube."""
    result = MODULE.transit_pose((0.36, -0.154, 0.121), 0.220)

    assert result == (0.36, -0.154, 0.220)


def test_pick_transit_pose_rejects_non_clearance_height():
    """A transit point at or below pick-safe height cannot protect inventory."""
    try:
        MODULE.transit_pose((0.36, -0.154, 0.121), 0.121)
    except ValueError as exception:
        assert "exceed" in str(exception)
    else:
        raise AssertionError("Expected invalid transit height to be rejected")


def test_scan_operation_does_not_apply_pick_clearance_validation():
    """The scan pose may intentionally be higher than pick transit height."""
    result = MODULE.transit_pose_for_operation(
        "scan",
        (0.300, -0.070, 0.250),
        0.220,
    )

    assert result == (0.300, -0.070, 0.250)


def test_low_transport_retracts_without_lifting_or_lateral_sweep():
    """A grasped cube moves inward at the same Y/Z coordinates."""
    result = MODULE.low_transport_pose((0.36, -0.16, 0.08), 0.28)

    assert result == (0.28, -0.16, 0.08)
    payload = (result[0], result[1], result[2] - 0.045)
    assert MODULE.transport_payload_within_envelope(payload)


def test_low_transport_rejects_extension_and_oversized_lateral_pose():
    """Transport must fold inward and remain inside the chassis radius."""
    try:
        MODULE.low_transport_pose((0.36, 0.0, 0.08), 0.40)
    except ValueError as exception:
        assert "retract" in str(exception)
    else:
        raise AssertionError("Expected an extending transport pose to fail")

    assert not MODULE.transport_payload_within_envelope((0.32, 0.20, 0.035))
    assert MODULE.transport_payload_within_envelope((0.216, 0.004, -0.018))
    assert not MODULE.transport_payload_within_envelope((0.216, 0.004, -0.040))


def test_cartesian_paths_keep_absolute_joint_jump_limit():
    """Disable only the noisy relative heuristic, not the safety bound."""
    assert MODULE.CARTESIAN_RELATIVE_JUMP_THRESHOLD == 0.0
    assert MODULE.CARTESIAN_REVOLUTE_JUMP_THRESHOLD == 0.50


def test_carry_window_accepts_observed_fixed_joint_settling():
    """Transport settling must not be mistaken for a lost attachment."""
    attached = (0.0125, 0.0018, 0.0683)
    transported = (0.0033, -0.0030, 0.0176)

    assert (
        MODULE.point_distance(attached, transported)
        > MODULE.MAX_ATTACHED_DRIFT
    )
    # The grasp window was intentionally widened in the validated 2/2
    # layout (GRASP_XY_LIMIT 0.018 -> 0.035, GRASP_Z_LIMITS
    # (0.025, 0.075) -> (0.010, 0.090)), so the observed settled pose now
    # also falls inside it. The strict transport check is the carry window;
    # asserting it directly is what proves settling is not a lost cube.
    assert MODULE.position_within_window(
        transported,
        MODULE.CARRY_XY_LIMIT,
        MODULE.CARRY_Z_LIMITS,
    )
    assert MODULE.position_within_window(
        (0.0497, 0.0050, -0.0097),
        MODULE.CARRY_XY_LIMIT,
        MODULE.CARRY_Z_LIMITS,
    )
    # A dynamic physical base can settle the fixed attachment a few
    # millimetres below the former -0.020 m limit during a long transit.
    assert MODULE.position_within_window(
        (0.0488, -0.0055, -0.0224),
        MODULE.CARRY_XY_LIMIT,
        MODULE.CARRY_Z_LIMITS,
    )


def test_constraint_settling_uses_fresh_joint_feedback():
    """Use the reference grasp method after a fresh feedback settle period."""
    source = MODULE_PATH.read_text(encoding="utf-8")

    assert "settle_deadline = None" in source
    assert "self.latest_joint_state is not previous_message" in source
    assert "time.monotonic() >= settle_deadline" in source
    assert "if not self.refresh_joint_state_after_constraint():" in source


def test_detach_treats_missing_joint_as_idempotent_success():
    """A prior release must not turn a completed place into a failure."""
    source = MODULE_PATH.read_text(encoding="utf-8")

    assert '"Joint does not exist" in message' in source
    assert 'f"released:{self.cube_name}"' in source


def test_carry_window_still_rejects_a_dropped_cube():
    """The relaxed transport window remains local to the end effector."""
    assert not MODULE.position_within_window(
        (0.003, 0.003, -0.040),
        MODULE.CARRY_XY_LIMIT,
        MODULE.CARRY_Z_LIMITS,
    )
    assert not MODULE.position_within_window(
        (0.080, 0.000, 0.020),
        MODULE.CARRY_XY_LIMIT,
        MODULE.CARRY_Z_LIMITS,
    )


def test_persistent_scan_avoids_disposable_base_constraints():
    """Camera-only motion must not churn Gazebo fixed joints per cube."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    scan_body = source.split("def run_scan_only", 1)[1].split(
        "def run_place_only", 1
    )[0]
    pick_body = source.split("def run_pick_only", 1)[1].split(
        "def run_scan_only", 1
    )[0]

    assert "self.stabilize_base()" not in scan_body
    assert "self.release_base()" not in scan_body
    assert 'self.pick_safe_pose,\n                "pick_approach"' in pick_body
    assert '"pick_transit"' not in pick_body
    assert '"pick_low_retract"' in pick_body
    assert 'self.transport_pose,\n                "pick_low_retract"' in pick_body
    assert '"pick_lift_clearance"' not in pick_body
    assert '"carry_home"' not in pick_body
    assert "self.validate_transport_envelope()" in pick_body
    assert pick_body.count("self.validate_transport_envelope()") >= 2
    assert "Loaded chassis became unstable after base release" in pick_body
    assert '"pick_clearance"' not in pick_body
    assert '"pick_descend_retry"' in pick_body
    place_body = source.split("def run_place_only", 1)[1].split(
        "def run_release_only", 1
    )[0]
    assert "self.start_joint_target_async(" in place_body
    assert '"return_home"' in place_body
    assert (
        place_body.index("self.release_base()")
        < place_body.index("world_slot_stabilized")
    )
    assert "world_slot_release:defer_return_home" in place_body
    direct_release = place_body.split(
        "direct_world_slot = self.release_world_pose is not None", 1
    )[1].split("elif not self.move_to_pose", 1)[0]
    assert "world_slot_release:direct_from_low_carry" in direct_release
    assert "self.validate_transport_envelope()" in direct_release
    assert (
        'move_to_pose(self.place_safe_pose, "place_safe")'
        not in direct_release
    )
    assert "self.stabilize_placed_cube()" in place_body
    assert "gripper_command_skipped:" in source
    assert "carry_window_recheck" in source
    assert "self.validate_carry_window_after_settle()" in pick_body


def test_place_opens_fingers_before_detaching_cube():
    """Prevent closed fingers from kicking an unconstrained placed cube."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    body = source.split("def run_place_only", 1)[1].split(
        "def run_release_only", 1
    )[0]
    open_index = body.index('self.command_gripper(0.0, "open")')
    detach_index = body.index("self.detach_cube()")

    assert open_index < detach_index


def test_failed_place_retains_attachment_for_state_machine_retry():
    """A transient controller timeout must not make the retry impossible."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    body = source.split("def run_place_only", 1)[1].split(
        "def run_release_only", 1
    )[0]
    failure_cleanup = body.split("finally:", 1)[1]

    assert "retaining cube for retry" in failure_cleanup
    assert "self.detach_cube()" not in failure_cleanup
    assert "self.command_gripper" not in failure_cleanup


def test_pick_confirms_attachment_before_low_retraction():
    """Do not retract until LinkAttacher is stable across physics frames."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    body = source.split("def run_pick_only", 1)[1].split(
        "def run_scan_only", 1
    )[0]
    assert "self.attach_cube_with_stability_retry()" in body
    after_attach = body.split(
        "self.attach_cube_with_stability_retry()", 1
    )[1]
    before_retract = after_attach.split(
        '"pick_low_retract"', 1
    )[0]
    retry_body = source.split(
        "def attach_cube_with_stability_retry", 1
    )[1].split("def verify_placement", 1)[0]

    assert "refresh_joint_state_after_constraint" not in before_retract
    assert "confirm_attachment_constraint" in retry_body
    assert "attachment_anchor_within_window(reference)" in retry_body
    assert "self.detach_cube()" in retry_body
    assert "self.set_entity_pose(self.cube_name, original_pose)" in retry_body


def test_attachment_anchor_rejects_observed_bad_fixed_joint():
    """A stable but displaced anchor must be rebuilt before retraction."""
    assert MODULE.attachment_anchor_within_window((0.0137, 0.0016, 0.0141))
    assert not MODULE.attachment_anchor_within_window(
        (0.0472, -0.0005, -0.0110)
    )
    assert not MODULE.attachment_anchor_within_window(
        (0.0494, 0.0026, 0.0223)
    )


def test_joint_limits_match_the_arm_model():
    """The correction must use the URDF revolute limits, not one value."""
    urdf = (
        MODULE_PATH.parents[1]
        / "mybot_description"
        / "urdf"
        / "arm.xacro"
    ).read_text(encoding="utf-8")

    for joint_name, limit in MODULE.ARM_JOINT_LIMITS.items():
        block = urdf.split(f'<joint name="{joint_name}"', 1)[1]
        declaration = block.split("<limit", 1)[1].split("/>", 1)[0]
        assert f'lower="-{limit}"' in declaration
        assert f'upper="{limit}"' in declaration


def test_in_bound_joints_are_never_moved():
    """A valid arm pose must not be disturbed by the limit guard."""
    positions = {
        "joint1": -2.9233,
        "joint2": -0.6167,
        "joint3": -1.0803,
        "joint4": 0.0008,
        "joint5": 1.4451,
        "joint6": -2.9175,
    }

    assert MODULE.joint_limit_corrections(positions) == {}


def test_out_of_bound_joints_return_inside_the_limit():
    """Observed a15r2 overshoot must be pulled back within the bound."""
    positions = {
        "joint1": -3.25840,
        "joint2": 0.0,
        "joint3": 0.0,
        "joint4": 0.0,
        "joint5": -2.45020,
        "joint6": -3.27580,
    }

    corrections = MODULE.joint_limit_corrections(positions)

    assert sorted(corrections) == ["joint1", "joint5", "joint6"]
    for joint_name, target in corrections.items():
        limit = MODULE.ARM_JOINT_LIMITS[joint_name]
        assert abs(target) < limit
        # Same side of zero, so the correction is the short way back.
        assert target * positions[joint_name] > 0.0


def test_correction_clears_moveit_start_state_allowance():
    """Land far enough inside the bound to satisfy the planner's check."""
    # MoveIt rejects a start state further than this outside a bound.
    start_state_max_bounds_error = 0.10

    assert MODULE.ARM_JOINT_LIMIT_MARGIN > 0.0
    assert MODULE.ARM_JOINT_LIMIT_MARGIN < start_state_max_bounds_error


def test_deferred_stow_corrects_joint_limits_before_finishing():
    """The parked placement pose is where the breach was observed."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    body = source.split("def run_place_only", 1)[1].split(
        "def run_release_only", 1
    )[0]
    deferred = body.split("world_slot_release:defer_return_home", 1)[1]
    before_complete = deferred.split("place_complete", 1)[0]

    assert "self.enforce_arm_joint_limits()" in before_complete
    assert "return False" not in before_complete
    assert "return_home_after_world_slot" in before_complete


def test_scan_corrects_joint_limits_before_planning():
    """A retried scan must not replan from a rejected start state."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    body = source.split("def run_scan_only", 1)[1].split(
        "def run_place_only", 1
    )[0]
    guard_index = body.index("self.enforce_arm_joint_limits()")
    plan_index = body.index('"scan_pose"')

    assert guard_index < plan_index


def test_joint_limit_guard_retries_coupled_settling():
    """One corrected joint can expose a neighbouring bound."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    body = source.split("def enforce_arm_joint_limits", 1)[1].split(
        "def attach_cube", 1
    )[0]

    assert "attempt < 2" in body
    assert "self.enforce_arm_joint_limits(attempt + 1)" in body


def test_limit_guard_keeps_the_validated_safety_windows():
    """Keep strict grasp limits and the measured physical carry tolerance."""
    assert MODULE.GRASP_XY_LIMIT == 0.035
    assert MODULE.GRASP_Z_LIMITS == (0.010, 0.090)
    assert MODULE.CARRY_XY_LIMIT == 0.060
    assert MODULE.CARRY_Z_LIMITS == (-0.030, 0.080)
    assert MODULE.MAX_ATTACHED_DRIFT == 0.020


def test_attachment_rebuilds_stable_but_shifted_joint(monkeypatch):
    """Reject the measured red2 Z jump despite a valid absolute window."""
    from types import SimpleNamespace

    initial = (0.01488, -0.00003, 0.03575)
    bad = (0.01237, 0.00362, 0.06851)
    good = (0.01480, 0.00010, 0.03580)
    assert MODULE.attachment_anchor_within_window(bad)
    samples = iter([bad, good])
    events = []
    node = SimpleNamespace(
        cube_name="red_cube_2", attached=True,
        get_entity_pose=lambda *args: "original-pose",
        validate_carry_window=lambda: initial,
        attach_cube=lambda: events.append("attach") or True,
        confirm_attachment_constraint=lambda: next(samples),
        detach_cube=lambda: events.append("detach") or True,
        set_entity_pose=lambda *args: events.append(args) or True,
        get_logger=lambda: SimpleNamespace(warn=lambda *args: None),
    )
    monkeypatch.setattr(MODULE.time, "sleep", lambda seconds: None)
    result = MODULE.MoveItPickPlace.attach_cube_with_stability_retry(node)
    assert result == good
    assert events == ["attach", "detach", ("red_cube_2", "original-pose"), "attach"]


def test_attachment_shift_retries_are_bounded(monkeypatch):
    """A persistent bad joint must fail without permitting arm retraction."""
    from types import SimpleNamespace

    attempts = []
    node = SimpleNamespace(
        cube_name="red_cube_2", attached=True,
        get_entity_pose=lambda *args: "original-pose",
        validate_carry_window=lambda: (0.01488, -0.00003, 0.03575),
        attach_cube=lambda: attempts.append(1) or True,
        confirm_attachment_constraint=lambda: (0.01237, 0.00362, 0.06851),
        detach_cube=lambda: True,
        set_entity_pose=lambda *args: True,
        get_logger=lambda: SimpleNamespace(
            warn=lambda *args: None, error=lambda *args: None),
    )
    monkeypatch.setattr(MODULE.time, "sleep", lambda seconds: None)
    assert MODULE.MoveItPickPlace.attach_cube_with_stability_retry(node) is None
    assert len(attempts) == 3
