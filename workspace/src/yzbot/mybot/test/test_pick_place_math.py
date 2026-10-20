"""Unit tests for deterministic pick-and-place geometry helpers."""

import importlib.util
import math
from unittest.mock import Mock
from pathlib import Path
from types import SimpleNamespace

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
import pytest


MODULE_PATH = Path(__file__).parents[1] / "pick_place_moveit.py"
SPEC = importlib.util.spec_from_file_location("pick_place_moveit", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize('yaw', [0., math.pi/2, math.pi, -math.pi/2])
@pytest.mark.parametrize('offset_z', [.035, .045, .065])
def test_loaded_delivery_reaches_slot_xy_and_clears_fingers(yaw, offset_z):
    model = (1., 2., 0.)
    quaternion = (0., 0., math.sin(yaw/2), math.cos(yaw/2))
    slot = MODULE.local_to_world_position(model, quaternion, (.4, .02, -.0686))
    relative = (.018, -.004, offset_z)
    tool = MODULE.loaded_slot_tool_pose(model, quaternion, slot, relative, (0.,1.,0.,0.))
    offset = MODULE.rotate_vector((0.,1.,0.,0.), relative)
    cube = MODULE.local_to_world_position(
        model, quaternion, tuple(tool[i]+offset[i] for i in range(3)))
    assert math.hypot(cube[0]-slot[0],cube[1]-slot[1]) <= .040
    assert .04-1e-9 <= cube[2]-slot[2] <= .085
    fingertip_world_z = model[2]+MODULE.BASE_LINK_HEIGHT+tool[2]-.082
    assert fingertip_world_z >= slot[2]-.015+.020-1e-9
    assert tool[0] == .38  # clear 100mm physical extension from carry


def test_measured_49cm_slot_uses_reachable_point_inside_the_pad():
    target=MODULE.loaded_slot_tool_pose((0.,0.,0.),(0.,0.,0.,1.),
                                      (.476,.09,.0348),(.016,-.004,.068),
                                      (0.,1.,0.,0.))
    assert target[0] == .38
    offset=MODULE.rotate_vector((0.,1.,0.,0.),(.016,-.004,.068))
    point=MODULE.local_to_world_position((0.,0.,0.),(0.,0.,0.,1.),
                                        tuple(target[i]+offset[i] for i in range(3)))
    assert abs(point[0]-.476) < .18
    assert abs(point[1]-.09) < .03
    assert .01 < point[2]-.0348 < .085


def test_unreachable_physical_slot_is_rejected_not_clamped():
    with pytest.raises(ValueError, match='workspace'):
        MODULE.loaded_slot_tool_pose((0.,0.,0.), (0.,0.,0.,1.),
                                     (.8,0.,.0348),(.02,0.,.04),(0.,1.,0.,0.))


def test_loaded_delivery_separates_lift_and_extension_at_safe_height():
    # Reproduce the B/blue1 target that truncated the old high diagonal.
    target = (.38, .0027185784470125556, .04112999093643639)
    lift, extend = MODULE.loaded_slot_approach_waypoints(target)
    assert lift == (.28, target[1], .10)
    assert extend == (.38, target[1], .10)
    assert lift[2] == extend[2] > target[2] + .05
    assert MODULE.BASE_LINK_HEIGHT + extend[2] - .082 > .0348 - .015 + .020


def test_loaded_delivery_approach_preserves_clearance_for_higher_slots():
    lift, extend = MODULE.loaded_slot_approach_waypoints((.36, -.03, .12))
    assert lift == pytest.approx((.28, -.03, .17))
    assert extend == pytest.approx((.36, -.03, .17))
    with pytest.raises(ValueError, match='finite'):
        MODULE.loaded_slot_approach_waypoints((.38, 0., float('nan')))


def test_failed_loaded_extension_never_opens_detaches_or_teleports():
    robot = Pose()
    robot.orientation.w=1.
    worker = SimpleNamespace(
        wait_until_ready=lambda:True, validate_carry_window=lambda:(.02,0.,.04),
        validate_transport_envelope=lambda:True,
        validate_transport_envelope_after_settle=lambda:True, attached=False,
        stabilize_base=lambda:True, update_stand_collision_objects=lambda *args:True,
        publish_state=Mock(), release_world_pose=(.4,0.,.0348),
        cube_name='blue_cube_1', tool_orientation=(0.,1.,0.,0.),
        get_logger=lambda:Mock(), relative_cube_position=lambda:(.02,0.,.04),
        get_entity_pose=lambda *args:robot, _model_values=lambda pose:((0.,0.,0.),(0.,0.,0.,1.)),
        cartesian_move=Mock(return_value=False), command_gripper=Mock(),
        detach_cube=Mock(), set_entity_pose=Mock(), base_stabilized=False)
    assert not MODULE.MoveItPickPlace.run_place_only(worker)
    worker.cartesian_move.assert_called_once()
    worker.command_gripper.assert_not_called()
    worker.detach_cube.assert_not_called()
    worker.set_entity_pose.assert_not_called()


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


def test_grasp_correction_reduces_measured_link6_x_error():
    """Replay AB1's 5 cm miss through the actual correction call path."""
    class FakeGrasp:
        pick_pose = (0.36, 0.0, 0.08)
        pick_safe_pose = (0.36, 0.0, 0.13)
        pick_lift_duration = 1.0

        def __init__(self):
            self.relative = [-0.05111, -0.01842, 0.06467]
            self.tool_xy = [0.36, 0.0]

        def relative_cube_position(self):
            return tuple(self.relative)

        def get_logger(self):
            return SimpleNamespace(warn=lambda message: None)

        def cartesian_move(self, pose, label, **kwargs):
            # At this downward link6 orientation, moving the arm in local X
            # changes the cube's reported link6 X in the same direction.
            delta_x = pose[0] - self.tool_xy[0]
            delta_y = pose[1] - self.tool_xy[1]
            self.relative[0] += delta_x
            self.relative[1] -= delta_y
            self.tool_xy[:] = pose[:2]
            return True

        def validate_grasp_window(self):
            return (
                tuple(self.relative)
                if MODULE.position_within_window(
                    self.relative,
                    MODULE.GRASP_XY_LIMIT,
                    MODULE.GRASP_Z_LIMITS,
                )
                else None
            )

    grasp = FakeGrasp()
    assert MODULE.MoveItPickPlace._retry_grasp_with_correction(grasp)
    assert math.isclose(grasp.relative[0], 0.014, abs_tol=0.005)
    assert grasp.validate_grasp_window() is not None


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
    assert timestamps == sorted(timestamps)
    assert len(set(timestamps)) == len(timestamps)
    assert timestamps[-1] == 4.0
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


def test_low_transport_retracts_with_clearance_lift_and_no_lateral_sweep():
    """A grasped cube moves inward with a small residual vertical lift."""
    result = MODULE.low_transport_pose((0.36, -0.16, 0.08), 0.28)

    assert result == (0.28, -0.16, 0.08 + MODULE.CARRY_CLEARANCE_LIFT)
    assert MODULE.CARRY_CLEARANCE_LIFT == 0.045
    payload = (result[0], result[1], result[2] - 0.045)
    assert MODULE.transport_payload_within_envelope(payload)


def test_transport_retract_clears_tray_before_folding_to_low_carry():
    """The loaded wrist rises 5 cm only during the tray exit path."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    pick = source.split("def run_pick_only", 1)[1].split(
        "def run_scan_only", 1
    )[0]
    assert 'declare_parameter("transport_lift", 0.080)' in source
    assert "via=[retract_lift, retract_clear]" in pick
    assert '"pick_low_retract"' in pick


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


def test_clearance_close_preserves_air_for_lateral_grasp_offsets():
    """Keep the fixed-joint cube from being pushed by a finger plate."""
    close = MODULE.clearance_gripper_position
    assert close(0.0) == MODULE.GRIPPER_MAX_CLOSE_POSITION
    assert close(0.010) < close(0.0)
    assert close(0.020) == 0.0


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
    assert MODULE.position_within_carry_window(transported)
    assert MODULE.position_within_carry_window((0.0497, 0.0050, -0.0097))
    # A dynamic physical base can settle the fixed attachment a few
    # millimetres below the former -0.020 m limit during a long transit.
    assert MODULE.position_within_carry_window((0.0488, -0.0055, -0.0224))


def test_carry_window_accepts_bounded_low_retract_settling():
    """A 64 mm attached-cube offset is not a dropped payload."""
    assert MODULE.position_within_carry_window(
        (0.0640, 0.0046, 0.0699),
    )
    assert not MODULE.position_within_carry_window(
        (0.0660, 0.0046, 0.0699),
    )
    assert not MODULE.position_within_carry_window(
        (0.0640, 0.0610, 0.0699),
    )


def test_constraint_settling_uses_fresh_joint_feedback():
    """Use the reference grasp method after a fresh feedback settle period."""
    source = MODULE_PATH.read_text(encoding="utf-8")

    assert "settle_deadline = None" in source
    assert "self.latest_joint_state is not previous_message" in source
    assert "time.monotonic() >= settle_deadline" in source
    assert "if not self.refresh_joint_state_after_constraint(" in source


def test_base_stabilization_keeps_the_ground_joint_but_shortens_the_settle():
    """The ground joint is load bearing; only its settle was over-budget.

    Measured 2026-09-10: holding the base with the drive gate alone shifted
    the grasp reference from x=+0.015 m to x=-0.022 m and the pick failed
    (the free chassis recoils while the arm extends, and the perception
    target is captured before that motion). The joint therefore stays, but
    the post-constraint settle drops from 0.4 s to the configured
    base_constraint_settle, which still crosses one physics boundary.
    """
    source = MODULE_PATH.read_text(encoding="utf-8")
    stabilize_body = source.split("def stabilize_base", 1)[1].split(
        "def release_base", 1
    )[0]

    assert 'request.link1_name = "base_footprint"' in stabilize_body
    assert 'request.model2_name = "ground_plane"' in stabilize_body
    assert "self.refresh_joint_state_after_constraint(" in stabilize_body
    assert "settle_sec=self.base_constraint_settle" in stabilize_body
    assert "settle_sec=0.4" not in stabilize_body
    assert 'self.declare_parameter("base_constraint_settle", 0.15)' in source
    assert "self.base_constraint_settle = float(" in source


def test_detach_treats_missing_joint_as_idempotent_success():
    """A prior release must not turn a completed place into a failure."""
    source = MODULE_PATH.read_text(encoding="utf-8")

    assert '"Joint does not exist" in message' in source
    assert 'f"released:{self.cube_name}"' in source


def test_carry_window_still_rejects_a_dropped_cube():
    """The relaxed transport window remains local to the end effector."""
    assert not MODULE.position_within_carry_window((0.003, 0.003, -0.040))
    assert not MODULE.position_within_carry_window((0.080, 0.000, 0.020))


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
    assert "self.move_to_joint_target(" in place_body
    assert "self.start_joint_target_async(" not in place_body
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


def test_deferred_stow_must_succeed_before_finishing():
    """Do not acknowledge placement before checked stow has completed."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    body = source.split("def run_place_only", 1)[1].split(
        "def run_release_only", 1
    )[0]
    deferred = body.split("world_slot_release:defer_return_home", 1)[1]
    before_complete = deferred.split("place_complete", 1)[0]

    assert "self.enforce_arm_joint_limits()" in before_complete
    assert "return False" in before_complete
    assert "self.move_to_joint_target(" in before_complete
    assert "self.start_joint_target_async(" not in before_complete
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
    assert MODULE.CARRY_X_LIMIT == 0.065
    assert MODULE.CARRY_Y_LIMIT == 0.060
    assert MODULE.CARRY_Z_LIMITS == (-0.030, 0.080)
    assert MODULE.MAX_ATTACHED_DRIFT == 0.020


def test_pick_descend_falls_back_to_an_ik_move():
    """A twice-truncated descend must not throw the cube away.

    BC3 (2026-09-11): `pick_descend` reached 0.400 and `pick_descend_retry`
    0.333, the pick aborted after 3.9 s, the coordinator excluded the cube and
    picked another blue one, which cost ~30 s. The cube is anchored until the
    gripper closes, so the IK fallback is safe there.
    """
    source = MODULE_PATH.read_text(encoding="utf-8")
    descend_block = source.split('"pick_descend_retry"', 1)[1].split(
        "if self.validate_grasp_window() is None:", 1
    )[0]
    assert "pick_descend Cartesian truncated twice" in descend_block
    assert '"pick_descend_fallback"' in descend_block
    assert "self.move_to_pose(" in descend_block
    # The anchor is still held at that point and the window is checked after.
    assert "pick_cube_stabilized" in source


def test_low_retract_climbs_over_the_tray_and_returns_to_carry():
    """The retract climbs, pulls back clear of the tray, then lowers.

    The pickup trays are 0.10 m boxes and the cube rests exactly on the top
    face, so a retract at tray height drags it (measured offsets of
    0.056-0.073 m against the 0.065 m limit aborted picks on 2026-09-10 and
    2026-09-11). Carrying the extra height all the way kept the payload in
    the lidar plane: campaigns 12-14 logged 175-350 collision-monitor
    NearSlow toggles per round against 5-14 before, and ran 10-15% slower.
    """
    source = MODULE_PATH.read_text(encoding="utf-8")
    retract = source.split("retract_lift = (", 1)[1].split(
        "self.get_logger().warn(", 1
    )[0]
    assert "self.pick_pose[2] + self.transport_lift" in retract
    assert "retract_clear = (" in retract
    assert "self.transport_pose[0]," in retract
    assert "via=[retract_lift, retract_clear]" in retract
    # The carry pose keeps only a small residual clearance lift.
    assert MODULE.low_transport_pose((0.40, 0.0, 0.08), 0.28) == (
        0.28,
        0.0,
        0.08 + MODULE.CARRY_CLEARANCE_LIFT,
    )
    helper = source.split("def cartesian_move(", 1)[1].split(
        "def command_gripper(", 1
    )[0]
    assert "via: Optional[Sequence[Sequence[float]]] = None" in helper
    assert "for point in via" in helper


def test_carry_recheck_window_covers_the_entity_state_lag():
    """The post-move recheck must outlast a stale entity-state frame.

    The carry reference is the cube pose relative to link6; right after the
    0.28 m retract at 0.23 m/s the two can be sampled 0.2-0.3 s apart, which
    reads as a 5 cm offset - exactly the 0.066-0.073 m values that aborted
    otherwise clean picks (BC42 2026-09-12).

    Zone9 campaign 20260912T151705 showed those readings converge on their
    own in 0.15-0.75 s: AB2 blue_cube_1 came back into the window on sample
    5, AC2 on sample 3, BC3 on sample 2, and each pick then completed. AC1
    aborted on sample 5 with the same 0.06565 m level and cost 32 s of
    rework, so the window has to cover about 3x the longest measured settle
    while still rejecting a slip that never converges.
    """
    source = MODULE_PATH.read_text(encoding="utf-8")
    helper = source.split("def validate_carry_window_after_settle(", 1)[1].split(
        "def validate_transport_envelope(", 1
    )[0]
    assert "retries: int = 14" in helper
    assert "delay_sec: float = 0.15" in helper
    assert 14 * 0.15 >= 1.5


def test_cartesian_retime_ramps_from_and_to_rest():
    """A retimed trajectory must not step from rest to full speed.

    The constant-speed profile applied the whole move's impulse in the first
    control cycle; the carried cube slipped out of the gripper and fell back
    onto the tray in about a quarter of all picks (2026-09-12), which the
    carry window then saw as a settled 4-6 cm offset.
    """
    trajectory = RobotTrajectory()
    for position in (0.0, 0.1, 0.2, 0.3, 0.4):
        point = JointTrajectoryPoint()
        point.positions = [position]
        trajectory.joint_trajectory.points.append(point)
    MODULE.retime_joint_trajectory(trajectory, 1.2)
    times = [
        point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        for point in trajectory.joint_trajectory.points
    ]
    assert abs(times[-1] - 1.2) < 1e-6
    assert times == sorted(times)
    # Waypoint positions, rather than their timestamps, are evenly spaced.
    # Their actual joint speed must ease in and out of the trajectory.
    speeds = [0.1 / (times[index] - times[index - 1])
              for index in range(1, len(times))]
    assert speeds[0] < speeds[1]
    assert speeds[-1] < speeds[-2]


def test_legacy_cycle_does_not_pause_on_post_grasp_joint_refresh():
    """An attached cube should leave the tray without a fresh-state dwell."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    body = source.split("def run_once", 1)[1].split(
        "def update_request", 1
    )[0]
    after_attach = body.split("if not self.attach_cube():", 1)[1].split(
        '"pick_lift"', 1
    )[0]
    assert "refresh_joint_state_after_constraint" not in after_attach


def test_world_slot_lifts_released_wrist_before_background_stow():
    """Clear the placed cube before the unloaded joint-space return."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    place = source.split("def run_place_only", 1)[1].split(
        "def run_release_only", 1
    )[0]
    lift = place.index('"place_lift"')
    stow = place.index('"return_home_after_world_slot"')
    assert lift < stow
    assert "if self.release_world_pose is None:\n                lifted =" not in place


def test_placement_cartesian_paths_keep_self_collision_checks():
    """Reject the folded link3/link6 branch before executing placement."""
    import ast
    tree = ast.parse(MODULE_PATH.read_text(encoding="utf-8"))
    method = next(node for node in ast.walk(tree)
                  if isinstance(node, ast.FunctionDef)
                  and node.name == "run_place_only")
    checked = set()
    for call in ast.walk(method):
        if not isinstance(call, ast.Call) or len(call.args) < 2:
            continue
        label = call.args[1]
        if not isinstance(label, ast.Constant):
            continue
        if label.value not in ("place_extend_loaded", "place_lift"):
            continue
        collision = next(item.value for item in call.keywords
                         if item.arg == "avoid_collisions")
        assert isinstance(collision, ast.Constant) and collision.value is True
        checked.add(label.value)
    assert checked == {"place_extend_loaded", "place_lift"}


def test_failed_cartesian_lift_uses_checked_ik_not_unchecked_retry():
    """An invalid straight-line branch must not poison the next pickup."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    place = source.split("def run_place_only", 1)[1].split(
        "def run_release_only", 1)[0]
    recovery = place.split("if not lifted:", 1)[1].split(
        "if self.release_world_pose is not None:", 1)[0]
    assert 'self.place_safe_pose, "place_lift_safe"' in recovery
    assert "self.move_to_pose(" in recovery
    assert "return False" in recovery
    assert "avoid_collisions=False" not in recovery


def test_base_release_retries_once_then_preserves_hold_on_timeout():
    """A lost detach acknowledgement cannot authorize driving away."""
    from unittest.mock import Mock
    worker = SimpleNamespace(
        stop_base_motion=Mock(), detach_client=Mock(),
        _wait_for_future=Mock(return_value=None),
        set_base_hold=Mock(), base_stabilized=True,
        get_logger=lambda: SimpleNamespace(error=lambda message: None))
    assert not MODULE.MoveItPickPlace.release_base(worker)
    assert worker.detach_client.call_async.call_count == 2
    assert worker.base_stabilized
    worker.set_base_hold.assert_called_once_with(True)


def test_base_release_acknowledged_absence_is_idempotent():
    """A retry after a lost first acknowledgement accepts confirmed absence."""
    from unittest.mock import Mock
    worker = SimpleNamespace(
        stop_base_motion=Mock(), detach_client=Mock(),
        _wait_for_future=Mock(side_effect=[
            None, SimpleNamespace(
                success=False,
                message="DETACHED -- ERROR (Joint does not exist!)")]),
        set_base_hold=Mock(), base_stabilized=True, publish_state=Mock(),
        get_logger=lambda: SimpleNamespace(error=lambda message: None))
    assert MODULE.MoveItPickPlace.release_base(worker)
    assert not worker.base_stabilized
    worker.set_base_hold.assert_called_once_with(False)


def test_timed_out_cartesian_goal_is_canceled_and_drained_before_retry():
    """Never submit a fallback while MoveIt still owns the old trajectory."""
    from unittest.mock import Mock

    result_future = object()
    cancel_future = object()
    goal_handle = SimpleNamespace(
        cancel_goal_async=Mock(return_value=cancel_future)
    )
    worker = SimpleNamespace(
        _wait_for_future=Mock(side_effect=[object(), object()]),
        wait_for_joint_state=Mock(return_value=True),
        get_logger=lambda: SimpleNamespace(
            warn=lambda message: None,
            error=lambda message: None,
            info=lambda message: None,
        ),
    )
    assert MODULE.MoveItPickPlace._cancel_and_drain_execution(
        worker, goal_handle, result_future, "pick_low_retract"
    )
    assert worker._wait_for_future.call_args_list[0].args == (cancel_future,)
    assert worker._wait_for_future.call_args_list[0].kwargs == {
        "timeout_sec": 3.0
    }
    assert worker._wait_for_future.call_args_list[1].args == (result_future,)
    assert worker._wait_for_future.call_args_list[1].kwargs == {
        "timeout_sec": 5.0
    }
    worker.wait_for_joint_state.assert_called_once_with(timeout_sec=1.0)


def test_joint_targets_snap_to_the_nearest_turn():
    """A +-2*pi IK branch must not be planned as a full revolution.

    20261014: the arm swung a large circle while grasping (tripping a joint
    error) and spun a full circle while placing.
    """
    two_pi = 2.0 * MODULE.math.pi
    # A target one turn away snaps back onto the current angle.
    assert abs(MODULE.nearest_equivalent_angle(two_pi + 0.20, 0.20) - 0.20) < 1e-9
    assert abs(MODULE.nearest_equivalent_angle(-two_pi + 0.20, 0.20) - 0.20) < 1e-9
    # Genuinely different angles are untouched.
    assert abs(MODULE.nearest_equivalent_angle(1.40, 0.20) - 1.40) < 1e-9
    # Half a turn keeps the sign the planner was given.
    assert abs(MODULE.nearest_equivalent_angle(0.20 + MODULE.math.pi, 0.20)
               - (0.20 + MODULE.math.pi)) < 1e-9
