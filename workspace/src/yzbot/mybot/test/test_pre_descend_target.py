"""Regression checks for the pre-descend target verification.

Baseline evidence (campaign policy_20260918T190835, commit c8b2bd1): after a
perception_slot_fallback target the arm descended onto poses 0.78 m and 2.01 m
away from the cube, and 2rB_3bA red_cube_4 was commanded with the pre-centring
lateral offset (centring residual 0.0039 m, grasp window read 0.07812 m). Each
wasted descend ended in the very grasp-window rejection the live check can make
before the arm moves.
"""

import importlib.util
from pathlib import Path
from types import SimpleNamespace


MODULE_PATH = Path(__file__).parents[1] / "pick_place_moveit.py"
SPEC = importlib.util.spec_from_file_location("pick_place_moveit", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

REFINE = MODULE.MoveItPickPlace.refine_pick_target_before_descend


class FakeArm:
    """Minimal stand-in for the persistent pick node."""

    def __init__(self, relative, cube_name="red_cube_5"):
        self.relative = relative
        self.cube_name = cube_name
        self.pick_pose = (0.400, 0.0, 0.080)
        self.pick_safe_pose = (0.400, 0.0, 0.150)
        self.transport_pose = (0.400, 0.0, 0.080)
        self.transport_x = 0.30
        self.target_source = "slot_fallback"
        self.target_age_sec = 0.5
        self.approach_rejected_reason = None
        self.descend_calls = 0
        self.events = []
        self.moves = []
        self.tool_orientation = (0., 1., 0., 0.)
        self.pick_lift_duration = 1.0
        self.shift_succeeds = True
        self.apply_shift = True

    def relative_cube_position(self):
        if self.relative is None:
            return None
        return tuple(self.relative)

    def get_logger(self):
        return SimpleNamespace(
            info=lambda message: self.events.append(("info", message)),
            warn=lambda message: self.events.append(("warn", message)),
            error=lambda message: self.events.append(("error", message)),
        )

    def cartesian_move(self, pose, label, **kwargs):
        self.moves.append((pose, label))
        if not self.shift_succeeds:
            return False
        if self.apply_shift:
            self.relative = (
                self.relative[0] + pose[0] - self.pick_safe_pose[0],
                self.relative[1] - pose[1] + self.pick_safe_pose[1],
                self.relative[2],
            )
        return True

    def messages(self, marker):
        return [text for _level, text in self.events if marker in text]


def test_normal_target_is_accepted_at_the_current_pose():
    """A cube already inside the window must not be disturbed."""
    arm = FakeArm((0.01513, -0.00028, 0.13790))
    assert REFINE(arm, "pick_descend") is True
    assert arm.approach_rejected_reason is None
    assert arm.pick_pose == (0.400, 0.0, 0.080)
    assert arm.pick_safe_pose == (0.400, 0.0, 0.150)
    assert arm.messages("approach_verified")


def test_slot_fallback_target_0_78_m_away_is_refused_before_descending():
    """2rB_3bA red_cube_5 drove the descend to x=-0.76294."""
    arm = FakeArm((-0.76294, 0.19833, 0.14184))
    assert REFINE(arm, "pick_descend") is False
    assert arm.approach_rejected_reason in {
        "outside_correction_limit", "target_out_of_reach"
    }
    assert arm.descend_calls == 0
    rejection = arm.messages("approach_rejected")
    assert rejection, "refusal must be logged"
    assert "target=red_cube_5" in rejection[0]
    assert "source=slot_fallback" in rejection[0]
    assert "target_age_sec=0.5" in rejection[0]


def test_two_metre_transform_is_refused_before_descending():
    """1bA_4rC red_cube_3 read grasp_window x=2.02827 after a fallback."""
    arm = FakeArm((2.02827, -0.08678, 0.24246))
    assert REFINE(arm, "pick_descend") is False
    assert arm.descend_calls == 0
    assert arm.pick_pose == (0.400, 0.0, 0.080)


def test_target_identity_mismatch_is_refused():
    """A target that belongs to another cube cannot be descended onto."""
    arm = FakeArm((1.40000, 0.10000, 0.07000), cube_name="red_cube_5")
    assert REFINE(arm, "pick_descend") is False
    assert arm.approach_rejected_reason in {
        "outside_correction_limit", "target_out_of_reach"
    }
    assert arm.descend_calls == 0


def test_bounded_offset_is_recomputed_at_the_current_pose():
    """A small residual must be corrected here, not left for the descend."""
    arm = FakeArm((0.03000, 0.04000, 0.13800))
    assert REFINE(arm, "pick_descend") is True
    assert arm.approach_rejected_reason is None
    assert arm.pick_pose[0] < 0.400
    assert arm.pick_pose[1] > 0.0
    assert arm.pick_safe_pose[:2] == arm.pick_pose[:2]
    assert arm.messages("pick_target_refined")


def test_correction_without_retract_room_is_refused():
    """A correction that leaves no transport margin must not raise."""
    arm = FakeArm((0.03000, 0.00000, 0.13800))
    arm.pick_pose = (0.320, 0.0, 0.080)
    arm.pick_safe_pose = (0.320, 0.0, 0.150)
    assert REFINE(arm, "pick_descend") is True  # correction keeps retract room
    assert arm.pick_pose[0] > arm.transport_x
    arm2 = FakeArm((0.06000, 0.00000, 0.13800))
    arm2.pick_pose = (0.300, 0.0, 0.080)
    arm2.pick_safe_pose = (0.300, 0.0, 0.150)
    assert REFINE(arm2, "pick_descend") is False
    assert arm2.approach_rejected_reason == "transport_pose_invalid"
    assert arm2.descend_calls == 0


def test_unreadable_cube_pose_refuses_the_descend():
    """Without a live cube pose the arm must not move."""
    arm = FakeArm(None)
    assert REFINE(arm, "pick_descend") is False
    assert arm.approach_rejected_reason == "cube_pose_unavailable"
    assert arm.descend_calls == 0


def test_approach_correction_limits_keep_the_historical_bounds():
    """The pre-descend correction accepts the same offsets as before."""
    assert MODULE.APPROACH_CORRECTION_X_LIMIT == 0.12
    assert MODULE.APPROACH_CORRECTION_Y_LIMIT == 0.08
    assert MODULE.GRASP_XY_LIMIT == 0.035


def test_both_pick_entry_points_gate_the_descend():
    """run_pick_only and run_once must verify before the descend."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    for entry in ("    def run_pick_only", "    def run_once"):
        body = source.split(entry, 1)[1]
        guard = body.index('refine_pick_target_before_descend("pick_descend")')
        descend = body.index("self.descend_pick_vertically()")
        assert guard < descend, entry
        assert "return False" in body[guard:descend]


def test_refusal_is_reported_with_a_distinct_result_detail():
    """The coordinator distinguishes refused descends from failed grasps."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert '"approach_rejected "' in source
    assert "elif self.approach_rejected_reason:" in source


def test_safe_height_z_does_not_trigger_a_lateral_correction():
    # Actual 20261021 safe poses had relative Z ~109 mm and valid XY.
    arm = FakeArm((.009, -.001, .109))
    assert REFINE(arm, "pick_descend")
    assert arm.moves == []
    assert arm.pick_pose == (.4, 0., .08)
    assert not arm.messages("pick_target_refined")


def test_real_xy_correction_is_executed_at_safe_height():
    arm = FakeArm((.06, .04, .109))
    assert REFINE(arm, "pick_descend")
    assert len(arm.moves) == 1
    pose, label = arm.moves[0]
    assert label == "pick_align_at_safe_height"
    assert pose[2] == .15
    assert pose[:2] == arm.pick_pose[:2] == arm.pick_safe_pose[:2]
    assert abs(arm.relative[0] - .014) < 1e-10
    assert abs(arm.relative[1] + .009) < 1e-10


def test_failed_or_unverified_shift_never_retargets_the_descent():
    for failed, unverified in ((True, False), (False, True)):
        arm = FakeArm((.06, .04, .109))
        arm.shift_succeeds = not failed
        arm.apply_shift = not unverified
        assert not REFINE(arm, "pick_descend")
        assert arm.pick_pose == (.4, 0., .08)
        assert arm.pick_safe_pose == (.4, 0., .15)
        assert arm.approach_rejected_reason in {
            "safe_alignment_failed", "safe_alignment_unverified"}


def test_bad_projected_height_is_refused_without_xy_motion():
    for z in (.075, .20, float('nan')):
        arm = FakeArm((.06, .04, z))
        assert not REFINE(arm, "pick_descend")
        assert arm.moves == []
