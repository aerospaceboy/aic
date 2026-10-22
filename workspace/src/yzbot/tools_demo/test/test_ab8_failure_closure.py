"""Two failure modes from the 2026-10-21 AB8 series must stay closed."""

from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PICK_PLACE = REPO / "mybot" / "pick_place_moveit.py"
NAVIGATOR = REPO / "tools_demo" / "tools_demo" / "task_navigator.py"


def test_proven_delivery_survives_a_return_home_timeout():
    """AB8-5: the cube was released and stabilized, then stow timed out."""
    source = PICK_PLACE.read_text(encoding="utf-8")
    block = source[source.index("def run_place_only"):]
    block = block[:block.index("def run_release_only")]
    # The helper retracts and warns after a stow timeout. Its caller must
    # not turn that post-delivery failure into a failed mission.
    assert "return_home_after_world_slot_failed_after_delivery" in block
    assert "deferred_stow" in block
    assert "self.finish_deferred_place_stow(staging_arm)" in block
    assert "if not self.finish_deferred_place_stow" not in block


def test_release_still_requires_verification_before_the_stow():
    """Only a verified delivery may survive a stow failure."""
    source = PICK_PLACE.read_text(encoding="utf-8")
    block = source[source.index("def run_place_only"):]
    block = block[:block.index("def run_release_only")]
    verify = block.index("verify_placement()")
    stow = block.index("return_home_after_world_slot")
    assert verify < stow
    assert "Placement verification failed" in block[verify:stow]

def test_stow_gets_a_short_budget_after_a_proven_delivery():
    """AB8-5 spent 60 s on a stalled stow that no longer mattered."""
    source = PICK_PLACE.read_text(encoding="utf-8")
    assert "WORLD_SLOT_STOW_TIMEOUT = 8.0" in source
    block = source[source.index("def run_place_only"):]
    block = block[:block.index("def run_release_only")]
    assert "result_timeout_sec=WORLD_SLOT_STOW_TIMEOUT" in block
    # Every other arm move keeps the original mission-critical budget.
    assert "result_timeout_sec: float = 60.0" in source


def test_stow_timeout_logs_the_joint_state():
    source = PICK_PLACE.read_text(encoding="utf-8")
    assert "arm_joint_state_at_timeout" in source
