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
    stow = block.index("return_home_after_world_slot")
    tail = block[stow:]
    # The failure branch retracts and warns instead of returning False.
    assert "return_home_after_world_slot_failed_after_delivery" in tail
    assert "deferred_stow" in tail
    failed_branch = tail[:tail.index("self.publish_state(f\"place_complete")]
    assert "return False" not in failed_branch


def test_release_still_requires_verification_before_the_stow():
    """Only a verified delivery may survive a stow failure."""
    source = PICK_PLACE.read_text(encoding="utf-8")
    block = source[source.index("def run_place_only"):]
    block = block[:block.index("def run_release_only")]
    verify = block.index("verify_placement()")
    stow = block.index("return_home_after_world_slot")
    assert verify < stow
    assert "Placement verification failed" in block[verify:stow]
