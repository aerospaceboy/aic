"""Contracts for physics-safe pickup stabilization."""

from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "pick_place_moveit.py"


def test_pick_cube_is_anchored_and_correction_retracts_vertically_first():
    source = MODULE_PATH.read_text(encoding="utf-8")
    pick_body = source.split("def run_pick_only", 1)[1].split(
        "def run_scan_only", 1
    )[0]
    assert "self.stabilize_pick_cube()" in pick_body
    assert "self.release_pick_cube_stabilization()" in pick_body
    vertical = source.index('"pick_correct_retract_vertical"')
    lateral = source.index('"pick_correct_shift"')
    assert vertical < lateral
    assert "pick_stabilized_cubes" in source
