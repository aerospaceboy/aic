"""A transport-envelope drift at the delivery dock must not abort the mission."""

from pathlib import Path
from types import SimpleNamespace

REPO = Path(__file__).resolve().parents[2]
PICK_PLACE = REPO / "mybot" / "pick_place_moveit.py"
COORDINATOR = REPO / "tools_demo" / "tools_demo" / "mission_coordinator.py"


def test_place_start_tolerates_a_measured_transport_drift():
    """The transport leg is over at place time; placement gates still apply."""
    source = PICK_PLACE.read_text(encoding="utf-8")
    start = source.index("def run_place_only")
    block = source[start:source.index("def run_release_only", start)]
    assert "transport_envelope_drift_continuing_to_place" in block
    # The old fatal branch must be gone: a drift now records the sample.
    assert "Wrist extended outside the transport envelope in transit" not in block
    assert "self.place_envelope_violation = self.last_envelope_sample" in block


def test_release_still_refuses_a_new_envelope_break_while_placing():
    """Only an already-recorded drift is tolerated; a fresh one still fails."""
    source = PICK_PLACE.read_text(encoding="utf-8")
    block = source[source.index("def run_place_only"):]
    block = block[:block.index("def run_release_only")]
    assert "if self.place_envelope_violation is None:" in block
    assert "Low carry envelope became invalid before release" in block
    assert "transport_envelope_drift_at_release_continuing" in block


def test_drift_reaches_the_delivery_truth():
    """The tolerated drift is reported, not silently swallowed."""
    place = PICK_PLACE.read_text(encoding="utf-8")
    assert '"place_envelope_drift"' in place
    coordinator = COORDINATOR.read_text(encoding="utf-8")
    assert 'self.last_place_envelope_drift = result.get("place_envelope_drift")' in coordinator
    assert "f\"envelope_drift={drift_text}\"" in coordinator


def test_coordinator_captures_the_drift_sample():
    harness = SimpleNamespace(
        last_place_arm_residual=None,
        last_place_envelope_drift=None,
        _refused_place_geometry=None,
    )
    from tools_demo.mission_coordinator import MissionCoordinator

    MissionCoordinator._note_place_outcome(
        harness,
        {
            "success": True,
            "place_slot_correction": [0.0, 0.0],
            "place_envelope_drift": [0.3041, -0.0721, -0.0460, 0.0, 0.0],
        },
        "place",
        True,
    )
    assert harness.last_place_envelope_drift == [
        0.3041, -0.0721, -0.0460, 0.0, 0.0]
    assert harness.last_place_arm_residual == (0.0, 0.0)


def test_non_place_results_leave_the_drift_untouched():
    harness = SimpleNamespace(
        last_place_arm_residual=None,
        last_place_envelope_drift="kept",
        _refused_place_geometry=None,
    )
    from tools_demo.mission_coordinator import MissionCoordinator

    MissionCoordinator._note_place_outcome(
        harness, {"success": True}, "pick", True)
    assert harness.last_place_envelope_drift == "kept"
