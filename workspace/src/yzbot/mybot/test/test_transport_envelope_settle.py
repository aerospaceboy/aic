"""The place precondition must not fail on one stale entity-state sample."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

MODULE_PATH = Path(__file__).parents[1] / "pick_place_moveit.py"
SPEC = importlib.util.spec_from_file_location(
    "pick_place_moveit_settle_probe", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Envelope:
    """Return a scripted sequence of envelope verdicts."""

    def __init__(self, verdicts):
        self.verdicts = list(verdicts)
        self.calls = 0

    def get_logger(self):
        return SimpleNamespace(warn=lambda *a, **k: None)

    def validate_transport_envelope(self):
        self.calls += 1
        if len(self.verdicts) > 1:
            return self.verdicts.pop(0)
        return self.verdicts[0]


def test_a_stale_first_sample_recovers_inside_the_window():
    node = Envelope([False, False, True])
    assert MODULE.MoveItPickPlace.validate_transport_envelope_after_settle(
        node, retries=5, delay_sec=0.0) is True
    assert node.calls == 3


def test_a_persistent_violation_is_still_refused():
    node = Envelope([False])
    assert MODULE.MoveItPickPlace.validate_transport_envelope_after_settle(
        node, retries=4, delay_sec=0.0) is False
    assert node.calls == 5


def test_a_valid_first_sample_is_accepted_without_rechecking():
    node = Envelope([True])
    assert MODULE.MoveItPickPlace.validate_transport_envelope_after_settle(
        node, retries=4, delay_sec=0.0) is True
    assert node.calls == 1


def test_the_place_precondition_is_the_only_settled_call_site():
    source = MODULE_PATH.read_text()
    assert source.count("validate_transport_envelope_after_settle()") == 1
    # The pick path and the pre-release gate keep the strict single sample.
    assert source.count("self.validate_transport_envelope()") >= 4


def test_the_tilt_limit_is_unchanged():
    assert MODULE.TRANSPORT_BASE_TILT_LIMIT == 0.15
