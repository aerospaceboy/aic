"""Keep the low-cost Flash default and the original Max configuration."""
from pathlib import Path

import yaml

CONFIG = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "task_parser.yaml"
)


def _parser_params():
    data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    return data["competition_task_parser"]["ros__parameters"]


def test_thinking_disabled_by_default():
    """Verify-path calls must not use Qwen thinking (latency)."""
    assert _parser_params()["llm_enable_thinking"] is False


def test_active_model_matches_one_complete_saved_profile():
    """Both explicit profile choices must remain deployable and testable."""
    profiles = {"qwen3.8-flash": "task_parser.flash.yaml",
                "qwen3.8-max": "task_parser.max.yaml"}
    model = _parser_params()["model"]
    assert model in profiles
    active = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    selected = yaml.safe_load(CONFIG.with_name(profiles[model]).read_text())
    assert active == selected


def test_max_and_flash_preserve_the_same_non_model_configuration():
    """Switching model must retain endpoint, budgets, topics and protections."""
    flash = yaml.safe_load(CONFIG.with_name("task_parser.flash.yaml").read_text())
    original = yaml.safe_load(CONFIG.with_name("task_parser.max.yaml").read_text())
    original_model = original["competition_task_parser"]["ros__parameters"].pop("model")
    assert original_model == "qwen3.8-max"
    flash["competition_task_parser"]["ros__parameters"].pop("model")
    assert original == flash


def test_node_declare_defaults_to_no_thinking():
    """Source default must match the yaml contract."""
    source = (
        Path(__file__).resolve().parents[1]
        / "tools_demo"
        / "task_parser_node.py"
    ).read_text(encoding="utf-8")
    assert 'declare_parameter("llm_enable_thinking", False)' in source
