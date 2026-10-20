"""Lock LLM verify-path defaults: thinking off, model qwen3.8-max."""
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


def test_model_is_qwen38_max_unless_user_renames():
    """Ship with the agreed competition model id."""
    assert _parser_params()["model"] == "qwen3.8-max"


def test_node_declare_defaults_to_no_thinking():
    """Source default must match the yaml contract."""
    source = (
        Path(__file__).resolve().parents[1]
        / "tools_demo"
        / "task_parser_node.py"
    ).read_text(encoding="utf-8")
    assert 'declare_parameter("llm_enable_thinking", False)' in source
