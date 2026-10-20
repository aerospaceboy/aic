"""Resolve competition Nav2 parameters under a different installed prefix."""

import copy
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
import yaml


def test_behavior_tree_follows_relocated_package(tmp_path, monkeypatch):
    """Keep all control values while resolving the project BT from install."""
    root = Path(__file__).parents[2]
    filename = root / "bot_navigation/launch/competition_navigation.launch.py"
    spec = importlib.util.spec_from_file_location("usb_nav_launch", filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    relocated = tmp_path / "another-user/install/bot_navigation/share"
    config = relocated / "config"
    config.mkdir(parents=True)
    tree = "navigate_to_pose_recovery.xml"
    (config / tree).write_bytes(
        (root / "bot_navigation/config" / tree).read_bytes()
    )
    monkeypatch.setattr(
        module, "get_package_share_directory",
        lambda name: str(relocated) if name == "bot_navigation"
        else get_package_share_directory(name),
    )
    description = module.generate_launch_description()
    group = next(e for e in description.entities if isinstance(e, GroupAction))
    include = next(
        e for e in group.get_sub_entities()
        if isinstance(e, IncludeLaunchDescription)
    )
    context = LaunchContext()
    source = root / "bot_navigation/param/originbot_nav2.yaml"
    context.launch_configurations["params_file"] = str(source)
    substitution = dict(include.launch_arguments)["params_file"]
    generated = perform_substitutions(
        context, normalize_to_list_of_substitutions(substitution)
    )
    original = yaml.safe_load(source.read_text())
    actual = yaml.safe_load(Path(generated).read_text())
    expected = copy.deepcopy(original)
    expected["bt_navigator"]["ros__parameters"][
        "default_nav_to_pose_bt_xml"
    ] = str(config / tree)
    assert actual == expected
    assert Path(actual["bt_navigator"]["ros__parameters"][
        "default_nav_to_pose_bt_xml"
    ]).is_file()
