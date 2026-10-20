"""Exercise the campaign's DDS bootstrap without starting Gazebo."""

import ast
import importlib.util
from pathlib import Path

import pytest


REPOSITORY = Path(__file__).resolve().parents[2]


def bootstrap():
    path = REPOSITORY / 'tools_demo/tools_demo/runtime_environment.py'
    spec = importlib.util.spec_from_file_location('runtime_environment', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.configure_campaign_transport


def test_campaign_configures_dds_before_importing_rclpy():
    campaign = REPOSITORY / 'tools_demo/scripts/run_zone9_campaign.py'
    source = campaign.read_text()
    statements = ast.parse(source).body
    calls = [i for i, node in enumerate(statements)
             if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
             and isinstance(node.value.func, ast.Name)
             and node.value.func.id == 'configure_campaign_transport']
    ros_imports = [i for i, node in enumerate(statements)
                   if isinstance(node, ast.Import)
                   and any(x.name == 'rclpy' for x in node.names)]
    assert calls, 'campaign DDS does not match the loopback UDP launcher'
    assert calls[0] < ros_imports[0]
    call = statements[calls[0]].value
    argument = ast.Expression(call.args[0])
    workspace = eval(
        compile(argument, '<bootstrap-path>', 'eval'),
        {'pathlib': __import__('pathlib'), '__file__': str(campaign)},
    )
    assert workspace == REPOSITORY.parent.parent


def test_transport_matches_launcher_and_preserves_domain(tmp_path):
    profile = (tmp_path / 'install/tools_demo/share/tools_demo'
               / 'config/cyclonedds_local.xml')
    profile.parent.mkdir(parents=True)
    profile.write_text('<profiles/>')
    env = {'ROS_DOMAIN_ID': '23', 'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
           'FASTRTPS_DEFAULT_PROFILES_FILE': '/wrong.xml'}
    bootstrap()(tmp_path, env)
    assert env['ROS_DOMAIN_ID'] == '23'
    assert env['RMW_IMPLEMENTATION'] == 'rmw_cyclonedds_cpp'
    assert env['CYCLONEDDS_URI'] == profile.as_uri()
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE' not in env


def test_explicit_fastdds_override_clears_cyclone_profile(tmp_path):
    profile = (tmp_path / 'install/tools_demo/share/tools_demo'
               / 'config/fastdds_udp.xml')
    profile.parent.mkdir(parents=True)
    profile.write_text('<profiles/>')
    env = {'YZBOT_RMW': 'rmw_fastrtps_cpp', 'CYCLONEDDS_URI': 'wrong'}
    bootstrap()(tmp_path, env)
    assert env['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
    assert env['FASTRTPS_DEFAULT_PROFILES_FILE'] == str(profile)
    assert 'CYCLONEDDS_URI' not in env


def test_unknown_rmw_fails_before_changing_environment(tmp_path):
    env = {'YZBOT_RMW': 'invalid'}
    with pytest.raises(ValueError, match='Unsupported'):
        bootstrap()(tmp_path, env)
    assert env == {'YZBOT_RMW': 'invalid'}


def test_transport_fails_before_launch_when_profile_missing(tmp_path):
    env = {}
    with pytest.raises(FileNotFoundError, match='DDS profile'):
        bootstrap()(tmp_path, env)
    assert env == {}
