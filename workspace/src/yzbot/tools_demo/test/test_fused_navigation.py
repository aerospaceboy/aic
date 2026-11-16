"""The live fused profile preserves wheel motion and existing safety geometry."""
from pathlib import Path
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
BASE = yaml.safe_load((ROOT / 'bot_navigation/param/originbot_nav2.yaml').read_text())
FUSED = yaml.safe_load((ROOT / 'bot_navigation/config/fusion_nav2.yaml').read_text())


@pytest.mark.parametrize('name', [
    'FollowPath', 'CentralTransit', 'SmoothTransit', 'CornerFlow', 'CornerFlowA'])
def test_dynamic_candidates_obey_same_nonholonomic_model_as_execution(name):
    controllers = FUSED['controller_server']['ros__parameters']
    controller = controllers[name]
    assert not controller['continuous_transit']
    assert not controller['enable_dynamic_avoidance']
    assert controller['require_nonholonomic']
    assert controller['Dynamic']['motion_model'] == 'DiffDrive'
    assert controller['Dynamic']['vy_max'] == 0.
    assert controller['Dynamic']['vy_std'] == 0.
    assert controller['Dynamic']['model_dt'] == 1. / controllers['controller_frequency']
    assert controller['Dynamic']['StaticClearanceCritic']['clearance'] >= .48
    assert controller['dynamic_release_delay'] >= .5
    assert controller['dynamic_reentry_delay'] >= .75


@pytest.mark.parametrize('group', ['local_costmap', 'global_costmap'])
def test_fusion_retains_original_map_radius_and_live_collision_sources(group):
    old = BASE[group][group]['ros__parameters']
    new = FUSED[group][group]['ros__parameters']
    for name in ['robot_radius', 'plugins', 'obstacle_layer', 'static_layer', 'inflation_layer']:
        assert new[name] == old[name]
    assert new['prediction_layer']['enabled']
    # The selected release follower is RPP. Its conservative future spatial
    # costs must remain visible while experimental timed MPPI is disabled.
    assert not new['prediction_layer']['timed_future_cost']


def test_collision_monitor_limits_are_unchanged_and_precision_adjustments_remain_bounded():
    assert FUSED['collision_monitor'] == BASE['collision_monitor']
    assert FUSED['velocity_smoother'] == BASE['velocity_smoother']


def test_all_existing_nominal_followers_keep_their_wheel_motion_settings():
    old = BASE['controller_server']['ros__parameters']
    new = FUSED['controller_server']['ros__parameters']
    for name in old['controller_plugins']:
        assert new[name].get('Nominal', new[name]) == old[name]
