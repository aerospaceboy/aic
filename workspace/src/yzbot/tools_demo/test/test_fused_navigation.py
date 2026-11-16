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
    for name in ['robot_radius', 'obstacle_layer', 'static_layer', 'inflation_layer']:
        assert new[name] == old[name]
    assert set(new['plugins']) == set(old['plugins'])
    # StaticLayer overwrites existing cells when use_maximum is false.
    # Sensed obstacles must be applied after the static map in both maps.
    assert new['plugins'].index('static_layer') < new['plugins'].index('obstacle_layer')
    assert new['prediction_layer']['enabled']
    # The selected release follower is RPP. Its conservative future spatial
    # costs must remain visible while experimental timed MPPI is disabled.
    assert not new['prediction_layer']['timed_future_cost']


def test_collision_monitor_preserves_original_zones_and_adds_stopping_preview():
    new=dict(FUSED['collision_monitor']['ros__parameters'])
    preview=new.pop('FootprintApproach')
    new['polygons']=[p for p in new['polygons'] if p!='FootprintApproach']
    assert new == BASE['collision_monitor']['ros__parameters']
    assert FUSED['velocity_smoother'] == BASE['velocity_smoother']
    cruise=max(FUSED['controller_server']['ros__parameters'][name]['Nominal']['desired_linear_vel']
               for name in ('FollowPath','CentralTransit','SmoothTransit','CornerFlow','CornerFlowA'))
    deceleration=abs(FUSED['velocity_smoother']['ros__parameters']['max_decel'][0])
    assert preview['time_before_collision']>=cruise/deceleration+.15
    assert preview['footprint_topic']=='/local_costmap/published_footprint'
    assert preview['action_type']=='approach' and preview['enabled']


def test_coherent_pursuit_preserves_geometry_and_bounded_path_search():
    old = BASE['controller_server']['ros__parameters']
    new = FUSED['controller_server']['ros__parameters']
    for name in old['controller_plugins']:
        actual = new[name].get('Nominal', new[name])
        assert actual['plugin'] == 'bot_navigation::CoherentPursuitController'
        allowed = {'plugin'}
        if 'Nominal' in new[name]:
            allowed.add('max_robot_pose_search_dist')
        assert {k: v for k, v in actual.items() if k not in allowed} == {
            k: v for k, v in old[name].items() if k not in allowed}
        if 'Nominal' in new[name]:
            assert actual['max_robot_pose_search_dist'] == 1.5
            assert actual['use_collision_detection']
