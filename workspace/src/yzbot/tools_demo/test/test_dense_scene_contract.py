from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET
import yaml
import xacro

ROOT = Path(__file__).resolve().parents[2]


def test_dense_sensor_resolution_and_original_scene_recovery():
    path = ROOT / 'mybot_description/urdf/originbot_with_rgbd_gazebo_arm.xacro'
    for mode, rate, samples in [('0', 10, 240), ('1', 20, 720)]:
        robot = ET.fromstring(xacro.process_file(str(path), mappings={'dense_scene': mode}).toxml())
        sensor = robot.find(".//sensor[@name='lidar']")
        assert float(sensor.findtext('update_rate')) == rate
        assert int(sensor.findtext('ray/scan/horizontal/samples')) == samples
        assert float(sensor.findtext('ray/range/min')) == .38


def test_node_scoped_overrides_cover_exactly_every_actor():
    config = ROOT / 'bot_navigation/config'
    scene = json.loads((config / 'dense_transport_scene.json').read_text())
    override = yaml.safe_load((config / 'dense_scene_overrides.yaml').read_text())
    sensor = override['competition_dynamic_obstacles']['ros__parameters']
    assert sensor['prediction_exclusion_bounds'] == [0., 0., 0., 0.]
    monitor = override['competition_avoidance_monitor']['ros__parameters']
    assert monitor['expected_obstacle_count'] == 4 + len(scene['actors'])
    assert len(set(monitor['obstacle_topics'])) == monitor['expected_obstacle_count']
    for actor in scene['actors']:
        assert '/competition/obstacles/' + actor['name'] in monitor['obstacle_topics']


def test_mppi_and_actuator_lateral_limits_are_consistent():
    config = yaml.safe_load((ROOT / 'bot_navigation/config/dense_nav2.yaml').read_text())
    controller = config['controller_server']['ros__parameters']
    smoother = config['velocity_smoother']['ros__parameters']
    assert controller['min_y_velocity_threshold'] < .01
    for name in ('FollowPath', 'CentralTransit', 'SmoothTransit', 'CornerFlow', 'CornerFlowA'):
        assert controller[name]['Dynamic']['vy_max'] <= smoother['max_velocity'][1]
        assert controller[name]['Dynamic']['model_dt'] == 1/controller['controller_frequency']


def test_stage_models_keep_full_lateral_authority_without_reset_speed_increase():
    config = yaml.safe_load((ROOT / 'bot_navigation/config/dense_nav2.yaml').read_text())
    controller = config['controller_server']['ros__parameters']
    for name, speed in [('CornerFlowA', .75), ('CornerFlow', 1.2),
                        ('SmoothTransit', 1.5), ('CentralTransit', 1.)]:
        assert controller[name]['Dynamic']['vx_max'] == speed
        assert controller[name]['Dynamic']['vy_max'] == .6
    for name in ('navigate_through_corner_a_dense.xml', 'navigate_corner_a_dense_to_pose.xml'):
        tree = ET.parse(ROOT / 'tools_demo/config' / name)
        for follower in tree.iter('FollowPath'):
            assert follower.attrib['controller_id'] == 'CornerFlowA'
            assert 'CornerFlowA' in controller['controller_plugins']


def test_live_costmap_marks_survive_static_map_composition():
    config = yaml.safe_load((ROOT / 'bot_navigation/config/dense_nav2.yaml').read_text())
    for name in ('local_costmap', 'global_costmap'):
        plugins = config[name][name]['ros__parameters']['plugins']
        # Native StaticLayer overwrites free-space cells. Dynamic observations
        # and placed cargo must be composed afterwards, before inflation.
        for live in ('obstacle_layer', 'dynamic_layer', 'prediction_layer'):
            if live in plugins:
                assert plugins.index('static_layer') < plugins.index(live)
                assert plugins.index(live) < plugins.index('inflation_layer')


def test_dense_route_guidance_retains_static_and_cargo_with_live_local_protection():
    config = yaml.safe_load((ROOT / 'bot_navigation/config/dense_nav2.yaml').read_text())
    global_map = config['global_costmap']['global_costmap']['ros__parameters']
    local_map = config['local_costmap']['local_costmap']['ros__parameters']
    # Narrow sections use the original follower with current-body global
    # routing. Long-horizon moving risk is evaluated by the timed controller
    # only in open space, rather than painting an all-times global union.
    assert global_map['obstacle_layer']['enabled']
    assert 'placed_cubes' in global_map['obstacle_layer']['observation_sources']
    assert global_map['sweep_band_layer']['enabled']
    assert global_map['dynamic_layer']['enabled']
    assert global_map['prediction_layer']['enabled']
    assert global_map['prediction_layer']['timed_future_cost']
    assert global_map['prediction_layer']['robot_egress_radius'] > global_map['robot_radius'] + .05
    # Only global guidance is persistent: current local laser occupancy and
    # current/forecast sensor tracks continue to constrain executable motion.
    assert local_map['obstacle_layer']['enabled']
    assert 'scan' in local_map['obstacle_layer']['observation_sources']
    assert local_map['prediction_layer']['enabled']


def test_time_indexed_predictions_replace_only_future_spatial_union():
    base = ROOT / 'bot_navigation/config'
    config = yaml.safe_load((base / 'dense_nav2.yaml').read_text())
    local = config['local_costmap']['local_costmap']['ros__parameters']
    assert local['prediction_layer']['timed_future_cost']
    # MPPI may not cut into the global planner's static inflation shell.
    assert local['static_layer']['map_topic'] == '/competition/dense_static_map'
    global_map = config['global_costmap']['global_costmap']['ros__parameters']
    assert global_map['static_layer']['map_topic'] == '/competition/dense_global_static_map'
    assert math.isclose(local['robot_radius'] + .20, global_map['robot_radius'] + .10)
    sensor = yaml.safe_load((base / 'dense_scene_overrides.yaml').read_text())['competition_dynamic_obstacles']['ros__parameters']
    assert sensor['prediction_robot_radius'] == local['robot_radius']
    assert sensor['small_body_clearance'] + local['robot_radius'] >= .46 - 1.e-9
    for name in ('FollowPath', 'CentralTransit', 'SmoothTransit', 'CornerFlow', 'CornerFlowA'):
        model = config['controller_server']['ros__parameters'][name]['Dynamic']
        assert model['SensorTimeCritic']['enabled']
        assert model['GoalAngleCritic']['enabled'] == (name == 'FollowPath')
        # The largest bounded future penalty must not dominate a hard
        # current/static collision penalty, which native CostCritic normalizes.
        future_bound = (2*model['SensorTimeCritic']['collision_cost'] + 20)*model['model_dt']*model['time_steps']
        hard = model['CostCritic']['collision_cost']*model['CostCritic']['cost_weight']/254/model['time_steps']
        assert hard > 2*future_bound
