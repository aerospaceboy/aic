from pathlib import Path
import json
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
    for name in ('FollowPath', 'CentralTransit', 'SmoothTransit', 'CornerFlow'):
        assert controller[name]['Dynamic']['vy_max'] <= smoother['max_velocity'][1]
        assert controller[name]['Dynamic']['model_dt'] == 1/controller['controller_frequency']


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
