"""Behavioral checks for the read-only CoStudio scene snapshot."""

from builtin_interfaces.msg import Time
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from tools_demo.scene_markers import RouteHistory, build_markers


def snapshot(names):
    states = ModelStates()
    states.name = names
    for index, _ in enumerate(names):
        pose = Pose()
        pose.position.x = float(index)
        pose.position.y = -float(index)
        pose.position.z = 0.5
        pose.orientation.w = 1.0
        states.pose.append(pose)
    return states


def test_all_scored_entities_and_labels_have_truth_positions():
    names = [f'{color}_cube_{i}' for color in ('red', 'blue')
             for i in range(1, 6)]
    names += ['zone_a', 'zone_b', 'zone_c', 'six_arm',
              'competition_moving_obstacle_1', 'competition_moving_obstacle_2']
    result = build_markers(snapshot(names), Time(sec=12))
    assert len(result.markers) == 33
    assert result.markers[0].action == Marker.DELETEALL
    objects = result.markers[1::2]
    for i, marker in enumerate(objects):
        assert marker.pose.position.x == float(i)
        assert marker.pose.position.y == -float(i)
        assert marker.header.frame_id == 'map'
        assert marker.header.stamp.sec == 12
        assert marker.lifetime.sec == 2
    assert [m.text for m in result.markers[2::2]][10:13] == ['A', 'B', 'C']
    zone_labels = result.markers[2::2][10:13]
    assert all(marker.scale.z == 0.23 for marker in zone_labels)
    assert all(marker.color.r == 0.05 for marker in zone_labels)


def test_transport_and_obstacle_movement_update_without_changing_ids():
    states = snapshot(['red_cube_1', 'competition_moving_obstacle_1'])
    before = build_markers(states, Time())
    states.pose[0].position.x = 7.0
    states.pose[0].position.z = 1.2
    states.pose[1].position.y = 4.0
    after = build_markers(states, Time(sec=1))
    assert after.markers[1].pose.position.x == 7.0
    assert after.markers[1].pose.position.z == 1.2
    assert after.markers[3].pose.position.y == 4.0
    assert [(m.ns, m.id) for m in before.markers] == [
        (m.ns, m.id) for m in after.markers]
    assert before.markers[1].pose.position.x == 0.0


def test_reset_removes_old_entities_and_ignores_anchors_and_invalid_poses():
    states = snapshot(['red_cube_1_anchor', 'ground_plane', 'blue_cube_2'])
    states.pose[2].position.x = float('nan')
    result = build_markers(states, Time())
    assert len(result.markers) == 1
    assert result.markers[0].action == Marker.DELETEALL
    assert len(build_markers(ModelStates(), Time()).markers) == 1


def path_message(points, frame='map'):
    message = Path()
    message.header.frame_id = frame
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        message.poses.append(pose)
    return message


def test_plan_persists_until_goal_completion_and_is_visually_distinct():
    history = RouteHistory()
    history.position(0.0, 0.0)
    history.position(0.1, 0.0)
    history.route(path_message([(0, 0), (3, 4)]), 10.0)
    markers = history.markers(Time(), 11.0)
    assert [m.ns for m in markers] == [
        'navigation/planned', 'navigation/traveled']
    assert markers[0].points[-1].x == 3.0
    assert markers[1].points[-1].x == 0.1
    assert all(m.type == Marker.LINE_STRIP for m in markers)
    assert [m.ns for m in history.markers(Time(), 200.0)] == [
        'navigation/planned', 'navigation/traveled']
    assert markers[0].scale.x == 4 * markers[1].scale.x
    assert markers[0].points[0].z > markers[1].points[0].z
    history.navigation({'state': 'succeeded'})
    assert [m.ns for m in history.markers(Time(), 201.0)] == [
        'navigation/traveled']


def test_goal_change_clears_route_and_ignores_late_terminal_plan():
    history = RouteHistory()
    history.navigation({'state': 'active', 'zone': 'A', 'goal': [3, 4]})
    history.route(path_message([(0, 0), (3, 4)]), 10.0)
    history.navigation({'state': 'active', 'zone': 'A', 'goal': [3, 4],
                        'phase': 'alignment'})
    assert history.plan
    history.navigation({'state': 'active', 'zone': 'B', 'goal': [5, 6]})
    assert not history.plan
    history.route(path_message([(3, 4), (5, 6)]), 20.0)
    assert history.plan[-1] == (5.0, 6.0)
    history.navigation({'state': 'failed'})
    history.route(path_message([(3, 4), (5, 6)]), 21.0)
    assert not history.plan


def test_new_round_and_teleport_clear_old_routes_but_phase_change_does_not():
    history = RouteHistory()
    history.mission({'task_id': 'one', 'state': 'navigating_source'})
    history.position(1.0, 2.0)
    history.position(1.2, 2.0)
    history.mission({'task_id': 'one', 'state': 'picking'})
    assert len(history.trail) == 2
    history.mission({'task_id': 'two', 'state': 'navigating_source'})
    assert not history.trail
    history.position(1.0, 2.0)
    history.route(path_message([(1, 2), (2, 2)]), 10.0)
    history.position(10.0, 2.0)
    assert list(history.trail) == [(10.0, 2.0)]
    assert not history.plan


def test_invalid_path_and_terminal_updates_cannot_draw_stale_route():
    history = RouteHistory()
    for message in (path_message([(1, 2), (3, 4)], 'camera'),
                    path_message([(float('nan'), 2)])):
        history.route(message, 10.0)
        assert not history.plan
    history.route(path_message([(1, 2), (3, 4)], 'odom'), 10.0)
    assert history.plan
    history.mission({'task_id': 'one', 'state': 'completed'})
    history.route(path_message([(1, 2), (3, 4)]), 11.0)
    assert not history.plan


def test_storage_is_bounded_and_display_retains_plan_endpoint():
    history = RouteHistory(max_points=4)
    for i in range(100):
        history.position(i * 0.1, 0.0)
    assert len(history.trail) == 4
    history.position(float('inf'), 0.0)
    assert len(history.trail) == 4
    history.route(path_message([(i, 0) for i in range(5000)]), 10.0)
    assert len(history.plan) <= 1000
    assert history.plan[-1] == (4999.0, 0.0)


