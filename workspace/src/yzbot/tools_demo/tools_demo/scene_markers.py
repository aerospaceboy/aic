"""Display live Gazebo entities in the competition map frame for CoStudio.

The competition map, odom and Gazebo world share an origin (the navigation
launch supplies identity map->odom). This is visualization-only ground truth;
no marker data enters navigation, perception or manipulation control.
"""

from copy import deepcopy
from collections import deque
import json
import math
import time

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def entity_style(name):
    """Return stable labels, geometry and colors for known models."""
    if name in {f'{color}_cube_{i}' for color in ('red', 'blue')
                for i in range(1, 6)}:
        red = name.startswith('red')
        color = (1.0, 0.15, 0.22, 1.0) if red else (0.15, 0.45, 1.0, 1.0)
        return ('cubes', name, Marker.CUBE, (0.10, 0.10, 0.10), color)
    if name in ('zone_a', 'zone_b', 'zone_c'):
        zone = name[-1].upper()
        zone_colors = {
            'A': (0.18, 0.82, 0.36, 0.78),
            'B': (0.96, 0.68, 0.10, 0.82),
            'C': (0.08, 0.72, 0.82, 0.80),
        }
        return ('zones', zone, Marker.CUBE,
                (1.0, 0.5, 0.025), zone_colors[zone])
    if name in ('crossing_cart_1', 'crossing_cart_2'):
        return ('obstacles', 'Central cart ' + name[-1], Marker.CYLINDER,
                (0.28, 0.28, 0.50), (0.65, 0.18, 0.85, 0.9))
    if name in ('preview_moving_partition_1', 'preview_moving_partition_2'):
        size = (1.6, 0.12, 1.2) if name.endswith('1') else (0.12, 1.6, 1.2)
        return ('obstacles', 'Moving wall ' + name[-1], Marker.CUBE,
                size, (0.65, 0.18, 0.85, 0.9))
    if name == 'six_arm':
        return ('robot', 'YZBot', Marker.ARROW,
                (0.7, 0.18, 0.18), (0.1, 1.0, 1.0, 1.0))
    if name in ('competition_moving_obstacle_1',
                'competition_moving_obstacle_2'):
        return ('obstacles', 'Obstacle ' + name[-1], Marker.CYLINDER,
                (0.36, 0.36, 0.50), (1.0, 0.45, 0.05, 0.9))
    return None


def build_markers(states, stamp, frame='map'):
    """Build a complete snapshot with stable IDs and stale deletion."""
    result = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    clear.header.frame_id = frame
    clear.header.stamp = stamp
    result.markers.append(clear)
    for name, pose in zip(states.name, states.pose):
        style = entity_style(name)
        if style is None:
            continue
        values = (pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w)
        if not all(math.isfinite(value) for value in values):
            continue
        namespace, label, kind, size, color = style
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = stamp
        marker.ns = namespace + '/' + name
        marker.id = 0
        marker.type = kind
        marker.action = Marker.ADD
        marker.pose = deepcopy(pose)
        if namespace in ('zones', 'robot'):
            marker.pose.position.z += 0.04
        marker.scale.x, marker.scale.y, marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime.sec = 2
        result.markers.append(marker)
        text = deepcopy(marker)
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.position.z += 0.45
        text.pose.orientation.x = 0.0
        text.pose.orientation.y = 0.0
        text.pose.orientation.z = 0.0
        text.pose.orientation.w = 1.0
        text.scale.x = text.scale.y = 0.0
        text.scale.z = 0.23
        if namespace == 'zones':
            text.color.r = text.color.g = text.color.b = 0.05
            text.color.a = 1.0
        else:
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = label
        result.markers.append(text)
    return result


class RouteHistory:
    """Bounded display data, independent of the robot's control state."""

    def __init__(self, max_points=1500):
        self.trail = deque(maxlen=max_points)
        self.plan = []
        self.plan_received = 0.0
        self.task_id = None
        self.terminal = False
        self.navigation_open = True
        self.navigation_goal = None

    def mission(self, payload):
        """Reset on a new task and stop showing terminal navigation goals."""
        task_id = payload.get('task_id')
        if task_id and task_id != self.task_id:
            self.trail.clear()
            self.plan.clear()
            self.task_id = task_id
            self.navigation_open = True
            self.navigation_goal = None
        self.terminal = payload.get('state') in (
            'completed', 'failed', 'canceled')
        if self.terminal:
            self.plan.clear()
            self.navigation_open = False

    def navigation(self, payload):
        """Retain the route until its navigation leg actually ends."""
        state = payload.get('state')
        if state in ('succeeded', 'failed', 'canceled', 'idle'):
            self.plan.clear()
            self.navigation_open = False
            self.navigation_goal = None
        elif state == 'active' and not self.terminal:
            goal = payload.get('goal')
            identity = (payload.get('zone'),
                        tuple(goal) if isinstance(goal, list) else None)
            if self.navigation_goal not in (None, identity):
                self.plan.clear()
            self.navigation_goal = identity
            self.navigation_open = True

    def position(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return
        if self.trail:
            distance = math.hypot(x - self.trail[-1][0], y - self.trail[-1][1])
            if distance > 2.0:
                # Gazebo reset/teleport must not draw a line across the map.
                self.trail.clear()
                self.plan.clear()
            elif distance < 0.06:
                return
        self.trail.append((x, y))

    def route(self, message, now):
        # This competition uses identity map->odom. Do not relabel an
        # unrelated coordinate frame as map when consuming a foreign path.
        frames = {message.header.frame_id}
        frames.update(p.header.frame_id for p in message.poses
                      if p.header.frame_id)
        if (self.terminal or not self.navigation_open
                or not frames.issubset({'map', 'odom'})):
            self.plan.clear()
            return
        points = [(p.pose.position.x, p.pose.position.y)
                  for p in message.poses]
        if not all(math.isfinite(v) for p in points for v in p):
            self.plan.clear()
            return
        stride = max(1, math.ceil(len(points) / 999))
        self.plan = points[::stride]
        if points and self.plan[-1] != points[-1]:
            self.plan.append(points[-1])
        self.plan_received = now

    def markers(self, stamp, now):
        result = []
        for name, points, color, height, width, alpha in (
                ('planned', self.plan, (1.0, 0.85, 0.05), 0.14, 0.14, 1.0),
                ('traveled', self.trail, (0.05, 0.95, 1.0), 0.06, 0.035, 0.7)):
            if len(points) < 2:
                continue
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'navigation/' + name
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = width
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = alpha
            marker.lifetime.sec = 2
            marker.points = [Point(x=float(x), y=float(y), z=height)
                             for x, y in points]
            result.append(marker)
        return result


class SceneMarkers(Node):
    """Publish bounded-rate snapshots and a compatible retained map alias."""

    def __init__(self):
        super().__init__('competition_scene_markers')
        retained = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            MarkerArray, '/competition/scene_markers', retained)
        self.map_publisher = self.create_publisher(
            OccupancyGrid, '/competition/map', retained)
        self.create_subscription(OccupancyGrid, '/map',
                                 self.map_publisher.publish, retained)
        self.create_subscription(ModelStates, '/gazebo/model_states',
                                 self.receive, qos_profile_sensor_data)
        self.routes = RouteHistory()
        self.create_subscription(Path, '/plan', self.receive_plan, 1)
        self.create_subscription(String, '/competition/mission/state',
                                 self.receive_mission, retained)
        self.create_subscription(String, '/competition/navigation/state',
                                 self.receive_navigation, retained)
        self.last_publish = 0.0
        self.last_receive = time.monotonic()
        self.stale = True
        # A wall-clock timer is deliberate: remove frozen positions even when
        # the simulator (and therefore /clock) stops publishing.
        self.wall_clock = rclpy.clock.Clock(
            clock_type=rclpy.clock.ClockType.STEADY_TIME)
        self.create_timer(0.5, self.expire, clock=self.wall_clock)

    def receive(self, states):
        """Refresh markers at at most 5 Hz without additional Gazebo RPCs."""
        now = time.monotonic()
        self.last_receive = now
        if now - self.last_publish < 0.2:
            return
        for name, pose in zip(states.name, states.pose):
            if name == 'six_arm':
                self.routes.position(pose.position.x, pose.position.y)
                break
        stamp = self.get_clock().now().to_msg()
        markers = build_markers(states, stamp)
        # Same snapshot: DELETEALL would erase paths from a separate publisher.
        markers.markers.extend(self.routes.markers(stamp, now))
        self.publisher.publish(markers)
        self.last_publish = now
        self.stale = False

    def receive_plan(self, message):
        self.routes.route(message, time.monotonic())

    def receive_mission(self, message):
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if isinstance(payload, dict):
            self.routes.mission(payload)

    def receive_navigation(self, message):
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if isinstance(payload, dict):
            self.routes.navigation(payload)

    def expire(self):
        """Clear cached scene when the truth source stops for two seconds."""
        if not self.stale and time.monotonic() - self.last_receive > 2.0:
            self.publisher.publish(build_markers(
                ModelStates(), self.get_clock().now().to_msg()))
            self.routes.trail.clear()
            self.routes.plan.clear()
            self.stale = True


def main(args=None):
    """Run the read-only scene visualization node."""
    rclpy.init(args=args)
    node = SceneMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

