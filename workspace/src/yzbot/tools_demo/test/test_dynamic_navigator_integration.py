"""Integration tests for the navigator <-> dynamic supervisor wiring."""

from types import SimpleNamespace

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from tools_demo.dynamic_local_supervisor import SupervisorOutput
from tools_demo.task_navigator import TaskNavigator


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Clock:
    def __init__(self, stamp=100.0):
        self.stamp = stamp

    def now(self):
        return SimpleNamespace(nanoseconds=int(self.stamp * 1e9))


class StubSupervisor:
    def __init__(self, output=None, error=None):
        self.output = output
        self.error = error
        self.calls = 0

    def step(self, now, robot, payload, obstacles, goal_local, **kwargs):
        self.calls += 1
        if self.error is not None:
            raise self.error
        return self.output


def build_node(supervisor, goal=SimpleNamespace(x=5.0, y=0.0)):
    node = SimpleNamespace(
        latest_x=0.0, latest_y=0.0, latest_yaw=0.0,
        latest_vx=0.3, latest_vy=0.0,
        dynamic_supervisor=supervisor,
        dynamic_owner=False,
        dynamic_supervisor_error=None,
        dynamic_supervisor_last_log_ns=0,
        delivery_slot_world=None,
        nav_goal_pose=goal,
        obstacle_positions={},
        obstacle_source_stamps={},
        dynamic_obstacle_history={},
        dynamic_obstacle_velocity={},
        obstacle_radius_map={},
        obstacle_topics={},
        alignment_velocity_publisher=Recorder(),
        direct_control_publisher=Recorder(),
        _direct_control=False,
        _ignored=[],
    )
    node.get_clock = lambda: Clock()
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *a, **k: None, warn=lambda *a, **k: None,
        error=lambda *a, **k: None)
    node.obstacle_positions = {'/competition/obstacles/obstacle_1': (8.5, 0.2)}
    node.obstacle_source_stamps = {'/competition/obstacles/obstacle_1': 100.0}
    node.dynamic_obstacle_history = {
        '/competition/obstacles/obstacle_1': [(99.9, 8.5, 0.5), (100.0, 8.5, 0.2)]}
    node.dynamic_obstacle_velocity = {
        '/competition/obstacles/obstacle_1': (0.0, -0.3)}
    node._dynamic_planner_obstacles = (
        TaskNavigator._dynamic_planner_obstacles.__get__(node))
    node._publish_direct_velocity = (
        TaskNavigator._publish_direct_velocity.__get__(node))
    node._release_direct_control = (
        TaskNavigator._release_direct_control.__get__(node))
    node._costmap_cell_free = lambda x, y: True
    node._dynamic_static_check = (
        TaskNavigator._dynamic_static_check.__get__(node))
    return node


def test_active_supervisor_command_takes_direct_ownership():
    supervisor = StubSupervisor(SupervisorOutput(
        'active', (0.25, -0.1, 0.0), True, False, {'kind': 'before'}))
    node = build_node(supervisor)
    owned = TaskNavigator._dynamic_planner_step(node)
    assert owned is True
    assert node.dynamic_owner is True
    assert node._direct_control is True
    command = node.alignment_velocity_publisher.messages[-1]
    assert isinstance(command, Twist)
    assert abs(command.linear.x - 0.25) < 1e-9
    assert abs(command.linear.y + 0.1) < 1e-9


def test_observe_supervisor_never_touches_the_base():
    supervisor = StubSupervisor(SupervisorOutput(
        'observing', None, False, False, {'kind': 'before'}))
    node = build_node(supervisor)
    owned = TaskNavigator._dynamic_planner_step(node)
    assert owned is False
    assert node.dynamic_owner is False
    assert node._direct_control is False
    assert not node.alignment_velocity_publisher.messages


def test_supervisor_exception_disables_innovation_and_releases_control():
    supervisor = StubSupervisor(error=RuntimeError('planner boom'))
    node = build_node(supervisor)
    node._direct_control = True
    node.dynamic_owner = True
    owned = TaskNavigator._dynamic_planner_step(node)
    assert owned is False
    assert node.dynamic_supervisor is None
    assert 'planner boom' in node.dynamic_supervisor_error
    assert node.dynamic_owner is False
    assert node._direct_control is False
    assert any(isinstance(message, Bool) for message in
               node.direct_control_publisher.messages)


def test_no_goal_releases_ownership():
    supervisor = StubSupervisor(SupervisorOutput(
        'active', (0.2, 0.0, 0.0), True, False, {}))
    node = build_node(supervisor, goal=None)
    node._direct_control = True
    node.dynamic_owner = True
    owned = TaskNavigator._dynamic_planner_step(node)
    assert owned is False
    assert node.dynamic_owner is False
