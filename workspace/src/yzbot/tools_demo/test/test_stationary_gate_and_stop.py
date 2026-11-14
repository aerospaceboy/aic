"""Behaviour tests for the G0 placement stop/freshness gate.

The review of the previous candidate found two safety holes:
  1. ``_wait_base_stationary`` compared the same cached odom twice, so a
     frozen /odom stream looked stationary and placement proceeded from a
     moving base;
  2. every ``_bounded_place_shift`` exit reused the same Twist, so the last
     angular.z stayed alive after the "stop" and the base could still rotate
     while placement geometry was measured.

These tests drive the real production methods with executable stubs, not
formula mirrors.
"""

import math
import threading
import time
from types import SimpleNamespace

from geometry_msgs.msg import Twist

from tools_demo.mission_coordinator import (
    MissionCoordinator,
    PLACE_REPOSITION_ITERATIONS,
)


class FakePublisher:
    def __init__(self):
        self.messages = []
        self.lock = threading.Lock()

    def publish(self, message):
        with self.lock:
            self.messages.append((message.linear.x, message.linear.y,
                                  message.angular.z))

    def last(self):
        with self.lock:
            return self.messages[-1]


class OdomDriver:
    """Executable odom stream for the stationarity gate."""

    def __init__(self, pose=(0.0, 0.0, 0.0), speeds=(0.0, 0.0),
                 step=0.02, moving=None):
        self.pose = pose
        self.speeds = speeds
        self.step = step
        self.moving = moving
        self.sequence = 0
        self.received = time.monotonic()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)

    def _run(self):
        while not self.stop_event.wait(self.step):
            if self.moving is not None:
                x, y, yaw = self.pose
                dx, dy, dyaw = self.moving
                self.pose = (x + dx, y + dy, math.remainder(yaw + dyaw,
                                                           2 * math.pi))
            self.sequence += 1
            self.received = time.monotonic()


def coordinator_node(driver, publisher=None):
    node = SimpleNamespace(
        odom_pose=driver.pose,
        odom_sequence=driver.sequence,
        odom_received_monotonic=driver.received,
        odom_linear_speed=driver.speeds[0],
        odom_angular_speed=driver.speeds[1],
        cmd_vel_publisher=publisher or FakePublisher(),
    )
    original = MissionCoordinator._odom_sample

    def sample():
        return {
            'pose': driver.pose,
            'sequence': driver.sequence,
            'received': driver.received,
            'linear_speed': driver.speeds[0],
            'angular_speed': driver.speeds[1],
        }

    node._odom_sample = sample
    node._odom_pose = lambda: driver.pose
    node._odom_pose.__name__ = 'odom'
    return node


def test_frozen_cached_pose_is_not_stationary():
    driver = OdomDriver().start()
    try:
        time.sleep(0.1)
        frozen = coordinator_node(driver)
        # Simulate a /odom stream that stopped updating: the sequence never
        # advances even though the cached pose is identical every look.
        stuck = SimpleNamespace(
            odom_pose=driver.pose,
            odom_sequence=driver.sequence,
            odom_received_monotonic=driver.received,
            odom_linear_speed=math.inf,
            odom_angular_speed=math.inf,
        )
        stuck._odom_sample = lambda: {
            'pose': driver.pose,
            'sequence': driver.sequence,
            'received': driver.received,
            'linear_speed': stuck.odom_linear_speed,
            'angular_speed': stuck.odom_angular_speed,
        }
        assert MissionCoordinator._wait_base_stationary(
            stuck, timeout=0.4, required_samples=3) is False
        assert MissionCoordinator._wait_base_stationary(
            frozen, timeout=0.4, required_samples=3) is True
    finally:
        driver.stop()


def test_fresh_moving_samples_are_not_stationary():
    driver = OdomDriver(speeds=(0.2, 0.0),
                        moving=(0.004, 0.0, 0.0)).start()
    try:
        node = coordinator_node(driver)
        assert MissionCoordinator._wait_base_stationary(
            node, timeout=0.5, required_samples=3) is False
    finally:
        driver.stop()


def test_missing_odometry_blocks_instead_of_passing():
    node = SimpleNamespace(odom_pose=None, odom_sequence=0,
                           odom_received_monotonic=None,
                           odom_linear_speed=math.inf,
                           odom_angular_speed=math.inf)
    node._odom_sample = lambda: None
    assert MissionCoordinator._wait_base_stationary(
        node, timeout=0.2, required_samples=3) is False


def test_bounded_shift_stops_with_a_full_zero_twist_after_exception():
    publisher = FakePublisher()
    node = SimpleNamespace(cmd_vel_publisher=publisher,
                           get_logger=lambda: SimpleNamespace(
                               info=lambda *a, **k: None, warn=lambda *a, **k: None,
                               error=lambda *a, **k: None))
    node._odom_pose = lambda: (_ for _ in ()).throw(RuntimeError('odom boom'))
    try:
        MissionCoordinator._bounded_place_shift(node, 0.02, 0.0)
    except RuntimeError:
        pass
    assert publisher.messages, 'a zero stop must be published in finally'
    assert publisher.last() == (0.0, 0.0, 0.0)


def test_bounded_shift_ends_with_zero_twist_on_normal_exit():
    publisher = FakePublisher()
    node = SimpleNamespace(cmd_vel_publisher=publisher,
                           get_logger=lambda: SimpleNamespace(
                               info=lambda *a, **k: None, warn=lambda *a, **k: None,
                               error=lambda *a, **k: None),
                           _odom_pose=lambda: (0.0, 0.0, 0.0))
    MissionCoordinator._bounded_place_shift(node, 0.02, 0.02)
    assert publisher.last() == (0.0, 0.0, 0.0)
    assert any(message != (0.0, 0.0, 0.0) for message in publisher.messages)


def test_reposition_reports_stationarity_failure_instead_of_proceeding():
    node = SimpleNamespace(
        _wait_base_stationary=lambda **kwargs: False,
        _place_slot_local=lambda job: (0.33, 0.0),
        _bounded_place_shift=lambda *a: None,
    )
    local, error = MissionCoordinator._reposition_for_place(node, object())
    assert local is None
    assert error == 'base_not_stationary_before_place'


def test_reposition_blocks_after_a_failed_settle():
    state = {'calls': 0}

    def stationary(**kwargs):
        state['calls'] += 1
        return state['calls'] == 1

    node = SimpleNamespace(
        _wait_base_stationary=stationary,
        _place_slot_local=lambda job: (0.203, -0.13),
        _bounded_place_shift=lambda *a: None,
    )
    local, error = MissionCoordinator._reposition_for_place(node, object())
    assert local is None
    assert error == 'base_not_stationary_after_shift'
    assert state['calls'] == 2
