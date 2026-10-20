"""Check exclusive command ownership without initializing ROS nodes."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from tools_demo.velocity_selector import VelocitySelector


def harness(direct, stamp=10.0, changed=9.0):
    """Build the selector state with distinct upstream commands."""
    nav = Twist()
    nav.linear.x = 0.8
    direct_command = Twist()
    direct_command.angular.z = 0.3
    return SimpleNamespace(
        direct=direct, changed_at=changed,
        nav_stamp=stamp, direct_stamp=stamp,
        nav_message=nav, direct_message=direct_command,
        publisher=Mock())


def test_direct_owner_suppresses_nav_translation():
    """A direct turn cannot alternate with Nav2's forward velocity."""
    node = harness(True)
    with patch('tools_demo.velocity_selector.time.monotonic',
               return_value=10.1):
        VelocitySelector.tick(node)
    value = node.publisher.publish.call_args.args[0]
    assert value.linear.x == 0.0
    assert value.angular.z == 0.3


def test_stale_direct_owner_stops_without_falling_back_to_nav():
    """A silent direct controller must stop rather than select Nav2."""
    node = harness(True)
    node.nav_stamp = 11.0
    with patch('tools_demo.velocity_selector.time.monotonic',
               return_value=11.1):
        VelocitySelector.tick(node)
    value = node.publisher.publish.call_args.args[0]
    assert value.linear.x == value.angular.z == 0.0


def test_owner_change_drops_cached_previous_commands():
    """Commands received before the new owner's lease are never replayed."""
    node = harness(False, changed=10.2)
    with patch('tools_demo.velocity_selector.time.monotonic',
               return_value=10.3):
        VelocitySelector.tick(node)
    assert node.publisher.publish.call_args.args[0].linear.x == 0.0


def test_fresh_nav_owner_passes_nav_command():
    """Fresh Nav2 motion passes when direct control is released."""
    node = harness(False)
    with patch('tools_demo.velocity_selector.time.monotonic',
               return_value=10.1):
        VelocitySelector.tick(node)
    assert node.publisher.publish.call_args.args[0].linear.x == 0.8


def test_mode_change_immediately_publishes_stop():
    """Every owner transition resets the output before another command."""
    node = harness(False)
    with patch('tools_demo.velocity_selector.time.monotonic',
               return_value=10.2):
        VelocitySelector.mode_callback(node, Bool(data=True))
    assert node.direct and node.changed_at == 10.2
    value = node.publisher.publish.call_args.args[0]
    assert value.linear.x == value.angular.z == 0.0


def test_manipulation_lease_overrides_navigation_then_expires():
    """Task alignment owns motion only while its commands stay fresh."""
    node = harness(False)
    command = Twist()
    command.linear.y = 0.15
    node.manip_message = command
    node.manip_stamp = 10.0
    with patch('tools_demo.velocity_selector.time.monotonic',
               return_value=10.1):
        VelocitySelector.tick(node)
    assert node.publisher.publish.call_args.args[0].linear.y == 0.15
    node.nav_stamp = 10.5
    with patch('tools_demo.velocity_selector.time.monotonic',
               return_value=10.6):
        VelocitySelector.tick(node)
    assert node.publisher.publish.call_args.args[0].linear.x == 0.8


def test_arm_stop_overrides_all_motion_sources():
    """A fresh manipulation-worker stop precedes task and Nav2 motion."""
    node = harness(False)
    node.manip_message = node.nav_message
    node.manip_stamp = node.arm_stamp = 10.0
    node.arm_message = Twist()
    with patch('tools_demo.velocity_selector.time.monotonic',
               return_value=10.1):
        VelocitySelector.tick(node)
    assert node.publisher.publish.call_args.args[0].linear.x == 0.0
