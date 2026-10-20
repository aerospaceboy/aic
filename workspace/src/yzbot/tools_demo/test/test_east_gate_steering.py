"""Safe phase waiting must retain upstream collision-safe steering."""

from types import MethodType
from unittest.mock import Mock

from geometry_msgs.msg import Twist
import pytest

from test_east_gate_wait_sweep import make_gate_harness
from tools_demo.safety_limiter import SafetyLimiter


def make_command_harness():
    """Bind the real gate and velocity callbacks at a safe waiting point."""
    harness = make_gate_harness(9.60, 0.10)
    harness.base_hold = False
    harness.publisher = Mock()
    harness.east_gate_blocked = MethodType(
        SafetyLimiter.east_gate_blocked, harness,
    )
    harness.scale_for_obstacles = Mock(return_value=1.0)
    return harness


@pytest.mark.parametrize("angular", [-1.833, 1.533])
def test_phase_wait_preserves_collision_safe_rotation(angular):
    """Replay the observed gate input without admitting translation."""
    harness = make_command_harness()
    command = Twist()
    command.linear.x = 0.774
    command.linear.y = 0.1
    command.angular.z = angular
    SafetyLimiter.cmd_vel_callback(harness, command)
    output = harness.publisher.publish.call_args.args[0]
    assert output.linear.x == 0.0
    assert output.linear.y == 0.0
    assert output.angular.z == angular
    assert command.linear.x == 0.774


def test_phase_wait_does_not_invent_rotation_after_monitor_stop():
    """An upstream stop remains a stop even when the phase gate is closed."""
    harness = make_command_harness()
    command = Twist()
    command.linear.x = 0.774
    SafetyLimiter.cmd_vel_callback(harness, command)
    output = harness.publisher.publish.call_args.args[0]
    assert output.linear.x == output.angular.z == 0.0


def test_manipulation_base_hold_still_blocks_all_motion():
    """The phase steering exception does not override manipulation holds."""
    harness = make_command_harness()
    harness.base_hold = True
    command = Twist()
    command.linear.x = 0.774
    command.angular.z = -1.833
    SafetyLimiter.cmd_vel_callback(harness, command)
    output = harness.publisher.publish.call_args.args[0]
    assert output.linear.x == output.angular.z == 0.0
