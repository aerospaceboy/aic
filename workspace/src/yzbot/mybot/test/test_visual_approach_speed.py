"""Keep a shorter empty-arm approach within the shipped joint velocity limit."""

import importlib.util
import math
from pathlib import Path

import pytest
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

SPEC = importlib.util.spec_from_file_location(
    "visual_speed_module", Path(__file__).parents[1] / "pick_place_moveit.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize('duration', [.2, .9, 5.])
def test_faster_empty_approach_preserves_path_and_obeys_joint_velocity(duration):
    """An ambitious duration must stretch rather than overdrive any joint."""
    positions = [[0., 0.], [.01, .2], [.4, .21], [.5, .3], [.6, .3]]
    trajectory = RobotTrajectory()
    for values in positions:
        trajectory.joint_trajectory.points.append(JointTrajectoryPoint(positions=values))
    MODULE.retime_joint_trajectory(trajectory, duration, max_joint_velocity=.95)
    points = trajectory.joint_trajectory.points
    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]
    assert [list(p.positions) for p in points] == positions
    assert all(b > a for a, b in zip(times, times[1:]))
    assert times[-1] >= duration
    for previous, current, start, end in zip(points, points[1:], times, times[1:]):
        assert max(abs(b - a) / (end - start)
                   for a, b in zip(previous.positions, current.positions)) <= .95 + 1e-8
    if duration == 5.:
        assert times[-1] == duration


@pytest.mark.parametrize('limit', [0., -1., math.inf, math.nan])
def test_invalid_approach_velocity_cannot_retime_a_path(limit):
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.points.append(JointTrajectoryPoint(positions=[0.]))
    with pytest.raises(ValueError):
        MODULE.retime_joint_trajectory(trajectory, .9, max_joint_velocity=limit)
