import copy
import time
from types import SimpleNamespace
import pytest
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import ModelStates
from tools_demo.dense_safety_guard import DenseSafetyGuard
from tools_demo.transport_motion import dense_rotation_clear, dense_snapshot_valid


def test_expired_command_lease_actively_emits_a_complete_stop():
    sent = []
    command = Twist()
    command.linear.x = 1.
    command.linear.y = .3
    command.angular.z = 1.4
    guard = SimpleNamespace(command=command, command_time=time.monotonic()-1.,
                            publisher=SimpleNamespace(publish=sent.append))
    DenseSafetyGuard.tick(guard)
    assert len(sent) == 1
    assert sent[0] == Twist()


def test_absent_upstream_command_never_leaves_an_old_actuator_command_alive():
    sent = []
    guard = SimpleNamespace(command=None, command_time=None,
                            publisher=SimpleNamespace(publish=sent.append))
    DenseSafetyGuard.tick(guard)
    assert sent == [Twist()]


@pytest.mark.parametrize('scale', [0., .3, 1.])
def test_dense_slowdown_time_scales_the_entire_moving_arc(monkeypatch, scale):
    monkeypatch.setattr('tools_demo.dense_safety_guard.dense_velocity_scale',
                        lambda *args: scale)
    now = time.monotonic()
    sent = []
    command = Twist()
    command.linear.x, command.angular.z = 1.2, .6
    guard = SimpleNamespace(
        command=command, command_time=now, robot=(0., 0., 0.), world_pose_time=now,
        actors=[], payload=False, received=now, stamp=1., last_diag=now,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)),
        publisher=SimpleNamespace(publish=sent.append))
    DenseSafetyGuard.tick(guard)
    assert sent[0].linear.x == pytest.approx(1.2 * scale)
    assert sent[0].angular.z == pytest.approx(.6 * scale)


def test_nonfinite_optimizer_output_is_replaced_by_complete_zero():
    for value in (float('nan'), float('inf')):
        command = Twist()
        command.linear.x = 1.
        command.linear.y = value
        command.angular.z = 1.2
        guard = SimpleNamespace(command=None, command_time=None)
        DenseSafetyGuard.velocity(guard, command)
        assert guard.command == Twist()


def test_load_sensitive_rotation_reserve_blocks_spin_but_allows_open_space():
    actor = dict(x=.6, y=0., radius=.14)
    assert dense_rotation_clear((0., 0., 0.), [actor], False)
    assert not dense_rotation_clear((0., 0., 0.), [actor], True)
    assert dense_rotation_clear((0., 0., 0.), [dict(actor, x=1.)], True)


def test_blocked_turn_cannot_turn_a_moving_arc_into_forward_only_motion(monkeypatch):
    monkeypatch.setattr('tools_demo.dense_safety_guard.dense_velocity_scale',lambda *args:1.)
    now=time.monotonic();sent=[];cmd=Twist();cmd.linear.x=.4;cmd.angular.z=.6
    n=SimpleNamespace(command=cmd,command_time=now,robot=(0.,0.,0.),world_pose_time=now,
        actors=[dict(x=.6,y=0.,radius=.14)],payload=True,received=now,stamp=1.,last_diag=now,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_000_000_000)),
        publisher=SimpleNamespace(publish=sent.append))
    DenseSafetyGuard.tick(n)
    assert sent[-1]==Twist()
    cmd.angular.z=0.;cmd.linear.x=-.3
    DenseSafetyGuard.tick(n)
    assert sent[-1].linear.x==-.3  # A checked straight escape remains possible.


def test_world_model_pose_cannot_be_overwritten_by_drifting_encoder_odometry():
    from nav_msgs.msg import Odometry
    n=SimpleNamespace(robot=None)
    m=ModelStates();m.name=['six_arm'];p=Pose();p.position.x=9.142;p.position.y=-13.680;p.orientation.w=1.;m.pose=[p]
    DenseSafetyGuard.models(n,m)
    odom=Odometry();odom.pose.pose.position.x=9.075;odom.pose.pose.position.y=-13.806
    DenseSafetyGuard.odom(n,odom)
    assert n.robot[:2]==(9.142,-13.680)
    DenseSafetyGuard.models(n,ModelStates())
    assert n.robot is None


def snapshot():
    return dict(stamp_sec=3., expected_count=2, observed_count=2, actors=[
        dict(model=n, x=float(i), y=1., vx=.2, vy=0., radius=.14,
             stamp_sec=3., success=True) for i, n in enumerate(('one', 'two'))])


def test_complete_unique_finite_snapshot_is_required():
    original = snapshot()
    assert dense_snapshot_valid(original, {'one', 'two'})
    for defect in ('empty', 'duplicate', 'nan', 'speed', 'failed'):
        data = copy.deepcopy(original)
        if defect == 'empty': data.update(expected_count=0, observed_count=0, actors=[])
        elif defect == 'duplicate': data['actors'][1]['model'] = 'one'
        elif defect == 'nan': data['actors'][0]['x'] = float('nan')
        elif defect == 'speed': data['actors'][0]['vx'] = 5.
        else: data['actors'][0]['success'] = False
        assert not dense_snapshot_valid(data, {'one', 'two'}), defect
