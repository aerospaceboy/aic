import math
from types import SimpleNamespace
import pytest
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from tools_demo.payload_envelope import navigation_footprint
from tools_demo.dense_safety_guard import DenseSafetyGuard


def test_envelope_never_shrinks_original_base_and_covers_carried_assembly():
    for loaded in (False, True):
        points=navigation_footprint(loaded)
        for i in range(360):
            q=(.28*math.cos(math.radians(i)),.28*math.sin(math.radians(i)))
            for a,b in zip(points,points[1:]+points[:1]):
                assert (b[0]-a[0])*(q[1]-a[1])-(b[1]-a[1])*(q[0]-a[0])>=-1e-9
    assert max(x for x,y in navigation_footprint(True))>=.46
    assert max(x for x,y in navigation_footprint(False))<.30


def test_failed_or_canceled_mission_keeps_actual_attached_load_envelope():
    n=SimpleNamespace(attached=None,payload=False)
    DenseSafetyGuard.gripper(n,String(data='attached:blue_cube_5'))
    for state in ('failed','canceled','idle'):
        DenseSafetyGuard.mission(n,String(data='{"state":"'+state+'"}'))
        assert n.payload
    DenseSafetyGuard.gripper(n,String(data='released:blue_cube_5'))
    assert not n.payload


def test_tilted_base_blocks_all_recovery_commands():
    now=__import__('time').monotonic();sent=[];cmd=Twist();cmd.linear.x=-.2
    n=SimpleNamespace(robot=None,command=cmd,command_time=now,actors=[],payload=True,
        received=now,stamp=1.,last_diag=now,publisher=SimpleNamespace(publish=sent.append),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000)))
    m=ModelStates();m.name=['six_arm'];p=Pose();p.orientation.y=math.sin(.4/2);p.orientation.w=math.cos(.4/2);m.pose=[p]
    DenseSafetyGuard.models(n,m)
    assert not n.planar_valid
    DenseSafetyGuard.tick(n)
    assert sent[-1]==Twist()
