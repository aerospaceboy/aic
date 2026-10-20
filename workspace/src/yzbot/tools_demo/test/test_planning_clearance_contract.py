"""Physical footprint, carried planning envelope and static connectivity."""

import ast
from collections import deque
from pathlib import Path

import yaml

from tools_demo.mission_coordinator import CUBE_WORLD_SLOTS

REPO=Path(__file__).resolve().parents[2]


def test_carried_planning_pad_does_not_modify_physical_controller_or_speed():
    config=yaml.safe_load((REPO/'bot_navigation/param/originbot_nav2.yaml').read_text())
    local=config['local_costmap']['local_costmap']['ros__parameters']
    global_map=config['global_costmap']['global_costmap']['ros__parameters']
    control=config['controller_server']['ros__parameters']['FollowPath']
    arm=ast.parse((REPO/'mybot/pick_place_moveit.py').read_text())
    payload=next(n.value.value for n in arm.body if isinstance(n,ast.Assign)
                 and any(isinstance(t,ast.Name) and t.id=='TRANSPORT_PAYLOAD_RADIUS_LIMIT' for t in n.targets))
    assert local['robot_radius'] == .28
    assert payload == .35  # the separate transport guard is not rolled back
    assert global_map['robot_radius'] == .38 >= payload+global_map['resolution']/2
    assert local['inflation_layer']['inflation_radius'] == .45
    assert global_map['inflation_layer']['inflation_radius'] == .65
    assert control['desired_linear_vel'] == 1.78
    assert control['use_collision_detection'] is True
    assert control['max_allowed_time_to_collision_up_to_carrot'] == .65
    assert control['use_rotate_to_heading'] is False
    assert control['min_lookahead_dist'] == .60
    assert control['lookahead_time'] == .80


def test_every_enabled_pickup_entry_and_zone_remains_connected_with_larger_pad():
    # Static reachability only, not a claim about tracking or dynamic timing.
    # Inflate by a still more conservative 0.40m grid disk and use 4-neighbour
    # flood fill (never jump diagonally between blocked corners).
    metadata=yaml.safe_load((REPO/'bot_navigation/maps/mapn3.yaml').read_text())
    header=(REPO/'bot_navigation/maps/mapn3.pgm').read_bytes().split(b'\n',3)
    width,height=map(int,header[1].split())
    pixels=header[-1]
    assert len(pixels) == width*height
    blocked=bytearray(width*height)
    occupied=[divmod(i,width) for i,value in enumerate(pixels) if value==0]
    offsets=[(dy,dx) for dy in range(-8,9) for dx in range(-8,9) if dx*dx+dy*dy<=64]
    for y,x in occupied:
        for dy,dx in offsets:
            xx,yy=x+dx,y+dy
            if 0<=xx<width and 0<=yy<height:
                blocked[yy*width+xx]=1
    ox,oy=metadata['origin'][:2]
    resolution=metadata['resolution']
    def index(x,y):
        return (height-1-int((y-oy)/resolution))*width+int((x-ox)/resolution)
    points={'origin':(0.,0.),'A':(-5.,3.8),'B':(11.3,-5.),'C':(2.9,-12.1)}
    config=yaml.safe_load((REPO/'bot_navigation/config/competition_zones.yaml').read_text())
    allowed=config['competition_task_navigator']['ros__parameters']['cube_approach_allowlist']
    for item in allowed:
        name,side=item.split(':')
        x,y=CUBE_WORLD_SLOTS[name]
        assert side in ('east','west')
        points[item]=(x+(.9 if side=='east' else -.9),y)
    targets={index(*xy):name for name,xy in points.items()}
    assert not any(blocked[i] for i in targets),targets
    start=index(*points['origin'])
    seen=bytearray(width*height)
    seen[start]=1
    pending=deque([start])
    remaining=set(targets)
    while pending and remaining:
        i=pending.popleft()
        remaining.discard(i)
        y,x=divmod(i,width)
        for j in ((i-1 if x else -1),(i+1 if x+1<width else -1),
                  (i-width if y else -1),(i+width if y+1<height else -1)):
            if j>=0 and not seen[j] and not blocked[j]:
                seen[j]=1
                pending.append(j)
    assert not remaining,[targets[i] for i in remaining]
