"""Behavior checks for experimental patrols, sensor forecasts and final guard."""
import math
import numpy as np
import pytest
from tools_demo.transport_motion import patrol_position,sensor_forecast,dense_velocity_scale
from tools_demo.transport_motion import actor_reachable_distance


def test_patrol_has_bounded_speed_and_does_not_jump_at_reversal():
    actor=dict(start=[1.,2.],end=[2.,2.],speed=.4,phase_fraction=.23)
    p=[patrol_position(actor,t) for t in np.arange(0,15,.02)]
    assert all(1<=x<=2 and y==2 for x,y in p)
    assert max(math.dist(a,b) for a,b in zip(p,p[1:]))<=.00800001


def test_short_track_cannot_invent_patrol_boundaries():
    history=[[i*.1,i*.03,0] for i in range(7)]
    track=dict(x=.18,y=0.,vx=.3,vy=0.,confident=True,uncertainty=.04)
    forecast,kind=sensor_forecast(track,history)
    assert kind=='measured_velocity'
    assert forecast[-1][1]>.7


def test_observed_horizontal_reversals_are_not_hardcoded_to_world_y():
    actor=dict(start=[0.,1.],end=[1.,1.],speed=.3,phase_fraction=0.)
    history=[[float(t),*patrol_position(actor,t)] for t in np.arange(0,12,.1)]
    x,y=patrol_position(actor,11.9)
    track=dict(x=x,y=y,vx=-.3,vy=0.,confident=True,uncertainty=.04)
    forecast,kind=sensor_forecast(track,history)
    assert kind=='observed_reflection'
    assert all(-.01<=p[1]<=1.01 and abs(p[2]-1)<.01 for p in forecast)


def test_crossing_actor_is_not_ignored_by_final_guard():
    a=[dict(x=.9,y=-.3,vx=0.,vy=.4,radius=.14)]
    assert dense_velocity_scale((0,0,0),(1.,0,0),a)<1.


def test_rail_reversal_and_delayed_samples_are_in_the_collision_bound():
    a=dict(x=.8,y=0.,vx=.28,vy=0.,radius=.14,patrol_start=(0.,0.),
           patrol_end=(2.,0.),speed_bound=.28,source_age=.2)
    # Even if the latest velocity points away, a reversal can approach us.
    assert actor_reachable_distance((0.,0.),a,.5)==pytest.approx(.8-.28*.7)
    assert dense_velocity_scale((0.,0.,0.),(1.,0.,0.),[a],True)<1.
    # Full rail bounds, rather than an isotropic expanding disk, preserve a
    # clear parallel lane at high speed.
    parallel=dict(a,x=1.,y=1.3,patrol_start=(0.,1.3),patrol_end=(2.,1.3))
    assert dense_velocity_scale((0.,0.,0.),(1.7,0.,0.),[parallel],True)==1.


def test_nearby_parallel_but_clear_motion_does_not_cause_distance_only_slowing():
    a=[dict(x=1.,y=1.2,vx=.5,vy=0.,radius=.14)]
    assert dense_velocity_scale((0,0,0),(1.,0,0),a)==1.


def test_backstop_allows_motion_away_from_a_near_actor():
    a=[dict(x=.6,y=0.,vx=0.,vy=0.,radius=.14)]
    assert dense_velocity_scale((0,0,0),(-.3,0,0),a)==1.


def test_loaded_body_requires_more_clearance_than_empty():
    a=[dict(x=.8,y=.6,vx=0.,vy=0.,radius=.14)]
    assert dense_velocity_scale((0,0,0),(.8,0,0),a,True)<=dense_velocity_scale((0,0,0),(.8,0,0),a,False)


def test_rotation_and_wait_do_not_create_a_false_forward_conflict():
    assert dense_velocity_scale((0,0,0),(0,0,1.),[dict(x=1,y=0,radius=.14)])==1.


def test_small_lateral_correction_keeps_independent_forward_braking():
    from tools_demo.transport_motion import stopping_sweep
    horizon, samples = stopping_sweep((0,0,0), (1.5,.3,0.))
    assert horizon == pytest.approx(.7)
    assert samples[-1][1] == pytest.approx(.2*1.5+1.5**2/(2*3.))
    assert samples[-1][2] == pytest.approx(.2*.3+.3**2/2)
    actor = dict(x=.7,y=.6,vx=0.,vy=.36,radius=.14)
    assert dense_velocity_scale((0,0,0),(1.5,.3,0.),[actor],True) == 1.
    # Independent fine-time evaluation of both actual constant motion and the
    # stop sweep: allowing the command does not trade away physical clearance.
    minimum = min(math.hypot(.7-1.5*t,.6+(.36-.3)*t)-.34-.14
                  for t in [i*.001 for i in range(701)])
    assert minimum > .10


def test_lateral_braking_and_turn_uncertainty_are_not_treated_as_forward_braking():
    from tools_demo.transport_motion import stopping_sweep
    horizon, samples = stopping_sweep((0,0,0), (0.,.6,1.))
    assert horizon == pytest.approx(.8)
    assert samples[-1][2] == pytest.approx(.2*.6+.6**2/2)
    assert samples[-1][3] > 0.
    actor = [dict(x=.1,y=.5,vx=0.,vy=0.,radius=.14)]
    assert dense_velocity_scale((0,0,0),(0.,.6,1.),actor,True) < 1.
