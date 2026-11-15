"""Sensor-derived forecasts and deterministic experimental patrol geometry."""
import math
import numpy as np


def patrol_position(actor, elapsed):
    a, b = actor['start'], actor['end']
    length = math.dist(a, b)
    if length <= 0 or actor['speed'] <= 0:
        raise ValueError('non-degenerate patrol and positive speed required')
    distance = (elapsed * actor['speed'] + 2*length*actor.get('phase_fraction', 0)) % (2*length)
    f = min(distance, 2*length-distance) / length
    return a[0]+f*(b[0]-a[0]), a[1]+f*(b[1]-a[1])


def sensor_forecast(track, history, horizon=2.0, dt=.1):
    """Reflect only after TWO turning extrema were actually observed.

    History is in the stamped stationary sensor frame, never actor truth.
    Until both boundaries are confirmed the nominal model is local velocity.
    The independent live collision channel is retained in either case.
    """
    x,y=track['x'],track['y'];vx,vy=track['vx'],track['vy']
    speed=math.hypot(vx,vy); axis=None; bounds=None
    if len(history)>=12 and track.get('confident') and speed>.04:
        values=np.asarray(history,dtype=float)
        xy=values[:,1:3];mean=xy.mean(axis=0)
        _,_,v=np.linalg.svd(xy-mean,full_matrices=False)
        direction=v[0];s=(xy-mean)@direction
        residual=np.abs((xy-mean)@v[1]).max()
        low,high=float(s.min()),float(s.max())
        li,hi=int(s.argmin()),int(s.argmax())
        if (residual<.045 and high-low>.35
                and np.any(s[li:]>=low+.07) and np.any(s[hi:]<=high-.07)):
            axis=(mean,direction,float((np.array([x,y])-mean)@direction))
            bounds=(low,high)
    forecast=[]
    for i in range(int(round(horizon/dt))+1):
        t=i*dt;px,py=x+vx*t,y+vy*t
        if axis is not None:
            mean,direction,s0=axis;lo,hi=bounds;length=hi-lo
            signed=float(np.array([vx,vy])@direction)
            d=(s0-lo+signed*t)%(2*length)
            p=mean+direction*(lo+min(d,2*length-d));px,py=p.tolist()
        uncertainty=max(.035,track.get('uncertainty',.05))+.035*t
        forecast.append([round(t,3),float(px),float(py),float(uncertainty)])
    return forecast, 'observed_reflection' if axis else 'measured_velocity'


def dense_velocity_scale(robot, command, actors, payload=False):
    """Independent finite-horizon collision backstop, not the local planner.

    Uses actual actor positions only in the existing truth safety channel.
    Passing at adequate separation is not slowed merely for being nearby.
    A missing/stale actor snapshot must be handled by the ROS adapter.
    """
    x,y,yaw=robot;vx,vy,w=command
    c,s=math.cos(yaw),math.sin(yaw)
    wx,wy=c*vx-s*vy,s*vx+c*vy
    speed=math.hypot(wx,wy)
    if speed<1e-5:return 1.0
    # Vehicle smoother has >=1m/s² lateral braking; use the slower direction.
    brake=1.0 if abs(vy)>.05 else 3.0
    horizon=.20+speed/brake
    body=.34 if payload else .28
    scale=1.0
    for a in actors:
        dx,dy=a['x']-x,a['y']-y
        avx,avy=a.get('vx',0.),a.get('vy',0.)
        required=body+a.get('radius',.14)+.10
        for j in range(13):
            t=horizon*j/12
            # Stop-path sweep includes reaction, then bounded braking.
            travelled=speed*min(t,.20)
            u=max(0.,t-.20)
            travelled+=speed*u-.5*brake*u*u if u<speed/brake else speed*speed/(2*brake)
            bx,by=wx/speed*travelled,wy/speed*travelled
            clearance=math.hypot(dx+avx*t-bx,dy+avy*t-by)-required
            # Sampling error bound, including moving bodies between samples.
            allowance=(speed+math.hypot(avx,avy))*horizon/24
            if clearance<allowance:
                distance=math.hypot(dx,dy)
                margin=distance-required
                # A command moving away is a valid escape, not a forced hold.
                if dx*wx+dy*wy<0 and margin>.05:continue
                scale=min(scale,max(0.,min(1.,(margin-.08)/max(.15,speed*.4))))
    return scale


def dense_rotation_clear(robot, actors, payload=False):
    """Do not spin a loaded arm beside a moving cylinder while translation stops."""
    body = .34 if payload else .28
    return all(math.hypot(robot[0]-a['x'], robot[1]-a['y'])
               >= body+a.get('radius', .14)+.15 for a in actors)


def dense_snapshot_valid(snapshot, names):
    """A partial/duplicated/malformed simulator snapshot is never free space."""
    try:
        actors = snapshot['actors']
        if (snapshot['observed_count'] != len(names)
                or snapshot['expected_count'] != len(names)
                or len(actors) != len(names)
                or {a['model'] for a in actors} != set(names)
                or not math.isfinite(snapshot['stamp_sec'])):
            return False
        return all(a.get('success') is True and 0 < a['radius'] < 1
                   and all(math.isfinite(a[key]) for key in ('x', 'y', 'vx', 'vy', 'stamp_sec'))
                   and math.hypot(a['vx'], a['vy']) <= .8 for a in actors)
    except (KeyError, TypeError, ValueError):
        return False
