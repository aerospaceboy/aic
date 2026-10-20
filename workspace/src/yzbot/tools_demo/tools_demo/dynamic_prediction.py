"""Bounded, scan-only motion forecasts in the stationary map frame.

Never receives simulator identities or velocities. A circle fit avoids apparent
motion of a cylinder's visible face as the observing robot changes position.
Uncertain/non-circular returns remain in the ordinary live obstacle layer.
"""

from collections import deque
from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class Detection:
    x: float
    y: float
    radius: float


def circle_detection(points):
    """Fit a sufficiently observed circular return; reject ill-conditioned arcs."""
    if len(points) < 5:
        return None
    mx = sum(p[0] for p in points) / len(points)
    my = sum(p[1] for p in points) / len(points)
    centered = [(p[0] - mx, p[1] - my) for p in points]
    a = sum(u * u for u, _ in centered)
    b = sum(u * v for u, v in centered)
    c = sum(v * v for _, v in centered)
    determinant = a * c - b * b
    if determinant <= 1e-5 * (a + c) ** 2:
        return None
    d = .5 * sum(u * (u*u + v*v) for u, v in centered)
    e = .5 * sum(v * (u*u + v*v) for u, v in centered)
    cx = mx + (d*c - e*b) / determinant
    cy = my + (e*a - d*b) / determinant
    radii = [math.hypot(x-cx, y-cy) for x, y in points]
    radius = sum(radii) / len(radii)
    residual = math.sqrt(sum((r-radius)**2 for r in radii) / len(radii))
    if not (.08 <= radius <= .35 and residual <= .012):
        return None
    return Detection(cx, cy, radius)


def scan_detections(indexed_points):
    """Cluster consecutive sensor rays, without connecting unrelated objects."""
    clusters = []
    for index, x, y in indexed_points:
        if (not clusters or index - clusters[-1][-1][0] > 2 or
                math.hypot(x-clusters[-1][-1][1], y-clusters[-1][-1][2]) > .15):
            clusters.append([])
        clusters[-1].append((index, x, y))
    detections = [circle_detection([(x, y) for _, x, y in cluster])
                  for cluster in clusters]
    return [d for d in detections if d is not None]


@dataclass
class Track:
    identity: int
    radius: float
    samples: deque = field(default_factory=lambda: deque(maxlen=7))
    vx: float = 0.0
    vy: float = 0.0
    confident: bool = False
    uncertainty: float = .035
    was_moving: bool = False

    def append(self, stamp, detection):
        self.samples.append((stamp, detection.x, detection.y))
        self.radius = max(self.radius, detection.radius)
        self.confident = False
        if len(self.samples) < 4 or stamp-self.samples[0][0] < .5:
            return
        times = [s[0]-stamp for s in self.samples]
        mt = sum(times)/len(times)
        denominator = sum((t-mt)**2 for t in times)
        mx = sum(s[1] for s in self.samples)/len(times)
        my = sum(s[2] for s in self.samples)/len(times)
        self.vx = sum((t-mt)*(s[1]-mx) for t, s in zip(times, self.samples))/denominator
        self.vy = sum((t-mt)*(s[2]-my) for t, s in zip(times, self.samples))/denominator
        error = max(math.hypot(s[1]-(mx+self.vx*(t-mt)),
                               s[2]-(my+self.vy*(t-mt)))
                    for t, s in zip(times, self.samples))
        # A stepped/latency-jittered observation can have a stable direction
        # with 3-4 cm fit residual. Carry that measured error into the occupied
        # body instead of silently disabling all forecasts at 2.5 cm. Larger
        # disagreement still rejects the constant-velocity model entirely.
        self.uncertainty = max(.035, error + .02)
        # A reversal is not extrapolated using an obsolete direction. Raw
        # returns and the collision monitor remain active during re-acquisition.
        latest = self.samples[-1]
        previous = self.samples[-2]
        instant = ((latest[1]-previous[1])/(latest[0]-previous[0]),
                   (latest[2]-previous[2])/(latest[0]-previous[0]))
        speed, instant_speed = math.hypot(self.vx,self.vy),math.hypot(*instant)
        aligned = (speed > 0 and instant_speed > 0 and
                   (instant[0]*self.vx+instant[1]*self.vy)/(speed*instant_speed) >= .94)
        if speed >= .05 and instant_speed >= .05 and not aligned:
            # Re-acquire a corner/reversal from its new segment, rather than
            # retaining a mixed old/new heading for an entire history window.
            self.samples = deque([previous,latest],maxlen=7)
            return
        self.confident = error <= .05 and .05 <= speed <= .65 and aligned
        self.was_moving = self.was_moving or self.confident


class MotionTracker:
    """One-to-one bounded associations and short-lived directional forecasts."""

    def __init__(self, horizon=.65, expiry=2.0):
        if not (0 < horizon <= 1 and 0 < expiry <= 2):
            raise ValueError('Prediction horizon and expiry must be bounded')
        self.horizon = horizon
        self.expiry = expiry
        self.tracks = []
        self.last_stamp = -math.inf
        self.next_identity = 1

    def update(self, stamp, detections):
        if not math.isfinite(stamp) or stamp <= self.last_stamp:
            return False
        self.last_stamp = stamp
        self.tracks = [t for t in self.tracks if stamp-t.samples[-1][0] <= self.expiry]
        pairs = []
        for ti, track in enumerate(self.tracks):
            dt = stamp-track.samples[-1][0]
            _, x, y = track.samples[-1]
            for di, detection in enumerate(detections):
                distance = math.hypot(detection.x-x, detection.y-y)
                if distance <= .65*dt+.07 and abs(detection.radius-track.radius) <= .05:
                    pairs.append((distance, ti, di))
        used_tracks, used_detections = set(), set()
        for _, ti, di in sorted(pairs):
            if ti not in used_tracks and di not in used_detections:
                self.tracks[ti].append(stamp, detections[di])
                used_tracks.add(ti)
                used_detections.add(di)
        for di, detection in enumerate(detections):
            if di not in used_detections:
                track = Track(self.next_identity, detection.radius)
                self.next_identity += 1
                track.append(stamp, detection)
                self.tracks.append(track)
        return True

    def snapshot(self, stamp):
        result = []
        for track in self.tracks:
            observed, x, y = track.samples[-1]
            if stamp-observed > self.expiry:
                continue
            # No forecasts from an occluded track. It can be associated when
            # seen again, but a stale snapshot must not invent current motion.
            confidence = track.confident and stamp-observed < .05
            result.append({'id': track.identity, 'x': x, 'y': y,
                           'radius': track.radius, 'vx': track.vx, 'vy': track.vy,
                           'confident': confidence, 'stamp': observed})
            result[-1]['uncertainty'] = track.uncertainty
            result[-1]['was_moving'] = track.was_moving
        return result

    def forecast_points(self, stamp, resolution=.05):
        """Filled predicted bodies; robot inflation is applied once by Nav2."""
        points = []
        for track in self.snapshot(stamp):
            if not track['confident']:
                continue
            speed = math.hypot(track['vx'], track['vy'])
            steps = max(1, math.ceil(speed*self.horizon/resolution))
            # World-body + scored clear air + sensing uncertainty. Nav2 adds
            # its unchanged robot radius once; it is not baked into this disc.
            radius = track['radius'] + .10 + track['uncertainty']
            cells = math.ceil(radius/resolution)
            for i in range(steps+1):
                elapsed = self.horizon*i/steps
                cx = track['x'] + elapsed*track['vx']
                cy = track['y'] + elapsed*track['vy']
                for ix in range(-cells, cells+1):
                    for iy in range(-cells, cells+1):
                        if math.hypot(ix*resolution, iy*resolution) <= radius:
                            points.append((cx+ix*resolution, cy+iy*resolution, .25))
        return points

    def occupancy_points(self, stamp, resolution=.05):
        """Retain bounded reachable space when a known mover is occluded.

        Loss of a circular return (including payload occlusion) is not proof
        that its space became free. Never extrapolate its obsolete heading:
        use the same bounded association speed in ALL directions, until a
        fresh direction is acquired or the two-second memory expires.
        """
        points = self.forecast_points(stamp, resolution)
        for track in self.snapshot(stamp):
            if track['confident'] or not track['was_moving']:
                continue
            age = max(0, stamp-track['stamp'])
            radius = track['radius']+.10+track['uncertainty']+.65*(age+self.horizon)
            cells = math.ceil(radius/resolution)
            for ix in range(-cells, cells+1):
                for iy in range(-cells, cells+1):
                    if math.hypot(ix*resolution, iy*resolution) <= radius:
                        points.append((track['x']+ix*resolution,
                                       track['y']+iy*resolution, .25))
        return points

    def cost_points(self, stamp, robot_radius=.28, resolution=.05,
                    planning_clearance=.35):
        """Separate observed physical bodies from traversable future risk.

        Hard current bodies and soft future risk both protect the unchanged
        final-guard separation. Hard current geometry must not be narrowed:
        a payload-shadowed cart overlapped the base during an in-place turn
        when only the scored physical margin was lethal (soft-body R1).
        """
        # The independent final guard stops at centre separation 0.75 m.
        # A route planned at only the scored 0.10 m surface margin can thus
        # be physically safe yet undrivable: the guard stops INSIDE a patrol.
        # Give scan-derived moving bodies a larger planning buffer, without
        # changing the robot radius, final guard or scored safety threshold.
        if not math.isfinite(planning_clearance) or planning_clearance < .35:
            raise ValueError('Moving-body planning clearance must protect guard')
        points = []
        for track in self.snapshot(stamp):
            if not track['was_moving']:
                continue
            age = max(0, stamp-track['stamp'])
            body_radius = track['radius']+planning_clearance+track['uncertainty']
            if track['confident']:
                speed = math.hypot(track['vx'],track['vy'])
                steps = max(1,math.ceil(speed*self.horizon/resolution))
                risks = [(track['x']+self.horizon*i/steps*track['vx'],
                          track['y']+self.horizon*i/steps*track['vy'],
                          body_radius+robot_radius) for i in range(steps+1)]
            else:
                risks = [(track['x'],track['y'],body_radius+robot_radius+
                          .65*(age+self.horizon))]
            for x,y,radius in risks:
                points.extend((px,py,.25,200.) for px,py in disc(x,y,radius,resolution))
            if age < .05:
                points.extend((x,y,.25,254.) for x,y in disc(
                    track['x'],track['y'],body_radius,resolution))
        # Many future discs overlap. Send their occupied-cell union, retaining
        # the maximum cost: duplicate geometry must not overflow the bounded
        # subscriber and silently discard a fresh safety snapshot. Cell-centre
        # quantization is within the existing >=3.5 cm sensing uncertainty.
        return merge_cost_cells(points, resolution)


def merge_cost_cells(points, resolution=.05):
    """Preserve occupied grid cells and hard precedence without duplicates."""
    cells = {}
    for x,y,z,cost in points:
        key = (math.floor(x/resolution),math.floor(y/resolution))
        if key not in cells or cost > cells[key]:
            cells[key] = cost
    return [((ix+.5)*resolution,(iy+.5)*resolution,.25,cost)
            for (ix,iy),cost in cells.items()]


def disc(x, y, radius, resolution):
    """Rasterize one world disc without inventing a sensor ray origin."""
    cells = math.ceil(radius/resolution)
    for ix in range(-cells,cells+1):
        for iy in range(-cells,cells+1):
            if math.hypot(ix*resolution,iy*resolution) <= radius:
                yield x+ix*resolution,y+iy*resolution
