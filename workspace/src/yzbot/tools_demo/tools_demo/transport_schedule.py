"""Estimate complete transport legs using the navigator's layout routes."""

from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
import statistics

from tools_demo.corner_flow import round_navigation_corners
from tools_demo.navigation_math import (
    A_NORTH_WALL_INNER_Y, ZonePose, angle_delta, dynamic_gate_waypoints,
    tray_crossing_waypoints,
)
from tools_demo.transport_routes import (
    current_delivery_route, current_pickup_guides,
)


def centered_slot_offset(slot, count, spacing, maximum_abs_y):
    """Share the production zone-slot spacing with the route estimator."""
    if not 0 <= slot < count <= 5:
        raise ValueError('Invalid delivery slot index or count')
    pitch = max(spacing, 2.0 * maximum_abs_y /
                (count - 1)) if count > 1 else 0.0
    return (slot - (count - 1) / 2.0) * pitch


def load_timing_profile(path, geometry_files):
    """Reject stale-layout measurements rather than reusing old timings."""
    payload = json.loads(Path(path).read_text())
    expected = payload.get('geometry_sha256', {})
    if (payload.get('schema_version') != 1
            or set(expected) != set(geometry_files)
            or any(hashlib.sha256(Path(p).read_bytes()).hexdigest()
                   != expected[key]
                   for key, p in geometry_files.items())):
        raise ValueError(
            'Timing profile geometry does not match this deployment')
    samples = payload.get('samples')
    if not isinstance(samples, list):
        raise ValueError('Timing profile samples must be an array')
    return samples


@dataclass
class TransportTimeEstimator:
    """A deterministic seconds estimate, not a physical completion guarantee.

    Speeds are effective transit priors, below configured cruise maxima.
    Service times include scanning and arm motion; repeated failure history
    adds recovery cost. Unobserved dynamic phases stay uncertain rather than
    being optimistically predicted many legs into the future.
    """

    docks: dict
    strategies: dict
    cube_positions: dict
    zone_totals: dict
    spacing: float = 0.08
    maximum_abs_y: float = 0.16
    empty_speed: float = 1.2
    loaded_speed: float = 1.1
    turn_speed: float = 2.0
    stage_overhead: float = 0.6
    pick_seconds: float = 7.0
    place_seconds: float = 7.5
    uncertainty_weight: float = 0.15
    failures: dict = field(default_factory=dict)
    responses_complete: bool = False
    navigation_parameters: dict = field(default_factory=dict)
    timing_samples: list = field(default_factory=list)

    def __post_init__(self):
        for value in (self.empty_speed, self.loaded_speed, self.turn_speed,
                      self.pick_seconds, self.place_seconds):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    'Transport speeds and service times must be positive')
        for value in (
                self.spacing,
                self.maximum_abs_y,
                self.stage_overhead,
                self.uncertainty_weight):
            if not math.isfinite(value) or value < 0:
                raise ValueError(
                    'Transport penalties must be finite and nonnegative')

    def transit_seconds(self, start, poses, loaded=False):
        """Account for distance, heading changes and stage overhead."""
        speed = self.loaded_speed if loaded else self.empty_speed
        x, y, yaw = start
        result = 0.0
        previous_flow = None
        for pose in poses:
            distance = math.hypot(pose.x - x, pose.y - y)
            heading = math.atan2(pose.y - y,
                                 pose.x - x) if distance > 1e-6 else yaw
            family = (pose.zone.split('_')[1]
                      if pose.zone.startswith('FLOW_') else None)
            cruise = min(speed, .75 if family == 'A' else 1.0) if family else speed
            result += distance / cruise
            # Arc samples are one moving action, not independent stops and
            # stationary rotations. Charge entry overhead once per group.
            if family is None or family != previous_flow:
                result += abs(angle_delta(heading, yaw)) / self.turn_speed
                result += self.stage_overhead
            previous_flow = family
            x, y, yaw = pose.x, pose.y, heading
        if poses:
            result += abs(angle_delta(poses[-1].yaw, yaw)) / self.turn_speed
        return result

    def calibrated_seconds(self, phase, start, poses, candidate, zone, prior):
        """Blend local, same-route observations with the geometric prior."""
        values = []
        for sample in self.timing_samples:
            try:
                origin = sample['start_pose']
                points = sample['waypoints_xy']
                elapsed = float(sample['elapsed_sec'])
                if (sample['phase'] != phase
                        or (phase != 'spawn' and (
                            sample['cube'] != candidate.cube
                            or sample['side'] != candidate.side))
                        or (phase == 'delivery' and sample['zone'] != zone)
                        or len(points) != len(poses)
                        or not math.isfinite(elapsed)
                        or elapsed <= 0 or len(origin) != 3
                        or not all(math.isfinite(v) for v in origin)
                        or math.hypot(origin[0]-start[0],
                                      origin[1]-start[1]) > .6
                        or abs(angle_delta(origin[2], start[2])) > .5):
                    continue
                if all(len(p) == 2 and all(math.isfinite(v) for v in p)
                       and math.hypot(p[0]-q.x, p[1]-q.y) <= .4
                       for p, q in zip(points, poses)):
                    values.append(elapsed)
            except (KeyError, TypeError, ValueError):
                continue
        if not values:
            return prior, 0.0
        # A single fast run cannot overwrite the prior. Range is a limited
        # uncertainty signal, not a statistically established tail quantile.
        weight = min(.8, len(values) / (len(values) + 2.0))
        estimate = (1.0-weight)*prior + weight*statistics.median(values)
        return estimate, weight*(max(values)-min(values)) / 2.0

    def zone_dock(self, zone, slot):
        """Mirror the production centered slot order and lateral dock shift."""
        x, y, yaw = self.docks[zone]
        offset = centered_slot_offset(
            slot,
            self.zone_totals[zone],
            self.spacing,
            self.maximum_abs_y)
        return ZonePose(zone, x, y + offset, yaw)

    def zone_exit(self, zone, dock):
        """Predict the egress controller endpoint, preserving slot heading."""
        if zone == 'B' and math.cos(dock.yaw) >= 0:
            x, y, _ = self.strategies[zone]['retreat_pose']
        else:
            x = dock.x - .45 * math.cos(dock.yaw)
            y = dock.y - .45 * math.sin(dock.yaw)
        return (x, y, dock.yaw)

    def pickup_poses(self, start, candidate):
        """Use the identical safety and encounter anchors as TaskNavigator."""
        guides = current_pickup_guides(
            candidate.cube.upper(), start[0], start[1], candidate.entry,
            responses_complete=self.responses_complete)
        # C's next pickup still goes through its configured return waypoint
        # after the direct egress. Its command handler inserts this separately.
        c = self.docks.get('C')
        if c is not None and math.hypot(
                start[0] - c[0], start[1] - c[1]) < 1.0:
            values = self.strategies['C'].get('return_waypoint')
            if values:
                guides.insert(0, ZonePose('C_return', *values))
        return round_navigation_corners(
            (start[0], start[1]), guides + [candidate.entry], candidate.cube.upper())

    def delivery_poses(self, candidate, zone, dock):
        """Start carrying at the tray-clear entry reached by direct backoff."""
        entry = candidate.entry
        base = [ZonePose('configured', *p) for p in
                self.strategies[zone].get('primary_waypoints', ())]
        poses, dedicated = current_delivery_route(
            zone, entry.x, entry.y, dock, base_guides=base,
            bay_distance=self.navigation_parameters.get(
                'bay_exit_staging_distance', 2.0))
        if zone == 'A' and not dedicated:
            poses = list(tray_crossing_waypoints(
                self.cube_positions[candidate.cube], candidate.side,
                (dock.x, dock.y), north_wall_y=A_NORTH_WALL_INNER_Y)) + poses
        return round_navigation_corners((entry.x, entry.y), poses, zone)

    def components(self, start, candidate, zone, slot, final):
        """Separate explainable estimates for runtime prediction auditing."""
        dock = self.zone_dock(zone, slot)
        pickup_poses = self.pickup_poses(start, candidate)
        pickup = self.transit_seconds(start, pickup_poses)
        entry, grasp = candidate.entry, candidate.dock
        # The forward grasp approach and reverse loaded departure are bounded
        # direct control, not cruise-speed global navigation.
        length = math.hypot(entry.x - grasp.x, entry.y - grasp.y)
        pickup += length / .45 + .8
        spawn_uncertainty = 0.0
        if any(p.zone == 'universal_origin_staging' for p in pickup_poses):
            # The slow mandatory prefix belongs to every initial choice.
            # Attributing its delay only to the observed first cube would
            # systematically favour unmeasured cubes on the same corridor.
            spawn = list(dynamic_gate_waypoints(start, (entry.x, entry.y)))
            prior = self.transit_seconds(start, spawn)
            calibrated, spawn_uncertainty = self.calibrated_seconds(
                'spawn', start, spawn, candidate, zone, prior)
            pickup += calibrated - prior
        pickup, pickup_uncertainty = self.calibrated_seconds(
            'pickup', start, pickup_poses, candidate, zone, pickup)
        departure = length / .55 + .3
        delivery_start = (entry.x, entry.y, grasp.yaw)
        delivery_poses = self.delivery_poses(candidate, zone, dock)
        delivery = self.transit_seconds(
            delivery_start, delivery_poses, loaded=True)
        delivery, delivery_uncertainty = self.calibrated_seconds(
            'delivery', delivery_start, delivery_poses,
            candidate, zone, delivery)
        exit_pose = self.zone_exit(zone, dock)
        egress = 0.0 if final else math.hypot(exit_pose[0] - dock.x,
                                              exit_pose[1] - dock.y) / .5 + .8
        recovery = 10.0 * self.failures.get(candidate.cube, 0)
        travel = pickup + departure + delivery + egress
        return {'pickup': pickup, 'departure': departure, 'delivery': delivery,
                'egress': egress,
                'service': self.pick_seconds + self.place_seconds,
                'risk': self.uncertainty_weight * travel + recovery
                + pickup_uncertainty + delivery_uncertainty
                + spawn_uncertainty}

    def transition(self, start, candidate, zone, slot, final):
        """Return risk-adjusted seconds and the next egress state."""
        parts = self.components(start, candidate, zone, slot, final)
        dock = self.zone_dock(zone, slot)
        return sum(parts.values()), self.zone_exit(zone, dock)


def route_estimate(
        jobs,
        entries,
        options,
        start,
        transition,
        zone_counts=None):
    """Re-evaluate an incumbent with the same cost as a replacement route."""
    lookup = {f'{name}:{side}': candidate for items in options.values()
              for name, side, candidate in items}
    if len(entries) != len(jobs) or len(
            {e.split(':')[0] for e in entries}) != len(entries):
        return math.inf
    counts = dict(zone_counts or {})
    result = 0.0
    for i, (job, key) in enumerate(zip(jobs, entries)):
        candidate = lookup.get(key)
        if candidate is None or candidate.cube.split('_')[0] != job.color:
            return math.inf
        slot = counts.get(job.target_zone, 0)
        leg, start = transition(
            start, candidate, job.target_zone, slot, i == len(jobs) - 1)
        result += leg
        counts[job.target_zone] = slot + 1
    return result
