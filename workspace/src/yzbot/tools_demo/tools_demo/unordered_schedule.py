"""Choose a bounded five-job route without changing goods requirements."""

from dataclasses import replace
from functools import lru_cache
import math


def optimize_schedule(jobs, options, start_pose, zone_poses, cost,
                      transition=None, required_cubes=(), zone_counts=None):
    """Minimize the supplied route estimate over legal unique cube choices.

    Options contain (cube, side, approach); cost receives a pose, approach
    and destination. Optional transition(pose, approach, zone, slot, final)
    returns (cost, next_pose), so slot-aware egress can define the next state.
    Required cubes constrain encounter routes; visiting one is never proof
    that a real dynamic response occurred.
    """
    if not 1 <= len(jobs) <= 5:
        raise ValueError("Scheduling requires one to five jobs")
    groups = sorted({(job.color, job.target_zone) for job in jobs})
    counts = tuple(sum((j.color, j.target_zone) == group for j in jobs)
                   for group in groups)
    names = sorted({item[0] for items in options.values() for item in items})
    bits = {name: 1 << i for i, name in enumerate(names)}
    if len(names) > 10:
        raise ValueError("Scheduling supports at most ten physical cubes")
    if not set(required_cubes) <= set(names):
        raise ValueError("Required encounter cube is unavailable")
    required_mask = sum(bits[name] for name in set(required_cubes))
    initial_counts = dict(zone_counts or {})

    @lru_cache(None)
    def edge(pose, name, side, zone, slot, final):
        approach = next(c for n, s, c in options[name.split('_', 1)[0]]
                        if n == name and s == side)
        if transition is not None:
            value, following = transition(pose, approach, zone, slot, final)
            following = tuple(float(v) for v in following)
        else:
            value, following = cost(
                pose, approach, zone), tuple(
                zone_poses[zone])
        value = float(value)
        if len(following) != 3 or not all(math.isfinite(v) for v in following):
            raise ValueError("Scheduling transition needs a finite next pose")
        return value, following

    @lru_cache(None)
    def visit(remaining, used, pose):
        if not any(remaining):
            return (
                0.0,
                ()) if used & required_mask == required_mask else (
                math.inf,
                ())
        best = (math.inf, ())
        for i, (color, zone) in enumerate(groups):
            if not remaining[i]:
                continue
            for name, side, approach in options.get(color, ()):
                if used & bits[name] or name.split("_", 1)[0] != color:
                    continue
                slot = initial_counts.get(zone, 0) + sum(
                    counts[k] - remaining[k] for k, (_, z) in enumerate(groups)
                    if z == zone)
                leg, following_pose = edge(
                    pose, name, side, zone, slot, sum(remaining) == 1)
                if not math.isfinite(leg) or leg < 0:
                    continue
                following = list(remaining)
                following[i] -= 1
                tail_cost, tail = visit(tuple(following), used | bits[name],
                                        following_pose)
                candidate = (leg + tail_cost,
                             ((color, zone, name, side),) + tail)
                if candidate < best:
                    best = candidate
        return best

    estimate, route = visit(counts, 0, tuple(start_pose))
    if not math.isfinite(estimate) or len(route) != len(jobs):
        raise ValueError("No complete legal schedule")
    templates = {(job.color, job.target_zone): job for job in jobs}
    scheduled = tuple(replace(templates[color, zone], index=i,
                              total=len(jobs), cube_name=name)
                      for i, (color, zone, name, side)
                      in enumerate(route, 1))
    entries = [f"{name}:{side}" for color, zone, name, side in route]
    return scheduled, entries, estimate
