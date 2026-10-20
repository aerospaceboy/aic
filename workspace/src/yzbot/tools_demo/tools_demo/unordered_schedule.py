"""Choose a bounded five-job route without changing goods requirements."""

from dataclasses import replace
from functools import lru_cache
import math


def optimize_schedule(jobs, options, start_pose, zone_poses, cost):
    """Minimize the supplied route estimate over legal unique cube choices.

    Options contain (cube, side, approach); cost receives a pose, approach
    and destination. This is an estimate, not a physical-time guarantee.
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
    poses = [tuple(start_pose)] + [tuple(zone_poses[z])
                                   for z in sorted(zone_poses)]
    zone_index = {z: i + 1 for i, z in enumerate(sorted(zone_poses))}

    @lru_cache(None)
    def visit(remaining, used, pose_index):
        if not any(remaining):
            return 0.0, ()
        best = (math.inf, ())
        for i, (color, zone) in enumerate(groups):
            if not remaining[i]:
                continue
            for name, side, approach in options.get(color, ()):
                if used & bits[name] or name.split("_", 1)[0] != color:
                    continue
                leg = float(cost(poses[pose_index], approach, zone))
                if not math.isfinite(leg) or leg < 0:
                    continue
                following = list(remaining)
                following[i] -= 1
                tail_cost, tail = visit(tuple(following), used | bits[name],
                                        zone_index[zone])
                candidate = (leg + tail_cost,
                             ((color, zone, name, side),) + tail)
                if candidate < best:
                    best = candidate
        return best

    estimate, route = visit(counts, 0, 0)
    if not math.isfinite(estimate) or len(route) != len(jobs):
        raise ValueError("No complete legal schedule")
    templates = {(job.color, job.target_zone): job for job in jobs}
    scheduled = tuple(replace(templates[color, zone], index=i,
                              total=len(jobs), cube_name=name)
                      for i, (color, zone, name, side)
                      in enumerate(route, 1))
    entries = [f"{name}:{side}" for color, zone, name, side in route]
    return scheduled, entries, estimate
