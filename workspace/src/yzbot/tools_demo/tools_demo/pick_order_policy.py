"""Validate entity choices without changing ordered task instructions."""

from tools_demo.navigation_math import CUBE_APPROACH_SIDES


def ordered_task_key(jobs):
    """Encode contiguous color/zone groups in the actual instruction order."""
    groups = []
    for job in jobs:
        identity = (job.color, job.target_zone)
        if groups and groups[-1][:2] == identity:
            color, zone, count = groups[-1]
            groups[-1] = (color, zone, count + 1)
        else:
            groups.append((*identity, 1))
    return "_".join(f"{n}{color[0]}{zone}" for color, zone, n in groups)


def validate_pick_entries(entries, jobs):
    """Return a complete legal per-job entity order, or reject it atomically."""
    if (not isinstance(entries, (list, tuple))
            or len(entries) != len(jobs)):
        return []
    result, seen = [], set()
    for entry, job in zip(entries, jobs):
        if not isinstance(entry, str):
            return []
        cube, _, side = entry.strip().lower().partition(":")
        side = side.strip()
        allowed = CUBE_APPROACH_SIDES.get(cube)
        if (
            not allowed or cube in seen
            or cube.split("_", 1)[0] != job.color
            or (side and side not in allowed)
        ):
            return []
        seen.add(cube)
        result.append(cube + (f":{side}" if side else ""))
    return result
