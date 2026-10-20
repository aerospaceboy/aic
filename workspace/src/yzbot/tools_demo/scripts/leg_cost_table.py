#!/usr/bin/env python3
"""Empirical per-leg navigation costs from campaign launch logs.

``mission_timing_analysis.py`` reports what one round spent. This script
turns every recorded round into the cost model the pick-order policy needs:
how long a *pickup* leg takes between a zone exit (or the spawn) and a tray,
and how long a *delivery* leg takes between a tray and a destination zone.
``--optimize`` then enumerates every legal cube selection and pick order for a
task key and prints the cheapest one -- the measurement base used to repair
the (C,A)/(C,B) reverse policies, whose trays are chosen against the layout.

Costs are medians of the observed legs; ``n`` is the sample count.
"""
import argparse
import json
import pathlib
import statistics
import sys

RUNS = pathlib.Path("/home/ros/huxuyang/YZBot-224/runs")
ANCHORS = {
    "spawn": (13.2, -0.95),
    "A": (-2.41, 2.99),
    "B": (-0.40, -12.00),
    "C": (-9.40, -7.00),
}
# Cube dock poses from task_navigator.DEFAULT_CUBE_DOCKS: the fallback
# cost model needs a distance for cells no round has exercised yet, and a
# flat default would silently invent a cheap route (spawn -> blue 5 was
# scored 18 s while the measured spawn -> blue 1 leg is 37.8 s).
DOCKS = {
    "red_cube_1": (-4.36, 3.00),
    "red_cube_2": (-0.36, 3.00),
    "red_cube_3": (-6.36, -2.50),
    "red_cube_4": (-0.36, -1.00),
    "red_cube_5": (-4.36, -11.00),
    "blue_cube_1": (4.00, 1.91),
    "blue_cube_2": (-10.36, -3.00),
    "blue_cube_3": (-6.36, -7.00),
    "blue_cube_4": (1.04, -3.90),
    "blue_cube_5": (2.24, -5.80),
}
ZONES = ("A", "B", "C")
COLORS = {"red": "r", "blue": "b"}
OVERHEAD = {"pick": 6.0, "depart": 1.2, "place": 4.6, "exit": 2.0}


def analyzer():
    """Import the sibling reporting script (not an installed package)."""
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import mission_timing_analysis

    return mission_timing_analysis


def nearest_anchor(x, y):
    return min(
        ANCHORS,
        key=lambda name: (
            (x - ANCHORS[name][0]) ** 2 + (y - ANCHORS[name][1]) ** 2
        ),
    )


def collect(root):
    """Return pickup/delivery leg samples keyed by route endpoints."""
    pickup, delivery, failures = {}, {}, []
    analyze = analyzer().analyze
    for path in sorted(root.glob("*/results.jsonl")):
        for line in path.read_text(
            encoding="utf-8", errors="ignore"
        ).splitlines():
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except ValueError:
                continue
            log = record.get("log_path")
            if not log or not pathlib.Path(log).exists():
                continue
            try:
                legs = analyze(pathlib.Path(log))["nav_legs"]
            except Exception as error:  # noqa: BLE001 - report, keep going
                failures.append((path.parent.name, str(error)[:60]))
                continue
            for leg in legs:
                cube = leg["cube"]
                if leg["location"] == cube.upper():
                    origin = nearest_anchor(leg["x"], leg["y"])
                    pickup.setdefault((origin, cube), []).append(
                        leg["duration"]
                    )
                elif leg["location"] in ZONES:
                    delivery.setdefault((cube, leg["location"]), []).append(
                        leg["duration"]
                    )
    return pickup, delivery, failures


def median(samples):
    return statistics.median(samples), len(samples)


def distance(start, cube):
    x, y = ANCHORS[start]
    dx = x - DOCKS[cube][0]
    dy = y - DOCKS[cube][1]
    return (dx * dx + dy * dy) ** 0.5


def fit(samples):
    """Least-squares ``seconds = a + b * metres`` over measured cells."""
    pairs = [
        (distance(start, cube), median(values)[0])
        for (start, cube), values in samples.items()
        if start in ANCHORS and cube in DOCKS
    ]
    if len(pairs) < 3:
        return 0.0, 3.0
    count = len(pairs)
    sum_x = sum(p[0] for p in pairs)
    sum_y = sum(p[1] for p in pairs)
    sum_xx = sum(p[0] * p[0] for p in pairs)
    sum_xy = sum(p[0] * p[1] for p in pairs)
    denominator = count * sum_xx - sum_x * sum_x
    if not denominator:
        return 0.0, sum_y / count
    slope = (count * sum_xy - sum_x * sum_y) / denominator
    intercept = (sum_y - slope * sum_x) / count
    return max(intercept, 0.0), max(slope, 0.5)


def pickup_cost(pickup, origin, cube, fits=None):
    values = pickup.get((origin, cube))
    if values:
        return median(values)[0]
    if fits is None:
        fits = {}
    intercept, slope = fits.setdefault(
        "pickup", fit(pickup)
    )
    return intercept + slope * distance(origin, cube)


def delivery_metres(cube, zone):
    x, y = ANCHORS[zone]
    dx = x - DOCKS[cube][0]
    dy = y - DOCKS[cube][1]
    return (dx * dx + dy * dy) ** 0.5


def fit_delivery(delivery):
    pairs = [
        (delivery_metres(cube, zone), median(values)[0])
        for (cube, zone), values in delivery.items()
        if cube in DOCKS and zone in ANCHORS
    ]
    if len(pairs) < 3:
        return 0.0, 3.0
    count = len(pairs)
    sum_x = sum(p[0] for p in pairs)
    sum_y = sum(p[1] for p in pairs)
    sum_xx = sum(p[0] * p[0] for p in pairs)
    sum_xy = sum(p[0] * p[1] for p in pairs)
    denominator = count * sum_xx - sum_x * sum_x
    if not denominator:
        return 0.0, sum_y / count
    slope = (count * sum_xy - sum_x * sum_y) / denominator
    intercept = (sum_y - slope * sum_x) / count
    return max(intercept, 0.0), max(slope, 0.5)


def delivery_cost(delivery, cube, zone, fits=None):
    values = delivery.get((cube, zone))
    if values:
        return median(values)[0]
    if fits is None:
        fits = {}
    intercept, slope = fits.setdefault(
        "delivery", fit_delivery(delivery)
    )
    return intercept + slope * delivery_metres(cube, zone)


def task_clauses(tag):
    clauses = []
    for part in tag.split("_"):
        count, colour, zone = part[0], part[1], part[2:]
        clauses.append((int(count), COLORS_INV[colour], zone))
    return clauses


COLORS_INV = {"r": "red", "b": "blue"}


def cubes_of(colour):
    return [f"{colour}_cube_{index}" for index in range(1, 6)]


def route_cost(order, clauses, pickup, delivery, fits=None):
    """Predicted seconds for one concrete order (list of cube names)."""
    total, index, origin = 0.0, 0, "spawn"
    for count, colour, zone in clauses:
        for _ in range(count):
            cube = order[index]
            index += 1
            total += pickup_cost(pickup, origin, cube, fits)
            total += OVERHEAD["pick"] + OVERHEAD["depart"]
            total += delivery_cost(delivery, cube, zone, fits)
            total += OVERHEAD["place"]
            if index < len(order):
                total += OVERHEAD["exit"]
            origin = zone
    return total


def enumerate_orders(clauses):
    """Every legal (selection, order) pair for the parsed clauses."""
    import itertools

    choices = []
    for count, colour, _zone in clauses:
        choices.append(
            [
                list(combo)
                for combo in itertools.permutations(
                    cubes_of(colour), count
                )
            ]
        )
    for combination in itertools.product(*choices):
        yield [cube for group in combination for cube in group]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", default=str(RUNS))
    parser.add_argument("--optimize", nargs="*", default=None)
    parser.add_argument("--show", action="store_true")
    options = parser.parse_args()

    pickup, delivery, failures = collect(pathlib.Path(options.runs))
    if options.show:
        print("fitted fallback: pickup a=%.1f b=%.2f s/m | "
              "delivery a=%.1f b=%.2f s/m"
              % (fit(pickup) + fit_delivery(delivery)))
        print("pickup legs (median s, n)")
        for (origin, cube), samples in sorted(pickup.items()):
            value, count = median(samples)
            print(f"  {origin:5} -> {cube:12} {value:6.1f}  n={count}")
        print("delivery legs (median s, n)")
        for (cube, zone), samples in sorted(delivery.items()):
            value, count = median(samples)
            print(f"  {cube:12} -> {zone} {value:6.1f}  n={count}")
    if failures:
        print(f"unparsed logs: {len(failures)}")

    if options.optimize is None:
        return 0

    tags = options.optimize or [
        "1bB_4rC", "1rC_4bB", "2bB_3rC", "2rC_3bB",
        "3bB_2rC", "3rC_2bB", "4bB_1rC", "4rC_1bB",
        "1bA_4rC", "1rC_4bA", "2bA_3rC", "2rC_3bA",
        "3bA_2rC", "3rC_2bA", "4bA_1rC", "4rC_1bA",
    ]
    for tag in tags:
        clauses = task_clauses(tag)
        best, best_cost = None, None
        for order in enumerate_orders(clauses):
            cost = route_cost(order, clauses, pickup, delivery)
            if best_cost is None or cost < best_cost:
                best, best_cost = list(order), cost
        print(f"{tag:10} best={best_cost:6.1f}s  {' '.join(best)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
