#!/usr/bin/env python3
"""Report one full-matrix campaign: per-round outcome plus family totals.

Reads a single ``runs/policy_<stamp>/results.jsonl`` (a full 48-case pass)
and prints the gate outcome per case, the family totals, and the timing
distribution, so a regression run can be judged without opening logs.
"""
import argparse
import json
import pathlib
import statistics
import sys

SCRIPTS = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))
import coverage48_summary as cov          # noqa: E402


def family_of(tag):
    """Reds-to-zone / blues-to-zone pair; clause order does not change it."""
    zones = {}
    for part in tag.split("_"):
        zones[part[1]] = part[2:]
    return f"{zones.get('r', '?')},{zones.get('b', '?')}"


def load(directory):
    path = pathlib.Path(directory) / "results.jsonl"
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory")
    parser.add_argument("--quiet", action="store_true")
    options = parser.parse_args()

    rows = load(options.directory)
    cases = cov.legal_cases()
    passed = [r for r in rows if r.get("passed")]
    failed = [r for r in rows if not r.get("passed")]

    if not options.quiet:
        print(f"{'case':12} {'ok':>3} {'elapsed':>8} {'margin':>7} "
              f"{'clear':>7} {'src':>4}  notes")
        for record in sorted(rows, key=lambda r: r["tag"]):
            mission = record["mission"]
            avoid = record.get("avoidance") or {}
            metrics = record.get("metrics") or {}
            elapsed = mission.get("elapsed_sec")
            margin = (290.0 - elapsed
                      if isinstance(elapsed, (int, float)) else 0)
            notes = []
            for key, label in (
                ("pick_failed", "pick"),
                ("carry_abort", "carry"),
                ("lethal_start", "lethal"),
                ("collision_fault", "coll"),
                ("nav_retry", "retry"),
            ):
                if metrics.get(key):
                    notes.append(f"{label}={metrics[key]}")
            if not record.get("passed"):
                notes.append(record.get("reason", "")[:60])
            print(f"{record['tag']:12} {str(record.get('passed'))[:3]:>3} "
                  f"{(f'{elapsed:.1f}' if elapsed is not None else '-'):>8} "
                  f"{margin:>7.1f} "
                  f"{avoid.get('minimum_clearance', 0):>7.4f} "
                  f"{str(metrics.get('plan_source')):>4}  {' '.join(notes)}")

    print()
    print(f"rounds={len(rows)}  passed={len(passed)}  failed={len(failed)}")
    if passed:
        times = [r["mission"]["elapsed_sec"] for r in passed]
        clears = [r["avoidance"]["minimum_clearance"] for r in passed]
        margins = [290.0 - t for t in times]
        print(f"elapsed  min={min(times):.1f} median="
              f"{statistics.median(times):.1f} max={max(times):.1f}")
        print(f"margin   min={min(margins):.1f} "
              f"median={statistics.median(margins):.1f}")
        print(f"clearance min={min(clears):.4f}")
        print(f"<280s: {sum(1 for t in times if t < 280.0)}/{len(passed)}")
    families = {}
    for record in rows:
        key = cov.key_of(record)
        tag = cases.get(key) if key else record["tag"]
        if not tag:
            continue
        families.setdefault(family_of(tag), []).append(record.get("passed"))
    for family in sorted(families):
        marks = families[family]
        print(f"  ({family}) passed {sum(1 for m in marks if m)}/{len(marks)}")
    for record in failed:
        print(f"FAILED {record['tag']}: {record.get('reason')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
