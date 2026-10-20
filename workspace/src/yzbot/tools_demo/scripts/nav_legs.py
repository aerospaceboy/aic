#!/usr/bin/env python3
"""Per-robot-leg nav summary across a campaign (durations, retries, recoveries)."""
import json
import re
import sys

TS = re.compile(r"\[(\d+\.\d+)\]")


def legs(path):
    rows = []
    cur = None
    for line in open(path, encoding="utf-8", errors="ignore"):
        m = TS.search(line)
        if not m:
            continue
        t = float(m.group(1))
        if "nav_start location=" in line:
            loc = line.split("nav_start location=", 1)[1].split()[0]
            cur = {"loc": loc, "t0": t, "retry": 0, "slow": 0, "recov": 0,
                   "probe": 0, "result": None}
            rows.append(cur)
            continue
        if cur is None:
            continue
        if "nav_retry" in line:
            cur["retry"] += 1
        if "slow_progress recovery" in line:
            cur["slow"] += 1
        if "Bounded recovery" in line:
            cur["recov"] += 1
        m2 = re.search(r"nav_result state=(\w+) elapsed=([0-9.]+)s", line)
        if m2:
            cur["result"] = m2.group(1)
            cur["dur"] = float(m2.group(2))
            cur = None
    return rows


def report(path, tag):
    rows = legs(path)
    out = []
    for r in rows:
        out.append((tag, r["loc"], r.get("dur", 0.0), r["retry"], r["slow"],
                    r["recov"], r["result"]))
    return out


if __name__ == "__main__":
    d = sys.argv[1]
    allrows = []
    for line in open(d + "/results.jsonl"):
        r = json.loads(line)
        allrows.extend(report(r["log_path"], r["tag"]))
    allrows.sort(key=lambda x: -x[2])
    print("%-5s %-16s %7s %5s %5s %5s %s" % ("tag", "leg", "dur", "retry", "slow", "recov", "result"))
    for row in allrows[:22]:
        print("%-5s %-16s %7.1f %5d %5d %5d %s" % row)
    tot = sum(r[2] for r in allrows)
    print("legs=%d total=%.1f mean=%.1f" % (len(allrows), tot, tot / len(allrows)))
