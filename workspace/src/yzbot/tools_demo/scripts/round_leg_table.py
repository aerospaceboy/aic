"""Per-round leg timing table from a zone9 campaign's mission state stream."""
import json
import re
import sys

CAMP = sys.argv[1]
TAGS = sys.argv[2].split(",") if len(sys.argv) > 2 else None
D = "/home/ros/huxuyang/YZBot-224/runs/" + CAMP
STATE = re.compile(r"\{.*\}")


def states(tag, record):
    text = open(record["log_path"], encoding="utf-8", errors="ignore").read()
    out = []
    for line in text.splitlines():
        if "[mission_coordinator" not in line:
            continue
        match = STATE.search(line)
        if not match:
            continue
        try:
            payload = json.loads(match.group(0))
        except json.JSONDecodeError:
            continue
        if "detail" in payload and "elapsed_sec" in payload:
            out.append(payload)
    deduped = []
    for payload in out:
        key = (payload["detail"], payload.get("cube_name"))
        if deduped and (deduped[-1]["detail"], deduped[-1].get("cube_name")) == key:
            continue
        deduped.append(payload)
    return deduped


for line in open(D + "/results.jsonl"):
    record = json.loads(line)
    tag = record["tag"]
    if TAGS and tag not in TAGS:
        continue
    seq = states(tag, record)
    print("== %s total=%.1f state=%s" % (
        tag, record["mission"]["elapsed_sec"], record["mission"]["state"]))
    previous = None
    for payload in seq:
        if previous is None:
            previous = payload
            continue
        detail = payload["detail"]
        gap = payload["elapsed_sec"] - previous["elapsed_sec"]
        if gap >= 3.0 or "completed" in detail or "recogniz" in detail:
            print("   %6.1f -> %6.1f  %5.1fs  %-22s %s" % (
                previous["elapsed_sec"], payload["elapsed_sec"], gap,
                detail[:22], payload.get("cube_name") or ""))
        previous = payload
