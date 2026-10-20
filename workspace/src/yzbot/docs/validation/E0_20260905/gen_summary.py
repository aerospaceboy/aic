#!/usr/bin/env python3
"""Render E0_SUMMARY.md from e0_summaries.json (offline diagnostic output)."""
import json
import os
import sys

inp, out = sys.argv[1], sys.argv[2]
runs = json.load(open(inp, encoding="utf-8"))

HEAD = """# E0 per-action timetable — sprint 20260905 archive (offline)

Produced by `extract_e0.py` from the four runs archived in
`backups/yzbot-sprint-20260905-evidence.tar.gz`. No simulation was re-run;
no control behavior is involved.

## What this table establishes

- Wall clock closes within 2.9-5.7 s across all four runs:
  nav(monotonic `nav_result` sums) + paired manip deltas + residual = total.
- The unexplained residual is at most 5.7 s, so fixed software waits (E3)
  have a hard ceiling around ~6 s per task. Deprioritized against nav work.
- Inbound + delivery legs are 233-239 s of every ~322-327 s run: the main
  lever for <300 s. Cruise capability on open straight samples is ~1.5 m/s
  (`open_straight` moving mean), while full inbound legs blend to
  0.6-0.8 m/s effective — roughly 40-50% of nav time is not translating at
  cruise (rotation, shim, docking, safety slow zones).
- Worst big single segment: blue4 -> B (from the south tray dock,
  14.5 m displacement): 40.10 s in baseline (single route, and the only
  slow leg whose detail shows no `nav_handoff` sequence at all), 28.9 s in
  candidate_1rC4bB, ~23.4 s historically. An ~11-16 s intermittent
  variance on one leg; needs per-sample (v,w)/plan evidence (E0 §8.4)
  before touching anything.
- Manip totals 64-69 s per 5-cube task (scan/pick/place + validation);
  E4 target only if logs prove redundant scans/planning.

## Caveats

- `chain_len_m` sums consecutive `nav_plan` waypoints and robot-start ->
  first waypoint; it is a lower bound on the true Smac path. Batch routes
  include the anchor chain; direct routes use straight displacement.
- Manip rows pair `manip_start` -> next `persistent_result` FIFO; place
  completions near the next nav boundary can be approximate. Aggregate
  manip time is corroborated by the small residual, so totals are sound.
- Continuous turning-event statistics (E0 §8.5, `|v|<0.1 && |w|>=0.2`)
  need per-sample odom/cmd streams that this archive does not contain;
  they are scheduled as next-round low-bandwidth observation.
- Log wall timestamps are used only for association; durations use
  monotonic `nav_result` elapsed where available (nav rows) and event
  deltas otherwise (manip rows).
"""

rows = []
rows.append(HEAD)
rows.append("\n## Per-run totals\n")
rows.append("| run | task | total s | inbound s | delivery s | exit s | nav sum | manip s | residual s | min clearance |")
rows.append("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
for r in runs:
    cs = r["nav_class_sums"]
    rows.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
        r["run_id"], (r["task"] or "")[:60], r["total_elapsed_sec"],
        cs.get("inbound"), cs.get("delivery"), cs.get("exit"),
        r["nav_sum_monotonic"], r["manip_sum_event_delta"],
        r["residual_vs_total"], r["avoidance_minimum_clearance"]))

for r in runs:
    rows.append("\n## %s — top-5 actions\n" % r["run_id"])
    rows.append("| s | kind | class | location/cube | displacement m | chain m |")
    rows.append("| ---: | --- | --- | --- | ---: | ---: |")
    for a in r["top5_actions"]:
        rows.append("| %.1f | %s | %s | %s | %s | %s |" % (
            a["sec"], a["kind"], a["class"],
            a["location"] or a["cube"],
            a["displacement_m"] if a["displacement_m"] is not None else "",
            a["chain_len_m"] if a["chain_len_m"] is not None else ""))
    rows.append("\n### inbound/delivery segments (elapsed > 5 s)\n")
    rows.append("| s | location | side | route | chain m | disp m | eff m/s |")
    rows.append("| ---: | --- | --- | --- | ---: | ---: | ---: |")
    for seg in r["nav_segments_chain_vs_elapsed"]:
        if seg["elapsed"] is None or seg["elapsed"] <= 5.0:
            continue
        if seg["location"].endswith("_EXIT"):
            continue
        rows.append("| %.1f | %s | %s | %s | %s | %s | %s |" % (
            seg["elapsed"], seg["location"], seg["side"], seg["route"],
            seg["chain_len_m"] if seg["chain_len_m"] is not None else "",
            seg["disp_m"] if seg["disp_m"] is not None else "",
            seg["eff_mps"] if seg["eff_mps"] is not None else ""))

rows.append("\n## Next experiment decision (recorded 2026-09-05)\n")
rows.append("""1. E1 single-variable candidate: the blue4->B south-tray leg
   variance (40.1 / 28.9 / ~23.4 s). First add low-bandwidth per-action
   observation (/plan length, odom (v,w), command chain, handoff/alignment
   phase timing) per E0 §8.4, reproduce the leg at least 3 times, then
   localize before changing any controller parameter.
2. E2 shadow: extract the pure route-description function (E2b step 1) and
   validate against logged `nav_plan`/`cube_entry_choice` snapshots; then
   evaluate blue1 vs blue4 as first blue for `1 blue B + 4 red A` offline.
3. E3 is capped by the 2.9-5.7 s residual; do not expect it to close the
   ~23 s gap. E5 stays deprioritized (RTF ~1).
""")

with open(out, "w", encoding="utf-8") as fh:
    fh.write("\n".join(rows))
print("wrote", out)
