# E0 per-action timetable — sprint 20260905 archive (offline)

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


## Per-run totals

| run | task | total s | inbound s | delivery s | exit s | nav sum | manip s | residual s | min clearance |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| baseline | Move 1 red cube to zone C and 4 blue cubes to zone B. | 327.3 | 134.1 | 104.3 | 18.77 | 257.17 | 64.43 | 5.7 | 0.1418 |
| candidate_1bB4rA | Move 1 blue cube to zone B and 4 red cubes to zone A. | 322.7 | 128.51 | 99.42 | 25.24 | 253.17 | 66.67 | 2.86 | 0.1369 |
| candidate_1rC4bB | Move 1 red cube to zone C and 4 blue cubes to zone B. | 321.9 | 134.88 | 94.48 | 19.44 | 248.8 | 69.47 | 3.63 | 0.1432 |
| injected_batch_rejection | Move 1 red cube to zone C and 1 blue cube to zone B. | 144.2 | 60.25 | 37.24 | 7.62 | 105.11 | 36.23 | 2.86 | 0.1313 |

## baseline — top-5 actions

| s | kind | class | location/cube | displacement m | chain m |
| ---: | --- | --- | --- | ---: | ---: |
| 40.1 | nav | delivery | B | 14.486 | 14.536 |
| 33.5 | nav | inbound | BLUE_CUBE_3 | 9.378 | 21.859 |
| 32.7 | nav | inbound | BLUE_CUBE_4 | 14.343 | 26.172 |
| 28.3 | nav | inbound | BLUE_CUBE_2 | 18.189 | 19.475 |
| 23.7 | nav | delivery | B | 18.23 | 18.725 |

### inbound/delivery segments (elapsed > 5 s)

| s | location | side | route | chain m | disp m | eff m/s |
| ---: | --- | --- | --- | ---: | ---: | ---: |
| 23.3 | RED_CUBE_1 | east | single | 19.725 | 19.315 | 0.845 |
| 8.6 | C | auto | batch | 6.931 | 6.494 | 0.803 |
| 33.5 | BLUE_CUBE_3 | west | batch | 21.859 | 9.378 | 0.652 |
| 23.7 | B | auto | batch | 18.725 | 18.23 | 0.789 |
| 16.2 | BLUE_CUBE_1 | east | single | 2.648 | 3.011 | 0.163 |
| 10.3 | B | auto | single | 3.663 | 3.609 | 0.354 |
| 32.7 | BLUE_CUBE_4 | east | batch | 26.172 | 14.343 | 0.8 |
| 40.1 | B | auto | single | 14.536 | 14.486 | 0.362 |
| 28.3 | BLUE_CUBE_2 | east | batch | 19.475 | 18.189 | 0.687 |
| 21.5 | B | auto | single | 18.853 | 18.769 | 0.877 |

## candidate_1bB4rA — top-5 actions

| s | kind | class | location/cube | displacement m | chain m |
| ---: | --- | --- | --- | ---: | ---: |
| 37.9 | nav | inbound | BLUE_CUBE_4 | 13.945 | 27.398 |
| 32.4 | nav | inbound | RED_CUBE_2 | 14.531 | 14.044 |
| 26.0 | nav | delivery | A | 15.029 | 15.073 |
| 24.8 | nav | delivery | B | 14.499 | 14.533 |
| 23.7 | nav | delivery | A | 13.972 | 14.02 |

### inbound/delivery segments (elapsed > 5 s)

| s | location | side | route | chain m | disp m | eff m/s |
| ---: | --- | --- | --- | ---: | ---: | ---: |
| 37.9 | BLUE_CUBE_4 | east | single | 27.398 | 13.945 | 0.723 |
| 24.8 | B | auto | single | 14.533 | 14.499 | 0.586 |
| 16.2 | RED_CUBE_1 | east | single | 10.632 | 11.168 | 0.657 |
| 6.4 | A | auto | single | 0.719 | 0.632 | 0.112 |
| 32.4 | RED_CUBE_2 | west | single | 14.044 | 14.531 | 0.434 |
| 18.5 | A | auto | single | 12.794 | 12.78 | 0.691 |
| 21.8 | RED_CUBE_4 | west | single | 16.276 | 16.693 | 0.747 |
| 26.0 | A | auto | single | 15.073 | 15.029 | 0.58 |
| 20.3 | RED_CUBE_5 | east | single | 14.658 | 14.683 | 0.723 |
| 23.7 | A | auto | single | 14.02 | 13.972 | 0.592 |

## candidate_1rC4bB — top-5 actions

| s | kind | class | location/cube | displacement m | chain m |
| ---: | --- | --- | --- | ---: | ---: |
| 33.7 | nav | inbound | BLUE_CUBE_3 | 9.379 | 21.869 |
| 32.4 | nav | inbound | BLUE_CUBE_4 | 14.338 | 26.179 |
| 28.9 | nav | delivery | B | 14.504 | 14.529 |
| 28.4 | nav | inbound | BLUE_CUBE_2 | 18.175 | 19.467 |
| 23.9 | nav | inbound | RED_CUBE_1 | 19.315 | 19.726 |

### inbound/delivery segments (elapsed > 5 s)

| s | location | side | route | chain m | disp m | eff m/s |
| ---: | --- | --- | --- | ---: | ---: | ---: |
| 23.9 | RED_CUBE_1 | east | single | 19.726 | 19.315 | 0.825 |
| 9.6 | C | auto | batch | 6.932 | 6.501 | 0.72 |
| 33.7 | BLUE_CUBE_3 | west | batch | 21.869 | 9.379 | 0.649 |
| 23.2 | B | auto | batch | 18.726 | 18.222 | 0.808 |
| 16.4 | BLUE_CUBE_1 | east | single | 2.647 | 3.015 | 0.161 |
| 10.0 | B | auto | single | 3.674 | 3.612 | 0.366 |
| 32.4 | BLUE_CUBE_4 | east | batch | 26.179 | 14.338 | 0.807 |
| 28.9 | B | auto | single | 14.529 | 14.504 | 0.503 |
| 28.4 | BLUE_CUBE_2 | east | batch | 19.467 | 18.175 | 0.686 |
| 22.7 | B | auto | single | 18.843 | 18.778 | 0.829 |

## injected_batch_rejection — top-5 actions

| s | kind | class | location/cube | displacement m | chain m |
| ---: | --- | --- | --- | ---: | ---: |
| 36.0 | nav | inbound | BLUE_CUBE_3 | 9.401 | 21.878 |
| 26.0 | nav | delivery | B | 18.243 | 18.725 |
| 24.2 | nav | inbound | RED_CUBE_1 | 19.297 | 19.711 |
| 11.2 | nav | delivery | C | 6.507 | 6.932 |
| 8.8 | manip | pick | red_cube_1 | 0.015 |  |

### inbound/delivery segments (elapsed > 5 s)

| s | location | side | route | chain m | disp m | eff m/s |
| ---: | --- | --- | --- | ---: | ---: | ---: |
| 24.2 | RED_CUBE_1 | east | single | 19.711 | 19.297 | 0.814 |
| 11.2 | C | auto | batch | 6.932 | 6.507 | 0.617 |
| 36.0 | BLUE_CUBE_3 | west | batch | 21.878 | 9.401 | 0.607 |
| 26.0 | B | auto | batch | 18.725 | 18.243 | 0.72 |

## Next experiment decision (recorded 2026-09-05)

1. E1 single-variable candidate: the blue4->B south-tray leg
   variance (40.1 / 28.9 / ~23.4 s). First add low-bandwidth per-action
   observation (/plan length, odom (v,w), command chain, handoff/alignment
   phase timing) per E0 §8.4, reproduce the leg at least 3 times, then
   localize before changing any controller parameter.
2. E2 shadow: extract the pure route-description function (E2b step 1) and
   validate against logged `nav_plan`/`cube_entry_choice` snapshots; then
   evaluate blue1 vs blue4 as first blue for `1 blue B + 4 red A` offline.
3. E3 is capped by the 2.9-5.7 s residual; do not expect it to close the
   ~23 s gap. E5 stays deprioritized (RTF ~1).
