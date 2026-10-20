# Agent 44 work log

## 2026-08-30: phase evidence and tray-departure diagnosis

- Confirmed tracked baseline `7071fc3` and archived it as
  `/home/ros/dev_ws/backups/yzbot-agent44-baseline-7071fc3-20260829.tar.gz`.
- Recorded synchronized rosbag evidence for obstacle_2, AMCL, odometry,
  navigation state, and mission state during two clean-restart
  `1 red -> C + 4 blue -> B` runs.
- Baseline rep1: 397.5 s, completed=5, skipped=0, CASE_ACCEPTED=true,
  min center distance 0.8994 m, no collision or new safety fault.
- Baseline rep2: 384.8 s, completed=5, skipped=0, CASE_ACCEPTED=true,
  min center distance 0.8845 m, no collision or new safety fault.
- The two runs contained three Nav2 retries. Synchronized evidence placed all
  three at cube trays, not the obstacle_2 east gate: blue_cube_3
  `(-6.357,-6.896)`, blue_cube_4 `(4.334,-11.057)`, and blue_cube_1
  `(4.660,1.716)`.
- During every sampled obstacle_2 gate crossing the robot remained moving at
  about 1.2-1.65 m/s; no gate stop was observed. This does not disprove the
  intermittent gate failure, but proves an independent repeatable departure
  defect: Nav2 starts while the base is still in the tray cost region.
- The single-variable trial restores direct position-only departure to the
  selected tray-clear entry. Yaw is delegated to Nav2 immediately after the
  entry is reached.

## 2026-08-30: departure trial 1 and attachment-anchor repair

- Departure trial 1 eliminated the prior tray-origin Nav2 retries: all five
  departures logged `departure_position_complete` at the tray-clear entry.
- The run ended completed=4, skipped=1 at 367.7 s, CASE_ACCEPTED=false, with
  no collision/new safety fault and min center distance 0.8736 m.
- The failure was manipulation, not navigation. blue_cube_5 and replacement
  blue_cube_2 both formed a stable but displaced LinkAttacher fixed joint.
  Their post-attach wrist-relative X jumped from about 0.015 m to
  0.047-0.049 m. Retraction then produced payload z=0.092-0.093 m, correctly
  rejected by the unchanged transport envelope.
- Added `attachment_anchor_within_window`: a stable anchor must also remain
  inside the existing grasp XY and carry Z windows before retraction. A bad
  anchor is detached, the original cube pose is restored, and the existing
  bounded three-attempt loop recreates it. No safety window was widened.
- Targeted result: 29/29 mybot tests passed, `git diff --check` passed, and
  `mybot` rebuilt successfully.

## 2026-08-30: constrained validation and environment observations

- One run was rejected as a global navigation-initialization anomaly:
  completed=0, skipped=3 at 646.7 s, with no manipulation request issued.
  It was archived separately and is not treated as manipulation regression.
- A subsequent low-real-time-factor run reached completed=4, skipped=0 before
  the 700 s wall-clock harness limit; the fifth pickup had already completed.
- An extended run ended completed=4, skipped=1 at 511.1 simulation seconds.
  The skipped placement followed repeated Gazebo service timeouts while the
  moving-obstacle service timed out concurrently, so this is classified as VM
  service starvation rather than a code defect. A later bad attachment anchor
  was successfully rejected and recreated by the new bounded validation.
- The latest clean-environment targeted run ended completed=4, skipped=1 at
  347.6 simulation seconds, with no collision and minimum center distance
  0.5974 m. Three deliveries completed normally. The fourth blue pickup route
  suffered repeated controller collision-ahead aborts near the tray, then the
  bounded BLUE_EXIT fallback spent 35 s in final yaw alignment and timed out;
  the replacement fifth cube was picked and delivered successfully.
- The latest failure is navigation fallback latency, not grasp, attachment,
  delivery truth, collision, or payload stability. It is retained as evidence
  for a later isolated fallback repair; it must not be conflated with the
  observed high host load (load average about 13 during the run).
- Per the operator's updated instruction, validation is now deliberately
  constrained: contract/unit tests plus one relevant simulation distribution
  per isolated change, followed by only a small final mixed set. Avoid broad
  run matrices while the computer is overloaded. Do not restart the VM unless
  SSH and simulation are genuinely unrecoverable.

## Next targeted optimization

- Restore the previously measured B-zone direct exit as an isolated change.
  It sends the bounded egress controller directly to the existing safe
  `(7.5, 3.0)` gateway, removing a redundant short Nav2 action after each B
  delivery. Prior evidence measured about 6.3-6.7 s saved per B exit, or about
  25 s for `1 red -> C + 4 blue -> B`, without weakening a safety parameter.
- Run navigation contract tests and one `1 red -> C + 4 blue -> B` simulation.
  Retain the change only if the mission remains safe and the timing evidence is
  positive; do not combine it with the separate cube-order lookahead trial.

## 2026-08-30: B direct-exit trial and C-exit pickup diagnosis

- Added the isolated B direct exit without changing speed or any safety
  threshold. Navigation contracts passed 72/72 and `tools_demo` rebuilt.
- The targeted `1 red -> C + 4 blue -> B` run ended completed=4, skipped=1 at
  335.8 s, with no collision/new safety fault and minimum center distance
  0.7179 m. The run is not accepted because a pickup was skipped.
- B direct exit itself triggered twice and safely completed in about 4.5 s and
  3.0 s, versus about 9 s for the prior split direct-plus-Nav2 exit. This is
  positive timing evidence and the failure occurred before the first B entry.
- The skip was isolated to the first blue pickup after leaving C. Greedy scores
  were blue_cube_3=35.06, blue_cube_4=35.50, blue_cube_2=35.64. Nav2 first
  attempted the diagonal route to blue_cube_3 and then blue_cube_4; both spent
  repeated controller-patience cycles reporting collision ahead. The robot
  finished the failed recovery at `(-2.86, -6.36)` and then used BLUE_EXIT.
- Added one constrained selection rule for this exact map transition: when a
  blue-to-B job starts at the west C exit, select the clear west-upper dock if
  it is within 1.0 m total estimated cost of greedy. In the observed layout it
  selects blue_cube_2 at `(-9.64, -3.0)`, only 0.58 m above greedy, avoiding the
  failed cross-map diagonal. The rule is inactive after B exits and for every
  other colour, target zone, and start region.

## 2026-08-30: candidate trial rejected; static dog-leg route prepared

- The candidate rule triggered and selected blue_cube_2 exactly as designed,
  but BLUE_CUBE_2 then failed on the same RPP collision-ahead chain. The
  replacement BLUE_CUBE_4 also failed. The run ended completed=1, skipped=4
  at 328.4 s, with no collision/new safety fault and minimum center distance
  1.0438 m. The candidate rule is therefore reverted and will not be retained.
- Map-to-log correlation found the common cause. C exit starts near
  `(-9.17, 2.01)`, north of a horizontal wall ending around x=-2.9. The lower
  pickup region lies behind a second offset wall, leaving a roughly 1.55 m
  dog-leg. NavFn cut the inflated corners; BLUE_CUBE_2 moved as far as
  `(-4.50, -1.21)` before RPP continuously predicted collision and exhausted
  controller patience. Changing the candidate cannot avoid this shared path.
- Prepared a TaskNavigator-only route for blue pickup commands that originate
  at the west C exit: stage at `(-2.10, 1.80)` north of the first wall, then
  `(-2.10, 0.00)` past its east tip. This preserves about 0.8 m lateral wall
  clearance and avoids obstacle_2's sweep without weakening collision checks.
  It is inactive for B exits, other start regions, red pickups, and deliveries.

## 2026-08-30: upper gates validated; lower L-wall isolated

- The next targeted run ended completed=3, skipped=2 at 307.0 s, with no
  collision/new safety fault and minimum center distance 0.7848 m.
- Both new upper gates succeeded on their first action: C exit to
  `(-2.10, 1.80)` took about 12.2 s and the wall-tip crossing to
  `(-2.10, 0.00)` about 5.7 s. This removed the previous failure at the upper
  dog-leg and is retained.
- The next legacy BLUE_CUBE_3 goal `(-7.0, -6.0)` failed after the robot reached
  `(-1.82, -3.25)`, precisely on the right side of the lower vertical wall.
  The map also shows a small obstacle beside that descent, explaining why a
  straight continuation remains inside RPP's collision projection.
- Prepared a BLUE_CUBE_3-only lower bypass: offset to `(-1.40, -1.00)`, descend
  on the open right side to `(-1.40, -7.30)`, then pass below the wall end to
  `(-4.20, -7.30)` before the tray entry. The failed legacy `(-7,-6)` staging
  goal is suppressed only for this C-exit route. Existing non-C approaches
  retain their prior staging logic.

## 2026-08-30: full lower bypass trial and obstacle_2 safety finding

- All six C-exit-to-BLUE_CUBE_3 stages completed once, the cube was delivered,
  and three B direct exits each completed in about 2.8-2.9 s. The run reached
  completed=4, skipped=0 at 274.6 s before the final pickup.
- Final state was completed=4, skipped=1 at 359.5 s. Entity-truth acceptance
  detected obstacle_2 minimum center distance 0.3710 m (clearance -0.089 m),
  so the run is unsafe and the current route cannot be accepted or committed.
- The near contact occurred after B exit while NavFn attempted the remaining
  south pickups. BLUE_CUBE_5 failed after moving from `(7.53, 2.99)` into the
  obstacle_2/wall mouth; BLUE_CUBE_2 fallback started at `(0.57, 1.35)` and
  failed on the same route. Collision Monitor did not emit BodyStop, so entity
  truth—not the absence of a monitor fault—is authoritative.
- Root cause in code: `dynamic_gate_waypoints()` only covers obstacle_1 at
  x=13.4. There is no proactive obstacle_2 crossing route for B-to-south-pickup
  legs; reactive reverse only triggers after velocity falls below 0.08 m/s and
  is therefore too late for a moving near-contact.
- Generalized the already validated west crossing into an obstacle_2 bypass
  for every north-to-south blue pickup: `(-2.10,1.80)->(-2.10,0.00)`, followed
  by a west-upper gate for BLUE_CUBE_2 or a lower-right gate for BLUE_CUBE_3/4/5.
  The lower route now uses one point `(-1.40,-7.60)` with 1.15 m wall-end
  clearance instead of three stop-and-turn points. This both increases moving
  and static clearance and removes two slow intermediate yaw alignments.

## 2026-08-30: obstacle_2 bypass accepted for safety; blue5 cost mismatch

- The proactive obstacle_2 bypass completed all five deliveries with skipped=0
  at 367.9 s. CASE_ACCEPTED=true, no collision/new safety fault, and minimum
  center distance 0.9485 m. This validates the bypass as safe.
- The compressed C-exit BLUE_CUBE_3 route reached recognition in 36.6 s versus
  44.0 s previously, saving 7.4 s. B-to-BLUE_CUBE_4 also completed safely.
- The final BLUE_CUBE_5 job alone consumed 93.4 s: 44.6 s outbound, 10.2 s
  pick/departure, 33.1 s return, and 5.5 s placement. Its selector score was
  only 33.5 versus blue_cube_2=48.6 because Euclidean scoring does not include
  the mandatory obstacle_2 and lower-wall bypass.
- Added a 30.0 ordering-only penalty for blue_cube_5 -> B. A four-blue task can
  use blue_cube_2 as the fourth cube and leave this measured longest spare
  unused. No route, speed, collision radius, or safety threshold is changed.

## 2026-08-30: skip-blue5 trial and bounded A-exit lookahead

- The skip-blue5 trial completed 5/5 with skipped=0 at 346.3 s,
  CASE_ACCEPTED=true, no collision/new safety fault, and minimum center
  distance 0.8977 m. This saved 21.6 s versus the prior safe 367.9 s run.
- Actual order was blue3 -> blue1 -> blue4 -> blue2. The final blue2 job took
  72.8 s, confirming it is materially cheaper than the 93.4 s blue5 job but
  insufficient by itself to reach 300 s on this C-heavy distribution.
- Restored the previously measured blue-to-B first-round lookahead near zone A
  exit. With at least two same-colour jobs remaining, it may choose the best
  lower candidate only when within 1.0 m of greedy. Prior isolated evidence on
  `3 red -> A + 2 blue -> B` was 286.3 s, 5/5 and safe. The rule is inactive at
  B/C exits, with one blue job left, and for all non-blue-B jobs.

## 2026-08-30: 277.5 s target achieved

- Targeted `3 red -> A + 2 blue -> B` completed 5/5 with skipped=0 at
  **277.5 s**. CASE_ACCEPTED=true, no collision/new safety fault, and minimum
  center distance 0.9955 m.
- The bounded A-exit lookahead triggered exactly once with remaining=2,
  selecting lower blue_cube_4 over greedy blue_cube_1. The final order then
  used blue_cube_1, so no requested colour/destination order changed.
- This is 8.8 s faster than the prior 286.3 s positive result for the same
  distribution. It validates the combined tray departure, attachment-anchor
  guard, B direct exit, proactive obstacle_2 bypass, blue5-B penalty, and
  bounded A-exit lookahead under the current safety envelope.
- The C-heavy `1 red -> C + 4 blue -> B` case remains safe and successful at
  346.3 s after skipping blue5, improved from 367.9 s but not yet below 300 s.
  Do not weaken obstacle clearance to chase that remaining worst-case time.
## 2026-09-03: obstacle scheme rebuild onto 78cfe37 + triggered-crossing dual gate (wall-safe)

### Session context
- User reported the inherited HEAD (d21f9ca) obstacle scheme was unreliable:
  the corridor_patrol actors in the y=-2.8 corridor were only probabilistically
  encountered, and the robot repeatedly jammed against walls there. Also the
  obstacle crossing trajectory passed through a wall.
- User direction: roll the VM back to the best verified baseline, bring in the
  current moving-obstacle scheme only, keep everything else untouched, then
  sprint toward stable <300 s. Baseline chosen after evidence review:
  **78cfe37** (8c1564c's 16-distribution 100% successor, 277.5/281.5 s
  sub-300 records, all safety fixes retained). WIP (5 files incl. base_hold)
  was committed to a temporary branch `wip-20260903` (c5aafae) and left there.

### What was wrong in the previous corridor_patrol port
- d21f9ca moved the actors to corridor_patrol at (10,-2.8)/(4,-2.8) inside the
  y=-2.8 corridor, with an origin staging route along that same corridor.
  Encounters were phase-luck; the narrow corridor plus conservative RPP
  produced wall-jams (patience exceeded, repeated aborts) and one physical
  rollover on an earlier run.
- The correct Agent47 geometry (0248d3d, verified) is **triggered_crossing**:
  actors park at (8.5,-0.9)/(6.5,-0.9) and cross to y=-0.1 only when odometry
  proves the robot is approaching the staging corridor; every first leg stages
  through (12.6,-0.6)->(0,0), crossing both gate columns deterministically.

### Work done (two commits on master over 78cfe37)
- `6d1394c obstacle: port triggered-crossing dual gate + origin staging from
  0248d3d onto 78cfe37 baseline`
  - moving_obstacle.py: continuous+triggered_crossing 441-line version (was 261).
  - navigation_math.py: DYNAMIC_GATE(x=13.4) -> UNIVERSAL staging constants
    (entry (12.6,-0.6), exit (0,0)) and new dynamic_gate_waypoints; kept 78cfe37
    ZONE_ROUTE_APPROACH, blue approach sides, etc. untouched.
  - mission_coordinator.py: selection_pose_after_universal_staging +
    blue_b_spawn_first_choice; cube scoring uses the staging pose so the first
    pickup matches the staged route.
  - zones/world/map/generate_static_map/dashboard/launch: two triggered_crossing
    actors (r 0.18, l 0.50), obstacle_3 removed, static map without actor ghosts.
  - Kept 78cfe37 nav speeds (1.65), egress caps, zone layout, launch
    TimerAction structure untouched.
- `bc755da obstacle: move obstacle-2 crossing column off the diagonal wall`
  - Map scan (0.01 m) showed a diagonal wall (5.5,-0.6)->(8.2,-1.25) crossing
    x=6.5 at y in [-0.80,-1.01], so obstacle 2's home (6.5,-0.9) and sweep
    started inside the wall. Repositioned to **x=5.0** (wall-free column over
    the whole sweep). obstacle 1 stays at x=8.5 (wall starts below -1.20).
  - Verified both crossings still trigger deterministically on the staging route.

### Validation (clean restart per run; DDS transient_local stale task issue:
always confirm mission idle AND let DDS discovery settle before publishing)
| case | elapsed | result | avoidance | BodyStop/collision |
|---|---|---|---|---|
| 5 blue -> B | 379.5 s | 5/5 skipped=0 | 2 yield events, demonstrated=true | 0 / 0 |
| 2 blue -> B | 142.6 s | 2/2 skipped=0 | 2 yield events | 0 / 0 |
| 3 red A + 2 blue B | 299.3 s | 5/5 skipped=0 | 1 yield (obs2 bounded_timeout) | 0 / 0 |
| 4 red B + 1 blue C | **291.3 s** | 5/5 skipped=0 | **2 yield, demonstrated=true** | 0 / 0 |
| 2 red C + 3 blue A | >420 s | 4/5 skipped=1 | 2 yields | 0 / 0 |

- Regression: 318 passed + 1 skipped (only pre-existing flake8/pep257).
- Build: 4 packages OK. Backups:
  `backups/yzbot-dual-trigger-crossing-78cfe37-20260903_234446.tar.gz`,
  `backups/obstacle-port-78cfe37-20260903_234446.patch`.
- `.orig` / HANDOFF_PROMPT_V5/V6 / AGENT_HISTORY_20260823.md preserved
  untracked. `wip-20260903` branch holds the pre-port uncommitted changes.

### Notes for the next agent
- The 2rC+3bA failure was one skipped cube on a far C delivery (elapsed >420 s);
  consistent with the historical C-heavy difficulty, not the obstacle scheme.
  Success-rate priority remains the near-300 mixed distributions.
- Two open items worth single-variable trials:
  1. obstacle 2 sometimes returns `bounded_timeout` instead of `yield` when the
     robot crosses the second gate at cruise speed (>response_speed 0.85).
     Acceptance currently has require_obstacle_avoidance=false (78cfe37), so
     this does not fail runs, but a clean 2-event demo may require slowing the
     corridor pass or tuning obstacle-2 response window.
  2. No-restart chaining of multiple tasks is unreliable (coordinator state
     leaks between missions). For any batch validation, restart the full
     system and clear DDS cache between runs, and wait for DDS discovery
     before publishing (publishing right after "ready" can be lost).

## 2026-09-04: stability-first 300-second sprint and rejected experiments

### Scope and invariant

- The operator confirmed that every legal five-cube distribution is in scope,
  but requested small, efficient validation during development. The priority
  is `completed=5, skipped=0` and physical safety first, deterministic dual
  obstacle avoidance second, and sub-300 seconds third.
- The legal generator space is 48 cases: either colour order, each 1+4 through
  4+1 count split, and two distinct destinations selected from A/B/C.
- Created the pre-trial backup
  `/home/ros/dev_ws/backups/yzbot-pre-retired-bypass-removal-20260904.tar.gz`.

### Rejected trial 1: remove the retired obstacle-2 bypass

- Hypothesis: `OBSTACLE_2_NORTH_SOUTH_GATES` appeared stale after the moving
  obstacles moved to triggered crossings at x=8.5 and x=5.0 and retired after
  the universal staging leg.
- In `1 red -> C + 4 blue -> B`, red1, blue3, and blue1 completed. Blue4 then
  stalled returning to B near `(2.1,-9.5)` with repeated RPP collision-ahead
  reports and recovery attempts.
- Static-map and log correlation proved the lower part of the old bypass also
  protects a real L-wall corner; it is not merely a dynamic-obstacle detour.
  Removing it globally is unsafe for reliability. The trial was stopped and
  reverted exactly to HEAD.

### Rejected trial 2: stock NavigateThroughPoses batching

- Hypothesis: preserve every configured waypoint but remove serial action
  stop/cancel/restart overhead by submitting each route as one
  `NavigateThroughPoses` goal. Unit/navigation regressions reached 151/151,
  and the full non-lint Python suite reached 320 passed, 1 skipped.
- The first trial showed that the task navigator's progress window must reset
  when `number_of_poses_remaining` decreases. After that correction, the
  universal three-pose route completed and both triggered obstacles yielded.
- The next C-exit-to-blue3 route passed its first anchor but cut about 0.5 m
  beside the second. Humble's stock behavior tree uses
  `RemovePassedGoals radius="0.7"`; it failed to remove that anchor, replanned
  backwards, then exhausted controller progress. Increasing this radius would
  weaken the mapped safety anchors and was rejected.
- Both batching trials were stopped without a physical collision. All source
  and test files were restored to exact HEAD hashes; the experimental XML was
  removed. Do not retry stock `NavigateThroughPoses` or enlarge its waypoint
  radius.

### Performance and planner diagnostics

- A headless `2 blue -> B` comparison completed 2/2, skipped=0, with two yield
  events, valid delivery truth, and no BodyStop/collision. Elapsed was 145.5 s
  versus the previous visible-mode 142.6 s result: no meaningful improvement.
- Its speed profile showed the controller is already moving quickly when able:
  open straight 10.7 s at 1.37 m/s, constrained straight 7.6 s at 1.52 m/s,
  ordinary turns 54.2 s at 1.21 m/s, sharp turns 21.4 s at 0.97 m/s,
  direct/docking 17.5 s at 0.43 m/s, and manipulation/wait 29.7 s.
- `gz stats -p` measured a live real-time factor around 0.94-0.96. Host/VM
  performance therefore changes wall-clock results, but closing gzclient alone
  is not a reliable optimization.
- An offline weighted static-map audit compared Smac `cost_travel_multiplier`
  8, 7, and 6 across all A/B/C-to-ten-cube pairs. The 8-to-7 change produced
  no path-length difference of even 0.05 m, so it was not taken to simulation.

### Stable handoff state

- Final tracked robot code/configuration is unchanged from commit `1824741`.
  This is intentional: neither experimental optimization met the project's
  reliability bar. Only this worklog and the V8 handoff prompt are committed.
- Final stable-source validation: `git diff --check` and Python compilation
  passed; the non-pre-existing-lint suite passed 318 tests with 1 skipped.
- Preserve the current safety envelope, route anchors, selector penalties, and
  triggered-crossing obstacle geometry. The next high-upside research path is
  a one-shot `ComputePathThroughPoses -> FollowPath` behavior tree that never
  deletes passed anchors, with an immediate fallback to the original serial
  route on any failure. It must exclude the universal dual-obstacle staging
  leg initially and must be validated on C-exit blue3 before timing work.

## 2026-09-04: strict anchored continuous paths accepted

### Implementation

- Added `navigate_through_poses_strict.xml`: Smac computes one complete path
  through every configured route pose, then RPP follows that path without the
  stock Humble `RemovePassedGoals` node or periodic backwards replanning.
- TaskNavigator batches routes containing two or more poses, but deliberately
  keeps every `universal_*` spawn/dual-obstacle stage on the proven serial
  NavigateToPose path. Final 0.20 m Nav2 handoff, direct docking, route poses,
  footprint, speeds, collision prediction, and Collision Monitor are unchanged.
- A batch action is attempted only once. Server unavailability, send failure,
  rejection, abort, or task-level slow progress restores the complete original
  pose queue and immediately resumes the proven serial implementation. Tests
  cover batching, universal-route exclusion, queue restoration, and rejection.

### Validation

- Targeted `1 red -> C + 1 blue -> B`: completed=2, skipped=0 at 128.7 s;
  two obstacle yields, zero batch fallback/retry, zero BodyStop/collision, and
  both delivery-truth records precise and valid. The strict C delivery took
  8.67 s; C-exit-to-blue3 four-anchor pickup took 34.17 s; blue3-to-B took
  22.69 s.
- Pressure `1 red -> C + 4 blue -> B`: completed=5, skipped=0 at **308.6 s**;
  two obstacle yields, zero batch fallback, zero Nav2 retry, zero
  BodyStop/collision, all five truth placements precise and valid, and all
  logged carry/transport envelopes valid. Every blue3/1/4/2 pickup and return
  completed on its first attempt.
- The pressure case improved 37.7 s over the earlier safe 346.3 s result and
  70.9 s over the V7 379.5 s result. It does not yet prove sub-300; the measured
  remaining gap is 8.6 s.
- Final gates: `git diff --check`, Python compilation, 154 navigation/risk/math
  tests, and the full non-pre-existing-lint suite (323 passed, 1 skipped).

### Next bounded sprint

- Do not change the accepted strict tree into the stock waypoint-removal tree.
  The previously proposed universal-route batch trial has now been completed
  and rejected below; retain the serial universal spawn leg.
- Alternatively reduce only wide-route intermediate action handoff/deceleration
  overhead; final dock handoff stays 0.20 m and all wall-corner profiles remain
  tight. One clean 5-cube run is not enough to broaden either change globally.

## 2026-09-05: universal strict-path trial rejected

- Tested the remaining high-upside proposal: allowing the initial
  `universal_*` dual-obstacle route and first pickup approach to use the same
  strict `ComputePathThroughPoses -> FollowPath` action as ordinary routes.
- Pressure case `1 red -> C + 4 blue -> B` completed 5/5 with skipped=0 in
  333.7 s. Both obstacles produced `response=yield`; measured minimum obstacle
  clearance was 0.1365 m versus the 0.10 m acceptance threshold, with zero
  collision/BodyStop, zero Nav2 retry, zero batch fallback, and five precise,
  valid delivery-truth records.
- The run was host-load affected: manipulation/wait was 74.5 s versus 64.5 s
  in the accepted 308.6 s baseline, ordinary turns were 115.5 s versus 104.4 s,
  and blue4-to-B alone was 35.42 s versus 23.41 s. However, the directly
  comparable initial navigation was 24.07 s versus 23.93 s, so batching the
  universal route demonstrated no action-level speed gain even before load
  normalization.
- Reverted the experiment completely. Stable robot code remains commit
  `73e5726`: universal staging stays serial, all later multi-anchor routes use
  the strict continuous path. Do not retry full universal-plus-pickup batching.
- Remaining credible optimization should target measured intermittent
  controller/host latency or a narrowly selected wide-route handoff; all
  direct-docking and zone-egress speed limits are already at their audited
  safety caps, so global speed increases are not justified.

## 2026-09-05: RGB-D load trials rejected at operator stop

- Profiling the benchmark launch showed that SLAM, dashboard, LLM, and gzclient
  are already disabled. The remaining software-rendered 640x480 RGB-D stream
  and color detector were investigated without changing lidar, collision
  monitoring, truth validation, route geometry, or motion limits.
- A same-machine targeted comparison used `1 red -> C + 1 blue -> B`:
  3 Hz RGB-D completed 2/2 in 153.2 s; the immediately following 5 Hz baseline
  completed 2/2 in 154.6 s. Both had two yields, no perception recovery, and
  precise valid placements. The 1.4 s difference is too small for acceptance.
- A combined 3 Hz plus callback duty gate (OpenCV/depth work only while mission
  state was `waiting_perception`) completed the same targeted case 2/2 in
  141.7 s, with both yields and no perception recovery. It then entered the
  five-cube pressure case under a degraded host (`gz stats` about 0.88 RTF;
  first navigation 42.02 s versus the accepted baseline's 23.93 s).
- On the operator's request to stop, the pressure case had four precise valid
  placements but had already consumed about 308 s and had not completed the
  fifth cube. The run was terminated and the candidate fully reverted because
  it lacked a completed five-cube proof and could not prove sub-300 stability.
- Stable robot code remains exactly `73e5726`. A future agent may revisit
  demand-driven image subscription, but must first prove that Gazebo rendering
  actually sleeps without subscribers and must run a completed five-cube case;
  callback-only gating still paid image transport/deserialization cost.

## 2026-09-05: half-hour audit, deterministic serial recovery, honest timing

### Scope and audit

- Started from `6a12674` at approximately 10:55 China time. VM was freshly
  restarted and idle. Preserved every pre-existing untracked history/original
  file. The user's request authorized implementation, validation, worklog and
  handoff updates, and pushing master within a half-hour budget.
- Reviewed navigation/action recovery, selector estimates, perception, arm
  release, runtime acceptance, profiling, and launch/cleanup. Kept route
  geometry, speeds, collision margins, world, and sensor rates unchanged.
- Found that the claimed one-shot batch fallback immediately re-entered the
  batch dispatcher; feedback could truncate original anchors; reverse recovery
  resent the batch. Eight executable failure tests failed against old code.
- Added a per-command attempt latch, preserved the original route poses,
  covered send/result errors, and resumed interrupted batches serially.
  The optional through-poses server now falls back immediately when unavailable
  instead of blocking the executor for its former 120-second discovery window.
- Added one further unavailable-server test and one profiler test. Profiler
  reports previously hidden wall-clock gaps without extrapolating stale motion.

### Experiments and validation

| Case | Wall seconds | Completed / skipped | Dual response | Min clearance |
| --- | ---: | --- | --- | ---: |
| Original baseline, 1 red C + 4 blue B | 327.3 | 5 / 0 | yield / yield | 0.1418 m |
| Recovery candidate, 1 red C + 4 blue B | 321.9 | 5 / 0 | yield / yield | 0.1432 m |
| Invalid batch-tree injection, 1 red C + 1 blue B | 144.2 | 2 / 0 | yield / yield | 0.1313 m |

- Baseline/candidate had five precise, valid final delivery records each,
  zero BodyStop events, zero Nav2 retries, and zero batch fallbacks. Aggregate
  sim/real deltas were 0.9938 and 0.9940 respectively. The 5.4-second difference
  is not a proven algorithmic speedup: successful control paths are unchanged.
- Fault injection temporarily replaced only the strict batch XML with an
  unregistered BT node. All THREE batch-eligible commands (C delivery,
  C-to-blue3 pickup, blue3-to-B delivery) failed once and then finished the
  original serial queues. No rebatching loop, no Nav2 retry, no BodyStop,
  and two precise/valid final placements. Restored the exact original XML and
  verified `git diff --exit-code` before the next run.
- The two-cube diagnostic deliberately returns `CASE_ACCEPTED=false` because
  the existing runner hardcodes five completed cubes. Mission state is
  completed=2, skipped=0 and safety evidence is valid; it is not a five-cube
  acceptance proof. One RPP prediction warning in this fault case was a
  controller avoidance prediction, not a reported physical collision.
- The new timing coverage on the fault case was 144.181 s sampled,
  0.005 s unobserved, maximum sample gap 0.217 s, zero delayed samples.
- Final functional regression after restoring the injected XML: **333 passed,
  1 skipped**. Command: `python3 -m pytest -q src/yzbot/mybot/test
  src/yzbot/tools_demo/test` with only both packages' existing `test_flake8.py`
  and `test_pep257.py` ignored. Changed modules compile and `git diff --check`
  passes. A regression invoked during intentional XML corruption failed only
  the strict-tree contract; it passed after restoration. No test was weakened
  to accept the injected tree.
- One local harness update occurred while bash was still reading that script;
  it produced a shell EOF error after the candidate had already reported its
  successful terminal result. Cleanup ran, no simulator processes remained,
  and the matching speed report was copied explicitly. The corrected harness
  preserves result logs even for nonzero diagnostic exits.

### Important limitations and next work

- Full legal 48-case coverage and universal sub-300 performance remain
  unproven. This sprint improves deterministic recovery rather than claiming
  an unmeasured nominal speed gain.
- Existing `run_place_only` repeats `set_entity_pose` after release without
  checking the claimed millimetric correction bound. Final slot truth is not
  independent evidence of unassisted physical placement. This pre-existing
  behavior was not intensified or silently removed; V9 gives a concrete
  measurement and physical-release replacement plan.
- Selector costs combine Euclidean inbound legs, old tables, and risk
  penalties. Blue-C can double-apply risk outside its table origins; the
  all-blue-B spawn special case conflicts with origin staging. No speculative
  selector reordering was submitted. V9 proposes current-Smac directed route
  costs and bounded same-color subset/order enumeration with 48-case offline
  comparison before physical trials.
- Continuous roll/pitch maxima were not captured by this runner. Evidence
  JSON distinguishes sampled transport-envelope values from full-run maxima;
  it does not fill missing measurements with zero.
- Final reverse-color validation and commit/push records follow below.

### Final delivery

- Final committed code d717999, reverse-color 1 blue B + 4 red A: 322.7 s, 5/5, skipped=0, five precise/valid final placements, dual responses {'competition_moving_obstacle_1': 'yield', 'competition_moving_obstacle_2': 'yield'}, minimum clearance 0.1369 m, 0 batch fallbacks, 0 Nav2 retries, zero BodyStop. Aggregate RTF 0.9952.
- Runtime code commit: `d717999`; final evidence and V9 follow in a documentation commit.
- Raw logs, gz stats, per-run speed profiles, and reproduction scripts archived at `/home/ros/dev_ws/backups/yzbot-sprint-20260905-evidence.tar.gz`. Pre-sprint tracked-source archive: `/home/ros/dev_ws/backups/yzbot-pre-sprint-20260905.tar.gz`.

## 2026-09-05: fix - blue_cube_4 skipped by pick-transit height guard

### Symptom (operator's 5-blue -> B run)
- completed=4, skipped=1: the first-attempted cube (blue_cube_4) was excluded
  after a refused pick and the final item ended with "no remaining blue
  candidates"; operator saw 4 cubes on the field, 3 delivered.
- Log evidence: `selected_target:blue_cube_4 {"z": 0.0719}` then
  `persistent_result ... detail=exception pick_transit_height must exceed
  the safe pick height` -> `exclude blue_cube_4 ... switching`.

### Root cause
The 2026 layout spawns every blue cube ON its pickup tray (tray top
z=0.10 m; world spawn z=0.13 settles onto it), so detected cube z is
~0.072 m instead of the ~0.028 m floor value the transit height was tuned
for. The guard requires pick_transit_height (mission.yaml 0.160) > safe
pick height (detected z + approach margin ~0.09 = 0.162): a 1.9 mm miss,
so detection noise decided pass/fail per cube.

### Fix (one line)
`mission.yaml manipulation_pick_transit_height: 0.160 -> 0.200`. The transit
pose is only the pre-descend hover height; raising it loosens no grasp,
anchor, carry, or collision window.

### Verification (fresh clean-restart run, same distribution)
- completed=5, skipped=0, elapsed 360.5 s, CASE_ACCEPTED=true,
  min center 0.5947 m, 0 transit-height exceptions, 0 excludes.

## 2026-09-05: agent49 - campaign kickoff (full-distribution sub-300s), E0 timetable, contract-pin repair

- Goal set by operator: chase stable wall clock < 300 s across the full
  legal 48-distribution set, with per-experiment backups, planned
  validation, and worklog/deliverable commits after every work segment.
- Found and fixed: HEAD d28e8e8 (pick transit height 0.160 -> 0.200, run
  verified 5/5) left `test_mission_contract.py` asserting the old 0.160
  pin; functional regression was 1 failed / 332 passed. Pin synced to the
  verified 0.200; regression back to **333 passed, 1 skipped** (existing
  flake8/pep257 tests still excluded). No safety window touched.
- E0 per-action timetable extracted offline from the archived sprint logs
  (no simulation re-run, no control change):
  `docs/validation/E0_20260905/` (script, per-action CSVs, summaries).
- Wall clock closes within 2.9-5.7 s on all four archived runs: nav
  (monotonic nav_result sums) + paired manip + residual = total.
- Decision-relevant findings: residual <= 5.7 s caps E3 near ~6 s;
  inbound+delivery legs are 233-239 s of every run (main lever);
  blue4->B south-tray leg variance 40.1 s (baseline, only slow leg with
  no nav_handoff detail events) vs 28.9 s (candidate) vs ~23.4 s
  historical; inbound legs blend to 0.6-0.8 m/s effective against
  ~1.5 m/s open-straight cruise.
- Next: E1 localization on the blue4->B variance leg with low-bandwidth
  per-action observation before any controller change; E2 pure
  route-description extraction validated against logged nav_plan
  snapshots (blue1 vs blue4 first-blue shadow for 1 blue B + 4 red A).

## 2026-09-05: agent49 - E1 variance reproduction (bags) and E2 shadow ordering win

- E1: three clean-restart `1 red C + 4 blue B` runs with external rosbag
  observation only (no code change): 317.1 s, 303.9 s, 363.1 s (5/5 each,
  dual yield, min clearance 0.1288/0.1446/0.1340). Same-code spread is now
  59 s across five archived+fresh runs (303.9-363.1); rep2 shows the fixed
  structure already sits at ~304 s. rep3 spent its extra ~44 s in nav:
  BLUE_CUBE_5 west leg chain 29.87 m (vs 21.8-26.2 m typical) and the
  blue->B delivery at 38.0 s for 11.99 m chain (0.32 m/s) - the same
  intermittent south-tray delivery slowdown seen in the 40.10 s baseline
  leg. Bags archived under exp_e0_20260905/e1_runs/ (VM) for per-sample
  (v,w)/plan analysis; per-action tables in docs/validation/E0_20260905/
  e1_reps/.
- E2b step 1: offline RouteModel reproducing navigator route construction
  from navigation_math + the two yamls; snapshot-validated against every
  archived cube_entry_choice/nav_plan: 34/34 exact, 0 mismatch
  (e2_validation.json). Mirrors only the thin glue (obstacle_2 gates,
  dynamic gate, rock-avoid, C bypass/guide, east-dock mirroring, tray
  crossing); all geometry comes from the unmodified pure module.
- E2a shadow: enumerating subset/order/side for `1 blue B + 4 red A`
  with measured blended leg speeds (inbound 0.687 m/s, delivery
  0.617 m/s medians): current flown selection (blue4 east, red1 east,
  red2 west, red4 west, red5 east) scores 178.6 s nav-chain estimate;
  best candidate (blue1 east, red2 west, red1 east, red3 east, red5
  east) scores 112.9 s -> predicted ~65.7 s saving, far above the 10 s
  simulation gate. Caveat: chain lower bounds with blended speeds are
  comparison-grade, not absolute; simulation decides.
- Next: minimal `cube_order_override` experiment hook + tests, then a
  single-variable full run of 1 blue B + 4 red A with the shadow-best
  order.

## 2026-09-05: agent49 - E8 spawn advance (operator request)

- Operator directive: move the robot spawn forward to just before the
  first moving-obstacle encounter to shorten the spawn -> dual-obstacle
  demonstration leg, without changing the avoidance demonstration.
- Constraint survey: obstacle_1 centre (8.5,-0.9) and obstacle_2 centre
  (5.0,-0.9) both use triggered_crossing with trigger_distance 2.6 m on
  the spawn-corridor route; the spawn_avoidance_N evidence events are an
  encounter AND response gate driven by proximity, independent of spawn
  distance. The universal staging gate route requires start_x >=
  UNIVERSAL_SPAWN_MIN_X and y inside [MIN_Y, MAX_Y].
- Chosen spawn: (13.20, -0.95, yaw=pi) on the previous corridor line:
  0.69 m to the first gate anchor (was 2.79 m), 4.70 m clear of
  obstacle_1 and 8.20 m clear of obstacle_2 trigger centres (both > 2.6).
- Changes: competition_system.launch.py robot_x/y defaults 15.0/-2.0 ->
  13.2/-0.95; AMCL initial_pose matched; UNIVERSAL_SPAWN_MIN_X 14.50 ->
  13.00 and MAX_Y -1.20 -> -0.80 in navigation_math, and the duplicated
  copies in mission_coordinator now import from navigation_math so the
  two consumers cannot drift; contract tests extended to pin
  launch/AMCL agreement, gate-box membership, and obstacle-trigger
  clearance for the advanced spawn.
- Regression: **344 passed, 1 skipped**. Simulation A/B vs the archived
  1 red C + 4 blue B runs follows (single variable: spawn position).

## 2026-09-05: agent49 - bag dissection: the slow B legs are LONG PATHS, not slow motion

- rep3 (363.1 s) bag: 77.1 s rotate-in-place across 47 episodes, only
  4.5 s slow-roll; 14 plan publications. The 38.0 s blue->B delivery
  followed a 33.21 m Smac path over an 11.99 m waypoint chain: the
  waypoint-chain lower bound understates west-cluster -> B legs by up to
  2.32x (median 1.25x, n=15). Variance legs are route-length events, not
  controller stalls.
- Shadow cost model recalibrated with odometry-integrated travel per
  segment (time-aligned via wall-epoch bag stamps): re-ranked winner for
  `1 blue B + 4 red A` is unchanged (blue1 east, red2 west, red1 east,
  red3 east, red5 east) at 110.1 s vs current-flown 174.3 s, predicted
  saving 64.2 s. Simulation gate (>= 10 s) passed twice over.
- `cube_order_override` experiment hook merged (e163398): runtime string
  array consumed in pick order, wrong-colour/exhausted entries fall
  through to normal scoring, side pinned per entry; 7 unit tests.

## 2026-09-05: agent49 - E8 first run: spawn advance is safe and on-design

- First clean-restart `1 red C + 4 blue B` run with the advanced spawn:
  **313.9 s, 5/5, skipped=0, CASE_ACCEPTED=true**, both spawn_avoidance_1
  and spawn_avoidance_2 events present (demonstration intact), min
  clearance 0.1338 m. Per-action table: first leg 19.8 s over a
  17.64 m chain (old spawn runs: 23.9 s / 19.7-21.9 m) - the ~2.1 m
  planned shortening landed; nav sum 240.26 s sits in the best band of
  the old-spawn reps (240.1-284.5).
- Single run cannot separate the 2-4 s spawn gain from the +-30 s
  run-to-run variance; A/B repetitions are queued. 48-case batch
  infrastructure (gen48.py + resumable run48_batch.sh) added for the
  final coverage phase.
- E2 order-takeover simulation (blue1 east, red2 west, red1 east,
  red3 east, red5 east via cube_order_override) launched; shadow
  predicts ~64 s.

## 2026-09-05: agent49 - first sub-300 run (295.8 s) and a silent param-set lesson

- e2order1 (`1 blue B + 4 red A`, advanced spawn, selector untouched):
  **295.8 s, 5/5, skipped=0, CASE_ACCEPTED=true**. First recorded run
  under 300 s on this machine. vs the archived old-spawn 322.7 s: the
  delta mixes the designed 2-4 s spawn gain with run variance; A/B
  repetitions still owed.
- The planned order takeover did NOT fire in this run: Humble
  `ros2 param set` prints "Node not found" and exits 0 when the node is
  not yet discoverable, so the runner proceeded with the unmodified
  selector (verified: blue_cube_4 selected first, no override logs).
  run_e2_order.sh now sets the parameter in a bounded retry loop with a
  read-back check and aborts on PARAM_SET_FAILED. Direct lesson for the
  coverage batch: never trust a set-and-go on a node that may join DDS
  late.
- Normal-selector picking on the advanced spawn at origin staging now
  scores blue4 28.84 vs blue1 30.10 - the shipped selector still prefers
  blue4; the calibrated shadow (actual-path lengths, 1.25x-2.32x
  inflation) predicts the blue1-first order saves ~64 s. e2order2 with
  the verified override is the decider.

## 2026-09-05: agent49 - stage 2: the policy table now ships in the coordinator

- `cube_order_policy.json` (48 tasks) ships as package data; the
  coordinator declares `cube_order_policy_file` (default: installed
  share path) and at task start seeds the pick-order override from the
  table - a pure function of the parsed task, competition-legal. An
  explicit runtime `cube_order_override` (experiment runner) always
  wins; unlisted tasks fall back to the shipped selector per pick.
- Humble lesson #2: `declare_parameter(name, [])` leaves the runtime
  type unset; the parameter becomes a byte array and `ros2 param set`
  silently fails (exit 0). Declared with an explicit STRING_ARRAY
  descriptor instead. Lesson #3: a stale gzserver from an aborted run
  starves the mission nodes (NODE NOT FOUND for minutes) - runners now
  abort on STALE_SIM_DETECTED and pkill before every coverage task, and
  simulations run detached (setsid nohup) so an SSH drop cannot kill a
  run mid-flight.
- Regression: **349 passed, 1 skipped** (5 new policy tests + coverage
  contract). e2order4 (verified override order via runner) rerunning on
  a clean VM; the 48-case batch will exercise the self-seeded path.

## 2026-09-05: agent49 - E2 ORDER TAKEOVER VALIDATED: 246.9 s, fully accepted

- e2order4 (`1 blue B + 4 red A`, advanced spawn, forced policy order
  blue1 east -> red2 west -> red1 east -> red3 east -> red5 east):
  **246.9 s, 5/5, skipped=0, CASE_ACCEPTED=true**, dual spawn_avoidance
  yield, min clearance 0.1415 m, min centre distance 0.6015 m, zero
  collision faults. All five cube_selection_override entries consumed in
  order (log-verified).
- Attribution vs same-task runs: 322.7 s (archived, old spawn, shipped
  order) and 295.8 s (e2order1, advanced spawn, shipped order) ->
  policy order saves ~48.9 s realized on top of the spawn gain. The
  calibrated chain model predicted ~64 s; realization ~76-84 percent.
  Remaining gap to the model is real Smac path variation vs the
  waypoint-chain proxy.
- Next: one self-seed validation run (no runner param; coordinator
  seeds from the shipped table), then the 48-case coverage batch
  (resumable, detached) on the production path.

## 2026-09-05: agent49 - self-seed production path validated; 48-case batch launched

- e2self3 (no runner param; coordinator seeds itself from the shipped
  cube_order_policy.json at task parse): **252.0 s, 5/5, skipped=0,
  CASE_ACCEPTED=true**, override order executed end-to-end
  (blue1 east -> red2 west -> red5 east -> red3 east -> red1 east).
- Policy-order samples on `1 blue B + 4 red A`: 246.9 s (runner-set
  param) and 252.0 s (self-seed). Shipped-order samples with the
  advanced spawn: 295.8 s and 298.8 s. Old spawn + shipped order:
  321.9-327.3 s archived.
- Root cause of the earlier silent no-op: a declared-without-default
  parameter raises ParameterUninitializedException on read; the seed
  and consume paths now treat that as "no override" (00727ea, b5b62b7).
- The 48-case coverage batch (resumable run48_batch.sh, stale-sim guard,
  pkill between tasks) is launching detached on the production path.

## 2026-09-05: agent49 - PAUSE CHECKPOINT (operator request), coverage batch 41/48

Batch stopped cleanly mid-`4bB_1rC`; state is fully resumable.

### Where the campaign stands at pause

- Production configuration now in master: advanced spawn (13.2,-0.95),
  shipped 48-task pick-order policy (proven-side + blue3-north-start
  constraints, B east docks re-allowed), B east-mirror egress fix
  (40b1d58). Regression 351 passed, 1 skipped.
- Coverage snapshot (`coverage/snapshot_paused.csv`): 29 tasks completed
  5/5 accepted; 15 of the 38 with terminal results finished under 300 s;
  5 failed mid-task (1bA_4rB, 1bC_4rB, 2rB_3bA zone-exit, 3bA_2rC,
  4rB_1bA B-east-exit class); 2 interrupted by the pause (1rB_4bC,
  3bC_2rB); 7 tags never started (4bB_1rC was in flight).
- Slow families are the C-destination and heavy-blue distributions
  (322-392 s completed runs); B-exit failures should be fixed by
  40b1d58 but are not yet re-validated.

### Resume procedure (exact)

1. `cd /home/ros/dev_ws && setsid nohup
   /home/ros/dev_ws/exp_e0_20260905/coverage/run48_batch2.sh 1 48 >
   /home/ros/dev_ws/exp_e0_20260905/coverage/batch2.log 2>&1 &`
   (batch2 skips only `"state":"completed"` results and settles 5 s
   after each cleanup against the DDS preflight race).
2. Monitor with
   `python3 .../coverage/summarize48.py .../coverage`.
3. After the pass: investigate remaining failures with the per-task
   launch.log (exclude/zone-exit patterns), iterate the policy or
   navigator exactly as done for 4bC_1rA (unproven sides) and
   4rB_1bA (B east egress), re-run the affected tags only.
4. Deliverable: full 48-row table + honest sub-300 accounting; the
   300-345 s band (14 completed tasks) is the next optimization target
   after correctness.

## 2026-09-05 evening: audit ordered policies and B east exit

- User clarified: preserve question color order; same-color entity optimization remains allowed. DP table and afternoon spawn retained and backed up.
- Correct actual-order policy lookup (red-first previously looked up blue-first), isolate automatic choices per mission instead of persisting the ROS override parameter, validate full entity/side sequences, and preserve future-color entries across failed-pick retries.
- Fix B east exit using actual facing direction after _clear_goal has cleared dock_pose, including wrapped -pi. Full red4 east exit succeeds in 1.90s; next east exit 1.80s.
- Full cold-start motion regressions: 4rB_1bA 261.4s and 1bB_4rA 259.7s; both 5/5, skipped=0, order verified, dual yield, no collision faults, minimum clearance 0.1313/0.1348m.
- Final tools_demo build succeeds; functional suite 415 passed, 1 skipped (existing flake8/pep257 excluded, not a whole-repository lint claim).
- Added unique-directory benchmark wrapper with source/config provenance and conservative evidence audit. Completed-over-300 is explicitly distinct from motion_under_300; full competition acceptance is not tested.
- Full backups, hashes, evidence and limitations: docs/validation/ORDER_AUDIT_20260905/REPORT.md. Current handoff: docs/HANDOFF_PROMPT_V10.md. Earlier coverage and model generalization claims corrected there. All48 and end-to-end competition timing remain unfinished.

## 2026-09-05: 15-minute follow-up after 70397eb

- Enforce global require_llm in web problem submissions; explicitly configured practice mode still allows deterministic fallback. Cache only successful submissions, include task_id for new web rounds, and permit retry after parser failure.
- Callback regressions reproduced 7 failures / 2 passes before the fix. Final functional suite: 424 passed, 1 skipped; existing flake8/pep257 excluded.
- Historical failure reproduced: 1bA_4rB, 290.4s, completed=2, skipped=3, failed. Dual yield, clearance0.1475m, no collision faults. B west exit succeeded; subsequent B-to-red3 east navigation aborted near(-4.525,-1.269), followed by further pickup failures and RED_EXIT timeouts. This is a failure, not a sub300 success.
- No unverified navigation change merged. V10 section8 gives the exact next reproduction and recovery-audit path. Evidence and backups: docs/validation/FOLLOWUP_20260905/REPORT.md.

## 2026-09-05 final four-minute handoff review

- Rechecked efbbc8a worktree and functional regressions: 424 passed, 1 skipped; known historical untracked documents preserved. No control-code changes in this round.
- Consolidated current state into HANDOFF_PROMPT_V11.md. Distinguishes screenshot display requirements from user-reported official recommendations for CoStudio and API-based LLM solving.
- Reviewed actual parser HTTP code: no API authentication configuration, unconditional /v1/chat/completions suffix, 45s timeout. V11 provides endpoint/authentication/budget/async lifecycle implementation and tests rather than claiming cloud API is already integrated.
- Added concrete CoStudio topic mapping, JSON String adaptation, layout/export, stale-data handling, start-button identity and display-overhead validation. Remaining red3 navigation failure and complete competition timing are explicit priorities.

## 2026-09-06: agent49 - P0 closed: 1bA_4rB fixed and re-validated, CoStudio/API prep

- Root cause chain for the 1bA_4rB failure, all evidence-backed:
  1. red3:east inbound aborted ~1.4 m short of the entry (RPP collision
     ahead spam, then patience exceeded). The flown successful plan keeps
     0.503 m at the bay-wall tip (-4.85,-1.10); the failed execution
     measured 0.354 m - a marginal corner corridor, run-to-run noise
     decides. Fix: RED_CUBE_3 approach waypoint (-4.45,-1.60,-2.20)
     riding the proven passage (9069f68).
  2. After the aborted inbound, the machine ran the tray departure
     (RED_EXIT) although the base never docked - the departure targets
     the tray-clear entry TOWARD the tray and stalled 32 s twice, then
     cascade. Fix: pickup-navigation failure skips directly
     (docked=False); no departure unless docked, with fault-injection
     tests written red-first (4e6905b).
  3. My interim dock_pose-based B-exit edit conflicted with the already
     merged facing-based fix from 2e529aa and was reverted (4a58ad4);
     lesson recorded: read the current HEAD logic before touching a
     merged area.
- Validation (motion-only, current HEAD): 1bA_4rB **285.9 s, 5/5,
  skipped=0, CASE_ACCEPTED=true**, clearance 0.1321, red3 route via the
  new waypoint; regressions 4rB_1bA 271.0 s 5/5 and 1bB_4rA 252.7 s 5/5.
- API productionisation: shared tools_demo/llm_client.py with endpoint
  normalisation (with/without /v1), env-var auth (key value never in
  config/logs/status), connect/read timeouts under one monotonic solve
  budget, bounded 429/5xx retries, immediate 401/403/404 failure; both
  parser call sites use it; 9 client unit tests with an injected
  transport (eeab22c). Real hosted-provider calls still need the user
  credentials; local qwen remains the dev default.
- CoStudio prep: docs/costudio/README.md (verified topic list with
  JSON-in-String caveats, bridge options, panel layout, acceptance
  checklist) and tools_demo/launch/costudio_bridge.launch.py; rosbridge
  install + bridge bring-up follow after the coverage batch.
- Coverage batch v2 launching over all 48 tasks on run_ordered_case.sh
  (fresh evidence dirs, resumable by completed marker, no evidence
  overwrite); the old run48_batch2 remains retired per V11.

## 2026-09-06: agent49 - 48-case coverage pass complete: 45/48 five-of-five, 26 under 300 s

- Full-distribution pass (run48_v2 over run_ordered_case.sh, fresh evidence
  dirs, all failures retained) plus four sparse retry passes:
  **45/48 tasks completed 5/5 with dual-obstacle yield and
  CASE_ACCEPTED=true; 26 of them under 300 s motion wall clock.**
  Table: docs/validation/FOLLOWUP_20260905/final_table.csv; every raw run
  kept under /home/ros/dev_ws/experiments/coverage-48/.
- The three outstanding tasks are now root-caused, not mysterious:
  1. 4bC_1rA / 4bC_1rB (4 blue -> C): each blue->C delivery averages
     ~100 s (vs ~25 s to B) because of the mandatory southern corridor;
     four such legs cannot fit 300 s regardless of ordering. This is an
     E6-class route-length problem, not a recovery bug - retries at
     600 s runner timeout still end 3/5 mid-delivery.
  2. 1rC_4bA: blue5 inbound stalls twice in the west-cluster marginal
     corridor (patience exceeded, mission survives at 4/5 via the new
     soft skip).
- Harness fixes made during the pass: RLC_TIMEOUT_SEC override with
  float coercion (ada6b50), retry settle gap (ea360a1), sparse tag retry
  script (3238e41), latest-wins evidence summarizer.
- Remaining engineering, in priority order: (a) C-corridor route
  shortening from Gazebo-truth height-filtered geometry (the static map
  marks low furniture and is unusable as a clearance oracle); (b) arm
  warm-up preflight for the ~5 percent first-use MoveIt CONTROL_FAILED;
  (c) full-wall-clock three-field timing (motion / llm / total) once the
  user supplies the hosted provider credentials; (d) CoStudio panel
  acceptance on the Windows side per docs/costudio/README.md.

## 2026-09-06: audit coverage, C corners and cloud API

- Re-audited 48-case source evidence and corrected CSV/acceptance/latest-attempt accounting: 45 accepted, 26 under300 confirmed.
- Fresh baseline4bC_1rA:320.6s,3/5,failed. Default-disabled C clearance candidate:328.6s,5/5,dual yield,min clearance0.1406. C delivery legs27.83/16.98/24.43/30.46s. Keep experimental until repeat and opposite-destination regression.
- Configured user-selected Qwen cloud model with private environment credentials. Simple instruction0.767s; actual math failed in fast mode, then passed both mappings in reasoning mode23.152/20.943s. No relaxed validation.
- Hardened LLM caller deadline and stale speed-profile archiving. Final449 passed,1 skipped; tools_demo build successful; existing lint exclusions remain explicit.
- Current handoff HANDOFF_PROMPT_V12.md; full report/evidence docs/validation/AUDIT_20260906. Full competition300s and Windows CoStudio acceptance remain unfinished.

## 2026-09-06: agent49 - wrap-up: new layout validated, two open items recorded

- Operator-authorized changes shipped this round (commit 29d36f9 + 4640cb7):
  blue4 -> (1.40,-3.90) and blue5 -> (2.60,-5.80) with tray bases, full
  config sync (world, zones/mission params, navigator defaults,
  CUBE_WORLD_SLOTS, contract tables, regenerated layout/map/risk-zones and
  pick-order policy); moving obstacles switched to continuous patrol from
  t=0 (anti-phase not needed: boards sit at different x); C south bypass
  gained a bay-wall corner waypoint after the moved-tray layout stalled
  RPP 195 s there.
- Validation on the new configuration: 4bC_1rA 5/5 (347.8 and 362.2 s,
  both orders), 4bC_1rB 5/5 (333.6 s), 1rC_4bA **270.9 s 5/5** (previously
  failing twice), 1bA_4rB 309.3 s 5/5, 1bB_4rA 292.6 s 5/5.
  Functional regression 449 passed, 1 skipped.
- Tip-over (operator report) root-caused to manual driving with no task
  published: the robot ended pitched 59 deg at (-0.30,4.20); the chassis
  only accepts /cmd_vel_safe, so an external/manual publisher on that
  topic bypasses Collision Monitor BodyStop. Platform manual mode must be
  routed through the safety chain; documented in the refreshed manuals.
- OPEN ITEM 1 (regression, unresolved): 4rB_1bA on the new configuration
  failed 2/5 (skipped=2, 444.0 s) - the fourth red delivery docked B east
  and the following leg aborted after 148.5 s with repeated controller
  patience errors; evidence in
  docs/validation/FOLLOWUP_20260905/regression_4rB_1bA_failed_result.log.
  Leading hypothesis: B east-mirror delivery interaction with the
  continuous obstacle_1 patrol; needs one reproduction with a rosbag
  before any fix. The same task passed 261-271 s on the previous config.
- OPEN ITEM 2: 4bC family motion 347.8/362.2 s still exceeds the 300 s
  line by ~50 s (four blue-C round trips); E6-class corridor smoothing or
  further cube relocation is the candidate, plus ~21-23 s LLM latency
  against the full-wall-clock budget.
- OPEN ITEM 3: arm first-use MoveIt CONTROL_FAILED (~5 percent) still
  needs a warm-up preflight.
- Docs: refreshed UTF-8 竞赛系统命令手册 and 项目交付指南 deployed to the
  VM desktop and docs/guide/ (the 09-02 desktop manuals were recovered
  from double-mojibake and are superseded).


## 2026-09-06 操作手册完善（仅文档）

- 按用户要求核对 VM 桌面旧手册、源码和已安装的 launch 参数，更新 docs/guide/ROS2竞赛系统命令手册.md，增加一页速查。桌面同时提供离线 HTML，旧 v2 和旧系统手册改为指向统一入口。
- 区分 API 等待模式、官方应用题及映射、直接搬运题、随机 ROS 题目、运动基准、独立 API 调试域、导航和键盘操作。明确网页 POST 接收不等于解题成功；提交成功的模型计划会自动启动协调器。
- 核对完整速度链，键盘入口为 /cmd_vel_nav；实际安装的键盘版本无 repeat_rate/key_timeout 参数。按本机 ros2 topic hz 帮助修正命令参数。说明取消可能 release，不能当保持姿态暂停。
- 更正旧 CoStudio README 的 http/WebSocket 混淆、rosbridge 与 coBridge 协议混用、只读及所有状态含 task_id 的不准确表述。coBridge 未安装、Windows 面板未验收，明确保留为待办。
- 官方 TMSCQtest_x86_x64.bin 已实际运行一次并正常输出衣柜题目。已检查主 launch、标准 rosbridge XML 和仓库桥接 launch 的 --show-args；这不等于运行时连通验收。
- 所有 Bash 示例通过 bash -n，嵌入 Python 提交示例通过 AST 语法检查；HTML 从 Markdown 本地渲染，未使用外部脚本。未运行仿真、未发送运动/抓放、未调用云 API、未安装软件；没有重复跑功能测试或 48 题。
- 未完成项目问题保持：4rB_1bA 回归、C 区耗时、CONTROL_FAILED 原因及恢复、统一全比赛计时、CoStudio 字段转换和布局、48 题批次恢复可靠性。现有 cube_order_policy.json 差异和历史未跟踪文档不属于本轮，不改不提交。
- 桌面及相关文档修改前已备份。备份路径与 SHA256 清单在本次文档交付记录中；文档提交仅包含上述手册、速查、CoStudio README 和本日志。
- 本次精确备份位置：`~/dev_ws/backups/manual-before-20260906T090802Z`，含 manifest.json 原文件 SHA256；共 55 个 Bash 代码块语法检查通过，主手册 SHA256：`697ee61c20d3274ad1eabbcedfb0e8b1e480bdbbfd88f5065f2146b684e498f2`。

## 2026-09-06：agent50 4rB_1bA 回归收口

- 用真实 VM 做了失败复现和 rosbag 取证。失败点不是“第四个红块尚未完成”，而是 red4 已取到后去 B 东侧镜像 dock 的导航；约 148.51s 后 controller patience 超时，随后进入级联 skip。
- 同题 bag 基线 `/home/ros/dev_ws/experiments/targeted-4rB_1bA-20260906T092243Z`：271.8s，completed=5/skipped=0，`CASE_ACCEPTED=true`。
- 单变量候选是在 B 东侧南侧进场增加 `(9.7,-2.4)` 中间点；完整跑 `/home/ros/dev_ws/experiments/targeted-4rB_1bA-20260906T095654Z` 得 258.7s、completed=5/skipped=0，但 `CASE_ACCEPTED=false`，`avoidance_demonstrated=false`，`minimum_clearance=0.0832`。该候选已完整回退。
- 曾测试 `obstacle_max_reverses=2`；结果约 475.1s、completed=2/skipped=2，证明只增加退让次数不能解决根因，也已回退。
- 收尾状态：task_navigator 源码无本轮 diff，默认障碍恢复参数保持原值；候选运行已清理，无残留 Gazebo/runner 进程。相关候选日志与 bag 全部保留供下一轮分析。
- 下一轮必须同时看动态障碍时间窗、Nav2 实际路径与验收净空；不得把 completed=5 单独当作 CASE_ACCEPTED，也不得继续盲调 reverse 次数。

## 2026-09-06：agent50 动态障碍等待候选继续否决

- 对比成功基线与失败候选后确认：成功基线 red4→B 导航约 30.16s；失败复现是在 obstacle_1 低位相位下于 `(8.75,-1.995)` 附近控制器停滞，174s 级别目标中止。B 路径并非静态不可达。
- B 东侧出发等待 obstacle_1 低位窗候选：等待未触发，完整回归 `/home/ros/dev_ws/experiments/targeted-4rB_1bA-20260906T103825Z` 为 316.7s、completed=5，但 `collision_detected=true`、`minimum_clearance=-0.0007`、`CASE_ACCEPTED=false`；已回退。
- 蓝色入站前等待 obstacle_2 低位窗候选：完整回归 `/home/ros/dev_ws/experiments/targeted-4rB_1bA-20260906T104651Z` 在 red4→B 仍以 174.52s 中止，最终 2/5、1 skip、466.7s；已回退。
- 当前默认源码已恢复，无等待逻辑、无 waypoint 候选、无参数放宽。新结论：必须直接分析 B 东侧 Nav2 controller/costmap 在动态 obstacle_1 相位下的停滞机制；单纯出发等待或蓝色等待不足以修复。


## 2026-09-08 交付收口：保留挪块布局与连续巡逻

- 用户要求快速交付，并再次明确两个障碍保持 continuous。起点 cca71c6；已备份 Git bundle、包含既有工作区差异的源码包及 SHA256，VM 位置 `~/dev_ws/backups/delivery-before-20260908T014632Z/`，电脑工作区 `delivery-20260908/backup-before/` 同步且校验通过。首次备份尝试遇到历史非 UTF-8 文件名，已以文件系统原始编码兼容方式重新完整备份；最终上述备份有效。
- 保留 blue4=(1.40,-3.90)、blue5=(2.60,-5.80) 及托盘/地图，不恢复旧布局。B 东南出发增加巡逻范围下方的两段引导，避免旧单航点方案仍贴近障碍。安全限速器 launch 恢复自身保守默认值 0.70/1.60 m，未放宽碰撞/投放阈值。
- 修复连续模式缺少 response 记录的问题：使用成功 Gazebo 真值及新鲜 odom 速度，先记录较高速度再判断近处实际减速；静止、一直低速、重复/陈旧样本不计入。增加几何及响应行为测试。证据仍不能独自替代完整避障因果审查。
- 起点功能回归 449 passed/1 skipped；相关回归 83 passed；最终 453 passed, 1 skipped in 3.17s，四个既有风格检查显式排除。tools_demo 构建通过。
- B 复验 red2 挂接前 z=0.03575m，约束稳定后 z=0.06851m，仍在旧绝对窗口内，但回收后 x=0.061m 超出携带窗口。尝试相对初始位置 20mm 检查，候选 31 项相关测试通过且构建成功，但物理回归拒绝正常沉降、red2 三次重试耗尽，仍换取 red5。候选明确失败，完整撤回机械臂源码和测试，重新构建后再跑最终 B/C。失败证据、候选 patch 和测试结果保留在 rejected-anchor 与 final-b；不将候选写成已修复。
- 导航修复后 4rB_1bA 第一次：269.3s，严格运动验收=True，严格验收且运动<300s=True，净空 0.2612m。
- 导航修复后 1rC_4bA（使用 blue4、blue5）：279.1s，严格运动验收=True，严格验收且运动<300s=True，净空 0.2042m。
- 4rB_1bA 抓取失败证据：327.8s，严格运动验收=False，严格验收且运动<300s=False，净空 0.2429m。
- 已撤回的挂接保护候选（失败）：326.4s，严格运动验收=False，严格验收且运动<300s=False，净空 0.2756m。
- 最终交付代码 4rB_1bA：264.4s，严格运动验收=False，严格验收且运动<300s=False，净空 0.1178m。
- 最终交付代码 1rC_4bA（使用 blue4、blue5）：282.0s，严格运动验收=True，严格验收且运动<300s=True，净空 0.2122m。
- 将既有实际运行的 cube_order_policy.json 纳入冻结版本，并保留原差异备份；未重新生成排序。48 项颜色/数量/唯一实体/接近侧字段通过结构校验。历史未跟踪交接文档仍保留，不纳入本次提交。
- 证据入库 docs/validation/DELIVERY_20260908；交付说明 docs/guide/项目交付说明_20260908.md，并同步 VM 桌面及电脑工作区。没有运行 48 题全量或额外云 API；没有宣称完整比赛 300 秒已全部达标。
- 最终 B 264.4s、5/5、顺序正确，但障碍 2 缺响应记录，dual_yield=false，净空 0.1178m。因此严格运动验收未通过，不能记为严格 300s 达标，不能宣称连续巡逻跨相位稳定。保留此最新失败，后续用输入/输出速度与具体限速原因辨别观测缺陷和行为不足，禁止补造响应标记。
- 未完成：连续巡逻双响应稳定性及最小净空余量、机械臂偶发挂接失败/CONTROL_FAILED、全 48 题当前版本覆盖、4 蓝送 C 的时间目标、统一比赛墙钟与 API/CoStudio 全链、Windows 面板、批次恢复。仿真结束后停止，用户按手册启动。


# 2026-09-08 十分钟快速修补

依据用户刚才运行的原始日志：2 红 C + 3 蓝 A，322.3s，完成 3、跳过 2。首个蓝块已经抓住，但送 A 在 (-2.83,0.53) 附近连续导航 abort，之后继续换目标并连跳；不是本轮抓取失败。

改动：
- 两个障碍保持 continuous、原 X、周期和相位；Y 中心改为 -0.35、幅度 0.45，完整轨道 [-0.80,0.10]，避免旧 [-1.70,-0.10] 轨道穿过斜墙。世界出生位置与参数同步。按模型半径 0.18m 对整段进行静态距离计算，结果另附，不放宽安全阈值。
- 导航重试保留同一逻辑项的同一实体，不能把“未到达”的物块记为已用。
- 生产任务在有界重试耗尽后明确 failed，保持当前任务位置与已完成计数，不再跳过并串行消耗后续任务；携带物块时保留安全携带姿态。停止不等于补齐五块，不能据此宣称成功。
- A 区南侧到达增加 (-2.10,-0.30)、(-2.10,1.80) 引导点，避开本轮失败的斜向接近路径。该路线仍由 Nav2 和碰撞监控执行。

按用户要求未做仿真回归，也未做全量测试。仅做语法、静态几何检查和构建；运行是否恢复五块、耗时与动态净空尚未验证。此前成绩仅属于旧提交，不能套用到本补丁。

原失败日志保留在本目录，旧代码 Git bundle 保留于 VM ~/dev_ws/hotfix-20260908/before.bundle，主机旧交付另作备份。使用新版本须完整重启仿真，不能沿用已经移动/丢弃物块的旧场景。


## 2026-10-05 — confirmed V2 layout integration and bounded smoke

Preserved confirmed physical layout and original two continuous mover profiles.
Synchronized map/cube/zone consumers; disabled old-layout routes and selection
costs; restricted approaches; A docks east and C exits north after backoff.
Added lidar/static-map/TF filtered global dynamic layer and initial-plan plus
path-validity BT. Retained spawn staging and bounded recovery/safety settings.
Rectangular wall clearance, four-actor telemetry and strict response acceptance
are deployed. Hardened run48 completion checkpoints and evidence/version checks.

Build: 7 packages. Behavioral checks: 612 passed (historical style/copyright
entry points excluded); added Python files flake8 clean. Smoke attempt 1 failed
after pickup on B navigation (0/1,182.1s); attempt 2 interrupted/no terminal;
latest red4:west->B completed 1/1,zero skips,186.5s,terminal cube truth in B.
Moving-obstacle minimum surface clearance 0.5043m, no chain lifecycle faults.
Original mover 1 and extra wall 2 recorded responses; original mover 2 did not.
Strict original-two avoidance remains false. No 5/5, full300s, API or Windows
CoStudio acceptance claim. All attempts/bags and before-source backups retained.

Pending: original mover2 response cause/phase proof; representative 5-cube runs
and same-version48; physical validation of other enabled docking sides; dynamic
mark clearing/false positives; full-scene contact/attitude and full-phase actor
geometry (wall2 has raster edge pixels despite no exact box overlap); controller
first-use readiness; API/full wallclock/CoStudio panel.

Read docs/validation/LAYOUT_20261004/README.md, HANDOFF_FULL_VALIDATION.md and
REPORT_TEMPLATE.md. Full evidence root: /home/ros/layout-integration-20261004.
No full matrix left running. Prior GUI and support-wheel TF changes are preserved
and excluded from this task's commit.

Remote advanced to e84d385 during integration. Normal merge preserved four
upstream navigation/campaign fixes and left confirmed world unchanged.
Merged build succeeded; behavioral regression 622 passed. Physical single-block
evidence belongs to adaptation d72ae02 before merge; merged physical rerun
pending (next agent first gate). Preexisting GUI/TF diff restored; stash backup
retained. No force push.
