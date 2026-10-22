# East arc handoff and blue5 delivery return

The CORNER-FLOW-AB-2 round completed all deliveries but still contained two
navigation defects. Its earlier east-corner audit used x >= 12.75, excluding
the arc handoff at (12.4, -2.5). The broader exit window shows 2.8 s below
0.08 m/s, with a stationary turn away from the westbound path and back again.
Completing 5/5 did not establish a smooth handoff.

## Completed arc: preserve path-based heading ownership

The navigator previously canceled the completed arc and invoked manual
pre-alignment toward the next pickup's straight-line bearing. For blue1,
that bearing pointed northwest through intervening walls. Nav2 subsequently
aligned the base back toward its westbound obstacle-aware path.

Both odometry-triggered handoff and normal successful action completion now
identify a completed FLOW batch or single FLOW recovery pose. They delegate
the next stage's heading to Nav2 without the intervening bearing turn. The
old action must still stop and return its result before the next action
starts. Ordinary stages retain their existing alignment contract. Failed
actions and unproven physical arc anchors still enter recovery; they cannot
advance through the new continuation branch.

Two development rounds exposed an additional approach problem: the long
blue1-to-east-arc goal directed the roof-tip departure toward moving
obstacle 2 before the robot had reached the south aisle. Its crossing
recovery could end in the roof's inflated cells. The upper-right loaded
return now first reaches the existing open lane at (4.4, -2.5), then heads
east to the arc. The point has 1.033 m of static surface clearance. Tests
also verify the full eastbound line with the original 0.28 m body radius,
0.20 m tracking reserve and 0.10 m extra physical clearance.

This approach is a single FLOW_APPROACH action ahead of the FLOW_EAST
batch. The exact planner, original 0.20 m physical arrival proof and
collision-protected CornerFlow controller apply. It must be physically
reached before dispatching the arc. The separate family prevents the long
approach from being silently merged across this proof, and its successful
handoff also retains path-based heading ownership. The shared scheduler
sees the same approach route.

## blue5: remove the unsafe translated staging point

The deployed route helper used to translate every southeast departure
north by 2 m. At blue5's west exit (10.93, -13.00), this creates a waypoint
at (10.93, -11.00), only 0.141 m from Wall_132 in the loaded Gazebo world.
The configured local chassis radius is 0.28 m. Its occupancy-grid cost was
99 before navigation even started.

Recorded odometry went around the east wall end, reached y=-9.117, then
turned south to y=-10.818 while pursuing this waypoint. RPP withheld commands
for a predicted collision and aborted after the existing 2 s tolerance.
The serial fallback retained the same unsafe waypoint. The first batch had
never physically visited it: its closest approach was 0.528 m. This was not
an already-visited waypoint being erroneously replayed.

The deployed layout no longer adds coordinate-translated bay staging.
Without a required clearance guide, it gives Nav2 the complete delivery
goal so planning can go around the real wall without returning to it.
Other explicit bay-mouth, tray, patrol and wall-clearance routes remain.
The legacy staging helper is retained for legacy callers. The scheduler
uses the same deployed route helper, so it estimates the route actually
sent to Nav2.

No planner/controller tuning, collision envelopes, arrival proofs,
perception provenance, manipulation behavior, frozen sources or scoring
thresholds were changed.

## Validation

Behavior tests cover both arc-completion paths, serial FLOW recovery,
failed/unproven anchors, the physical clearance defect, primary/backup
blue5 routes and preservation of the other bay-mouth anchors.

GUI evidence is recorded under `runs/corner-flow-20261022/`, with source
receipts, 20 Hz odometry/commands and expanded exit-window analysis. Full
mission and independent perception/Gazebo truth evidence are under
`runs/freeze-20261021/rounds/` and `runs/vision-driven-20261021/`.

CORNER-EXIT-FIX-AB-1 was canceled after an incorrectly inverted A/B mapping
was submitted. It is not an acceptance run. CORNER-EXIT-FIX-AB-2 uses the
same question and mapping as CORNER-FLOW-AB-2 (1 red to A, 4 blue to B).

The complete isolated non-linter suite passed **1733 tests**, with 3 linter
tests deselected. The tools_demo package built successfully. The first
shared-graph test invocation also exposed two live velocity-topic arrivals
in an idle timer unit test; the final run used ROS_DOMAIN_ID=88 so unit tests
could not receive the operating simulator's messages.

CORNER-EXIT-FIX-AB-2 did **not** pass full-round acceptance. Its east pickup
exit was continuous (minimum 0.651 m/s, no samples below 0.08 m/s), but the
subsequent blue1-to-B approach was interrupted by moving obstacle 2 before
reaching the arc. The crossing recovery ended at (5.64, -0.83), cost 99,
beside the zigzag roof; repeated recovery consumed the mission budget.
The round was canceled at the existing deadline during the fourth
placement (completed counter 3/5; wall time 306.792 s). It never exercised
blue5. This failure remains in the evidence and must not be reported as a
successful round or as proof of full-round stability.

CORNER-EXIT-FIX-AB-3 repeated the same roof-side recovery and was canceled
at 252.684 s while holding blue1 (completed counter 2/5). Both AB-2 and AB-3
precede the physically checked south-aisle approach added above. AB-4 tests
that final approach together with the two requested repairs.

### Final-source GUI round: CORNER-EXIT-FIX-AB-4

The final source completed 5/5 in **295.750 s**, with zero skipped cubes,
no collision and correct Gazebo placement truth for all five deliveries.
All five picks used fresh perception targets without slot fallback or
post-detection homing. Navigation source receipts matched the executed
version. The added south-aisle approach completed without wall recovery.

Identical before/after windows now include the east handoff at x=12.4:

| Measured behavior | Before (CORNER-FLOW-AB-2) | AB-4 |
| --- | ---: | ---: |
| East-exit stopped time, speed below 0.08 m/s | 2.80 s | 0 s |
| East-exit minimum speed | 0.00012 m/s | 0.975 m/s |
| blue5 southward excursion beside red4 | 1.701 m | 0 m |
| blue5 return navigation duration | 21.6 s | 11.5 s |
| blue5 RPP collision prediction warnings | 31 | 0 |
| blue5 serial batch fallbacks | 1 | 0 |

The trajectory repair audit passed, but the independent **full-score audit
did not**: this round recorded only one distinct effective obstacle
response (moving obstacle 2), below the unchanged requirement of two.
Completion, time, perception and truth passed; this is not a full-score
acceptance pass. No response was fabricated and no demonstration detour
was added. AB-5 repeats the same final source to check natural encounters.

Recorded before/after plot:
`runs/corner-flow-20261022/CORNER-EXIT-FIX-AB-4/before-after.png`.

### Same-source repeat: CORNER-EXIT-FIX-AB-5

The repeat completed 5/5 in **293.742 s**, again with no collision, all
deliveries verified by Gazebo truth and all five picks sourced from
perception. Source receipts matched. The east-exit stopped time remained
zero (minimum speed 0.321 m/s); blue5's southward excursion remained zero,
with 11.9 s return navigation and no collision prediction warnings or
serial fallback. Both requested navigation-repair audits passed in both
final-source rounds. The simulator and RViz remain open after completion.

The repeat again recorded only moving obstacle 2 as a distinct effective
response. Therefore **full-score acceptance remains unfulfilled** despite
both final-source rounds completing within 300 s. The original two-obstacle
response requirement is unchanged. These results establish the two local
navigation repairs, not guaranteed full-score or identical-speed behavior
across moving-obstacle phases. The remaining response-count gap is recorded
explicitly in each independent vision/score audit and the aggregate report
at `runs/corner-flow-20261022/handoff-analysis/validation-summary.json`.
