# South pickup to A: remove the westward return and track one continuous path

The two recorded 4-red-to-A / 1-blue-to-B rounds at `manual-20261021T223642`
and `manual-20261021T225456` both delivered `red_cube_5` second. Their delivery
legs took 34.934 s and 34.343 s. The generic western pickup route forced the
base back to (-4.25, -7.15), then east again to (-2.10, -7.15), after the
planner had already passed the eastern tip of Wall_81. This was a route
constraint, not a Nav2 recovery or localization failure.

## Change

For departures in x=[-4.3,-3.3], y=[-13.0,-11.5] returning to the current A
slots, the shared scheduler/navigator route now plans north to (-1.20,-4.55),
then retains (-2.10,-0.30) and (-2.10,1.85) to clear the A wall tip before
turning toward (-4.15, slot_y), a free approach point before the original slot. Both old southern anchors are absent.
The first leg is a collision-checked global plan around the diagonal wall;
it is not a direct command through that wall. Other pickup regions, legacy
layouts and delivery destinations retain their existing clearance routes.

The four clearance poses use one NavigateThroughPoses action. The original
final docking leg follows separately, because its endpoint is intentionally
inside the tray's planning inflation and uses the existing guarded docking
contract. Its intermediate
anchors no longer terminate separate actions or trigger manual bearing-based
pre-alignment. The scoped SmoothTransit controller follows continuous arcs
with a short 0.25–0.35 m lookahead and a larger initial shim threshold. Its
full-route speed is bounded by the existing 0.75 m/s corner limit. The
0.65 s collision projection, cost and curvature regulation, independent
safety limiter, dynamic obstacle admission and final docking remain enabled.
The controller is not applied to narrow pickup pockets or spawn gates.
Ordinary single-goal deliveries also delegate initial orientation to the
planned path, avoiding a preliminary turn toward a target behind a wall.
Protected pickup/clearance stages retain their manual alignment.

Only an actual odometry visit within 0.20 m can retire a safety anchor.
Feedback pose counts do not prove arrival. The displayed command path also
orders the active batch before its suffix and omits physically proven south
anchors, avoiding a misleading line back behind the base. Replanning uses remaining goals;
serial fallback and bounded recovery resume unvisited anchors instead of
returning to already visited ones. An unproven batch cannot report success
or trigger an odometry shortcut to the final dock.

## Validation

Behavior tests cover the new departure region, unchanged other routes,
loaded-world clearance after the diagonal, continuous action/tree dispatch,
ordered physical anchor visits, partial failure and serial recovery, and
initial alignment ownership. The wall clearance check retains a 0.28 m body
radius, 0.20 m arrival reserve and 0.10 m gap. Frozen baseline files are not
modified. Simulation evidence and measured trajectories are recorded under
`runs/south-a-smooth-20261022/`.

### Recorded GUI results

All runs submit the same parsed plan: four red cubes to A and one blue cube
to B, with Gazebo, RViz and the normal vision-driven mission enabled.

| Run | red_5→A (sim s) | Distance (m) | Batch fallback | Manual pre-alignment |
| --- | ---: | ---: | ---: | ---: |
| SOUTH-A-SMOOTH-2 | 33.90 | 21.02 | 0 | 0 |
| SOUTH-A-SMOOTH-3 | 33.40 | 21.04 | 0 | 0 |

Measured anchor errors were below 0.065 m. At the first three anchors the
base remained moving (approximately 0.34–0.65 m/s at closest approach).
Neither trajectory visited the removed anchors: minimum distances to P1/P2
were approximately 3.82/1.86 m. The figures are recorded odometry, not an
offline path comparison. Necessary initial heading reversal and final dock
orientation can still rotate in place; the intervening clearance poses do
not terminate separate actions.

The first development run completed 5/5 in 267.668 s but fell back from the
initial batch because its endpoint was inside the A tray inflation. The
free approach endpoint above fixes that failure without expanding planner
tolerance or weakening physical arrival proof.

SOUTH-A-SMOOTH-2 completed 5/5 in 270.689 s with no collision and correct
Gazebo placement truth. It naturally recorded only one distinct effective
avoidance response, so the original two-response acceptance did **not** pass
in that run. Do not report that run as a full-score acceptance or add an
out-and-back route to manufacture the missing response. The route change
preserves real response counting and all original acceptance thresholds.

The full isolated non-linter suite passed: 1701 tests, 3 deselected. Both
modified ROS packages built successfully. Batch feedback also reserves the
pending final docking stage, avoiding an incorrect 6/5 progress display.

SOUTH-A-SMOOTH-3 completed 5/5 in 274.688 s with no collision, correct Gazebo
placement truth, five perception-sourced picks, no slot fallback, and two
distinct effective responses (spawn obstacle 2 and central cart 1). Its
unchanged full-score audit passed. Navigation acceptance passed in both
final-route runs; the two-response score remains phase dependent (one/two
responses respectively) and is not claimed to be stable in every round.
All captured navigation source hashes matched the code used for each run.
The only code change between the two final-route runs was the progress
counter correction, with the physical route/controller settings identical.
