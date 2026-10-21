# Continuous passage through the north and east wall corners

The previous south-to-A change removed the red5 detour, but did not cover
other commands passing the same A entrance or the long wall's east end.
Recorded odometry showed the A entrance reaching nearly zero speed, followed
by 1.4 rad/s rotation after a new navigation action began. These stops were
real action/heading transitions, not just angular-looking lines in RViz.

## Route and controller changes

`corner_flow.py` decorates only the deployed north/east corner routes. The
navigator applies the decoration before alignment/dispatch; the scheduler
uses the identical function when estimating the complete transport route.

* A returns retain the north safety pole at (-2.1, 1.85), then follow a 1 m
  radius arc above the wall, tangent to the free A approach. The red2 short
  return joins its existing pole and approach in one action. The previously
  fixed red5 route retains its southern anchors and gains the same arc.
* Departures from A to the lower west pickups follow the arc in reverse,
  reach the original north pole while heading south, and continue through
  the original south pole before the remaining narrow/patrol stages.
* East transfers pass outside Wall_72's tip (12.508, -3.608). The 1 m radius
  circle is centred at (13.4, -3.5), reaching x=14.4 while remaining inside
  Wall_10 at x=15.923. Transfers to/from the lower bay use a quarter circle;
  the B room uses a half circle. A tangent extension moves the batch endpoint
  beyond the turn. Upper-right blue1 transfers use the same arc even though
  they do not carry the older lower_east guide names.

Each contiguous arc group uses one NavigateThroughPoses action and the
CornerFlow RPP controller, with velocity-scaled preview, curvature regulation
and a 1.0 rad initial shim threshold. It does not terminate an action at each
arc sample. The loaded A corner keeps its 0.75 m/s stage limit; other arc
segments use at most 1.2 m/s. Ordinary navigation, narrow S stages and spawn
admission remain separately controlled. Final docking/alignment can stop.

CornerExact uses exact endpoints with the ordinary planner's wall-distance
cost preference (8.0). The central patrol planner remains separate. Keep the
full 0.65 s collision projection setting, cost regulation, independent
Collision Monitor/safety limiter and original physical arrival reserves.
The geometry tests retain body radius 0.28 m, tracking reserve 0.20 m and at
least 0.10 m physical wall gap. No near-goal tolerance was expanded.

Only ordered physical visits within 0.20 m retire arc anchors. Action
feedback counts cannot do so. A failed arc falls back to exact single-anchor
navigation, with already-proven samples removed and batching disabled for
the rest of that command. Successful separate arc groups can still run in
one command. Displayed paths omit proven samples and preserve execution order.
The route estimator charges arc samples as one moving action rather than
adding a stop/rotation penalty for every sample.

## Validation and development evidence

Evidence is under `runs/corner-flow-20261022/`, including source hashes,
20 Hz odometry/command samples, measured corner plots, full task results and
independent RGB-D/Gazebo delivery-truth evidence. The frozen baseline and
perception/manipulation logic were not edited.

CORNER-FLOW-BA-1 (4 red to B, 1 blue to A) completed 5/5 in 272.713 s. The
loaded north corner's minimum/median speeds were 0.476/0.750 m/s, with zero
stopped time and maximum yaw rate 0.625 rad/s. The original full-score audit
passed, including placement truth and three distinct effective responses.

CORNER-FLOW-AB-1 (1 red to A, 4 blue to B) was a failed development run:
4/5 before the original 300 s mission cutoff. North passages and the east
reverse arc had no stopped time (east minimum 0.854 m/s). The outward arc's
long approach used the central planner's low cost multiplier, hugged the
zigzag roof and triggered obstacle recovery. Its single-anchor recovery
also referenced an unregistered planner name. Both issues were corrected
with the independently registered CornerExact planner and a test resolving
all planner/controller/checker IDs used by the two corner behavior trees.
Do not count this failed run as successful acceptance.

The full isolated non-linter suite passed 1715 tests (3 deselected); both
changed ROS packages built successfully. Arc tests cover both directions,
upper-right transfers, conservation of original physical gates, wall
clearance through the loaded saved world, idempotence, ordered recovery,
multiple arc groups and preservation of unrelated/narrow routes.

### Final GUI acceptance: CORNER-FLOW-AB-2

With CornerExact registered and its ordinary wall-cost preference active,
the same 1-red-to-A / 4-blue-to-B task completed 5/5 in **287.759 s**.
The unchanged full-score audit passed: no collision, correct Gazebo truth
for all five placements, all five picks sourced from perception with no
slot fallback, and two distinct effective avoidance responses (spawn
obstacle 2 and central cart 1). Captured navigation source hashes matched.

| Region / passage | Minimum speed (m/s) | Median speed (m/s) | Stopped time (s) |
| --- | ---: | ---: | ---: |
| North, red2 carrying to A | 0.445 | 0.750 | 0 |
| North, departing A for blue3 | 0.799 | 1.200 | 0 |
| East, B to blue1 pickup | 0.883 | 1.072 | 0 |
| East, blue1 carrying to B | 0.869 | 1.091 | 0 |

Stopped time counts odometry below 0.08 m/s inside fixed corner windows;
the actual minima are well above that threshold. Each east half-circle
took about 4 s. The four corner actions completed without serial fallback,
missing-anchor advancement or invalid-planner errors. A separate existing
two-stage bay route on the final blue5 delivery did fall back to serial;
it is outside these corner windows and the mission still passed before
the original 300 s deadline. No acceptance limits were expanded to pass.
The final plotted lines are measured odometry, coloured by measured speed:
`runs/corner-flow-20261022/CORNER-FLOW-AB-2/measured-corners.png`.
