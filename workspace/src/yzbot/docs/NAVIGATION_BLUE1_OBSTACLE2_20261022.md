# blue1 pickup below Obstacle 2

The previous repair covered the eastern wall arc and blue5's unsafe bay
staging. It did not cover a separate crossing-owner error below Obstacle 2.

## Recorded cause

CORNER-EXIT-FIX-AB-4's B-to-blue1 pickup reached (4.867, -1.579), heading
2.962 rad (westward), with Obstacle 2 at x=5. The base had already crossed
the actor's X coordinate and was driving away from it. However, blue1's
remote goal was east of the actor because the robot still had to go west
around the long upper wall before returning along the northern aisle.

The original trigger checked the goal's X side and the actor's forward/
lateral projection into the body frame. An actor mostly above the base can
project slightly forward even when it is behind the base's horizontal
travel direction. The phase-dependent lateral threshold made the error
intermittent. A false trigger canceled Nav2 and acquired the corridor's
direct-control owner. That owner then deliberately aligned to yaw=0,
drove east past x=5.6 and resumed the original Nav2 goal. Nav2 had to turn
west again. Recorded commands show both rotations and the unnecessary
east/west excursion; this was not simply an RViz path drawing artifact.

## Repair

Crossing admission now also requires `dx * cos(yaw) > 0`: the base must be
heading toward the actor's X before the horizontal crossing controller can
take ownership. Previous response counts still cannot exempt a real
crossing. Admission windows, emergency collision protection, RPP prediction,
Collision Monitor and the safety limiter remain unchanged.

The B-to-blue1 route also keeps the westbound leg at y=-2.5, below the
whole actor patrol, before turning northwest. A 1.2 m radius tangent arc
centred at (5.1, -1.3) starts at (5.1, -2.5), turns 45 degrees and extends
along its tangent to approximately (3.2, -1.10). All these points stay in
the same continuous FLOW_EAST action as the earlier eastern arc. The next
ordinary pickup path starts aligned northwest instead of turning at an
artificial stage boundary below the mover.

The shared scheduler sees the same geometry. Tests sample every arc/chord
and the entire actor patrol envelope using the original body radius
0.28 m, tracking reserve 0.20 m and additional physical gap 0.10 m. Ordered
physical visits within 0.20 m still gate action completion. Both west and
east blue1 pickup entries are covered; unrelated route families are retained.

## Validation evidence

Artifacts are under `runs/obstacle2-blue1-20261022/`. The final isolated
non-linter suite covers the recorded false trigger, its mirrored direction,
both genuine crossing directions and full swept-envelope/wall clearance.
The tools_demo package builds successfully.

The regression uses the built-in runtime order hook to replay the exact
formerly failing legal route: red2-east to A, blue3-east to B, blue1-east
to B, blue4-east to B, blue5-west to B. This prevents task optimization from
avoiding the blue1 leg being tested. It changes neither the delivery targets
nor the original 300 s and safety limits. The order hook is cleared after
the final replay, and no production configuration file is changed.

OB2-BLUE1-REPLAY-1 is an intermediate version: it completed 5/5 in 286.713 s
and eliminated the wrong-direction direct-control takeover. However, its
first proposed clearance endpoint at (4.4, -2.5) still caused about 0.9 s
of stationary Nav2 path-entry alignment. This was not accepted as a smooth
repair. The final tangent arc above replaces that endpoint. A read-only
ComputePathToPose probe from the final handoff point confirmed a northwest
path entry aligned to the outgoing tangent; the physical replay remains
the acceptance evidence.

Audits identify the actual navigation zone rather than the mission's
unresolved job placeholder name during explicit-order replay. They cover
both the original error region and the expanded region x=[2.5,6.8],
y=[-2.9,0.4], including the new curve and its final handoff, so moving a
stationary turn outside the old crop cannot produce a passing result.

### Final replay: OB2-BLUE1-REPLAY-2

The final isolated suite passed **1739 non-linter tests** (3 deselected).
The final GUI replay completed **5/5 in 290.740 s**, with no collision and
two distinct real effective responses (moving obstacle 2 and central
cart 1). The independent full-score audit passed without changing its
criteria. All five visual picks and Gazebo placement truth passed, and
navigation source hashes matched. The runtime order override was cleared
and read back as an empty string array after the replay.

| Measured blue1 pickup behavior | Before (CORNER-EXIT-FIX-AB-4) | Final replay |
| --- | ---: | ---: |
| Stopped time below Obstacle 2 | 5.8 s | 0 s |
| Stopped time including new curve and handoff | 5.8 s | 0 s |
| Eastward backtracking while heading to the west flank | 1.239 m | 0 m |
| Erroneous Obstacle 2 crossing-owner holds | 1 | 0 |
| Direct angular-control samples in the original region | 76 | 0 |
| Minimum speed in the expanded region | — | 0.634 m/s |
| B-to-blue1 navigation duration | 31.3 s | 26.6 s |

Measured minimum clearance to the *whole* actor patrol surface, accounting
for the chassis radius, was 1.030 m in the original error region. The
south-lane entry and west handoff were physically reached within 0.006 m
and 0.033 m respectively. The enlarged audit includes both locations.

The before/after plot and the aggregate pass report are at
`runs/obstacle2-blue1-20261022/OB2-BLUE1-REPLAY-2/before-after.png` and
`validation-summary.json` in the same directory. These measurements prove
the replayed route and unchanged scoring contract; the replay does not
benchmark the production scheduler's unconstrained choice of inventory.
