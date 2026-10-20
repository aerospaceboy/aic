# Safe 300-Second Sprint Design

## Objective

Complete all five requested cube deliveries in less than 300 seconds with
`completed=5`, `skipped=0`, no real BodyStop event, and recorded obstacle
clearance greater than 0.56 m. Safety wins over speed whenever these goals
conflict.

## Evidence at the Start of This Sprint

- A manual default-DDS launch completed a full 2-red/3-blue mission in 397.7 s
  with `completed=4`, `skipped=1`.
- A fresh default-DDS `NavigateToPose` ActionClient found the server in 0.251 s.
  The same probe with `FASTRTPS_DEFAULT_PROFILES_FILE=fastdds_udp.xml` timed out
  after 15 s. The automation-only DDS override caused the stationary robot.
- `red_cube_3` was grasped and transported, but placement rejected its measured
  carry pose `(0.04199, 0.00897, -0.02221)` before arm motion. The strict carry
  Z lower bound is -0.020 m. Earlier in the same carry, the relative pose had
  already drifted by 29.3 mm after lift, so widening the window alone would
  conceal physical instability and is not an acceptable fix.
- The observed sharp-turn slowdown is consistent with active NearSlow safety
  intervention. Collision geometry, BodyStop, pickup docking speed, and world
  layout remain immutable.

## Selected Approach

Use a reliability-first, measurement-driven sequence. First make automation
match the proven manual launch and verify readiness with a real ActionClient.
Second reproduce and trace the intermittent carry/placement failure while
preserving all safety windows. Third collect phase timing, odometry speed, and
obstacle clearance in a successful 5/0 run. Only then change one speed-related
variable per round.

This is preferred over immediately reducing inflation or eliminating the lift.
Those alternatives may save time, but they directly worsen the user's observed
turning-clearance problem or repeat the documented fast-lift cube-slip failure.

## Components

### Reproducible startup

The Windows-to-VM launch scripts must explicitly remove inherited custom Fast
DDS profiles. Readiness must be tested by a new rclpy ActionClient, not the ROS
2 CLI daemon graph. A launch is unsuccessful unless the ActionClient, mission
API, camera stream, collision monitor, and single process set are all healthy.

### Manipulation reliability

Keep `GRASP_*`, `CARRY_*`, and `MAX_ATTACHED_DRIFT` safety constants unchanged
during diagnosis. Add a regression fixture for the observed relative-pose
sequence and enough event logging to distinguish fixed-joint failure, pose
measurement drift, and genuine cube loss. A fix must restore placement without
accepting the existing dropped-cube fixtures.

### Performance measurement

Record each mission state transition, navigation segment duration, odometry
linear/angular speed, collision-monitor state, and moving-obstacle distances.
Separate final docking from long-path navigation so slow docking is not
mistaken for a global velocity-limit problem.

### Safe optimization order

1. Remove avoidable waiting or failed retries.
2. Shorten manipulation motion only after verifying cube clearance and carry
   stability. Retain a positive lift; test a lower safe transit height before
   reducing lift duration.
3. Improve route/order choices backed by measured segment savings.
4. Tune local navigation only if the recorded slowdown source is known. Never
   reduce robot footprint, BodyStop, or the required 0.56 m obstacle margin.

## Validation Gates

Every behavior or parameter change gets a clean relaunch and one complete task.
Reject the change if the terminal state is not 5/0, any real BodyStop occurs,
minimum obstacle clearance is at most 0.56 m, a cube leaves the verified carry
window, or sharp turns show a new safety regression. Once under 300 s, repeat
both task distributions two or three times before delivery.

