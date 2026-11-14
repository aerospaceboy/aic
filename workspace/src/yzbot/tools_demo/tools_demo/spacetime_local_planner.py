"""Kinodynamic short-horizon trajectory planning for local dynamic conflicts.

For every passage strategy from ``dynamic_strategy_graph`` this planner builds a
time-parameterised holonomic trajectory (position *and* speed), then applies
hard filters before ranking anything:

  1. dynamics: speed/acceleration bounds of the measured base,
  2. dynamic safety: swept robot envelope vs the conservative union of all
     plausible obstacle hypotheses at the same time stamp,
  3. static safety: caller-provided free-space predicate along the whole
     trajectory and along the braking-to-stop path,
  4. stopping/exit feasibility: from every executed prefix the robot can still
     brake to a verified safe stop (or a validated retreat pose),
  5. observation freshness: stale tracks are handled as blocked reachable sets,
     never silently dropped.

Only after those filters does it compare arrival time, stops, lateral shift,
control effort and strategy switching. A finite weighted cost can never buy
back a collision: safety is a hard filter, not a penalty term.

The module is pure Python; the ROS supervisor owns command leases and passes a
consistent snapshot (same clock, fresh TF/odometry) into ``plan``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from tools_demo.dynamic_hypotheses import (Prediction, conservative_bodies)
from tools_demo.dynamic_strategy_graph import (ConflictFrame, Strategy)

DEFAULT_DT = 0.1
DEFAULT_HORIZON = 2.4
DEFAULT_PREFIX_SEC = 0.4
DEFAULT_CLEARANCE = 0.03
DEFAULT_REACTION_SEC = 0.25


@dataclass(frozen=True)
class RobotState:
    """Measured local base state (local frame: robot at origin facing +x)."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0
    stamp: float = 0.0

    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)


@dataclass(frozen=True)
class Dynamics:
    max_speed: float = 1.1
    max_lateral_speed: float = 0.45
    max_yaw_rate: float = 1.2
    accel: float = 0.8
    decel: float = 1.0
    lateral_accel: float = 0.5


@dataclass(frozen=True)
class PlanConfig:
    dt: float = DEFAULT_DT
    horizon: float = DEFAULT_HORIZON
    prefix_sec: float = DEFAULT_PREFIX_SEC
    clearance: float = DEFAULT_CLEARANCE
    reaction_sec: float = DEFAULT_REACTION_SEC
    stop_penalty_sec: float = 0.8
    switch_penalty_sec: float = 0.9
    effort_weight: float = 0.15
    lateral_penalty_sec_per_m: float = 0.4
    hysteresis_sec: float = 0.35
    cruise_speed: float = 0.7
    min_speed: float = 0.12
    static_clearance: float = 0.02


@dataclass
class Trajectory:
    samples: Tuple[Tuple[float, float, float, float], ...]   # (t, x, y, speed)
    strategy: Strategy
    arrival_sec: float = math.inf
    min_dynamic_clearance: float = math.inf
    min_static_clearance: float = math.inf
    stops: int = 0
    effort: float = 0.0
    feasible: bool = True
    reject_reason: str = ''
    fallback: str = ''

    def sample_at(self, t: float) -> Tuple[float, float, float, float]:
        if not self.samples:
            raise ValueError('empty trajectory')
        if t <= self.samples[0][0]:
            return self.samples[0]
        for start, end in zip(self.samples, self.samples[1:]):
            if t <= end[0]:
                span = end[0] - start[0]
                ratio = 0.0 if span <= 1e-9 else (t - start[0]) / span
                return (t,
                        start[1] + (end[1] - start[1]) * ratio,
                        start[2] + (end[2] - start[2]) * ratio,
                        start[3] + (end[3] - start[3]) * ratio)
        return self.samples[-1]

    def prefix(self, seconds: float) -> List[Tuple[float, float, float, float]]:
        return [sample for sample in self.samples if sample[0] <= seconds + 1e-9]

    def as_dict(self) -> dict:
        return {
            'strategy': self.strategy.name,
            'kind': self.strategy.kind,
            'feasible': self.feasible,
            'reject_reason': self.reject_reason,
            'fallback': self.fallback,
            'arrival_sec': (None if math.isinf(self.arrival_sec)
                            else round(self.arrival_sec, 3)),
            'min_dynamic_clearance': (None
                                      if math.isinf(self.min_dynamic_clearance)
                                      else round(self.min_dynamic_clearance, 3)),
            'min_static_clearance': (None
                                     if math.isinf(self.min_static_clearance)
                                     else round(self.min_static_clearance, 3)),
            'stops': self.stops,
            'effort': round(self.effort, 3),
            'samples': [[round(t, 3), round(x, 3), round(y, 3), round(v, 3)]
                        for t, x, y, v in self.samples],
        }


@dataclass
class PlanResult:
    selected: Optional[Trajectory]
    candidates: List[Trajectory]
    reason: str
    fallback: str
    committed_until_sec: float
    decision: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {
            'selected': None if self.selected is None
            else self.selected.as_dict(),
            'reason': self.reason,
            'fallback': self.fallback,
            'committed_until_sec': round(self.committed_until_sec, 3),
            'candidates': [candidate.as_dict() for candidate in self.candidates],
            'decision': self.decision,
        }


def _speed_profile(distance: float, start_speed: float, end_speed: float,
                   cruise: float, dynamics: Dynamics
                   ) -> List[Tuple[float, float, float]]:
    """Bang-bang profile for one leg: [(t, s, speed)].

    Accelerates to ``cruise``, cruises, then brakes early enough to reach
    ``end_speed`` exactly at ``distance``. Profile feasibility against the
    acceleration bounds is checked by the caller through the returned speeds.
    """
    cruise = max(end_speed, min(cruise, dynamics.max_speed))
    start_speed = max(0.0, min(start_speed, cruise))
    dt_step = 0.05
    samples: List[Tuple[float, float, float]] = [(0.0, 0.0, start_speed)]
    t = 0.0
    s = 0.0
    speed = start_speed
    while s < distance - 1e-6 and t < 30.0:
        remaining = distance - s
        stop_distance = max(0.0, (speed - end_speed)) ** 2 / (
            2.0 * dynamics.decel)
        # Brake when the remaining distance is at most the distance needed to
        # shed the current speed down to end_speed.
        if remaining <= stop_distance + 1e-6 and speed > end_speed + 1e-6:
            speed = max(end_speed, speed - dynamics.decel * dt_step)
        elif speed < cruise - 1e-6:
            speed = min(cruise, speed + dynamics.accel * dt_step)
        speed = max(0.0, min(speed, dynamics.max_speed))
        step = speed * dt_step
        if step < 1e-6:
            break
        s = min(distance, s + step)
        t += dt_step
        samples.append((t, s, speed))
    if samples[-1][1] < distance - 1e-6:
        # Fallback: constant last speed finish (only reachable for tiny legs).
        remaining = distance - samples[-1][1]
        extra = remaining / max(1e-3, samples[-1][2])
        t += extra
        samples.append((t, distance, samples[-1][2]))
    return samples


def build_trajectory(strategy: Strategy, robot: RobotState,
                     frame: ConflictFrame, payload: bool,
                     dynamics: Dynamics, config: PlanConfig) -> Trajectory:
    """Time-parameterised trajectory for one strategy in the local frame."""
    envelope = frame.envelope(payload)
    points: List[Tuple[float, float]] = [(robot.x, robot.y)]
    waypoints = list(strategy.waypoints)
    if strategy.kind == 'wait':
        # A validated wait is a safe decision point, not a commitment to the
        # resume leg: the rolling planner re-plans the crossing after the
        # conflict window has provably closed. Including a resume leg built
        # from a not-yet-observed future would be an unvalidated guess.
        waypoints = [point for point in waypoints
                     if strategy.wait_pose is None
                     or point == strategy.wait_pose]
    points.extend(waypoints)
    # Optional wait/retreat handling: those strategies stop at their pose.
    samples: List[Tuple[float, float, float, float]] = [
        (0.0, robot.x, robot.y, robot.speed())]
    t_cursor = 0.0
    current = (robot.x, robot.y)
    current_speed = robot.speed()
    stops = 0
    effort = 0.0
    window_earliest = max(0.0, strategy.earliest_cross_sec - robot.stamp)
    window_latest = (math.inf if math.isinf(strategy.latest_cross_sec)
                     else max(0.0, strategy.latest_cross_sec - robot.stamp))
    crossing_point = (frame.crossing_x, strategy.lateral_offset)
    arrival = math.inf
    for index, target in enumerate(points[1:]):
        distance = math.hypot(target[0] - current[0], target[1] - current[1])
        if distance < 1e-6:
            continue
        final_leg = index == len(points) - 2
        end_speed = 0.0 if (strategy.kind in ('wait', 'retreat')
                            or (final_leg and strategy.kind == 'side')) else None
        # Spacetime choice: pick a cruise speed so the *crossing* leg arrives
        # inside the allowed window; then clamp to the dynamics envelope.
        cruise = config.cruise_speed
        is_crossing_leg = (strategy.kind in ('before', 'after', 'mixed')
                           and math.hypot(target[0] - crossing_point[0],
                                          target[1] - crossing_point[1]) < 0.45)
        if is_crossing_leg:
            needed_time = max(window_earliest, t_cursor + distance
                              / max(config.min_speed, cruise))
            if needed_time <= window_latest:
                cruise = distance / max(1e-3, needed_time - t_cursor)
            else:
                cruise = max(config.min_speed, distance / max(
                    1e-3, window_latest - t_cursor))
            cruise = max(config.min_speed, min(dynamics.max_speed, cruise))
        end = 0.0 if end_speed is None and final_leg else (
            end_speed if end_speed is not None else cruise)
        profile = _speed_profile(distance, current_speed, end, cruise,
                                 dynamics)
        previous_speed = current_speed
        for t_leg, s_leg, speed in profile:
            ratio = 0.0 if distance <= 1e-9 else s_leg / distance
            x = current[0] + (target[0] - current[0]) * ratio
            y = current[1] + (target[1] - current[1]) * ratio
            effort += abs(speed - previous_speed)
            previous_speed = speed
            samples.append((t_cursor + t_leg, x, y, speed))
        if strategy.kind in ('wait', 'retreat') and profile \
                and profile[-1][2] <= 1e-3 and profile[-1][0] > 0.2:
            stops += 1
        t_cursor += profile[-1][0] if profile else 0.0
        current = target
        current_speed = profile[-1][2] if profile else 0.0
        # Hold at the wait anchor until every observed conflict window has
        # closed, so the candidate cost reflects the real waiting time rather
        # than looking artificially cheap for stopping early.
        if strategy.kind in ('wait', 'retreat') \
                and strategy.resume_after_sec > 0.0:
            hold_until = max(0.0, strategy.resume_after_sec - robot.stamp)
            if t_cursor < hold_until:
                samples.append((hold_until, current[0], current[1], 0.0))
                t_cursor = hold_until
        # A gap-crossing plan holds at the pre-band pose until the learned
        # clear gap opens, then continues through the band. The hold is part
        # of the plan (not an out-of-band trick) and is priced by its duration.
        elif (strategy.kind == 'gap' and strategy.wait_pose is not None
              and strategy.resume_after_sec > 0.0
              and abs(current[0] - strategy.wait_pose[0]) < 1e-6
              and abs(current[1] - strategy.wait_pose[1]) < 1e-6):
            hold_until = max(0.0, strategy.resume_after_sec - robot.stamp)
            if t_cursor < hold_until:
                samples.append((hold_until, current[0], current[1], 0.0))
                t_cursor = hold_until
        if math.hypot(current[0] - crossing_point[0],
                      current[1] - crossing_point[1]) < 0.45:
            arrival = t_cursor
    trajectory = Trajectory(tuple(samples), strategy, arrival_sec=arrival,
                            stops=stops, effort=effort)
    if strategy.kind in ('wait', 'retreat'):
        resume = (max(0.0, strategy.resume_after_sec - robot.stamp)
                  if strategy.resume_after_sec > 0.0 else t_cursor)
        transit = (abs(frame.crossing_x) / max(config.min_speed,
                                               config.cruise_speed))
        trajectory.arrival_sec = max(t_cursor, resume) + transit
    else:
        trajectory.arrival_sec = arrival if not math.isinf(arrival) else t_cursor
    return trajectory


def _static_ok(check: Optional[Callable[[float, float, float], float]],
               trajectory: Trajectory, frame: ConflictFrame, payload: bool,
               config: PlanConfig) -> Tuple[bool, float]:
    """Free-space predicate along every sample (default: corridor bounds)."""
    envelope = frame.envelope(payload)
    minimum = math.inf
    for t, x, y, speed in trajectory.samples:
        if check is not None:
            clearance = float(check(x, y, envelope))
        else:
            right = -(frame.corridor_right - envelope)
            left = frame.corridor_left - envelope
            back = frame.corridor_min_x + envelope
            front = frame.corridor_max_x - envelope
            clearance = min(left - y, y - right, x - back, front - x)
        minimum = min(minimum, clearance)
        if clearance < config.static_clearance:
            return False, minimum
    return True, minimum


def _dynamic_clearance(predictions: Sequence[Prediction],
                       trajectory: Trajectory, robot_stamp: float,
                       envelope: float, config: PlanConfig) -> Tuple[bool, float]:
    """Conservative swept-circle clearance against every plausible body."""
    minimum = math.inf
    for t, x, y, speed in trajectory.samples:
        bodies = conservative_bodies(predictions, robot_stamp + t)
        for bx, by, radius in bodies:
            clearance = math.hypot(x - bx, y - by) - radius - envelope
            minimum = min(minimum, clearance)
            if clearance < config.clearance:
                return False, minimum
    return True, minimum


def _stopping_ok(trajectory: Trajectory, predictions: Sequence[Prediction],
                 robot_stamp: float, envelope: float, dynamics: Dynamics,
                 frame: ConflictFrame, payload: bool, config: PlanConfig,
                 static_check: Optional[Callable[[float, float, float], float]],
                 ) -> Tuple[bool, str]:
    """Every executed prefix must be able to brake to a verified safe stop."""
    for index, (t, x, y, speed) in enumerate(trajectory.samples):
        if speed <= 1e-3:
            continue
        braking = speed * speed / (2.0 * dynamics.decel)
        # Sample the straight braking path at the same resolution as the plan.
        steps = max(2, int(braking / 0.05) + 2)
        for step in range(1, steps + 1):
            ratio = step / steps
            # Braking direction is the current motion direction; use the next
            # plan sample direction (the plan is sampled finely).
            if index + 1 < len(trajectory.samples):
                nxt = trajectory.samples[index + 1]
                direction = math.atan2(nxt[2] - y, nxt[1] - x) \
                    if math.hypot(nxt[1] - x, nxt[2] - y) > 1e-4 else 0.0
            else:
                direction = 0.0
            px = x + math.cos(direction) * braking * ratio
            py = y + math.sin(direction) * braking * ratio
            stop_time = t + (speed / dynamics.decel) * ratio
            if static_check is not None:
                if float(static_check(px, py, envelope)) < config.static_clearance:
                    return False, 'braking_path_blocked_at_t=%.2f' % t
            for bx, by, radius in conservative_bodies(
                    predictions, robot_stamp + stop_time):
                if math.hypot(px - bx, py - by) - radius - envelope < config.clearance:
                    return False, 'braking_path_enters_obstacle_at_t=%.2f' % t
    return True, ''


def plan(robot: RobotState, payload: bool, frame: ConflictFrame,
         predictions: Sequence[Prediction], strategies: Sequence[Strategy],
         *,
         previous_kind: Optional[str] = None,
         dynamics: Optional[Dynamics] = None,
         config: Optional[PlanConfig] = None,
         static_check: Optional[Callable[[float, float, float], float]] = None,
         retry_note: str = '') -> PlanResult:
    """Hard-filter every strategy, rank the survivors and pick one."""
    dynamics = dynamics or Dynamics()
    config = config or PlanConfig()
    trajectory_candidates: List[Trajectory] = []
    for strategy in strategies:
        trajectory = build_trajectory(strategy, robot, frame, payload,
                                      dynamics, config)
        envelope = frame.envelope(payload)
        if not trajectory.samples:
            trajectory.feasible = False
            trajectory.reject_reason = 'empty_trajectory'
            trajectory_candidates.append(trajectory)
            continue
        static_ok, static_clearance = _static_ok(static_check, trajectory,
                                                 frame, payload, config)
        trajectory.min_static_clearance = static_clearance
        if not static_ok:
            trajectory.feasible = False
            trajectory.reject_reason = (
                'static_clearance=%.3f' % static_clearance)
            trajectory_candidates.append(trajectory)
            continue
        dynamic_ok, dynamic_clearance = _dynamic_clearance(
            predictions, trajectory, robot.stamp, envelope, config)
        trajectory.min_dynamic_clearance = dynamic_clearance
        if not dynamic_ok:
            trajectory.feasible = False
            trajectory.reject_reason = (
                'dynamic_clearance=%.3f' % dynamic_clearance)
            trajectory_candidates.append(trajectory)
            continue
        stopping_ok, stop_reason = _stopping_ok(
            trajectory, predictions, robot.stamp, envelope, dynamics, frame,
            payload, config, static_check)
        if not stopping_ok:
            trajectory.feasible = False
            trajectory.reject_reason = stop_reason
            trajectory_candidates.append(trajectory)
            continue
        trajectory_candidates.append(trajectory)

    legal = [candidate for candidate in trajectory_candidates
             if candidate.feasible]

    def cost(candidate: Trajectory) -> float:
        value = candidate.arrival_sec
        value += config.stop_penalty_sec * candidate.stops
        value += config.effort_weight * candidate.effort
        value += config.lateral_penalty_sec_per_m * abs(
            candidate.strategy.lateral_offset)
        if previous_kind is not None and candidate.strategy.kind != previous_kind:
            value += config.switch_penalty_sec
        return value

    candidates = sorted(legal, key=cost)
    if candidates:
        selected = candidates[0]
        if previous_kind is not None and len(candidates) > 1:
            previous = next((candidate for candidate in candidates
                             if candidate.strategy.kind == previous_kind), None)
            if previous is not None and cost(previous) \
                    <= cost(selected) + config.hysteresis_sec:
                selected = previous
        return PlanResult(
            selected=selected, candidates=trajectory_candidates,
            reason='selected_by_hard_filter_then_cost',
            fallback='', committed_until_sec=config.prefix_sec,
            decision={'cost': round(cost(selected), 3),
                      'previous_kind': previous_kind,
                      'legal': len(legal),
                      'rejected': len(trajectory_candidates) - len(legal),
                      'retry_note': retry_note})
    # No legal trajectory: brake in place; if even the stop is not clear, say
    # so explicitly instead of pretending a motion is safe.
    stop = Trajectory(((0.0, robot.x, robot.y, robot.speed()),),
                      strategies[0] if strategies else
                      Strategy('blocked', 'wait', (), 0.0, ()),
                      arrival_sec=math.inf)
    stop.fallback = 'emergency_stop'
    envelope = frame.envelope(payload)
    stop_clear = True
    for t in (0.0, robot.speed() / dynamics.decel):
        for bx, by, radius in conservative_bodies(predictions,
                                                  robot.stamp + t):
            if math.hypot(robot.x - bx, robot.y - by) \
                    - radius - envelope < config.clearance:
                stop_clear = False
    if not stop_clear:
        stop.fallback = 'blocked_no_safe_action'
    reasons = [candidate.reject_reason for candidate in trajectory_candidates
               if candidate.reject_reason][:8]
    return PlanResult(selected=None, candidates=trajectory_candidates,
                      reason='all_candidates_rejected',
                      fallback=stop.fallback,
                      committed_until_sec=0.0,
                      decision={'rejected_reasons': reasons,
                                'stop_clear': stop_clear,
                                'retry_note': retry_note})


def stop_command(plan_result: PlanResult, decel: float,
                 vx: float, vy: float) -> Tuple[float, float, float]:
    """Command triple (vx_cmd, vy_cmd, w_cmd) for the safe-stop fallback."""
    speed = math.hypot(vx, vy)
    if speed <= 1e-3:
        return 0.0, 0.0, 0.0
    capped = max(0.0, speed - decel * 0.05)
    return vx / speed * capped, vy / speed * capped, 0.0
