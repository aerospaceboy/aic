"""Local supervisor for the observation-driven dynamic-avoidance planner.

The supervisor is the only new component that can take the base's direct
command lease. It is deliberately narrow:

  * it only activates inside a conflict corridor (approaching actors whose
    predicted paths cross the current route),
  * it owns the direct channel only while a plan is being executed; when the
    corridor clears it releases the lease and hands control back to Nav2,
  * it replans on a bounded cadence, executes only a short safety prefix and
    is allowed to fail closed (safe stop / hand back) but never to invent a
    motion when no verified option exists,
  * it records a compact causal decision record for every activation so the
    same snapshot can be replayed offline.

The class is pure Python (no rclpy) and receives a consistent snapshot from
the navigator; the ROS side only supplies measurements and publishes the
returned command triple.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Sequence, Tuple

from tools_demo import dynamic_hypotheses as dh
from tools_demo import dynamic_strategy_graph as dsg
from tools_demo import spacetime_local_planner as sp

DEFAULT_REPLAN_SEC = 0.20
DEFAULT_PREFIX_SEC = 0.30
DEFAULT_RELEASE_SEC = 0.60
DEFAULT_ACTIVATION_AHEAD_M = 7.0
DEFAULT_ACTIVATION_LATERAL_M = 1.6
DEFAULT_STOP_DECEL = 1.0
RobotState = sp.RobotState   # re-exported for the ROS layer


@dataclass
class SupervisorConfig:
    mode: str = 'observe'                 # off | observe | active
    replan_sec: float = DEFAULT_REPLAN_SEC
    prefix_sec: float = DEFAULT_PREFIX_SEC
    release_sec: float = DEFAULT_RELEASE_SEC
    activation_ahead_m: float = DEFAULT_ACTIVATION_AHEAD_M
    activation_lateral_m: float = DEFAULT_ACTIVATION_LATERAL_M
    horizon_sec: float = 4.0
    stop_decel: float = DEFAULT_STOP_DECEL
    empty_envelope: float = 0.28
    carrying_envelope: float = 0.34
    corridor_left: float = 0.80
    corridor_right: float = 0.80
    limit_speed: float = 1.0

    def valid(self) -> bool:
        return (self.mode in ('off', 'disabled', 'observe', 'active')
                and 0.05 <= self.replan_sec <= 1.0
                and 0.05 <= self.prefix_sec <= 1.0
                and 0.0 <= self.release_sec <= 2.0)


@dataclass
class SupervisorOutput:
    """One supervisor step result; the ROS layer maps it to the wire."""

    state: str
    command: Optional[Tuple[float, float, float]]   # (vx, vy, w)
    acquire_lease: bool
    release_lease: bool
    decision: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {
            'state': self.state,
            'command': None if self.command is None else
            [round(value, 3) for value in self.command],
            'acquire_lease': self.acquire_lease,
            'release_lease': self.release_lease,
            'decision': self.decision,
        }


def world_to_local(x: float, y: float, origin_x: float, origin_y: float,
                   yaw: float) -> Tuple[float, float]:
    dx = x - origin_x
    dy = y - origin_y
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return dx * cos_yaw + dy * sin_yaw, -dx * sin_yaw + dy * cos_yaw


def local_to_world(vx: float, vy: float, yaw: float) -> Tuple[float, float]:
    """Rotate a local-frame velocity back to the world frame."""
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return vx * cos_yaw - vy * sin_yaw, vx * sin_yaw + vy * cos_yaw


def actor_snapshots(obstacles: Dict[str, dict],
                    local_frame: Tuple[float, float, float],
                    now: float,
                    confidence_age: float = 0.45) -> List[dh.TrackSnapshot]:
    """Convert navigator obstacle records into local-frame track snapshots.

    ``obstacles`` maps a topic to a record with ``x``, ``y``, ``stamp_sec``,
    ``vx``, ``vy`` (world frame), ``history`` (list of (stamp, x, y)) and a
    ``radius``. Missing/old records become confidence=False so the prediction
    layer keeps their reachable space blocked instead of dropping them.
    """
    origin_x, origin_y, yaw = local_frame
    tracks = []
    for index, (topic, record) in enumerate(sorted(obstacles.items()), start=1):
        world_x = record.get('x')
        world_y = record.get('y')
        if world_x is None or world_y is None:
            continue
        if not (math.isfinite(world_x) and math.isfinite(world_y)):
            continue
        stamp = float(record.get('stamp_sec', record.get('stamp', now)))
        age = max(0.0, now - stamp)
        local_x, local_y = world_to_local(world_x, world_y, origin_x,
                                          origin_y, yaw)
        vx_world = float(record.get('vx', 0.0))
        vy_world = float(record.get('vy', 0.0))
        vx_local, vy_local = world_to_local(vx_world, vy_world, 0.0, 0.0, yaw)
        history = []
        for sample in record.get('history', ()):
            if len(sample) < 3:
                continue
            sample_stamp, sample_x, sample_y = sample[:3]
            local_sample_x, local_sample_y = world_to_local(
                sample_x, sample_y, origin_x, origin_y, yaw)
            history.append((float(sample_stamp), local_sample_x,
                            local_sample_y))
        tracks.append(dh.TrackSnapshot(
            track_id=int(record.get('track_id', index)),
            stamp=stamp,
            x=local_x,
            y=local_y,
            radius=float(record.get('radius', 0.18)),
            vx=vx_local,
            vy=vy_local,
            uncertainty=0.05 if age < 0.25 else 0.10,
            confident=(age <= confidence_age),
            was_moving=math.hypot(vx_world, vy_world) > 0.04,
            history=tuple(history[-7:]),
        ))
    return tracks


def crossing_point(tracks: Sequence[dh.TrackSnapshot],
                   predictions: Sequence[dh.Prediction],
                   config: SupervisorConfig
                   ) -> Optional[Tuple[float, float]]:
    """Return the local crossing point (x, y) of the nearest plausible path."""
    best = None
    for prediction in predictions:
        for hypothesis in prediction.near_field(until=None):
            positions = [sample for sample in hypothesis.positions
                         if sample[0] <= prediction.stamp + config.horizon_sec]
            if len(positions) < 3:
                continue
            previous = positions[0]
            for sample in positions[1:]:
                if abs(sample[2]) <= 0.05:
                    forward = sample[1]
                    if -0.2 <= forward <= config.activation_ahead_m:
                        if best is None or abs(forward) < abs(best[0]):
                            best = (forward, 0.0)
                    break
                if previous[2] * sample[2] < 0.0:
                    # Interpolate the sign change to get the crossing abscissa.
                    ratio = previous[2] / (previous[2] - sample[2])
                    forward = previous[1] + (sample[1] - previous[1]) * ratio
                    if -0.2 <= forward <= config.activation_ahead_m:
                        if best is None or abs(forward) < abs(best[0]):
                            best = (forward, 0.0)
                    break
                previous = sample
    return best


def conflict_active(tracks: Sequence[dh.TrackSnapshot],
                    config: SupervisorConfig) -> bool:
    """Immediate-activation test: a mover approaching our forward corridor."""
    for track in tracks:
        if track.x < -0.5 or track.x > config.activation_ahead_m:
            continue
        if abs(track.y) > config.activation_lateral_m:
            continue
        return True
    return False


class DynamicLocalSupervisor:
    """Rolling local planner/executor with an explicit lease handshake."""

    def __init__(self, config: SupervisorConfig):
        if not config.valid():
            raise ValueError('invalid supervisor configuration')
        self.config = config
        self.state = 'idle'            # idle | observing | active | released
        self.last_plan_sec: Optional[float] = None
        self.last_plan = None
        self.previous_kind: Optional[str] = None
        self.conflict_lost_sec: Optional[float] = None
        self.activation_count = 0
        self.executed_steps = 0
        self.fallback_count = 0
        self.decisions: List[dict] = []

    def step(self, now: float, robot: sp.RobotState, payload: bool,
             obstacles: Dict[str, dict], goal_local: Tuple[float, float],
             static_check=None, takeover_allowed: bool = True
             ) -> SupervisorOutput:
        """Advance one control cycle and return the command/lease decision."""
        config = self.config
        if config.mode in ('off', 'disabled'):
            return self._release('off')
        already_owner = self.state == 'active'
        frame_origin = (robot.x, robot.y, robot.yaw)
        # ``robot`` arrives with a world-frame pose and world-frame velocity;
        # the planner works in the robot-local frame (+x forward, +y left).
        local_vx, local_vy = world_to_local(robot.vx, robot.vy, 0.0, 0.0,
                                            robot.yaw)
        local_robot = sp.RobotState(stamp=robot.stamp, vx=local_vx,
                                    vy=local_vy, w=robot.w, yaw=0.0)
        tracks = actor_snapshots(obstacles, frame_origin, now)
        # A real observation dropout very close to the base is never certified
        # clear. Release the direct channel at once (the downstream collision
        # monitor and Nav2 stay in charge) rather than holding a stop command
        # that fights the navigation stack; the event is recorded.
        stale_nearby = any(
            now - track.stamp > 1.0
            and math.hypot(track.x, track.y) <= 2.5
            for track in tracks)
        if stale_nearby:
            was_owner = self.state == 'active'
            self.state = 'released'
            self.last_plan = None
            decision = {
                'stamp': round(now, 3),
                'state': 'stale_occluded',
                'released_owner': was_owner,
                'tracks': [{'track_id': track.track_id,
                            'x': round(track.x, 2),
                            'y': round(track.y, 2),
                            'age': round(now - track.stamp, 2)}
                           for track in tracks],
            }
            self.decisions.append(decision)
            return SupervisorOutput('fallback', None, False,
                                    was_owner, decision)
        # The conflict is only relevant when it lies *on the way to the
        # current navigation goal*. A moving obstacle behind the goal (typical
        # during the final pickup/delivery dock) must not drag the base away
        # from its target, and a docking robot is handled by the existing
        # precision-zone logic, not by this transit planner.
        goal_forward = math.hypot(goal_local[0], goal_local[1])
        active = conflict_active(tracks, config) and goal_forward >= 1.5
        if not active:
            if self.state == 'released':
                self.state = 'observing'
            if self.state == 'active':
                self.conflict_lost_sec = self.conflict_lost_sec or now
                if now - self.conflict_lost_sec >= config.release_sec:
                    self.state = 'released'
                    self.conflict_lost_sec = None
                    self.last_plan = None
                    return self._release('corridor_clear')
                if self.last_plan is not None:
                    return self._execute_prefix(now, self.last_plan)
                return self._release('corridor_clear')
            self.state = 'observing'
            self.conflict_lost_sec = None
            return self._release('no_conflict')
        self.conflict_lost_sec = None
        if self.state != 'active':
            self.state = 'active'
            self.activation_count += 1
            self.last_plan_sec = None
        if (self.last_plan_sec is not None
                and now - self.last_plan_sec < config.replan_sec
                and self.last_plan is not None):
            return self._execute_prefix(now, self.last_plan)
        predictions = dh.predict(tracks, now=now, horizon=config.horizon_sec)
        crossing = crossing_point(tracks, predictions, config)
        if crossing is None:
            return self._release('no_predicted_crossing')
        if crossing[0] > goal_forward + 0.40:
            return self._release('conflict_beyond_goal')
        if robot.speed() < 0.10 and not already_owner:
            return self._release('base_not_transiting')
        crossing_x = max(0.30, min(crossing[0], config.activation_ahead_m))
        frame = dsg.ConflictFrame(
            crossing_x=crossing_x,
            obstacle_radius=0.18,
            corridor_left=config.corridor_left,
            corridor_right=config.corridor_right,
            corridor_min_x=-1.0,
            corridor_max_x=max(4.0, crossing_x + 2.0),
            robot_speed=max(0.15, robot.speed()),
            limit_speed=config.limit_speed,
            empty_envelope=config.empty_envelope,
            carrying_envelope=config.carrying_envelope,
        )
        legal, rejected, windows = dsg.strategy_candidates(
            predictions, frame, payload=payload, now=now,
            retreat_available=None)
        result = sp.plan(local_robot, payload, frame, predictions, legal,
                         previous_kind=self.previous_kind,
                         config=sp.PlanConfig(prefix_sec=config.prefix_sec),
                         static_check=static_check)
        decision = {
            'stamp': round(now, 3),
            'activation': self.activation_count,
            'payload': payload,
            'crossing_x': round(crossing_x, 2),
            'hypotheses': [
                {'track_id': prediction.hypotheses[0].track_id,
                 'modes': {hypothesis.mode: round(hypothesis.weight, 2)
                           for hypothesis in prediction.hypotheses},
                 'stale': prediction.stale}
                for prediction in predictions],
            'windows': {str(key): [round(value[0], 2), round(value[1], 2)]
                        for key, value in windows.items()},
            'legal': [strategy.name for strategy in legal],
            'rejected': {item.name: item.reason for item in rejected},
            'selected': (None if result.selected is None
                         else result.selected.strategy.name),
            'kind': (None if result.selected is None
                     else result.selected.strategy.kind),
            'reason': result.reason,
            'fallback': result.fallback,
            'plan_candidates': [
                {'strategy': candidate.strategy.name,
                 'feasible': candidate.feasible,
                 'reject_reason': candidate.reject_reason}
                for candidate in result.candidates],
            'plan_decision': result.decision,
            'goal_local': [round(goal_local[0], 2),
                           round(goal_local[1], 2)],
        }
        if config.mode in ('observe', 'disabled') or (not takeover_allowed
                                        and not already_owner):
            # Evidence-only cycle: the normal navigation/mover logic keeps
            # control unless the existing coordinator was about to hold for
            # this mover. This prevents the planner from taking the base away
            # from an ordinary transit/docking just because a mover is visible.
            self.state = 'observing'
            self.last_plan_sec = now
            self.last_plan = None
            self.decisions.append(decision)
            return SupervisorOutput('observing', None, False, False, decision)
        if result.selected is None:
            self.fallback_count += 1
            self.decisions.append(decision)
            was_owner = self.state == 'active'
            self.state = 'released'
            self.last_plan = None
            command = (self._fallback_command(result, robot)
                       if was_owner else None)
            return SupervisorOutput('fallback', command, False, was_owner,
                                    decision)
        kind = result.selected.strategy.kind
        if kind in ('wait', 'retreat'):
            # A hold/withdraw plan is exactly what the shipped mover-hold
            # machinery already does (and has been validated on the real
            # route). Delegate that decision back to it instead of duplicating
            # a hand-built hold from a proxy corridor model: only an actual
            # passage plan justifies taking the direct channel.
            self.state = 'observing'
            was_owner = already_owner
            self.last_plan = None
            self.last_plan_sec = now
            self.decisions.append(decision)
            return SupervisorOutput('observing', None, False, was_owner,
                                    decision)
        self.previous_kind = kind
        self.last_plan = result
        self.last_plan_sec = now
        self.decisions.append(decision)
        return self._execute_prefix(now, result)

    def _execute_prefix(self, now: float, result) -> SupervisorOutput:
        selected = result.selected
        if selected is None:
            return self._release('no_plan')
        sample = selected.sample_at(min(self.config.prefix_sec,
                                        max(0.05, selected.samples[-1][0])))
        # Local-frame command: vx along robot heading, vy to the left.
        speed = sample[3]
        if speed <= 1e-3:
            command = (0.0, 0.0, 0.0)
        else:
            heading = math.atan2(sample[2] - 0.0, sample[1] - 0.0) \
                if math.hypot(sample[1], sample[2]) > 1e-3 else 0.0
            command = (speed * math.cos(heading), speed * math.sin(heading),
                       0.0)
        self.executed_steps += 1
        decision = {'stamp': round(now, 3),
                    'state': 'executing',
                    'strategy': selected.strategy.name,
                    'prefix_sec': self.config.prefix_sec,
                    'sample': [round(value, 3) for value in sample]}
        return SupervisorOutput('active', command, True, False, decision)

    def _fallback_command(self, result, robot: sp.RobotState
                          ) -> Tuple[float, float, float]:
        speed = robot.speed()
        if speed <= 1e-3:
            return 0.0, 0.0, 0.0
        capped = max(0.0, speed - self.config.stop_decel * 0.1)
        ratio = capped / speed
        local_vx, local_vy = world_to_local(
            robot.vx * ratio, robot.vy * ratio, 0.0, 0.0, robot.yaw)
        return local_vx, local_vy, 0.0

    def _release(self, reason: str) -> SupervisorOutput:
        return SupervisorOutput('idle', None, False, self.state == 'released',
                                {'release_reason': reason})
