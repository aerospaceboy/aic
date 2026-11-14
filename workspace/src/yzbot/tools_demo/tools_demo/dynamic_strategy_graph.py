"""Finite local passage-strategy graph for non-cooperative dynamic obstacles.

The planner does not only slow down for one obstacle. Around a conflict it
enumerates a small, explainable candidate set in the robot's crossing frame:

  * pass an obstacle's swept band *before* it arrives,
  * let it pass first and cross *after* it,
  * pass on the *left/right* side when the payload envelope allows the lateral
    shift and the shifted corridor is free,
  * wait at a validated *safe anchor* outside the predicted reachable set,
  * withdraw to a bounded, previously traversed *retreat* position and re-enter.

Candidates are combinations of per-obstacle order decisions plus one lateral
offset. Contradictory or infeasible combinations are pruned with a recorded
reason before any trajectory is generated; this graph is a *finite heuristic
candidate set*, not a complete homotopy classification and not claimed
globally optimal.

All geometry is in a 2D local frame whose +x axis points from the robot
towards the conflict and +y points to the robot's left.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from tools_demo.dynamic_hypotheses import (Prediction, conservative_bodies)

# Prototype parameters (engineering defaults, re-measure before claiming proof)
DEFAULT_BODY_MARGIN = 0.10
DEFAULT_MAX_LATERAL = 0.80
WAIT_DWELL_SEC = 1.5
DEFAULT_ORDER_OPTIONS = ('before', 'after', 'wait')
DEFAULT_MIN_CLEAR_SEC = 1.2     # robot needs this long inside the clear gap
# Near-term (<= NEAR_HARD_SEC) every plausible mode is a hard obstacle; beyond
# that only the per-track nominal mode is used for ordering/preheating. The
# executed prefix is always re-validated, so a far planned crossing becomes
# near-field checked before the robot actually enters it.
NEAR_HARD_SEC = 1.2
FAR_NOMINAL_ONLY = True
FAR_GROWTH = 0.03          # far-field uncertainty growth (m per second)
BAND_MARGIN = 0.03         # same clearance the trajectory filter requires


@dataclass(frozen=True)
class ConflictFrame:
    """Local crossing geometry and robot state for one dynamic conflict."""

    crossing_x: float = 0.0           # conflict centre along local x (m)
    obstacle_radius: float = 0.18
    corridor_left: float = 0.9        # free lateral room to the left (m)
    corridor_right: float = 0.9       # free lateral room to the right (m)
    corridor_min_x: float = -1.0      # free backward extent (m)
    corridor_max_x: float = 4.0       # free forward extent (m)
    robot_speed: float = 0.4          # measured forward speed (m/s)
    robot_lateral_speed: float = 0.0
    lateral_speed: float = 0.20       # achievable side-shift speed (m/s)
    limit_speed: float = 1.0
    accel_limit: float = 0.8
    decel_limit: float = 1.0
    lateral_accel_limit: float = 0.5
    empty_envelope: float = 0.28      # robot body footprint radius (m)
    carrying_envelope: float = 0.34   # footprint + payload half-width (m)

    def envelope(self, payload: bool) -> float:
        return self.carrying_envelope if payload else self.empty_envelope

    def crossing_duration(self, lateral_offset: float = 0.0,
                          payload: bool = False) -> float:
        """Conservative time to clear the conflict band from local x=0."""
        speed = max(0.15, min(self.limit_speed, self.robot_speed))
        band = 2.0 * (self.obstacle_radius + self.envelope(payload)
                      + DEFAULT_BODY_MARGIN)
        distance = max(0.2, abs(self.crossing_x)) + band
        lateral = abs(lateral_offset) / max(0.15, self.lateral_speed)
        return distance / speed + lateral


@dataclass(frozen=True)
class PassageDecision:
    """Per-obstacle relation the robot intends to establish."""

    track_id: int
    relation: str                 # 'before' | 'after' | 'wait'
    reason: str = ''


@dataclass
class Strategy:
    """One candidate passage plan before trajectory generation."""

    name: str
    kind: str                     # 'before' | 'after' | 'side' | 'wait' | 'retreat'
    decisions: Tuple[PassageDecision, ...]
    lateral_offset: float
    waypoints: Tuple[Tuple[float, float], ...]
    wait_pose: Optional[Tuple[float, float]] = None
    retreat_pose: Optional[Tuple[float, float]] = None
    earliest_cross_sec: float = 0.0
    latest_cross_sec: float = math.inf
    resume_after_sec: float = 0.0      # wait strategies only (absolute sim s)
    resume_pose: Optional[Tuple[float, float]] = None
    tag: str = ''

    def as_dict(self) -> dict:
        return {
            'name': self.name,
            'kind': self.kind,
            'lateral_offset': round(self.lateral_offset, 3),
            'waypoints': [[round(x, 3), round(y, 3)] for x, y in self.waypoints],
            'wait_pose': None if self.wait_pose is None else
            [round(self.wait_pose[0], 3), round(self.wait_pose[1], 3)],
            'retreat_pose': None if self.retreat_pose is None else
            [round(self.retreat_pose[0], 3), round(self.retreat_pose[1], 3)],
            'decisions': [{'track_id': d.track_id, 'relation': d.relation}
                          for d in self.decisions],
            'window_sec': [round(self.earliest_cross_sec, 3),
                           None if math.isinf(self.latest_cross_sec)
                           else round(self.latest_cross_sec, 3)],
            'tag': self.tag,
        }


@dataclass
class RejectedStrategy:
    name: str
    reason: str


def _body_at(prediction: Prediction, t: float, min_weight: float = 0.02
             ) -> List[Tuple[float, float, float]]:
    """All non-negligible mode bodies of one track at time t (local frame)."""
    bodies = []
    for hypothesis in prediction.near_field(until=None, min_weight=min_weight):
        if hypothesis.positions[0][0] > t:
            continue
        _, x, y = hypothesis.position_at(t)
        bodies.append((x, y, hypothesis.radius_at(t)))
    return bodies


def _nominal_body(prediction: Prediction, t: float
                  ) -> List[Tuple[float, float, float]]:
    """Highest-weight mode only: far-field ordering, never near-field safety."""
    bodies = []
    for hypothesis in prediction.nominal():
        if hypothesis.positions[0][0] > t:
            continue
        _, x, y = hypothesis.position_at(t)
        bodies.append((x, y, hypothesis.radius_at(t)))
    return bodies


def band_occupied_interval(prediction: Prediction,
                           band_center: Tuple[float, float],
                           band_yaw: float,
                           band_half_length: float,
                           band_half_width: float,
                           t_from: float, t_to: float,
                           sample_dt: float = 0.05,
                           min_weight: float = 0.02
                           ) -> Optional[Tuple[float, float]]:
    """First contiguous interval where any plausible mode intersects a band.

    The band is an oriented rectangle expressed in the local frame. This is
    deliberately the *union over modes*: a low-weight reversal branch still
    blocks the band while its evidence remains non-negligible.
    """
    cos_yaw, sin_yaw = math.cos(band_yaw), math.sin(band_yaw)

    def inside(x: float, y: float, radius: float) -> bool:
        dx, dy = x - band_center[0], y - band_center[1]
        along = dx * cos_yaw + dy * sin_yaw
        lateral = -dx * sin_yaw + dy * cos_yaw
        return (abs(along) <= band_half_length + radius
                and abs(lateral) <= band_half_width + radius)

    interval_start = None
    t = t_from
    while t <= t_to + 1e-9:
        blocked = any(inside(x, y, radius)
                      for x, y, radius in _body_at(prediction, t, min_weight))
        if blocked and interval_start is None:
            interval_start = t
        elif not blocked and interval_start is not None:
            return (interval_start, t)
        t += sample_dt
    if interval_start is not None:
        return (interval_start, t_to)
    return None


def _crossing_windows(predictions: Sequence[Prediction],
                      frame: ConflictFrame, payload: bool,
                      lateral_offset: float,
                      now: float) -> Dict[int, Tuple[float, float]]:
    """Time intervals where each obstacle body occupies the robot's crossing band."""
    band_center = (frame.crossing_x, 0.0)
    half_length = frame.obstacle_radius + frame.envelope(payload) \
        + DEFAULT_BODY_MARGIN
    half_width = frame.obstacle_radius + frame.envelope(payload) \
        + DEFAULT_BODY_MARGIN
    windows = {}
    horizon = max((prediction.stamp + prediction.horizon)
                  for prediction in predictions) if predictions else now
    for prediction in predictions:
        window = band_occupied_interval(
            prediction, band_center, 0.0, half_length, half_width,
            now, min(horizon, now + max(2.0, frame.crossing_duration() + 1.5)))
        if window is not None:
            windows[prediction.hypotheses[0].track_id] = window
    return windows


def band_occupancy_timeline(predictions: Sequence[Prediction],
                            frame: ConflictFrame, payload: bool,
                            now: float, horizon: float,
                            sample_dt: float = 0.05
                            ) -> List[Tuple[float, float]]:
    """Merged time intervals in which any plausible body blocks the band.

    The band is the crossing strip the robot must clear (oriented along the
    robot heading at the conflict point). Taking the union over low-weight
    modes is intentional: a mode is only dropped below MIN_MODE_WEIGHT.
    """
    half_length = (frame.obstacle_radius + frame.envelope(payload)
                   + DEFAULT_BODY_MARGIN)
    half_width = half_length
    blocked = []
    t = now
    end = now + horizon
    while t <= end + 1e-9:
        occupied = False
        for prediction in predictions:
            if prediction.stale and not prediction.hypotheses:
                continue
            for x, y, radius in _body_at(prediction, t):
                if (abs(x - frame.crossing_x) <= half_length + radius
                        and abs(y) <= half_width + radius):
                    occupied = True
                    break
            if occupied:
                break
        blocked.append((t, occupied))
        t += sample_dt
    intervals: List[Tuple[float, float]] = []
    start = None
    for stamp, occupied in blocked:
        if occupied and start is None:
            start = stamp
        elif not occupied and start is not None:
            intervals.append((start, stamp))
            start = None
    if start is not None:
        intervals.append((start, end))
    return intervals


def clear_crossing_gaps(predictions: Sequence[Prediction],
                        frame: ConflictFrame, payload: bool,
                        now: float, horizon: float,
                        min_clear_sec: float = DEFAULT_MIN_CLEAR_SEC,
                        sample_dt: float = 0.05
                        ) -> List[Tuple[float, float]]:
    """Clear gaps of at least ``min_clear_sec`` inside the crossing band."""
    blocked = band_occupancy_timeline(predictions, frame, payload, now,
                                      horizon, sample_dt)
    gaps: List[Tuple[float, float]] = []
    cursor = now
    for start, end in blocked:
        if start - cursor >= min_clear_sec:
            gaps.append((cursor, start))
        cursor = max(cursor, end)
    if now + horizon - cursor >= min_clear_sec:
        gaps.append((cursor, now + horizon))
    return gaps


def offset_band_intervals(predictions: Sequence[Prediction],
                          frame: ConflictFrame, payload: bool,
                          now: float, horizon: float,
                          lateral_offset: float,
                          sample_dt: float = 0.05,
                          near_hard_sec: float = NEAR_HARD_SEC
                          ) -> List[Tuple[float, float]]:
    """Occupied intervals for a crossing path at a fixed lateral offset.

    A robot crossing the mover strip at ``lateral_offset`` conflicts with a
    body when the body is within its expanded clearance box around that path.
    Passing at a small lateral offset is therefore a real option even while a
    stopped/paused body sits inside the strip centre.
    """
    half_length = (frame.obstacle_radius + frame.envelope(payload)
                   + DEFAULT_BODY_MARGIN)
    half_width = half_length
    intervals: List[Tuple[float, float]] = []
    start = None
    t = now
    end = now + horizon
    while t <= end + 1e-9:
        occupied = False
        near = t <= now + near_hard_sec
        for prediction in predictions:
            if prediction.stale and not prediction.hypotheses:
                continue
            bodies = (_body_at(prediction, t) if near
                      else _nominal_body(prediction, t))
            growth = (0.0 if near
                      else FAR_GROWTH * max(0.0, t - now - near_hard_sec))
            for x, y, radius in bodies:
                radius += growth
                if (abs(x - frame.crossing_x)
                        <= half_length + radius + BAND_MARGIN
                        and abs(y - lateral_offset)
                        <= half_width + radius + BAND_MARGIN):
                    occupied = True
                    break
            if occupied:
                break
        if occupied and start is None:
            start = t
        elif not occupied and start is not None:
            intervals.append((start, t))
            start = None
        t += sample_dt
    if start is not None:
        intervals.append((start, end))
    return intervals


def gaps_from_intervals(blocked: Sequence[Tuple[float, float]],
                        now: float, horizon: float,
                        min_clear_sec: float) -> List[Tuple[float, float]]:
    gaps: List[Tuple[float, float]] = []
    cursor = now
    for start, end in blocked:
        if start - cursor >= min_clear_sec:
            gaps.append((cursor, start))
        cursor = max(cursor, end)
    if now + horizon - cursor >= min_clear_sec:
        gaps.append((cursor, now + horizon))
    return gaps


def _wait_anchor_candidates(frame: ConflictFrame, payload: bool
                            ) -> List[Tuple[float, float]]:
    """Back off along the approach line until outside every swept band."""
    standoff = frame.envelope(payload) + 0.15
    return [(-standoff, 0.0), (-standoff - 0.35, 0.0),
            (-standoff - 0.7, 0.0)]


def _anchor_safe(predictions: Sequence[Prediction],
                 anchor: Tuple[float, float], frame: ConflictFrame,
                 payload: bool, now: float, dwell: float) -> bool:
    """A wait anchor is valid only outside every plausible body's approach."""
    clearance = frame.envelope(payload) + 0.10
    for prediction in predictions:
        for hypothesis in prediction.near_field():
            for t, x, y in hypothesis.positions:
                if t > now + dwell:
                    break
                radius = hypothesis.radius_at(t)
                if math.hypot(x - anchor[0], y - anchor[1]) < radius + clearance:
                    return False
    return True


def _side_path_position(progress: float, points: Sequence[Tuple[float, float]],
                        lengths: Sequence[float]) -> Tuple[float, float]:
    """Piecewise-linear position along a side path for a distance progress."""
    remaining = progress
    for index, length in enumerate(lengths):
        if remaining <= length or index == len(lengths) - 1:
            ratio = 0.0 if length <= 1e-9 else min(1.0, remaining / length)
            start, end = points[index], points[index + 1]
            return (start[0] + (end[0] - start[0]) * ratio,
                    start[1] + (end[1] - start[1]) * ratio)
        remaining -= length
    return points[-1]


def _side_path_blocked(predictions: Sequence[Prediction],
                       frame: ConflictFrame, side: float, payload: bool,
                       now: float, transit_span: float) -> Optional[str]:
    """Check the timed lateral path against every plausible mode.

    Comparing the *timed robot position* with the *timed obstacle body* is the
    honest test for a side passage; checking a fixed crossing point for the
    whole transit window rejects geometrically valid shifts.
    """
    points = [(-0.2, 0.0), (frame.crossing_x - 0.4, side),
              (frame.crossing_x + 0.6, side), (frame.crossing_x + 1.1, 0.0)]
    lengths = [math.hypot(points[index + 1][0] - points[index][0],
                          points[index + 1][1] - points[index][1])
               for index in range(len(points) - 1)]
    # Segment timing respects the measured base limits: forward motion uses
    # the forward speed, lateral-only motion the (much slower) side speed, and
    # a diagonal leg is dominated by whichever axis needs longer.
    forward_speed = max(0.15, min(frame.robot_speed, frame.limit_speed))
    lateral_speed = max(0.12, frame.lateral_speed)
    durations = []
    for index, length in enumerate(lengths):
        dx = abs(points[index + 1][0] - points[index][0])
        dy = abs(points[index + 1][1] - points[index][1])
        durations.append(max(dx / forward_speed, dy / lateral_speed, 1e-3))
    # Skewed segments are slower; add a small margin instead of pretending
    # the diagonal is a straight blend of both limit speeds.
    durations = [duration * 1.15 for duration in durations]
    cumulative = [0.0]
    for duration in durations:
        cumulative.append(cumulative[-1] + duration)
    total_time = cumulative[-1]
    travel = min(transit_span, max(1.0, total_time))
    sample_dt = 0.05
    t = 0.0
    clearance = frame.envelope(payload) + 0.06
    while t <= travel + 1e-9:
        # Locate the segment and interpolate by fraction of its duration.
        index = 0
        while index + 1 < len(cumulative) - 1 and t > cumulative[index + 1]:
            index += 1
        span = max(1e-6, durations[index])
        ratio = min(1.0, max(0.0, (t - cumulative[index]) / span))
        start, end = points[index], points[index + 1]
        x = start[0] + (end[0] - start[0]) * ratio
        y = start[1] + (end[1] - start[1]) * ratio
        for prediction in predictions:
            for hypothesis in prediction.near_field():
                if hypothesis.positions[0][0] > now + t:
                    continue
                _, bx, by = hypothesis.position_at(now + t)
                if math.hypot(x - bx, y - by) < (
                        hypothesis.radius_at(now + t) + clearance):
                    return ('track %d %s mode intersects the timed side path '
                            'at t=+%.2f' % (hypothesis.track_id,
                                            hypothesis.mode, t))
        t += sample_dt
    return None


def strategy_candidates(predictions: Sequence[Prediction],
                        frame: ConflictFrame,
                        *,
                        payload: bool = False,
                        now: float = 0.0,
                        retreat_available: Optional[Tuple[float, float]] = None,
                        max_candidates: int = 12
                        ) -> Tuple[List[Strategy], List[RejectedStrategy],
                                   Dict[int, Tuple[float, float]]]:
    """Enumerate finite passage strategies and prune infeasible combinations.

    Returns ``(legal, rejected, windows)``. ``windows`` holds each obstacle's
    current crossing-band occupancy interval in sim time.
    """
    if now <= 0.0:
        now = max((prediction.stamp for prediction in predictions), default=0.0)
    windows = _crossing_windows(predictions, frame, payload, 0.0, now)
    track_ids = [prediction.hypotheses[0].track_id for prediction in predictions]
    duration = frame.crossing_duration(payload=payload)
    cross_start = now + max(0.0, (abs(frame.crossing_x)
                                  - frame.envelope(payload)) / max(
        .15, min(frame.limit_speed, max(frame.robot_speed, .2))))
    legal: List[Strategy] = []
    rejected: List[RejectedStrategy] = []

    def add(strategy: Strategy) -> None:
        legal.append(strategy)

    # 1. Crossing decisions from clear gaps at several lateral path offsets.
    #    A gap is real free space (all plausible modes outside the expanded
    #    clearance box around that path), so a paused/reversing body blocks
    #    only where it actually sits; passing at a small verified lateral
    #    offset remains available instead of waiting for a full strip clear.
    horizon = max(2.0, min(8.0, frame.crossing_duration(payload=payload) + 3.0))
    band_transit = 2.0 * (frame.obstacle_radius + frame.envelope(payload)
                          + DEFAULT_BODY_MARGIN) / max(
        0.20, min(frame.limit_speed, max(frame.robot_speed, 0.20)))
    min_clear = max(DEFAULT_MIN_CLEAR_SEC, band_transit)
    margin = 0.35
    room_left = frame.corridor_left - frame.envelope(payload) - 0.05
    room_right = frame.corridor_right - frame.envelope(payload) - 0.05
    offsets = [0.0]
    step = 0.15
    for distance in (step, 2 * step, 3 * step):
        if room_left >= distance:
            offsets.append(distance)
        if room_right >= distance:
            offsets.append(-distance)
    # Derive offsets that clear a body actually sitting in the strip: pass on
    # the side of it that the measured corridor still allows.
    clearance = (frame.obstacle_radius + frame.envelope(payload)
                 + DEFAULT_BODY_MARGIN)
    for prediction in predictions:
        for hypothesis in prediction.nominal():
            if hypothesis.positions[0][0] > now + 0.2:
                continue
            _, body_x, body_y = hypothesis.position_at(now)
            if abs(body_x - frame.crossing_x) > 1.0:
                continue
            for candidate in (body_y + clearance, body_y - clearance):
                if abs(candidate) <= 1e-9:
                    continue
                if candidate > 0 and candidate > room_left:
                    continue
                if candidate < 0 and -candidate > room_right:
                    continue
                offsets.append(round(candidate, 3))
    deduped = {}
    for offset in offsets:
        deduped[round(offset, 3)] = offset
    offsets = sorted(deduped.values(), key=abs)
    crossing_candidates = 0
    for offset_rank, lateral_offset in enumerate(offsets):
        blocked = offset_band_intervals(predictions, frame, payload, now,
                                        horizon, lateral_offset)
        gaps = gaps_from_intervals(blocked, now, horizon, min_clear)
        if not gaps:
            continue
        for gap_start, gap_end in gaps[:2]:
            if crossing_candidates >= 8:
                break
            relation = 'before' if (not blocked or gap_end <= blocked[0][0]) \
                else 'after'
            decisions = tuple(PassageDecision(track_id, relation)
                              for track_id in track_ids)
            if gap_start <= now + 0.20 and abs(lateral_offset) < 1e-9:
                name = 'cross_now'
                kind = 'before'
                wait_pose = None
                resume_after = 0.0
                waypoints = ((-0.2, 0.0), (frame.crossing_x, 0.0),
                             (frame.crossing_x + 0.8, 0.0))
            else:
                name = 'gap_%+.2f_%d' % (lateral_offset, crossing_candidates)
                kind = 'gap'
                wait_pose = (-0.45, 0.0)
                resume_after = gap_start
                waypoints = ((-0.2, 0.0), wait_pose,
                             (frame.crossing_x, lateral_offset),
                             (frame.crossing_x + 0.8, 0.0))
            strategy = Strategy(
                name, kind, decisions, lateral_offset, waypoints,
                wait_pose=wait_pose, resume_after_sec=resume_after,
                earliest_cross_sec=gap_start,
                latest_cross_sec=max(gap_start, gap_end - min_clear),
                tag='gap_cross' if wait_pose is not None else 'cross_now')
            add(strategy)
            crossing_candidates += 1
    if crossing_candidates == 0:
        rejected.append(RejectedStrategy(
            'order_crossing',
            'no clear crossing gap of %.2fs within %.1fs at any verified '
            'lateral offset (all plausible modes occupy the strip)'
            % (min_clear, horizon)))

    # 2. Lateral side passages: shift the crossing point sideways so the
    #    obstacle's swept band is never entered. Requires corridor room beyond
    #    the envelope plus margin *and* the shifted crossing point must be
    #    clear of every plausible mode during the actual transit window.
    needed = (frame.obstacle_radius + frame.envelope(payload)
              + DEFAULT_BODY_MARGIN + 0.10)
    transit_span = cross_start + duration + 0.5
    for direction, room in (('left', frame.corridor_left),
                            ('right', frame.corridor_right)):
        room_available = room - frame.envelope(payload) - 0.05
        if room_available < needed:
            rejected.append(RejectedStrategy(
                'side_%s' % direction,
                'lateral room %.2f m < required %.2f m '
                '(obstacle radius + envelope + margin)'
                % (room_available, needed)))
            continue
        side = needed if direction == 'left' else -needed
        blocked = _side_path_blocked(predictions, frame, side, payload,
                                     now, transit_span)
        if blocked:
            rejected.append(RejectedStrategy('side_%s' % direction, blocked))
            continue
        add(Strategy('side_%s' % direction, 'side',
                     tuple(PassageDecision(track_id, 'after')
                           for track_id in track_ids), side,
                     ((-0.2, 0.0), (frame.crossing_x - 0.4, side),
                      (frame.crossing_x + 0.6, side),
                      (frame.crossing_x + 1.1, 0.0)),
                     tag='lateral_%s' % direction))

    # 3. Validated wait anchor: outside every plausible reachable set.
    for anchor in _wait_anchor_candidates(frame, payload):
        if abs(anchor[1]) > frame.corridor_left + 1e-9:
            continue
        if anchor[0] < frame.corridor_min_x:  # backward room bound
            continue
        if not _anchor_safe(predictions, anchor, frame, payload, now,
                            WAIT_DWELL_SEC):
            rejected.append(RejectedStrategy(
                'wait_at_%.2f' % anchor[0],
                'wait anchor lies inside a plausible obstacle reachable set'))
            continue
        clear_times = [window[1] + margin for window in windows.values()]
        resume_after = (max(clear_times) if clear_times
                        else now + WAIT_DWELL_SEC)
        add(Strategy('wait_at_%.2f' % anchor[0], 'wait',
                     tuple(PassageDecision(track_id, 'wait')
                           for track_id in track_ids), 0.0,
                     ((-0.2, 0.0), anchor, (frame.crossing_x + 0.8, 0.0)),
                     wait_pose=anchor, resume_after_sec=resume_after,
                     resume_pose=(frame.crossing_x + 0.8, 0.0),
                     tag='wait_safe_anchor'))

    # 4. Bounded retreat then re-enter: only when the caller supplies a
    #    previously traversed retreat pose. Never a generic reverse.
    if retreat_available is not None:
        clear_times = [window[1] + margin for window in windows.values()]
        resume_after = (max(clear_times) if clear_times
                        else now + WAIT_DWELL_SEC)
        add(Strategy('retreat_reenter', 'retreat',
                     tuple(PassageDecision(track_id, 'after')
                           for track_id in track_ids), 0.0,
                     (retreat_available,),
                     retreat_pose=retreat_available,
                     resume_after_sec=resume_after, tag='bounded_retreat'))
    else:
        rejected.append(RejectedStrategy(
            'retreat_reenter', 'no validated retreat pose supplied'))

    legal = legal[:max_candidates]
    return legal, rejected, windows


def decision_features(legal: Sequence[Strategy],
                      rejected: Sequence[RejectedStrategy]) -> dict:
    """Compact loggable summary of the candidate set."""
    return {
        'legal': [{'name': s.name, 'kind': s.kind,
                   'lateral': round(s.lateral_offset, 3),
                   'tag': s.tag,
                   'window': [round(s.earliest_cross_sec, 2),
                              None if math.isinf(s.latest_cross_sec)
                              else round(s.latest_cross_sec, 2)]}
                  for s in legal],
        'rejected': [{'name': r.name, 'reason': r.reason} for r in rejected],
    }
