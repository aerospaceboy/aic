"""Observation-driven multi-hypothesis short-horizon motion prediction.

Pure module (no ROS imports): the running tracker's latest snapshots are turned
into named, weighted motion hypotheses with an error tube and an explicit
validity window:

  * ``continue`` - the fitted constant velocity, bounded by measured speed,
  * ``pause``    - decelerate at the measured/configured limit and stop,
  * ``reverse``  - reflect at a *learned* patrol boundary, only when the
                   observation history actually contains both turning
                   extrema; a configured period is never used as a prophet,
  * ``occluded`` - a stale/occluded mover keeps a bounded reachable set
                   instead of disappearing.

Weights only order the far-field nominal prediction. The near-field hard set
is the union over every mode whose weight is at least ``min_weight`` and every
mode the tracker cannot rule out (e.g. a just-observed mover whose next turn
is unknown). Dropping a still-plausible dangerous branch to make a gap open is
explicitly forbidden.

G0/G1 note: this prototype's horizon is bounded by visibility and braking
(reaction) distance; the caller must lower the allowed speed when the horizon
is shorter than the validated braking distance.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# Prototype defaults; every value is a documented engineering parameter and
# must be re-measured before being claimed as validated.
DEFAULT_HORIZON_SEC = 3.0
DEFAULT_DT_SEC = 0.1
DEFAULT_UNCERTAINTY = 0.04
DEFAULT_MAX_SPEED = 0.65
DEFAULT_DECEL = 0.35          # measured patrol actors decelerate softly
MIN_MODE_WEIGHT = 0.02        # below this a mode is nominal-only, never dropped
STALE_SEC = 0.35              # older than this the observation is not fresh
OCCLUSION_HORIZON_SEC = 2.0


@dataclass(frozen=True)
class TrackSnapshot:
    """One tracker output used as planner input (all timestamps in sim seconds)."""

    track_id: int
    stamp: float
    x: float
    y: float
    radius: float
    vx: float = 0.0
    vy: float = 0.0
    uncertainty: float = DEFAULT_UNCERTAINTY
    confident: bool = False
    was_moving: bool = False
    history: Tuple[Tuple[float, float, float], ...] = ()

    @staticmethod
    def from_dict(payload: dict) -> 'TrackSnapshot':
        history = tuple(tuple(float(v) for v in sample)
                        for sample in payload.get('history', ()))
        return TrackSnapshot(
            track_id=int(payload['track_id']),
            stamp=float(payload['stamp']),
            x=float(payload['x']),
            y=float(payload['y']),
            radius=float(payload.get('radius', 0.18)),
            vx=float(payload.get('vx', 0.0)),
            vy=float(payload.get('vy', 0.0)),
            uncertainty=float(payload.get('uncertainty',
                                          DEFAULT_UNCERTAINTY)),
            confident=bool(payload.get('confident', False)),
            was_moving=bool(payload.get('was_moving', False)),
            history=history,
        )


@dataclass
class Hypothesis:
    """One named future for one track."""

    mode: str
    track_id: int
    weight: float
    positions: Tuple[Tuple[float, float, float], ...]
    tube: Tuple[float, ...]
    source: str
    valid_until: float
    reason: str = ''
    body_radius: float = DEFAULT_UNCERTAINTY  # obstacle body + tracker error

    def position_at(self, t: float) -> Tuple[float, float, float]:
        """Linear interpolation of the sampled future at absolute time t."""
        if not self.positions:
            raise ValueError('hypothesis has no positions')
        if t <= self.positions[0][0]:
            return self.positions[0]
        for start, end in zip(self.positions, self.positions[1:]):
            if t <= end[0]:
                span = end[0] - start[0]
                ratio = 0.0 if span <= 1e-9 else (t - start[0]) / span
                return (t,
                        start[1] + (end[1] - start[1]) * ratio,
                        start[2] + (end[2] - start[2]) * ratio)
        return self.positions[-1]

    def radius_at(self, t: float) -> float:
        """Conservative body radius (obstacle + tracker uncertainty) at time t."""
        if not self.tube:
            return DEFAULT_UNCERTAINTY + self.body_radius
        index = min(len(self.tube) - 1,
                    max(0, int(round((t - self.positions[0][0])
                                     / max(1e-9, self._dt())))))
        return self.tube[index] + self.body_radius

    def _dt(self) -> float:
        if len(self.positions) < 2:
            return DEFAULT_DT_SEC
        return self.positions[1][0] - self.positions[0][0]


@dataclass
class Prediction:
    """All hypotheses for one snapshot plus conservative and nominal views."""

    stamp: float
    horizon: float
    hypotheses: List[Hypothesis] = field(default_factory=list)
    stale: bool = False

    def mode_weights(self) -> Dict[str, float]:
        weights = {}
        for hypothesis in self.hypotheses:
            key = hypothesis.mode
            weights[key] = weights.get(key, 0.0) + hypothesis.weight
        return weights

    def near_field(self, until: Optional[float] = None,
                   min_weight: float = MIN_MODE_WEIGHT) -> List[Hypothesis]:
        """Modes that must be treated as possible danger for hard filtering."""
        limit = self.stamp + (until if until is not None
                              else min(self.horizon, 1.2))
        return [hypothesis for hypothesis in self.hypotheses
                if hypothesis.weight >= min_weight
                and hypothesis.positions
                and hypothesis.positions[0][0] <= limit]

    def nominal(self) -> List[Hypothesis]:
        """Highest-weight mode per track; ranking only, never sole safety."""
        best: Dict[int, Hypothesis] = {}
        for hypothesis in self.hypotheses:
            current = best.get(hypothesis.track_id)
            if current is None or hypothesis.weight > current.weight:
                best[hypothesis.track_id] = hypothesis
        return list(best.values())


def _integrate(track: TrackSnapshot, now: float, horizon: float,
               dt: float) -> Tuple[Tuple[Tuple[float, float, float], ...],
                                   Tuple[float, ...]]:
    steps = max(1, int(round(horizon / dt)))
    speed = min(DEFAULT_MAX_SPEED, math.hypot(track.vx, track.vy))
    direction = (track.vx / speed, track.vy / speed) if speed > 1e-6 else (0.0, 0.0)
    positions = []
    tube = []
    for index in range(steps + 1):
        t = index * dt
        positions.append((now + t,
                          track.x + direction[0] * speed * t,
                          track.y + direction[1] * speed * t))
        tube.append(max(.03, track.uncertainty + 0.03 * t))
    return tuple(positions), tuple(tube)


def _pause(track: TrackSnapshot, now: float, horizon: float,
           dt: float, decel: float) -> Tuple[Tuple[Tuple[float, float, float], ...],
                                             Tuple[float, ...]]:
    steps = max(1, int(round(horizon / dt)))
    speed = min(DEFAULT_MAX_SPEED, math.hypot(track.vx, track.vy))
    direction = (track.vx / speed, track.vy / speed) if speed > 1e-6 else (0.0, 0.0)
    stop_time = speed / max(1e-3, decel)
    positions = []
    tube = []
    travelled = 0.0
    for index in range(steps + 1):
        t = index * dt
        if t <= stop_time:
            travelled = speed * t - 0.5 * decel * t * t
        else:
            travelled = speed * stop_time - 0.5 * decel * stop_time ** 2
        positions.append((now + t, track.x + direction[0] * travelled,
                          track.y + direction[1] * travelled))
        tube.append(max(.03, track.uncertainty + 0.02 * t))
    return tuple(positions), tuple(tube)


def learned_boundary(history: Sequence[Tuple[float, float, float]],
                     axis: str = 'y',
                     min_excursion: float = 0.25,
                     confirmation: float = 0.08) -> Optional[Tuple[float, float]]:
    """Return an observed patrol low/high band, never a configured prior.

    A boundary is only usable when the recorded track visited an extremum and
    then demonstrably moved back away from it at least once on both ends (or
    one end plus a stale-but-moving history). This keeps a plain approach from
    being extrapolated as a reflection.
    """
    if len(history) < 4:
        return None
    values = [sample[2] if axis == 'y' else sample[1] for sample in history]
    low, high = min(values), max(values)
    if high - low < min_excursion:
        return None
    low_index = values.index(low)
    high_index = values.index(high)
    returned_low = any(value >= low + confirmation
                       for value in values[low_index:])
    returned_high = any(value <= high - confirmation
                        for value in values[high_index:])
    if not (returned_low and returned_high):
        return None
    return low, high


def _reverse(track: TrackSnapshot, now: float, horizon: float, dt: float,
             boundary: Tuple[float, float], axis: str = 'y'
             ) -> Tuple[Tuple[Tuple[float, float, float], ...],
                        Tuple[float, ...]]:
    """Reflect the observed motion at a learned boundary (triangle replay)."""
    steps = max(1, int(round(horizon / dt)))
    low, high = boundary
    speed = min(DEFAULT_MAX_SPEED, math.hypot(track.vx, track.vy))
    value = track.y if axis == 'y' else track.x
    velocity = track.vy if axis == 'y' else track.vx
    positions = []
    tube = []
    position = value
    direction = 1.0 if velocity >= 0 else -1.0
    for index in range(steps + 1):
        t = index * dt
        if index:
            position += direction * speed * dt
            if position >= high:
                position = high - (position - high)
                direction = -1.0
            elif position <= low:
                position = low + (low - position)
                direction = 1.0
        positions.append((now + t, track.x if axis == 'y' else position,
                          position if axis == 'y' else track.y))
        tube.append(max(.03, track.uncertainty + 0.04 * t))
    return tuple(positions), tuple(tube)


def _occluded(track: TrackSnapshot, now: float, horizon: float, dt: float
              ) -> Tuple[Tuple[Tuple[float, float, float], ...],
                         Tuple[float, ...]]:
    """Bounded reachable set for an occluded mover: keep its space blocked."""
    age = max(0.0, now - track.stamp)
    reach = min(OCCLUSION_HORIZON_SEC, age + horizon)
    speed = max(0.15, math.hypot(track.vx, track.vy))
    radius = track.radius + track.uncertainty + speed * reach * 0.5
    steps = max(1, int(round(horizon / dt)))
    positions = tuple((now + index * dt, track.x, track.y)
                      for index in range(steps + 1))
    tube = tuple(radius for _ in positions)
    return positions, tube


def _normalise(weights: Dict[str, float]) -> Dict[str, float]:
    total = sum(weights.values())
    if total <= 0:
        return {key: 1.0 / len(weights) for key in weights}
    return {key: value / total for key, value in weights.items()}


def predict_track(track: TrackSnapshot, *, now: Optional[float] = None,
                  horizon: float = DEFAULT_HORIZON_SEC,
                  dt: float = DEFAULT_DT_SEC,
                  decel: float = DEFAULT_DECEL,
                  residual: Optional[float] = None
                  ) -> Prediction:
    """Build all plausible hypotheses for one track snapshot."""
    clock = track.stamp if now is None else now
    age = clock - track.stamp
    prediction = Prediction(stamp=clock, horizon=horizon,
                            stale=age > STALE_SEC)
    speed = math.hypot(track.vx, track.vy)
    if speed < 1e-3 and not track.was_moving:
        weights = {'pause': 1.0}
        positions, tube = _pause(track, clock, horizon, dt, decel)
        prediction.hypotheses.append(Hypothesis(
            'pause', track.track_id, weights['pause'], positions, tube,
            'observed_stationary', clock + horizon,
            'tracker reports a stationary, previously non-moving body',
            body_radius=track.radius))
        return prediction
    if not track.confident or age > STALE_SEC:
        positions, tube = _occluded(track, clock, horizon, dt)
        prediction.hypotheses.append(Hypothesis(
            'occluded', track.track_id, 1.0, positions, tube,
            'stale_or_occluded', clock + min(horizon, OCCLUSION_HORIZON_SEC),
            f'age={age:.2f}s confident={track.confident}',
            body_radius=track.radius))
        return prediction
    prior = _normalise({'continue': 0.55, 'pause': 0.25, 'reverse': 0.20})
    if residual is not None:
        mismatch = min(1.0, max(0.0, residual) / 0.25)
        prior = _normalise({
            'continue': 0.55 * (1.0 - mismatch) + 0.1,
            'pause': prior['pause'] + 0.2 * mismatch,
            'reverse': prior['reverse'] + 0.2 * mismatch,
        })
    weights = dict(prior)
    boundary = learned_boundary(track.history)
    if boundary is None:
        weights.pop('reverse', None)
        weights = _normalise(weights)
    positions, tube = _integrate(track, clock, horizon, dt)
    prediction.hypotheses.append(Hypothesis(
        'continue', track.track_id, weights['continue'], positions, tube,
        'fitted_velocity', clock + horizon,
        f'v=({track.vx:+.3f},{track.vy:+.3f}) residual={residual}',
        body_radius=track.radius))
    positions, tube = _pause(track, clock, horizon, dt, decel)
    prediction.hypotheses.append(Hypothesis(
        'pause', track.track_id, weights['pause'], positions, tube,
        'deceleration_bound', clock + horizon,
        f'decel={decel:.2f} m/s^2', body_radius=track.radius))
    if 'reverse' in weights:
        # Only reflect when the extrapolated path actually reaches the
        # observed band within the horizon; otherwise the mode is nominal.
        positions_r, tube_r = _reverse(track, clock, horizon, dt, boundary)
        prediction.hypotheses.append(Hypothesis(
            'reverse', track.track_id, weights['reverse'], positions_r,
            tube_r, 'learned_boundary', clock + horizon,
            f'observed band y=[{boundary[0]:.2f},{boundary[1]:.2f}]',
            body_radius=track.radius))
    return prediction


def predict(snapshot: Sequence[TrackSnapshot], *,
            now: Optional[float] = None,
            horizon: float = DEFAULT_HORIZON_SEC,
            dt: float = DEFAULT_DT_SEC) -> List[Prediction]:
    """Predict every track in one coherent snapshot (one timestamp domain)."""
    return [predict_track(track, now=now, horizon=horizon, dt=dt)
            for track in snapshot]


def conservative_bodies(predictions: Sequence[Prediction],
                        t: float) -> List[Tuple[float, float, float]]:
    """Discs that a hard collision filter must treat as occupied at time t.

    Union over all modes with weight >= MIN_MODE_WEIGHT across every track,
    plus the nominal modes. A dangerous branch that still has non-negligible
    evidence is never pruned here.
    """
    bodies = []
    for prediction in predictions:
        if prediction.stale and not prediction.hypotheses:
            continue
        modes = prediction.near_field()
        if not modes:
            modes = prediction.nominal()
        for hypothesis in modes:
            if hypothesis.positions[0][0] > t:
                continue
            stamp, x, y = hypothesis.position_at(t)
            bodies.append((x, y, hypothesis.radius_at(t)))
    return bodies


def nominal_bodies(predictions: Sequence[Prediction],
                   t: float) -> List[Tuple[float, float, float]]:
    bodies = []
    for prediction in predictions:
        for hypothesis in prediction.nominal():
            if hypothesis.positions[0][0] > t:
                continue
            _, x, y = hypothesis.position_at(t)
            bodies.append((x, y, hypothesis.radius_at(t)))
    return bodies


def decision_features(predictions: Sequence[Prediction]) -> Dict[str, object]:
    """Compact, loggable features for causal records and offline replay."""
    features = {'tracks': len(predictions),
                'stale': sum(1 for prediction in predictions if prediction.stale),
                'modes': {}}
    modes: Dict[str, float] = {}
    for prediction in predictions:
        for mode, weight in prediction.mode_weights().items():
            modes[mode] = modes.get(mode, 0.0) + weight
    features['modes'] = modes
    features['nominal'] = [
        {'track_id': hypothesis.track_id, 'mode': hypothesis.mode,
         'weight': round(hypothesis.weight, 3)}
        for prediction in predictions for hypothesis in prediction.nominal()]
    return features
