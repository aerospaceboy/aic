"""Measure a fresh slowdown near a continuously moving obstacle."""
import math


class ContinuousResponse:
    def __init__(self, trigger_distance, response_distance, response_speed):
        self.trigger_distance = trigger_distance
        self.response_distance = response_distance
        self.response_speed = response_speed
        self.peak = 0.0
        self.last_stamp = None
        self.evidence = None

    def observe(self, distance, speed, stamp):
        if not all(math.isfinite(v) for v in (distance, speed, stamp)):
            return False
        if distance < 0.0 or speed < 0.0:
            return False
        if self.last_stamp is not None:
            gap = stamp - self.last_stamp
            if gap <= 0.0:
                return False
            if gap > 0.75:
                self.peak = 0.0
        self.last_stamp = stamp
        if distance > self.trigger_distance:
            self.peak = 0.0
            return False
        slowed = (distance <= self.response_distance
                  and self.peak >= self.response_speed + 0.15
                  and speed <= self.response_speed
                  and self.peak - speed >= 0.15)
        if slowed and self.evidence is None:
            self.evidence = {'distance': round(distance, 4),
                             'peak_speed': round(self.peak, 4),
                             'response_speed': round(speed, 4),
                             'stamp_sec': round(stamp, 4)}
        self.peak = max(self.peak, speed)
        return slowed
