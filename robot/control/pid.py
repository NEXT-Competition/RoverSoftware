"""A small, reusable PID controller with output and integral clamping.

Used by the autonomy controllers (color alignment now; heading hold for waypoint
navigation later).
"""

from __future__ import annotations


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, out_limit=1.0, i_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_limit = out_limit
        self.i_limit = i_limit
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        self._integral = _clamp(self._integral + error * dt, -self.i_limit, self.i_limit)
        derivative = 0.0 if self._prev_error is None else (error - self._prev_error) / dt
        self._prev_error = error
        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        return _clamp(out, -self.out_limit, self.out_limit)
