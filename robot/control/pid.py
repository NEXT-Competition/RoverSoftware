"""A small, reusable PID controller with output and integral clamping.

Used by the autonomy controllers (color alignment; heading hold for waypoint
navigation). `update` optionally accepts a measured derivative (derivative-on-
measurement) so the heading loop can use the IMU gyro's yaw-rate instead of
finite-differencing a noisy heading.
"""

from __future__ import annotations

from typing import Optional


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

    def update(self, error: float, dt: float, derivative: Optional[float] = None) -> float:
        """Advance the loop one step and return the clamped output.

        `derivative` is the rate of change of `error` (d(error)/dt). Pass it when
        you have a clean measured rate — e.g. for heading hold with a constant
        setpoint, d(error)/dt = -yaw_rate, so pass `-yaw_rate`. This avoids the
        derivative kick and noise of finite-differencing. When omitted (None), the
        derivative is estimated from the change in `error`, as before.
        """
        if dt <= 0:
            return 0.0
        self._integral = _clamp(self._integral + error * dt, -self.i_limit, self.i_limit)
        if derivative is None:
            derivative = 0.0 if self._prev_error is None else (error - self._prev_error) / dt
        self._prev_error = error
        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        return _clamp(out, -self.out_limit, self.out_limit)
