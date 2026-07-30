"""A small, reusable PID controller with output and integral clamping.

Used by the autonomy controllers (object alignment; heading hold for waypoint
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
        # Last step, for the tuning graphs. Kept as plain floats written on the
        # way past rather than a log: this is a 50 Hz loop, and a controller that
        # allocates per tick to be observable is a controller that is worse for
        # being observed.
        self._error = 0.0
        self._p = 0.0
        self._i = 0.0
        self._d = 0.0
        self._out = 0.0
        self._saturated = False

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
        self._p = self.kp * error
        self._i = self.ki * self._integral
        self._d = self.kd * derivative
        out = self._p + self._i + self._d
        clamped = _clamp(out, -self.out_limit, self.out_limit)
        self._error = error
        self._out = clamped
        # Whether the limit ate the request. This is the one thing a gain plot
        # cannot show you: a loop pinned at its limit looks like a loop that has
        # chosen its output, and no amount of extra kp will change it.
        self._saturated = clamped != out
        return clamped

    def trace(self, setpoint: float = 0.0, measured: Optional[float] = None) -> dict:
        """The last step, small enough to ride a 57600-baud frame.

        Short keys and 3 decimals because this crosses a radio shared with
        driving and telemetry, at the telemetry rate. Setpoint and measurement
        come from the CALLER: a PID only ever sees the error between them, and
        which is which is the caller's business (for alignment the setpoint is
        the centre of the frame; for heading hold it is a bearing).

        `p`/`i`/`d` are the CONTRIBUTIONS, not the gains — kp times the error,
        not kp. Contributions are what answers "which term is driving this",
        which is the question a gain by itself cannot.
        """
        trace = {
            "sp": round(setpoint, 3),
            "e": round(self._error, 3),
            "o": round(self._out, 3),
            "p": round(self._p, 3),
            "i": round(self._i, 3),
            "d": round(self._d, 3),
        }
        if measured is not None:
            trace["m"] = round(measured, 3)
        if self._saturated:
            trace["sat"] = True
        return trace
