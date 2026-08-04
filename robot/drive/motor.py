"""ESC-driven motor controlled through a servo-style PWM channel.

On the Fusion HAT an ESC looks exactly like a servo: a PWM channel where the
pulse width sets throttle instead of shaft position. Neutral pulse = stop,
longer = forward, shorter = reverse (for a bidirectional ESC).

We map a normalized throttle in [-1.0, 1.0] onto a servo angle using the
per-motor MotorConfig.
"""

from __future__ import annotations

import os
import time

from ..config import MotorConfig

try:
    from fusion_hat.servo import Servo as _HardwareServo  # real hardware
    HAVE_HW = True
except Exception:  # pragma: no cover - lets the stack run on a dev laptop
    _HardwareServo = None
    HAVE_HW = False


class _MockServo:
    """Stand-in for a real Servo: remembers the last angle, drives nothing.

    Used when there's no Fusion HAT, or when motors are mocked on purpose (set
    RS_MOCK_MOTORS=1) so you can exercise comms/telemetry without the hardware.
    """

    def __init__(self, channel):
        self._channel = channel
        self._last = None

    def angle(self, value):
        self._last = value
        # Uncomment to trace commands when developing without hardware:
        # print(f"[MockServo ch{self._channel}] angle={value:.1f}")


def mock_motors() -> bool:
    """True if we should use the mock servo (no HAT, or forced via env)."""
    forced = os.environ.get("RS_MOCK_MOTORS", "").strip().lower() in ("1", "true", "yes", "on")
    return forced or not HAVE_HW


# Back-compat export for the tools (e.g. servo_sweep.py fallback import).
Servo = _MockServo if mock_motors() else _HardwareServo


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class ESCMotor:
    """One ESC + motor on a single PWM channel."""

    def __init__(self, config: MotorConfig):
        self.cfg = config
        # Decide real-vs-mock at construction, so a flag set at startup wins.
        servo_cls = _MockServo if mock_motors() else _HardwareServo
        self.servo = servo_cls(config.channel)
        self._throttle = 0.0
        self.stop()

    @property
    def throttle(self) -> float:
        """Last commanded throttle in [-1, 1] (pre-inversion, as requested)."""
        return self._throttle

    def set_throttle(self, throttle: float) -> None:
        throttle = _clamp(throttle, -1.0, 1.0)
        self._throttle = throttle

        cmd = -throttle if self.cfg.inverted else throttle

        if abs(cmd) < self.cfg.deadband:
            self.servo.angle(self.cfg.neutral_angle)
            return

        # Symmetric throw about neutral: use the SAME angular swing on each side
        # so an off-center neutral_angle doesn't make forward and reverse (and
        # thus the normal vs inverted motor) respond at different rates. A neutral
        # that isn't the midpoint of [min_angle, max_angle] would otherwise make
        # the two tracks start at different throttles and drive at mismatched
        # speeds. The narrower side sets the throw; the wider side is clamped to
        # match, so full forward and full reverse are equal magnitudes.
        throw = max(0.0, min(self.cfg.max_angle - self.cfg.neutral_angle,
                             self.cfg.neutral_angle - self.cfg.min_angle))
        cmd *= self.cfg.max_forward if cmd > 0 else self.cfg.max_reverse
        # Speed trim last, and in both directions: it corrects a mechanical
        # mismatch between two motors, which does not care which way they turn.
        # After the dead band on purpose — trimming a motor to 0.9 must not also
        # move where its dead band starts, or the two sides would begin moving
        # at different stick positions, which is the very thing this fixes.
        cmd *= _clamp(self.cfg.speed_scale, 0.0, 1.0)
        self.servo.angle(self.cfg.neutral_angle + cmd * throw)

    def set_angle(self, degrees: float) -> None:
        """Command an absolute angle, clamped to [min_angle, max_angle].

        Bypasses the throttle mapping entirely. Pulse mechanisms (the launcher
        and anything shaped like it) author their geometry in degrees — "swing
        to 30, hold, swing back to -30" — and turning that into a throttle only
        to have it mapped back into an angle would lose the endpoints to the
        symmetric throw. `set_throttle` remains the right call for anything
        that spins; this is for things that travel between two positions.
        """
        lo, hi = min(self.cfg.min_angle, self.cfg.max_angle), max(self.cfg.min_angle, self.cfg.max_angle)
        self.servo.angle(_clamp(degrees, lo, hi))

    def arm(self, seconds: float) -> None:
        """Hold neutral so the ESC recognizes the signal and arms."""
        self.servo.angle(self.cfg.neutral_angle)
        if seconds > 0:
            time.sleep(seconds)

    def stop(self) -> None:
        self._throttle = 0.0
        self.servo.angle(self.cfg.neutral_angle)
