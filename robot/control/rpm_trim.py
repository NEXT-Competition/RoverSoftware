"""Closed-loop wheel speed: make the tracks turn at the rate they were told to.

Everything above the drivetrain speaks in throttle, and a throttle is an
open-loop wish. `DriveCommand(0.5, 0.5)` says "both sides, half power"; it does
not say "both sides, the same speed", and on real hardware those are different
sentences. ESCs have different neutral points and different gain. Gearboxes have
different friction, especially a cold one against one that has been working.
Weight sits off-centre. One track is on grass and the other is on dirt. The rover
drives a slow arc while every number in the system reports a straight line.

This module is the correction. Given each side's measured RPM (sensors/encoder.py)
it returns adjusted throttles, and it does so in one of two ways:

    match     Hold the two sides to EACH OTHER. The error is the difference
              between their speeds; the correction is split, half added to the
              slow side and half taken off the fast one. Needs no calibration
              whatsoever — not even counts-per-rev, since a shared scale factor
              cancels out of a difference — which makes it the one to switch on
              first. It engages only while the two sides are commanded to the
              same speed, because a commanded turn is a difference you ASKED for
              and correcting it would fight the steering.

    velocity  Hold each side to `throttle x max_rpm` independently. This works
              while turning as well, and it makes the throttle mean something
              repeatable — half throttle is the same ground speed on grass as on
              tarmac, until the motor runs out of authority. The price is one
              honest measurement: `max_rpm`, which nobody can guess for you.

--- what it will not do ---
It never commands motion on its own. The correction is bounded by the loop's own
`out_limit`, it is released below `min_throttle`, and both integrators are reset
whenever the drivetrain stops. Above all it FAILS OPEN: any doubt about the
measurement — an encoder that never started, a wheel commanded for a second
while its encoder still reads a standstill — and the loop hands back exactly the
throttles it was given, latching a fault the dashboard shows. A speed controller
that trusts a dead sensor is a speed controller that winds that side to full
throttle, and that failure is worse than the drift it was fitted to fix.
"""

from __future__ import annotations

import time
from typing import Dict, Optional, Tuple

from ..config import TrimConfig
from .pid import PID

# What counts as "not turning" when deciding a wheel has stalled. Absolute RPM
# rather than a fraction of max_rpm, because `match` mode has no max_rpm to take
# a fraction of — and because a wheel doing less than this is not turning under
# any reasonable gearing.
STALL_RPM = 2.0


def _clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


def _sign(v: float) -> float:
    return 1.0 if v >= 0 else -1.0


class RpmTrim:
    """The wheel-speed loop, as one object the tank drivetrain owns.

    Holds a PID per side. `match` mode only ever steps the left one — there is a
    single difference to correct, not two errors — which is why the trace it
    publishes is keyed on the loop as a whole rather than per side.
    """

    def __init__(self, config: TrimConfig):
        self.cfg = config
        self.left_pid = PID(**vars(config.pid))
        self.right_pid = PID(**vars(config.pid))
        self._trim = (0.0, 0.0)      # last correction applied, for telemetry
        self._fault = ""             # latched; cleared by reset()
        # Per side: when we first noticed "commanded, but not moving". None
        # means it is either moving or not being asked to.
        self._stalled_since: Dict[str, Optional[float]] = {"left": None, "right": None}
        self._engaged = False
        # What the last step was aiming at and what it saw, in RPM, so the
        # settings page can graph the loop in the units its gains are written in.
        self._sp = 0.0
        self._m = 0.0

    # --- lifecycle ----------------------------------------------------------

    def retune(self) -> None:
        """Copy gains off the config onto the live loops, without resetting them.

        Same rule as robot.py::_retune: turning a knob mid-run should nudge the
        loop, not make it forget the mismatch it had already learned.
        """
        for pid in (self.left_pid, self.right_pid):
            pid.kp, pid.ki, pid.kd = self.cfg.pid.kp, self.cfg.pid.ki, self.cfg.pid.kd
            pid.out_limit, pid.i_limit = self.cfg.pid.out_limit, self.cfg.pid.i_limit

    def reset(self) -> None:
        """Drop everything the loop has accumulated, including a latched fault.

        Called when the drivetrain stops. That is also the only way out of a
        stall fault, and deliberately so: it means an encoder that came loose
        cannot re-arm the loop while the rover is still moving.
        """
        self.left_pid.reset()
        self.right_pid.reset()
        self._trim = (0.0, 0.0)
        self._fault = ""
        self._stalled_since = {"left": None, "right": None}
        self._engaged = False
        self._sp = self._m = 0.0

    # --- the loop -----------------------------------------------------------

    def apply(self, left: float, right: float,
              left_rpm: Optional[float], right_rpm: Optional[float],
              dt: float, now: Optional[float] = None) -> Tuple[float, float]:
        """Return the throttles to actually send, given what was measured.

        `left_rpm`/`right_rpm` are SIGNED and may be None, which means "this side
        has no working encoder" — not "it is stopped". The distinction is the
        whole safety argument; see the module docstring.
        """
        self._engaged = False
        self._trim = (0.0, 0.0)

        if self.cfg.mode not in ("match", "velocity"):
            return left, right
        if left_rpm is None or right_rpm is None:
            # Measure-only, silently. A build with no encoders is not a fault:
            # it is every build that existed before this module did.
            return left, right
        if self._fault:
            return left, right
        if dt <= 0:
            return left, right

        now = time.monotonic() if now is None else now
        if self._check_stall(left, right, left_rpm, right_rpm, now):
            return left, right

        # Nothing to hold at rest, and an integrator that keeps winding against
        # a wheel the ground is holding still is a lurch waiting for throttle.
        floor = self.cfg.min_throttle
        if abs(left) < floor and abs(right) < floor:
            self.left_pid.reset()
            self.right_pid.reset()
            return left, right

        self.retune()
        if self.cfg.mode == "match":
            return self._match(left, right, left_rpm, right_rpm, dt)
        return self._velocity(left, right, left_rpm, right_rpm, dt)

    def _match(self, left: float, right: float, left_rpm: float,
               right_rpm: float, dt: float) -> Tuple[float, float]:
        """Hold the two sides to each other, while they are asked to be equal."""
        if abs(left - right) > self.cfg.straight_tolerance:
            # Being asked to turn. Hold the integral rather than resetting it:
            # the mismatch between two motors is a physical property that is
            # still true after the corner, and throwing it away every time the
            # steering twitches means the trim never converges on anything.
            return left, right

        # Work in the commanded direction, not in raw signed RPM. In reverse
        # both speeds are negative, and "right is turning faster" is then the
        # MORE negative one — a subtraction on raw values gets that backwards
        # and the loop drives the rover into a tighter and tighter curve.
        forward = _sign(left + right)
        # The measurement is the mismatch itself, positive when the right side
        # is outrunning the left; the setpoint is zero. Stated that way round so
        # the tuning graph plots something an operator can read directly.
        measured = forward * (right_rpm - left_rpm)
        out = self.left_pid.update(-measured, dt)
        self._sp, self._m = 0.0, measured
        # Split, so the pair keeps the average speed the operator asked for.
        # Putting the whole correction on one side would make the robot slow
        # down (or speed up) every time it corrected, which reads as a throttle
        # that goes soft in a straight line.
        half = -out / 2.0
        self._trim = (forward * half, -forward * half)
        self._engaged = True
        return (_clamp(left + self._trim[0]), _clamp(right + self._trim[1]))

    def _velocity(self, left: float, right: float, left_rpm: float,
                  right_rpm: float, dt: float) -> Tuple[float, float]:
        """Hold each side to the speed its own throttle asked for."""
        scale = self.cfg.max_rpm
        if scale <= 0:
            return left, right
        left_target = left * scale
        left_out = self.left_pid.update(left_target - left_rpm, dt)
        right_out = self.right_pid.update(right * scale - right_rpm, dt)
        self._sp, self._m = left_target, left_rpm
        self._trim = (left_out, right_out)
        self._engaged = True
        return (_clamp(left + left_out), _clamp(right + right_out))

    def _check_stall(self, left: float, right: float, left_rpm: float,
                     right_rpm: float, now: float) -> bool:
        """Latch a fault if a commanded wheel is not turning. True once faulted.

        This is what makes a disconnected encoder safe. Without it the loop sees
        0 rpm against a real setpoint, integrates, and pins that side at full
        throttle — a rover that lunges when you ask it to creep.
        """
        seconds = self.cfg.stall_seconds
        if seconds <= 0:
            return False
        for side, cmd, rpm in (("left", left, left_rpm), ("right", right, right_rpm)):
            moving = abs(rpm) >= STALL_RPM
            commanded = abs(cmd) >= self.cfg.min_throttle
            if moving or not commanded:
                self._stalled_since[side] = None
                continue
            since = self._stalled_since[side]
            if since is None:
                self._stalled_since[side] = now
            elif (now - since) >= seconds:
                self._fault = side
                print(f"[RpmTrim] {side} wheel commanded {cmd:+.2f} for "
                      f"{seconds:.1f}s with no encoder motion — speed matching "
                      f"is off until the drivetrain stops (check the encoder "
                      f"wiring, or the wheel is stalled)")
                self.left_pid.reset()
                self.right_pid.reset()
                return True
        return False

    # --- observability ------------------------------------------------------

    @property
    def fault(self) -> str:
        """"left" / "right" once a stall has latched, else ""."""
        return self._fault

    @property
    def engaged(self) -> bool:
        """True when the last call actually corrected something."""
        return self._engaged

    def status(self) -> dict:
        """What the loop did, for a telemetry frame. The speeds come separately.

        Short keys and few of them: this rides a 57600-baud radio shared with
        driving. The corrections are reported only while the loop is actually
        engaged, so an operator can tell "trimming by nothing" from "not
        trimming" — which are the same two numbers and different situations.
        """
        t: dict = {"mode": self.cfg.mode}
        if self._engaged:
            t["tl"] = round(self._trim[0], 3)
            t["tr"] = round(self._trim[1], 3)
        if self._fault:
            t["fault"] = self._fault
        return t

    def trace(self) -> Optional[dict]:
        """The loop's last step, for the settings page's graph. None when idle.

        `match` publishes the difference loop (setpoint 0, error in RPM);
        `velocity` publishes the LEFT side's, because two curves on one chart
        with one scale is a picture nobody can read — and the two sides are
        tuned with the same gains anyway.
        """
        if not self._engaged:
            return None
        return self.left_pid.trace(setpoint=self._sp, measured=self._m)
