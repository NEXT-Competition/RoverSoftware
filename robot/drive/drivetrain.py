"""The drivetrain: turn a DriveCommand's left/right into actuator commands.

`DriveCommand(left, right)` stays the one command type in the system. Every
controller — teleop, object_align, shooter_align, waypoint — produces it, and
nothing upstream of here knows or cares what the robot is built like. That is
what lets a one-motor-and-a-steering-servo rover reuse the entire autonomy stack
unchanged: this module is the only thing that has to know.

Four kinds:

    tank        left/right track speeds, any number of motors per side
    servo_steer one or more drive motors plus a steering servo
    single      drive motors only; steering is discarded
    none        no drivetrain (a build that is only mechanisms)

--- the steered-chassis caveat, stated rather than hidden ---
A differential-drive controller expresses "point at it, then go" as
`arcade(0.0, steer)` — see object_align.py and waypoint.py. On a tank that
pivots in place. On a steered chassis it is throttle zero with the wheels
turned: the robot does not rotate, so object_align would sit there steering at a
cone until its search timeout. `DriveConfig.min_pivot_throttle` mitigates this by
creeping forward whenever steering is commanded with no throttle, so the
steering has authority. It is a mitigation, not a fix — proper Ackermann
autonomy would need the controllers themselves to plan arcs, which they don't.
"""

from __future__ import annotations

import time
from typing import Dict, Iterable, List, Optional

from ..config import DriveConfig
from ..control.rpm_trim import RpmTrim
from ..sensors.encoder import Encoder, build_encoder, close_backend
from .motor import ESCMotor


def _clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


def _slew(current, target, max_step):
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


class _SlewLimiter:
    """Caps how fast a commanded value may change, in units per second.

    Lifted verbatim out of TankDrive so every drivetrain kind limits the same
    way — including a steering servo, which is its own reason not to slam a
    linkage from lock to lock. One limiter per channel being limited, because
    they each need their own "where was I last time".
    """

    def __init__(self, rate: float, n: int):
        self.rate = rate
        self._current: List[float] = [0.0] * n
        self._last: float | None = None

    def apply(self, targets: Iterable[float]) -> List[float]:
        targets = list(targets)
        now = time.monotonic()
        if self.rate > 0 and self._last is not None:
            max_step = self.rate * (now - self._last)
            targets = [_slew(cur, tgt, max_step)
                       for cur, tgt in zip(self._current, targets)]
        self._last = now
        self._current = targets
        return targets

    def reset(self) -> None:
        self._current = [0.0] * len(self._current)
        self._last = None


class Drivetrain:
    """Base: owns the actuators, arms them, and stops them."""

    def __init__(self, config: DriveConfig):
        self.cfg = config
        # One ESCMotor per declared actuator, built once. `kind` decides how it
        # is armed, not how it is driven — the throttle-to-angle mapping is the
        # same for an ESC and a positional servo (see drive/motor.py).
        self.motors: Dict[str, ESCMotor] = {
            name: ESCMotor(actuator) for name, actuator in config.actuators.items()
        }
        # One quadrature encoder per actuator that declares a pair of pins;
        # nothing at all on a build that doesn't, which is every build that
        # existed before sensors/encoder.py did. Constructed here (not started —
        # `arm` does that) so the object exists before the GPIO is claimed, the
        # same order the camera and its consumers use.
        self.encoders: Dict[str, Encoder] = {}
        for name, actuator in config.actuators.items():
            enc = build_encoder(actuator, window=config.trim.rpm_window,
                                tau=config.trim.rpm_tau)
            if enc is not None:
                self.encoders[name] = enc

    def _named(self, names: Iterable[str]) -> List[ESCMotor]:
        return [self.motors[n] for n in names if n in self.motors]

    def recalibrate(self) -> None:
        """Copy live-tunable encoder settings onto the objects that cached them.

        The counts-per-rev of a wheel is exactly the kind of number you get right
        by parking the rover, turning the wheel and typing what you counted — a
        round trip that a service restart per attempt means nobody completes. So
        it is `live=True` in tuning.py, and this is what makes that true.
        Called from Robot._push_live_config; never from the control loop.
        """
        for name, enc in self.encoders.items():
            actuator = self.cfg.actuators.get(name)
            if actuator is not None:
                enc.counts_per_rev = float(actuator.encoder_cpr)
                enc.invert = bool(actuator.encoder_invert)
            enc.window = self.cfg.trim.rpm_window
            enc.tau = self.cfg.trim.rpm_tau

    def sample_encoders(self, now: Optional[float] = None) -> None:
        """Advance every encoder's speed estimate. Called once per drive tick."""
        for enc in self.encoders.values():
            enc.sample(now)

    def side_rpm(self, names: Iterable[str]) -> Optional[float]:
        """Mean measured RPM across one side's encoders, or None if it has none.

        A mean rather than the first, because a six-wheel side has three motors
        and the useful number is how fast the TRACK is going. None propagates
        the "no measurement" case all the way to RpmTrim, which is what makes it
        fail open instead of chasing a zero.
        """
        values = [self.encoders[n].rpm() for n in names if n in self.encoders]
        values = [v for v in values if v is not None]
        return sum(values) / len(values) if values else None

    def encoder_telemetry(self) -> Optional[dict]:
        """Per-actuator RPM for a telemetry frame, or None when there are none."""
        out = {}
        for name, enc in self.encoders.items():
            value = enc.telemetry()
            if value is not None:
                out[name] = value
        return out or None

    def status(self) -> Optional[dict]:
        """Wheel speed, keyed by actuator, or None on a build with no encoders.

        Per actuator rather than per side, because that is the resolution you
        need to find the one wheel that is dragging on a six-wheel build — and
        because the names are the operator's own.
        """
        rpm = self.encoder_telemetry()
        return None if rpm is None else {"rpm": rpm}

    def arm(self) -> None:
        """Hold every ESC at neutral long enough for it to arm.

        Servo-kind actuators are parked at neutral too, but they are not what
        the wait is for — an ESC needs to see a steady neutral pulse before it
        will accept throttle. One sleep for the whole drivetrain, not one per
        motor: they are all seeing the same signal at the same time.
        """
        for motor in self.motors.values():
            motor.stop()
        # Claim the encoder pins while the ESCs are holding neutral: nothing is
        # turning, so the first speed sample is taken against a known standstill
        # rather than mid-motion. A backend that isn't there logs once and the
        # encoder stays inert (sensors/encoder.py) — never a reason not to arm.
        for enc in self.encoders.values():
            enc.start()
        needs_arming = any(a.kind == "esc" for a in self.cfg.actuators.values())
        if needs_arming and self.cfg.arm_seconds > 0:
            time.sleep(self.cfg.arm_seconds)

    def drive(self, left: float, right: float) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        for motor in self.motors.values():
            motor.stop()

    def shutdown(self) -> None:
        """Release the GPIO the encoders claimed. Motors are stopped by `stop`.

        Separate from `stop` because stop() runs on every e-stop and mode
        change, and handing the pins back each time would mean re-claiming them
        (and losing the count) the moment the rover moved again.
        """
        for enc in self.encoders.values():
            enc.stop()
        if self.encoders:
            close_backend()


class TankDrivetrain(Drivetrain):
    """Left/right track speeds, fanned out to every motor on each side.

    This is the original TankDrive, generalized from exactly two motors to any
    number per side. With the default layout it drives one motor per side and
    the arithmetic is identical, which is what makes it a drop-in.
    """

    def __init__(self, config: DriveConfig):
        super().__init__(config)
        self._limiter = _SlewLimiter(config.slew_rate, 2)
        # Closed-loop wheel speed. Built whatever the mode, because the mode is
        # live-tunable from the dashboard and an operator switching it on should
        # not have to restart the rover to get an object that already exists.
        # Inert until both sides report a speed — see control/rpm_trim.py.
        self.trim = RpmTrim(config.trim)
        self._trim_at: float | None = None

    # `Robot` and the tests reach for .left/.right to inspect what the servos
    # were actually told. Properties rather than attributes so they follow the
    # roles if a layout reassigns them.
    @property
    def left(self) -> ESCMotor:
        return self._named(self.cfg.roles.left)[0]

    @property
    def right(self) -> ESCMotor:
        return self._named(self.cfg.roles.right)[0]

    def drive(self, left: float, right: float) -> None:
        """Command normalized track speeds in [-1, 1]."""
        self._limiter.rate = self.cfg.slew_rate  # live-tunable
        left, right = self._limiter.apply((_clamp(left), _clamp(right)))
        # AFTER the slew limiter, on purpose. The limiter shapes the operator's
        # intent; the trim corrects what the hardware then actually did with it,
        # and rate-limiting a correction would just add lag to the loop.
        left, right = self._apply_trim(left, right)
        for motor in self._named(self.cfg.roles.left):
            motor.set_throttle(left)
        for motor in self._named(self.cfg.roles.right):
            motor.set_throttle(right)

    def _apply_trim(self, left: float, right: float) -> tuple:
        """Close the speed loop around the two sides, if there is one to close.

        Timed off its own clock rather than the control loop's `dt`: `drive` is
        also called from the tools and the tests, which have no loop, and a PID
        stepped with somebody else's idea of elapsed time is a PID that behaves
        differently in every caller.
        """
        now = time.monotonic()
        dt = 0.0 if self._trim_at is None else now - self._trim_at
        self._trim_at = now
        self.sample_encoders(now)
        return self.trim.apply(left, right,
                               self.side_rpm(self.cfg.roles.left),
                               self.side_rpm(self.cfg.roles.right),
                               dt, now)

    def status(self) -> Optional[dict]:
        """Measured wheel speed and what the trim did with it, for telemetry."""
        base = super().status()
        return None if base is None else {**base, **self.trim.status()}

    def stop(self) -> None:
        self._limiter.reset()
        # Release the loop with the drivetrain. This is also the only thing that
        # clears a latched stall fault, which is deliberate — see RpmTrim.reset.
        self.trim.reset()
        self._trim_at = None
        super().stop()


class SteeredDrivetrain(Drivetrain):
    """Drive motors plus a steering servo.

    Recovers throttle and steer from the left/right pair — the exact inverse of
    `DriveCommand.arcade`, which is what produced them — then drives the wheels
    at throttle and the servo at steer. Throttle and steer are slew-limited
    separately: they are different mechanisms with different reasons to be
    gentle.
    """

    def __init__(self, config: DriveConfig):
        super().__init__(config)
        self._limiter = _SlewLimiter(config.slew_rate, 2)
        self._warned_no_steer = False

    def drive(self, left: float, right: float) -> None:
        # Measured, not corrected. The speed loop is a tank idea — matching two
        # tracks to each other is what keeps a skid-steer straight — and a
        # steered chassis holds its line with the linkage instead. The encoders
        # still report, because a wheel-speed readout is worth having on any
        # build.
        self.sample_encoders()
        left, right = _clamp(left), _clamp(right)
        throttle = (left + right) / 2.0
        steer = (left - right) / 2.0

        # A steered chassis cannot pivot, but the autonomy controllers ask it to
        # (see the module docstring). Creep so the steering bites, rather than
        # sitting still with the wheels turned and the search timer running.
        floor = self.cfg.min_pivot_throttle
        if floor > 0 and abs(steer) > 0.01 and abs(throttle) < floor:
            throttle = floor if throttle >= 0 else -floor

        self._limiter.rate = self.cfg.slew_rate
        throttle, steer = self._limiter.apply((throttle, steer))

        for motor in self._named(self.cfg.roles.throttle):
            motor.set_throttle(throttle)
        steer_motor = self.motors.get(self.cfg.roles.steer)
        if steer_motor is not None:
            steer_motor.set_throttle(_clamp(steer * self.cfg.steer_gain))
        elif not self._warned_no_steer:
            self._warned_no_steer = True
            print("[Drivetrain] servo_steer layout has no steering actuator; "
                  "steering commands are being discarded")

    def stop(self) -> None:
        self._limiter.reset()
        super().stop()


class SingleDrivetrain(Drivetrain):
    """Throttle only. Steering is discarded — logged once, never per tick."""

    def __init__(self, config: DriveConfig):
        super().__init__(config)
        self._limiter = _SlewLimiter(config.slew_rate, 1)
        self._warned = False

    def drive(self, left: float, right: float) -> None:
        self.sample_encoders()  # measured, not corrected — see SteeredDrivetrain
        throttle = (_clamp(left) + _clamp(right)) / 2.0
        if abs(left - right) > 0.05 and not self._warned:
            self._warned = True
            print("[Drivetrain] 'single' layout has no steering; "
                  "steer commands are being ignored")
        self._limiter.rate = self.cfg.slew_rate
        (throttle,) = self._limiter.apply((throttle,))
        for motor in self._named(self.cfg.roles.throttle):
            motor.set_throttle(throttle)

    def stop(self) -> None:
        self._limiter.reset()
        super().stop()


class NullDrivetrain(Drivetrain):
    """No drivetrain at all. Accepts commands and does nothing with them."""

    def drive(self, left: float, right: float) -> None:
        return


_KINDS = {
    "tank": TankDrivetrain,
    "servo_steer": SteeredDrivetrain,
    "single": SingleDrivetrain,
    "none": NullDrivetrain,
}


def build_drivetrain(config: DriveConfig) -> Drivetrain:
    """Construct the drivetrain this layout describes.

    An unknown kind falls back to tank rather than raising: the layout validator
    is what refuses bad kinds, and by the time we are here the robot is booting
    and must end up with *something* that can be stopped.
    """
    cls = _KINDS.get(config.kind)
    if cls is None:
        print(f"[Drivetrain] unknown kind {config.kind!r}; falling back to tank")
        cls = TankDrivetrain
    return cls(config)
