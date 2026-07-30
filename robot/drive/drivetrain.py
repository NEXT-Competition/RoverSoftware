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
from typing import Dict, Iterable, List

from ..config import DriveConfig, MotorConfig
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
        motors: Dict[str, ESCMotor] = {
            name: ESCMotor(actuator) for name, actuator in config.actuators.items()
        }
        for name in dict.fromkeys(config.roles.left + config.roles.right + config.roles.throttle):
            if name not in motors:
                motors[name] = ESCMotor(
                    MotorConfig(channel=len(motors), name=name, label=name)
                )
        self.motors: Dict[str, ESCMotor] = motors

    def _named(self, names: Iterable[str]) -> List[ESCMotor]:
        return [self.motors[n] for n in names if n in self.motors]

    def arm(self) -> None:
        """Hold every ESC at neutral long enough for it to arm.

        Servo-kind actuators are parked at neutral too, but they are not what
        the wait is for — an ESC needs to see a steady neutral pulse before it
        will accept throttle. One sleep for the whole drivetrain, not one per
        motor: they are all seeing the same signal at the same time.
        """
        for motor in self.motors.values():
            motor.stop()
        needs_arming = any(a.kind == "esc" for a in self.cfg.actuators.values())
        if needs_arming and self.cfg.arm_seconds > 0:
            time.sleep(self.cfg.arm_seconds)

    def drive(self, left: float, right: float) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        for motor in self.motors.values():
            motor.stop()


class TankDrivetrain(Drivetrain):
    """Left/right track speeds, fanned out to every motor on each side.

    This is the original TankDrive, generalized from exactly two motors to any
    number per side. With the default layout it drives one motor per side and
    the arithmetic is identical, which is what makes it a drop-in.
    """

    def __init__(self, config: DriveConfig):
        super().__init__(config)
        self._limiter = _SlewLimiter(config.slew_rate, 2)

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
        for motor in self._named(self.cfg.roles.left):
            motor.set_throttle(left)
        for motor in self._named(self.cfg.roles.right):
            motor.set_throttle(right)

    def stop(self) -> None:
        self._limiter.reset()
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
