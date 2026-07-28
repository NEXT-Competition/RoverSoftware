"""Mechanisms: the non-drivetrain subsystems a build declares in its layout.

An intake, an arm, a second launcher. Two shapes cover what people actually
build:

    PowerMechanism  hold a value. Several actuators move together, so a named
                    preset maps actuator -> value ("in" = roller 1.0, belt 0.8).
    PulseMechanism  a timed cycle: swing to active, hold, return to rest, settle.

`PulseMechanism` is `drive/shooter.py` with the geometry pulled out into
config, and it is non-blocking for exactly the same reason that module gives:
the obvious `angle(fire); sleep(0.3); angle(rest)` would freeze the 50 Hz loop
for 300 ms, trip the slow-tick watchdog, and hold the drive outputs at whatever
they last were. So `activate()` returns immediately and `update()` advances the
cycle on wall-clock deadlines, ticked from `Robot.run()` — never from a
controller, so that a mode switch or an e-stop mid-cycle still retracts instead
of stalling the servo against its stop.

--- why unchanged writes are elided ---
A routine can hold an action every tick ("run the intake while in this state").
Writing an unchanged value would then cost one I2C transaction per actuator per
tick — 300 a second on a six-actuator rover, inside a 100 ms tick budget. So
`set_power` returns early when nothing changed. The drivetrain deliberately does
NOT do this: its slew limiter changes the value nearly every tick anyway, and a
drivetrain that stops writing is a drivetrain whose failsafe stopped ticking.
"""

from __future__ import annotations

import time
from typing import Dict, Optional

from ..config import MechanismConfig
from .motor import ESCMotor


def _clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


class Mechanism:
    """Base: owns its actuators and knows how to make them safe."""

    def __init__(self, config: MechanismConfig):
        self.cfg = config
        self.name = config.name
        self.motors: Dict[str, ESCMotor] = {
            name: ESCMotor(actuator) for name, actuator in config.actuators.items()
        }

    # --- the interface Robot and the routine engine use ---

    def update(self) -> None:
        """Advance any timed behaviour. Cheap; called every control tick."""

    def stop(self) -> None:
        """Return to a safe, inert state. E-stop, shutdown, mode exit."""
        for motor in self.motors.values():
            motor.stop()

    def ready(self) -> bool:
        return True

    def status(self) -> dict:
        return {"kind": self.cfg.kind}


class PowerMechanism(Mechanism):
    """Continuous power on one or more actuators."""

    def __init__(self, config: MechanismConfig):
        super().__init__(config)
        self._values: Dict[str, float] = {name: 0.0 for name in self.motors}
        self._deadline: Optional[float] = None

    def set_power(self, power: float, actuator: Optional[str] = None) -> bool:
        """Drive one actuator, or all of them when `actuator` is None."""
        targets = ([actuator] if actuator is not None else list(self.motors))
        ok = True
        for name in targets:
            motor = self.motors.get(name)
            if motor is None:
                ok = False
                continue
            value = _clamp(power)
            # See the module docstring: never write an unchanged value.
            if self._values.get(name) == value:
                continue
            self._values[name] = value
            motor.set_throttle(value)
        self._arm_auto_stop()
        return ok

    def apply_preset(self, name: str) -> bool:
        """Drive every actuator to the values of a named preset.

        Actuators the preset doesn't mention are set to zero rather than left
        where they were: a preset describes the whole mechanism's state, and a
        roller still spinning because the previous preset named it and this one
        doesn't is a surprise nobody wants near their hands.
        """
        preset = self.cfg.presets.get(name)
        if preset is None:
            return False
        for act in self.motors:
            self.set_power(preset.get(act, 0.0), act)
        self._arm_auto_stop()
        return True

    def _arm_auto_stop(self) -> None:
        running = any(v != 0.0 for v in self._values.values())
        if running and self.cfg.auto_stop_seconds > 0:
            self._deadline = time.monotonic() + self.cfg.auto_stop_seconds
        elif not running:
            self._deadline = None

    def update(self) -> None:
        if self._deadline is not None and time.monotonic() >= self._deadline:
            self._deadline = None
            self.stop()

    def stop(self) -> None:
        self._deadline = None
        for name, motor in self.motors.items():
            self._values[name] = 0.0
            motor.stop()

    def status(self) -> dict:
        return {"kind": "power",
                "values": {k: round(v, 3) for k, v in self._values.items()}}


class PulseMechanism(Mechanism):
    """A timed rest -> active -> recovering cycle. The generalized launcher."""

    def __init__(self, config: MechanismConfig):
        super().__init__(config)
        self._state = "rest"  # rest | active | recovering
        self._until = 0.0
        self._last_activation = -1e9
        self._activations = 0
        self.stop()

    @property
    def activations(self) -> int:
        return self._activations

    @property
    def state(self) -> str:
        return self._state

    def ready(self) -> bool:
        """Home, off cooldown, and with rounds left."""
        if self._state != "rest":
            return False
        if self.cfg.cooldown > 0 and (time.monotonic() - self._last_activation) < self.cfg.cooldown:
            return False
        if self.cfg.max_activations and self._activations >= self.cfg.max_activations:
            return False
        return True

    def activate(self) -> bool:
        """Start a cycle. False (and no effect) if it isn't ready.

        Callers treat False as "not yet", not as an error — the mechanism is the
        authority on its own cycle, so something asking every tick gets one
        activation per cycle rather than needing its own timer.
        """
        if not self.ready():
            return False
        now = time.monotonic()
        for motor in self.motors.values():
            motor.set_angle(self.cfg.active_angle)
        self._state = "active"
        self._until = now + self.cfg.active_seconds
        self._last_activation = now
        self._activations += 1
        # Always logged: with mocked servos on a bench this line is the only
        # evidence anything happened, and in the field it timestamps the event
        # in the journal next to the telemetry that caused it.
        print(f"[{self.name}] activate #{self._activations}")
        return True

    # The launcher's name for the same thing, so a pulse mechanism satisfies the
    # ShooterLike protocol and can be handed to ShooterAlignController.
    def fire(self) -> bool:
        return self.activate()

    def update(self) -> None:
        if self._state == "rest":
            return
        now = time.monotonic()
        if now < self._until:
            return
        if self._state == "active":
            for motor in self.motors.values():
                motor.set_angle(self.cfg.rest_angle)
            self._state = "recovering"
            # A servo commanded to an angle has not reached it; re-activating
            # from half way back gives a short, weak throw.
            self._until = now + self.cfg.recover_seconds
        else:
            self._state = "rest"

    def stop(self) -> None:
        for motor in self.motors.values():
            motor.set_angle(self.cfg.rest_angle)
        self._state = "rest"
        self._until = 0.0

    def status(self) -> dict:
        cool = 0.0
        if self.cfg.cooldown > 0:
            cool = max(0.0, self.cfg.cooldown - (time.monotonic() - self._last_activation))
        return {"kind": "pulse", "state": self._state, "count": self._activations,
                "ready": self.ready(), "cool": round(cool, 2)}


def build_mechanism(config: MechanismConfig) -> Mechanism:
    """Construct the mechanism this config describes.

    An unknown kind becomes a power mechanism rather than raising — the layout
    validator refuses bad kinds, and anything that gets this far is booting and
    must end up with something that can be stopped.
    """
    if config.kind == "pulse":
        return PulseMechanism(config)
    if config.kind != "power":
        print(f"[{config.name}] unknown mechanism kind {config.kind!r}; "
              "treating it as 'power'")
    return PowerMechanism(config)
