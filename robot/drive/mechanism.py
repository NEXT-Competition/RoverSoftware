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
        # What was ASKED for, per actuator. Distinct from what is currently
        # written to the hardware once a ramp is involved — see _ramp_to.
        self._values: Dict[str, float] = {name: 0.0 for name in self.motors}
        self._output: Dict[str, float] = {name: 0.0 for name in self.motors}
        self._deadline: Optional[float] = None
        self._ramped: Optional[float] = None  # monotonic time of the last step

    def set_power(self, power: float, actuator: Optional[str] = None,
                  hold: bool = False) -> bool:
        """Drive one actuator, or all of them when `actuator` is None.

        `hold` marks this as a RUN-WHILE-HELD command — see `_arm_auto_stop`.
        """
        targets = ([actuator] if actuator is not None else list(self.motors))
        ok = True
        for name in targets:
            if name not in self.motors:
                ok = False
                continue
            self._command(name, power)
        self._write()
        self._arm_auto_stop(hold)
        return ok

    def _command(self, name: str, power: float) -> None:
        """Set one actuator's target, restarting the ramp if it moved.

        Restarting matters: `_write` measures a step from the last time the
        output moved, so a mechanism that sat at its target for a minute would
        otherwise be handed a minute's worth of step on the next command and
        jump straight there — exactly the step the ramp exists to avoid.
        """
        value = _clamp(power)
        if value != self._values.get(name):
            self._ramped = None
        self._values[name] = value

    def _write(self) -> None:
        """Push the current output at the hardware, ramping if asked to.

        With slew_rate 0 the output IS the commanded value and this is the
        straight-through write every mechanism has always done. Above 0 the
        output only closes on the command at that many units per second, so a
        flywheel's ESC sees a ramp instead of a step it will refuse.

        Winding DOWN is never limited: `stop()` has to be immediate, and a
        mechanism easing itself to a halt through an e-stop would be a bug.
        """
        now = time.monotonic()
        rate = max(0.0, float(self.cfg.slew_rate))
        if rate <= 0:
            step = None          # no limit: the output IS the command
        elif self._ramped is None:
            step = 0.0           # a ramp begins now, so nothing moves yet
        else:
            step = rate * (now - self._ramped)
        self._ramped = now
        for name, motor in self.motors.items():
            want = self._values.get(name, 0.0)
            out = self._output.get(name, 0.0)
            if step is not None and abs(want) > abs(out) and want * out >= 0:
                # Same direction and further from zero: this is a wind-up.
                out = out + _clamp(want - out, -step, step)
            else:
                out = want
            # See the module docstring: never write an unchanged value.
            if out == self._output.get(name):
                continue
            self._output[name] = out
            motor.set_throttle(out)

    def apply_preset(self, name: str, hold: bool = False) -> bool:
        """Drive every actuator to the values of a named preset.

        Actuators the preset doesn't mention are set to zero rather than left
        where they were: a preset describes the whole mechanism's state, and a
        roller still spinning because the previous preset named it and this one
        doesn't is a surprise nobody wants near their hands.

        `hold` marks this as a RUN-WHILE-HELD command — see `_arm_auto_stop`.
        """
        preset = self.cfg.presets.get(name)
        if preset is None:
            return False
        for act in self.motors:
            self._command(act, preset.get(act, 0.0))
        self._write()
        self._arm_auto_stop(hold)
        return True

    def _arm_auto_stop(self, hold: bool) -> None:
        """Arm the dead-man, but only for a run-while-held command.

        `auto_stop_seconds` is the dead-man for controls the operator is
        physically holding: those re-announce themselves several times a second,
        so a lost release frame costs a fraction of a second of extra running
        instead of a mechanism nobody can stop.

        It deliberately does NOT apply to a latched command — a press-once
        toggle, a routine's `mech_preset`. Those mean "run until told to stop",
        and one mechanism can be driven both ways: an intake that toggles IN and
        is held to SPIT wants the dead-man on the second and would be unusable
        with it on the first. Which one this is comes from the caller, because
        only the caller knows whether anything is still refreshing it.
        """
        running = any(v != 0.0 for v in self._values.values())
        if running and hold and self.cfg.auto_stop_seconds > 0:
            self._deadline = time.monotonic() + self.cfg.auto_stop_seconds
        else:
            # Stopped, or latched. Either way clear the deadline rather than
            # leaving it: toggling the intake on while a held spit's dead-man
            # was still armed would otherwise inherit it and stop a second later.
            self._deadline = None

    def update(self) -> None:
        if self._deadline is not None and time.monotonic() >= self._deadline:
            self._deadline = None
            self.stop()
            return
        # Advance a ramp. Ticked from Robot.run() for every mechanism, every
        # tick, which is what lets a wind-up continue after the command that
        # started it has been dealt with.
        if self.cfg.slew_rate > 0 and self._output != self._values:
            self._write()

    def stop(self) -> None:
        self._deadline = None
        self._ramped = None
        for name, motor in self.motors.items():
            self._values[name] = 0.0
            self._output[name] = 0.0
            motor.stop()

    def status(self) -> dict:
        # `values` is what the mechanism was TOLD to do, not what has reached
        # the hardware yet. Load-bearing: Robot._set_mechanism decides whether a
        # bare toggle means start or stop by comparing this against the preset,
        # and a value still ramping would never match — pressing the button
        # during a spin-up would restart it instead of stopping it.
        out = {"kind": "power",
               "values": {k: round(v, 3) for k, v in self._values.items()}}
        if self.cfg.slew_rate > 0:
            out["output"] = {k: round(v, 3) for k, v in self._output.items()}
        return out


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
