"""Mechanisms: the non-drivetrain subsystems a build declares in its layout.

An intake, an arm, a second launcher. Three shapes cover what people actually
build:

    PowerMechanism  hold a value. Several actuators move together, so a named
                    preset maps actuator -> value ("in" = roller 1.0, belt 0.8).
    PulseMechanism  a timed cycle: swing to active, hold, return to rest, settle.
    SequenceMechanism
                    an ordered queue: step 1 moves the servo, step 2 starts a
                    motor, step 3 starts another. Each leg ends on a dwell, on a
                    measured condition, or on both.

--- why a queue is its own kind, and not a routine ---
The FSM in robot/routine/ can already put actions in order, and for anything
that spans a whole behaviour (drive there, look, then fire) that is the right
tool. What it cannot do is describe ONE MECHANISM'S internal cycle:

  - Its legs run at the FSM's state granularity, so the shortest leg you can
    express is a whole state with its own transitions and timeout.
  - A routine is user-authored and abortable, which is right for a plan and
    wrong for a mechanism: the launcher's servo must retract on e-stop whether
    or not any routine is running. `Robot.run()` ticks mechanisms directly for
    exactly this reason (see PulseMechanism), and a queue that lives here
    inherits that.
  - A build has one shooter cycle, not one per routine. Putting it in the
    layout means every routine, every gamepad binding and the operator's own
    jog all fire the same, tested, sequence.

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
from typing import Any, Dict, Optional

from ..config import MechanismConfig, SequenceStep
from ..sensors.encoder import Encoder, build_encoder
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
        # One quadrature encoder per actuator that declares a pair of pins, on
        # the same terms as the drivetrain's: constructed here, started by
        # `start()`, and inert on a build with no GPIO backend. A mechanism has
        # them so a step can gate on a flywheel actually being at speed rather
        # than on a guess about how long it takes to get there.
        self.encoders: Dict[str, Encoder] = {}
        for name, actuator in config.actuators.items():
            enc = build_encoder(actuator)
            if enc is not None:
                self.encoders[name] = enc
        # Set by Robot so a gate can ask about a mechanism that is not this one.
        # A plain dict, shared by reference: a snapshot would stop reflecting
        # the robot the moment anything changed.
        self._registry: Dict[str, Any] = {}

    def bind(self, registry: Dict[str, Any]) -> None:
        """Give this mechanism the means to ask about the others."""
        self._registry = registry

    # --- the interface Robot and the routine engine use ---

    def start(self) -> None:
        """Claim any GPIO this mechanism reads. Called once, as the robot arms."""
        for enc in self.encoders.values():
            enc.start()

    def update(self) -> None:
        """Advance any timed behaviour. Cheap; called every control tick.

        Subclasses must call this: it is where the encoders' speed estimates are
        advanced, and an encoder that is never sampled reports nothing at all.
        """
        for enc in self.encoders.values():
            enc.sample()

    def stop(self) -> None:
        """Return to a safe, inert state. E-stop, shutdown, mode exit."""
        for motor in self.motors.values():
            motor.stop()

    def shutdown(self) -> None:
        """Hand back the encoder GPIO. Separate from `stop` for the reason
        Drivetrain.shutdown gives: stop() runs on every e-stop and mode change,
        and re-claiming the pins each time would lose the count."""
        for enc in self.encoders.values():
            enc.stop()

    def ready(self) -> bool:
        return True

    def rpm(self, actuator: str) -> Optional[float]:
        """Measured speed of one actuator, or None if it is not measured."""
        enc = self.encoders.get(actuator)
        return None if enc is None else enc.rpm()

    def status(self) -> dict:
        return {"kind": self.cfg.kind}

    def _encoder_status(self) -> dict:
        """The `rpm` block, only on a build that measures something."""
        speeds = {n: e.telemetry() for n, e in self.encoders.items()}
        speeds = {n: v for n, v in speeds.items() if v is not None}
        return {"rpm": speeds} if speeds else {}


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
        super().update()
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
                "values": {k: round(v, 3) for k, v in self._values.items()},
                **self._encoder_status()}


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
        super().update()
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
                "ready": self.ready(), "cool": round(cool, 2),
                **self._encoder_status()}


class SequenceMechanism(Mechanism):
    """An ordered queue of legs, advanced off the control tick.

    The shooter that motivated it: a feeder servo, a flywheel and a belt on one
    mechanism, which have to happen IN ORDER and cannot be described by either
    of the other two kinds. `power` writes every actuator at once, and `pulse`
    swings them all to the same angle together.

        steps:
          - {values: {flywheel: 1.0}, seconds: 0.2,
             wait_for: {kind: rpm, actuator: flywheel, at_least: 3000}}
          - {values: {feeder: 40}, seconds: 0.35}
          - {values: {belt: 0.8}, seconds: 0.6}

    Non-blocking, for the reason the module docstring gives: the obvious
    `sleep()` between legs freezes the 50 Hz loop, trips the slow-tick watchdog
    and holds the drive outputs whereever they last were. `activate()` returns
    at once and `update()` advances the queue on deadlines.

    --- what happens when it is interrupted ---
    `stop()` parks every actuator and drops the queue wherever it had got to. It
    runs on e-stop, mode exit and shutdown, which is what makes a half-finished
    sequence safe rather than a servo held against its stop and a flywheel left
    spinning. There is deliberately no `pause`/`resume`: a mechanism that can be
    resumed is one whose actuators can be left loaded while nothing is ticking
    it, and none of the failure modes that stop a sequence are ones where
    picking up from the middle is the right answer.
    """

    def __init__(self, config: MechanismConfig):
        super().__init__(config)
        self._state = "rest"        # rest | running
        self._index = 0
        self._step_at = 0.0         # when the current step was applied
        self._last_activation = -1e9
        self._activations = 0
        self._gate_warned = False
        self._aborted = ""          # why the last run ended early; "" = it didn't
        self.stop()

    # --- what the operator and the routine engine see ---

    @property
    def state(self) -> str:
        return self._state

    @property
    def activations(self) -> int:
        return self._activations

    @property
    def step_index(self) -> int:
        return self._index

    def ready(self) -> bool:
        """Idle, off cooldown, and with runs left.

        `mech_ready` is already a routine condition, so this doubling as "the
        sequence has finished" is what lets a routine wait for one without a
        second vocabulary for it.
        """
        if self._state != "rest":
            return False
        if self.cfg.cooldown > 0 and (time.monotonic() - self._last_activation) < self.cfg.cooldown:
            return False
        if self.cfg.max_activations and self._activations >= self.cfg.max_activations:
            return False
        return True

    def activate(self) -> bool:
        """Start the queue at step 1. False (and no effect) if it isn't ready.

        False means "not yet", not "error" — same contract as PulseMechanism, so
        something asking every tick gets one run per cycle rather than needing a
        timer of its own.
        """
        if not self.ready() or not self.cfg.steps:
            return False
        self._activations += 1
        self._aborted = ""
        self._state = "running"
        self._last_activation = time.monotonic()
        print(f"[{self.name}] sequence #{self._activations} "
              f"({len(self.cfg.steps)} steps)")
        self._enter(0)
        return True

    # A sequence answers to the launcher's verb too, so it satisfies the
    # ShooterLike protocol and can be handed to ShooterAlignController — and so
    # the existing `fire`/`pulse` routine actions drive it with no new vocabulary.
    def fire(self) -> bool:
        return self.activate()

    # --- running it ---

    def _enter(self, index: int) -> None:
        """Apply step `index` and start its clock."""
        self._index = index
        step = self.cfg.steps[index]
        if step.clear:
            for name in self.motors:
                if name not in step.values:
                    self._park(name)
        for name, value in step.values.items():
            self._write(name, value)
        self._step_at = time.monotonic()

    def update(self) -> None:
        super().update()
        if self._state != "running":
            return
        step = self.cfg.steps[self._index]
        elapsed = time.monotonic() - self._step_at
        if elapsed < step.seconds:
            return                        # inside the minimum dwell
        if not self._gate_open(step):
            timeout = step.timeout or self.cfg.step_timeout
            if elapsed < timeout:
                return                    # still waiting on the other factor
            if step.on_timeout == "abort":
                self._abort(f"step {self._step_name(self._index)} waited "
                            f"{timeout:.1f}s for {self._gate_text(step)}")
                return
            print(f"[{self.name}] step {self._step_name(self._index)} gave up "
                  f"waiting for {self._gate_text(step)} after {timeout:.1f}s; "
                  "carrying on because on_timeout is 'advance'")
        nxt = self._index + 1
        if nxt < len(self.cfg.steps):
            self._enter(nxt)
        elif self.cfg.loop:
            self._enter(0)
        else:
            self._finish()

    def _finish(self) -> None:
        self.stop()

    def _abort(self, why: str) -> None:
        # Always logged. A sequence that stops early looks, from the outside,
        # exactly like one that ran — the mechanism is at rest either way — so
        # this line is the only thing that distinguishes "it fired" from "it
        # never got to speed", which are opposite problems to go and fix.
        print(f"[{self.name}] sequence aborted: {why}")
        self._aborted = why
        self.stop()

    # --- the gates: the "other factor" half of a step ---

    def _gate_open(self, step: SequenceStep) -> bool:
        """Is this step's non-time condition satisfied? True when it has none."""
        spec = step.wait_for
        if not spec:
            return True
        kind = str(spec.get("kind", ""))
        if kind == "rpm":
            speed = self.rpm(str(spec.get("actuator", "")))
            if speed is None:
                # No encoder, or one that cannot be trusted (see Encoder.rpm).
                # Held closed rather than waved through: the timeout turns this
                # into a clean abort, and a shooter that declines to fire beats
                # one that feeds a ball into a wheel at an unknown speed.
                return False
            speed = abs(speed)
            at_least = _float(spec.get("at_least"), 0.0)
            at_most = _float(spec.get("at_most"), 0.0)
            if at_least > 0 and speed < at_least:
                return False
            if at_most > 0 and speed > at_most:
                return False
            return True
        if kind == "mech_ready":
            other = self._registry.get(str(spec.get("mech", "")))
            return other is not None and bool(other.ready())
        # Validation refuses unknown kinds, so this is a layout that changed
        # under a running robot. Held closed, so the step's timeout ends the run
        # rather than a gate nobody wrote silently passing.
        if not self._gate_warned:
            self._gate_warned = True
            print(f"[{self.name}] step {self._step_name(self._index)} waits on "
                  f"unknown condition {kind!r}; it cannot be satisfied")
        return False

    def _gate_text(self, step: SequenceStep) -> str:
        """The gate, as the sentence that goes in the abort line."""
        spec = step.wait_for
        kind = str(spec.get("kind", ""))
        if kind == "rpm":
            act = str(spec.get("actuator", ""))
            measured = self.rpm(act)
            at = ("no reading" if measured is None else f"{measured:.0f} rpm")
            bounds = []
            if _float(spec.get("at_least"), 0.0) > 0:
                bounds.append(f">= {_float(spec.get('at_least'), 0.0):.0f}")
            if _float(spec.get("at_most"), 0.0) > 0:
                bounds.append(f"<= {_float(spec.get('at_most'), 0.0):.0f} ")
            return f"{act} {' and '.join(bounds) or 'rpm'} (measured {at})"
        if kind == "mech_ready":
            return f"mechanism {spec.get('mech', '')!r} to be ready"
        return f"condition {kind!r}"

    # --- writing to actuators ---

    def _write(self, name: str, value: float) -> None:
        """One actuator to one value, in the units of its own kind."""
        motor = self.motors.get(name)
        if motor is None:
            return                       # validation refuses unknown names
        if self._is_servo(name):
            motor.set_angle(_clamp(value, -90.0, 90.0))
        else:
            motor.set_throttle(_clamp(value))

    def _park(self, name: str) -> None:
        """One actuator to its safe resting state."""
        motor = self.motors.get(name)
        if motor is None:
            return
        if self._is_servo(name):
            motor.set_angle(self.cfg.rest_angle)
        else:
            motor.stop()

    def _is_servo(self, name: str) -> bool:
        actuator = self.cfg.actuators.get(name)
        return actuator is not None and actuator.kind == "servo"

    def _step_name(self, index: int) -> str:
        step = self.cfg.steps[index] if index < len(self.cfg.steps) else None
        return (step.name if step is not None and step.name
                else f"{index + 1}/{len(self.cfg.steps)}")

    def stop(self) -> None:
        for name in self.motors:
            self._park(name)
        self._state = "rest"
        self._index = 0

    def status(self) -> dict:
        cool = 0.0
        if self.cfg.cooldown > 0:
            cool = max(0.0, self.cfg.cooldown - (time.monotonic() - self._last_activation))
        s = {"kind": "sequence", "state": self._state, "count": self._activations,
             "ready": self.ready(), "cool": round(cool, 2),
             "steps": len(self.cfg.steps),
             **self._encoder_status()}
        if self._state == "running":
            s["step"] = self._index + 1
            s["step_name"] = self._step_name(self._index)
        if self._aborted:
            s["aborted"] = self._aborted
        return s


def _float(value: Any, default: float) -> float:
    """A number out of a gate spec, or the default. Never raises: a mechanism
    mid-cycle must not take the robot down over a typo in one field."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def build_mechanism(config: MechanismConfig) -> Mechanism:
    """Construct the mechanism this config describes.

    An unknown kind becomes a power mechanism rather than raising — the layout
    validator refuses bad kinds, and anything that gets this far is booting and
    must end up with something that can be stopped.
    """
    if config.kind == "pulse":
        return PulseMechanism(config)
    if config.kind == "sequence":
        return SequenceMechanism(config)
    if config.kind != "power":
        print(f"[{config.name}] unknown mechanism kind {config.kind!r}; "
              "treating it as 'power'")
    return PowerMechanism(config)
