"""Shooter: a servo-actuated launcher on its own PWM channel.

Mechanically this is whatever your build uses to release a projectile — a servo
that trips a spring latch, pushes a ball into a flywheel, or swings an arm. All
this module knows is "swing to the fire angle, hold briefly, swing back".

--- Why firing is a state machine and not a sleep ---
The obvious implementation is `angle(fire); sleep(0.3); angle(rest)`. That would
block the control loop for 300ms. The loop runs at 5 Hz (200ms period) and warns
above SLOW_TICK_S = 0.1s, so a blocking fire would stall drive updates and trip
the watchdog every shot — and, worse, the robot would keep whatever throttle it
last had while the loop is frozen.

So `fire()` returns immediately and `update()` (called every tick from the run
loop) advances rest -> firing -> retracting -> rest on wall-clock deadlines.

--- Why update() is driven by Robot, not by the controller ---
A mode switch or an e-stop mid-pulse must still retract the servo. The control
manager stops calling the active controller's update() the moment either
happens, so if the shooter were ticked from the controller it would freeze at
the fire angle — stalling the servo against its stop, drawing current, and
leaving the mechanism cocked. Robot.run() ticks it unconditionally instead.
"""

from __future__ import annotations

import time

from ..config import ShooterConfig
# Re-resolved at construction rather than using motor.py's module-level `Servo`:
# run_robot.py sets RS_MOCK_MOTORS *after* the import graph is built, so the
# binding made at import time can be stale. ESCMotor does the same thing.
from .motor import _HardwareServo, _MockServo, mock_motors


class Shooter:
    """One servo-actuated launcher on a single PWM channel."""

    def __init__(self, config: ShooterConfig):
        self.cfg = config
        servo_cls = _MockServo if mock_motors() else _HardwareServo
        self.servo = servo_cls(config.channel)
        self._state = "rest"  # rest | firing | retracting
        self._until = 0.0
        self._shots = 0
        self.stop()

    @property
    def shots(self) -> int:
        """Rounds fired since construction (telemetry / magazine accounting)."""
        return self._shots

    @property
    def state(self) -> str:
        return self._state

    def ready(self) -> bool:
        """True when the mechanism is home and a new shot can start."""
        return self._state == "rest"

    def fire(self) -> bool:
        """Start a shot. Returns False (and does nothing) if still cycling.

        Callers treat False as "not yet", not as an error — the shooter is the
        authority on its own mechanical cycle, so a controller that asks every
        tick simply gets one shot per cycle instead of needing its own timer.
        """
        if not self.ready():
            return False
        now = time.monotonic()
        self.servo.angle(self.cfg.fire_angle)
        self._state = "firing"
        self._until = now + self.cfg.fire_seconds
        self._shots += 1
        # Always logged: on the bench with mocked servos this line is the only
        # evidence a shot happened, and in the field it timestamps every round
        # in the journal next to the telemetry that triggered it.
        print(f"[shooter] fire #{self._shots}")
        return True

    def update(self) -> None:
        """Advance the fire/retract cycle. Cheap; call every control tick."""
        if self._state == "rest":
            return
        now = time.monotonic()
        if now < self._until:
            return
        if self._state == "firing":
            self.servo.angle(self.cfg.rest_angle)
            self._state = "retracting"
            # Hold at rest a moment before declaring ready: a servo commanded to
            # an angle has not *reached* it, and re-firing from a half-retracted
            # position gives a short, weak throw.
            self._until = now + self.cfg.retract_seconds
        else:  # retracting
            self._state = "rest"

    def stop(self) -> None:
        """Return to rest immediately (shutdown, disarm, e-stop)."""
        self.servo.angle(self.cfg.rest_angle)
        self._state = "rest"
        self._until = 0.0

    def status(self) -> dict:
        """Same shape a PulseMechanism reports.

        This class is deliberately NOT a subclass of drive/mechanism.py's
        PulseMechanism: it keeps its own ShooterConfig so the RS_SHOOTER_* env
        vars, the `shooter.*` tuning paths and ShooterAlignController's firing
        policy all stay exactly as they were. It matches the interface instead —
        the same structural-protocol rule that ShooterLike already uses — so
        Robot can hold it in the mechanism registry alongside the rest.
        """
        return {"kind": "pulse", "state": self._state, "count": self._shots,
                "ready": self.ready(), "cool": 0.0}
