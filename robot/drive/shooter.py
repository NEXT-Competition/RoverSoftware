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

import math
import time

from ..config import ShooterConfig
# Re-resolved at construction rather than using motor.py's module-level `Servo`:
# run_robot.py sets RS_MOCK_MOTORS *after* the import graph is built, so the
# binding made at import time can be stale. ESCMotor does the same thing.
from .motor import _HardwareServo, _MockServo, mock_motors


class Shooter:
    """One servo-actuated launcher on a single PWM channel.

    The pulse-state machine remains the default behavior for a standard servo
    launcher. A second, optional closed-loop path can also be enabled for a
    flywheel-style shooter by setting a target RPM and feeding back the measured
    RPM; the tuning values below are kept unchanged from the reference script.
    """

    # Closed-loop flywheel speed control values from the reference script.
    # Kept unchanged here to preserve the requested behavior.
    _NEUTRAL_ANGLE = 5.0
    _DIRECTION = -1
    _MAX_THROTTLE = 25.0
    _KP = 0.0020
    _KD = 0.0200
    _MAX_ANGLE_CHANGE = 0.8
    _ERROR_DEADBAND = 20.0
    # Above this the wheel counts as turning, for the stall guard only.
    # Well under any real target and well over encoder noise at rest.
    _STALL_RPM = 5.0
    _CONTROL_INTERVAL_SECONDS = 0.1
    _RPM_PER_THROTTLE = 340.0
    _THROTTLE_DEADBAND = 4.54
    _THROTTLE_HEADROOM = 1.6
    _MAX_TRIM = 4.0
    _FLYWHEEL_DIAMETER_IN = 3.0
    _FLYWHEEL_RADIUS_M = _FLYWHEEL_DIAMETER_IN * 0.0254 / 2.0
    _MAX_LEGAL_RPM = (12.0 / _FLYWHEEL_RADIUS_M) * 60.0 / (2.0 * math.pi)

    def __init__(self, config: ShooterConfig):
        self.cfg = config
        servo_cls = _MockServo if mock_motors() else _HardwareServo
        self.servo = servo_cls(config.channel)
        self._state = "rest"  # rest | firing | retracting
        self._until = 0.0
        self._shots = 0
        self._pid_target_rpm = 0.0
        self._pid_measured_rpm = 0.0
        self._pid_active = False
        self._pid_last_control = 0.0
        self._pid_throttle = 0.0
        self._pid_trim = 0.0
        self._pid_previous_error = 0.0
        # Has anything ever fed a real reading in? Until something does, update()
        # supplies a modelled one (see _estimated_rpm). A single call to
        # set_measured_rpm() flips this for good and the model is never used
        # again — a real sensor always wins over the estimate.
        self._pid_has_sensor = False
        # Stall guard: the last time the wheel was seen actually turning while
        # under command. Only meaningful with a real sensor — the modelled rpm
        # always "moves", which is exactly why an open-loop build cannot have
        # this protection and a sensored one must.
        self._pid_moving_at = 0.0
        self._pid_stalled = False
        self.stop()

    @property
    def shots(self) -> int:
        """Rounds fired since construction (telemetry / magazine accounting)."""
        return self._shots

    @property
    def state(self) -> str:
        return self._state

    @property
    def pid_active(self) -> bool:
        return self._pid_active

    @property
    def pid_throttle(self) -> float:
        return self._pid_throttle

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def set_target_rpm(self, target_rpm: float) -> None:
        """Enable closed-loop RPM control for a flywheel-style mechanism.

        Capped at `max_target_rpm`. That cap is not the competition limit — it
        is the backstop against a FAILING ENCODER: one that reads slow makes the
        controller add throttle to compensate, so the wheel runs away while the
        dashboard stays calm. See EncoderConfig.
        """
        want = max(0.0, float(target_rpm))
        cap = float(getattr(self.cfg, "max_target_rpm", 0.0) or 0.0)
        if cap > 0 and want > cap:
            print(f"[shooter] capping target {want:.0f} -> {cap:.0f} rpm")
            want = cap
        self._pid_target_rpm = want
        self._pid_active = self._pid_target_rpm > 0.0
        if not self._pid_active:
            self._pid_reset()
        else:
            # Arm the stall guard from now, not from whenever this object was
            # built: a wheel that has been sitting still for ten minutes is not
            # a wheel that has been stalled under command for ten minutes.
            self._pid_moving_at = time.monotonic()

    def set_measured_rpm(self, measured_rpm: float) -> None:
        """Feed the latest sensor reading back into the PID control loop."""
        self._pid_measured_rpm = float(measured_rpm)
        self._pid_has_sensor = True
        if self._pid_target_rpm > 0.0:
            self._pid_active = True

    def _estimated_rpm(self) -> float:
        """Modelled wheel speed, for a rover with no tachometer.

        NOT a measurement: it is the assumption "the wheel reaches what we
        commanded". That makes the error identically zero, so the trim term
        never moves and the controller emits its pure feed-forward throttle —
        which is the honest behaviour of an open loop, and is stable.

        It deliberately does NOT invert the commanded throttle to synthesise a
        speed. That version looks more like a measurement but is worse: the
        model then tracks the command with a one-tick delay, and the derivative
        term reacting to the deadband-zeroed error drives trim between 0 and its
        clamp forever — a 5 Hz limit cycle of a few hundred rpm on the real
        wheel, with nothing in the logs to explain it.

        Either way the loop is blind to what the model cannot predict: battery
        sag, ball drag, a wheel stalling against a jam. It exists so the
        flywheel path is usable now and becomes genuinely closed-loop the moment
        a sensor calls set_measured_rpm() instead — one real reading sets
        _pid_has_sensor and this is never consulted again.
        """
        return self._pid_target_rpm

    def muzzle_mps(self, speed_rpm: float) -> float:
        """Rim speed in m/s — what the competition limit is actually written in.

        Rule 5.5 caps a launch at 12.0 m/s, which on this 3 inch wheel is
        _MAX_LEGAL_RPM (~3008 rpm). Reported rather than enforced: the bench
        script deliberately allowed higher targets and MARKED them, because
        tuning above the limit on blocks is a legitimate thing to do and a
        silent cap is how you end up trusting a number that was never applied.
        """
        return (speed_rpm * 2.0 * math.pi / 60.0) * self._FLYWHEEL_RADIUS_M

    def spin(self, on: bool) -> None:
        """Start or stop the flywheel at the configured target speed."""
        # Clear a previous stall trip: the operator pressing the button again is
        # the acknowledgement. Latched across the stop so the dashboard can say
        # WHY the wheel stopped rather than just showing it stopped.
        if on:
            self._pid_stalled = False
        self.set_target_rpm(float(getattr(self.cfg, "target_rpm", 0.0)) if on else 0.0)
        if not on:
            self.stop()

    @property
    def spinning(self) -> bool:
        return self._pid_active

    def _pid_reset(self) -> None:
        self._pid_trim = 0.0
        self._pid_previous_error = 0.0
        self._pid_throttle = 0.0
        self._pid_last_control = 0.0

    def _write_pid_throttle(self, value: float) -> None:
        self.servo.angle(
            self._NEUTRAL_ANGLE + self._DIRECTION * self._clamp(value, 0.0, self._MAX_THROTTLE)
        )

    def _run_pid_control(self, now: float) -> None:
        if not self._pid_active:
            return

        if now - self._pid_last_control < self._CONTROL_INTERVAL_SECONDS:
            return

        target = self._pid_target_rpm
        if target <= 0.0:
            self._pid_reset()
            self._pid_active = False
            self.stop()
            return

        # Stall: commanded, but nothing turning. A jam, a dead ESC and an
        # unplugged encoder all land here, and all three otherwise sit at full
        # throttle indefinitely — the controller reads zero, calls it a huge
        # error, and pushes harder against something that is not moving.
        #
        # Only armed when a real sensor is feeding us: _estimated_rpm() returns
        # the target, so an open-loop build always looks like it is turning and
        # would never trip this. That is a limitation of having no sensor, not a
        # reason to fake one.
        stall_s = float(getattr(self.cfg, "stall_seconds", 0.0) or 0.0)
        if self._pid_has_sensor and stall_s > 0:
            if self._pid_measured_rpm > self._STALL_RPM:
                self._pid_moving_at = now
            elif self._pid_moving_at and (now - self._pid_moving_at) > stall_s:
                print(f"[shooter] {target:.0f} rpm commanded but nothing turning "
                      f"for {stall_s:.0f}s — stopping. Wheel jammed, ESC dead, "
                      f"or the encoder is not reading.")
                self._pid_stalled = True
                self.stop()
                return

        error = target - self._pid_measured_rpm
        if abs(error) < self._ERROR_DEADBAND:
            error = 0.0

        step = self._clamp(
            self._KP * error + self._KD * (error - self._pid_previous_error),
            -self._MAX_ANGLE_CHANGE,
            self._MAX_ANGLE_CHANGE,
        )
        self._pid_previous_error = error
        self._pid_trim = self._clamp(self._pid_trim + step, -self._MAX_TRIM, self._MAX_TRIM)

        ceiling = min(
            self._MAX_THROTTLE,
            self._THROTTLE_DEADBAND + (target / self._RPM_PER_THROTTLE) * self._THROTTLE_HEADROOM,
            self._THROTTLE_DEADBAND + (self._MAX_LEGAL_RPM / self._RPM_PER_THROTTLE) * 1.25,
        )
        throttle = self._clamp(
            self._THROTTLE_DEADBAND + target / self._RPM_PER_THROTTLE + self._pid_trim,
            0.0,
            ceiling,
        )
        self._pid_throttle = throttle
        self._write_pid_throttle(throttle)
        self._pid_last_control = now

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
        """Advance the fire/retract cycle or run closed-loop RPM control."""
        now = time.monotonic()

        if self._pid_active:
            if not self._pid_has_sensor:
                self._pid_measured_rpm = self._estimated_rpm()
            self._run_pid_control(now)
            return

        if self._state == "rest":
            return
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

    def _idle_angle(self) -> float:
        """Where this mechanism sits when it is doing nothing.

        A servo launcher parks at its rest POSITION, which is what rest_angle
        describes. A flywheel must instead sit at NEUTRAL, because neutral is
        the pulse that ARMS its ESC: an ESC that has never been held at neutral
        ignores everything sent to it afterwards. Parking a flywheel at
        rest_angle (-30 by default, a servo geometry value that means nothing to
        an ESC) leaves it unarmed forever, so the robot logs a perfectly healthy
        "flywheel -> N rpm" while the wheel never moves and the button looks
        dead. Only the drivetrain is armed explicitly at start-up
        (Robot.start -> drive.arm), so for the shooter this idle value IS the
        arming signal.
        """
        if float(getattr(self.cfg, "target_rpm", 0.0)) > 0.0:
            return self._NEUTRAL_ANGLE
        return self.cfg.rest_angle

    def stop(self) -> None:
        """Return to rest immediately (shutdown, disarm, e-stop)."""
        self.servo.angle(self._idle_angle())
        self._state = "rest"
        self._until = 0.0
        self._pid_active = False
        self._pid_target_rpm = 0.0
        self._pid_measured_rpm = 0.0
        self._pid_moving_at = 0.0
        self._pid_reset()

    def status(self) -> dict:
        """Same shape a PulseMechanism reports.

        This class is deliberately NOT a subclass of drive/mechanism.py's
        PulseMechanism: it keeps its own ShooterConfig so the RS_SHOOTER_* env
        vars, the `shooter.*` tuning paths and ShooterAlignController's firing
        policy all stay exactly as they were. It matches the interface instead —
        the same structural-protocol rule that ShooterLike already uses — so
        Robot can hold it in the mechanism registry alongside the rest.
        """
        out = {"kind": "pulse", "state": self._state, "count": self._shots,
               "ready": self.ready(), "cool": 0.0, "pid_active": self.pid_active,
               "pid_throttle": self.pid_throttle}
        if self._pid_has_sensor or self._pid_active:
            # The measured speed, and the rim speed it implies. `over` is the
            # bench-mode flag the source script printed as "OVER": a target
            # above the 12.0 m/s competition limit is allowed and MARKED, never
            # silently permitted. `sensor` distinguishes a genuinely closed loop
            # from one running on the feed-forward model — without it a healthy
            # display and a robot with no encoder look identical.
            out["rpm"] = round(self._pid_measured_rpm, 0)
            out["target_rpm"] = round(self._pid_target_rpm, 0)
            out["mps"] = round(self.muzzle_mps(self._pid_measured_rpm), 2)
            out["sensor"] = self._pid_has_sensor
            out["over"] = self._pid_measured_rpm > self._MAX_LEGAL_RPM
            out["stalled"] = self._pid_stalled
        return out