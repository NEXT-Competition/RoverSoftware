"""Ball-intake controller: chase a ball, drive over it, run the intake.

Sibling to `object_align.py` and deliberately its opposite policy. Both steer a
PID on a detection's horizontal offset; what they disagree about is the ending.

    object_align   approach and STOP SHORT at a standoff. Right for a bucket:
                   you line up on it and shoot, you do not touch it.
    ball_intake    approach and DRIVE THROUGH, with the intake running. There
                   is no standoff — arriving at the ball is the failure mode,
                   collecting it is the goal.

That difference is not a parameter, which is why this is its own controller
rather than `approach=True, standoff=0`. It changes three things structurally:

  - Arrival is not a state. There is no latch, no hysteresis, no rangefinder.
  - The intake is an actuator this loop owns, so every branch must command it.
  - Losing sight of the target is EXPECTED and is not a failure. A ball
    disappears under the intake mouth a moment before it is collected, so this
    controller keeps driving blind on a timer where object_align refuses to
    advance without a live size reading.

--- The state machine ---
    no provider                    -> stop (perception not wired up)
    ball above the collect line    -> chase: PID steers, throttle eases off
                                      with proximity
    ball at/below the collect line -> collect: intake on, creep STRAIGHT
    just lost it                   -> push: blind straight, intake still on
    lost a little longer           -> hold: stopped, intake still on
    lost for a while               -> search: sweep, then step forward, repeat

--- Why "collect" drives straight instead of steering ---
Below the collect line the ball is within the intake's mouth, close enough that
a steering correction sweeps the mouth sideways past it. The ball is also at its
largest and most off-centre-looking there, so a PID that stayed engaged would
steer hardest exactly when steering hurts most. Straight and slow is what worked
on the robot.

--- Why blind motion is timed, not distance-based ---
This robot has no encoder feedback into this loop and no sensor that can see
under the intake, so "has the ball gone in" is unanswerable. The timers are
therefore open-loop, and honestly so: `collect_push_s` says how long to keep
DRIVING and `intake_hold_s` how long to keep the INTAKE turning, and the second
is longer because a ball in the throat is still being collected after the robot
has stopped moving. A beam break across the intake would replace both with a
real signal; until then these are stopwatch values, not derived ones.

Downstream, `control/collision.py` clamps whatever this returns, so the blind
legs are bounded by the same guard as everything else — worth knowing, because
they are the only place this controller drives with nothing in view.

--- Why the PID advances on detection stamps, not control ticks ---
Same reason as object_align: the detector and the control loop run at unrelated
rates, and `detection()` is a cached read, so the loop sees the same sample for
several ticks. On the IMX500 the sensor attaches inference to a fraction of
frames, which makes the gap wide and irregular. Feeding a tick dt to a loop that
only has new information every few ticks inflates the derivative term and winds
the integral on a measurement that has not changed.
"""

from __future__ import annotations

import time
from typing import Callable, Dict, Optional

from .commands import DriveCommand
from .controller import Controller
from .detection import Detection
from .pid import PID

DetectionProvider = Callable[[], Optional[Detection]]


class IntakeActuator:
    """The slice of `drive/mechanism.PowerMechanism` this controller needs.

    Structural, not inherited: a PowerMechanism satisfies it as-is, and the
    tests pass a recorder. Keeping it this narrow is what lets the whole
    controller run with no robot, no config and no hardware imports.
    """

    def set_power(self, power: float, actuator: Optional[str] = None) -> bool: ...
    def stop(self) -> None: ...


class BallIntakeController(Controller):
    name = "ball_intake"

    def __init__(
        self,
        detection_provider: Optional[DetectionProvider] = None,
        intake: Optional[IntakeActuator] = None,
        target_label: str = "ball",
        intake_power: float = 1.0,  # +1 takes in, -1 spits (PowerMechanism)
        # Geometry. error_y is normalized: 0 = frame centre, +1 = bottom edge.
        collect_line: float = 0.4,  # at/below this, the ball is at the mouth
        # Speeds, all normalized throttle.
        chase_speed: float = 0.5,  # throttle when the ball is far
        collect_speed: float = 0.3,  # creep once at the mouth
        push_speed: float = 0.3,  # blind, after losing sight
        # Easing: throttle falls from chase_speed to collect_speed as the ball
        # descends the frame, so the robot is already slow when it arrives
        # rather than braking at the line.
        pivot_threshold: float = 0.35,  # |error_x| above this => turn in place
        # Blind timers, measured from the moment the ball was last seen.
        collect_push_s: float = 1.0,  # keep DRIVING this long
        intake_hold_s: float = 3.0,  # keep the INTAKE on this long (>= push)
        # Search, once the ball has been gone longer than intake_hold_s.
        search_spin_s: float = 5.0,
        search_advance_s: float = 1.0,
        search_spin_speed: float = 0.25,
        search_advance_speed: float = 0.3,
        pid: Optional[PID] = None,
        # Injectable so the timers can be tested deterministically. Every
        # behaviour here is a function of elapsed time, and a test that reaches
        # a state by sleeping is a test that fails on a loaded CI box near a
        # cycle boundary - which is exactly where the interesting cases are.
        now_fn: Callable[[], float] = time.monotonic,
    ):
        self._now = now_fn
        self.detection_provider = detection_provider
        self.intake = intake
        self.target_label = target_label
        self.intake_power = intake_power
        self.collect_line = collect_line
        self.chase_speed = chase_speed
        self.collect_speed = collect_speed
        self.push_speed = push_speed
        self.pivot_threshold = pivot_threshold
        self.collect_push_s = collect_push_s
        self.intake_hold_s = intake_hold_s
        self.search_spin_s = search_spin_s
        self.search_advance_s = search_advance_s
        self.search_spin_speed = search_spin_speed
        self.search_advance_speed = search_advance_speed
        # Same starting point as object_align: gains sized for perception dead
        # time, not for how fast the drivetrain could react.
        self.pid = pid or PID(kp=0.5, ki=0.0, kd=0.05, out_limit=0.8)

        self._vision = None
        self._target_restore: Optional[str] = None
        self._last_seen = 0.0
        self._last_stamp: Optional[float] = None
        self._steer = 0.0
        self._state = "idle"
        self._intake_on = False
        # Gate for the blind push. Without it, "no ball yet" at startup is
        # indistinguishable from "ball just vanished", and the robot drives
        # forward the moment the mode is selected.
        self._had_ball = False

    # -- wiring -------------------------------------------------------------

    def set_detection_provider(self, provider: DetectionProvider) -> None:
        self.detection_provider = provider

    def set_intake(self, intake: IntakeActuator) -> None:
        self.intake = intake

    def set_vision_config(self, vision) -> None:
        """Narrow the DETECTOR to balls, not just this loop.

        Filtering by label here is not enough. The detector picks ONE box per
        frame (`vision.select`, default "largest") and only then does this
        controller look at the label — so with `vision.target_label` empty, a
        blue bucket in view wins the pick every frame simply by being bigger,
        this loop sees "not a ball" (i.e. nothing at all), and the robot
        searches with a ball plainly in frame.

        Setting the shared VisionConfig moves the filter ahead of the pick.
        Same object and same borrow-and-restore contract the RoutineController
        already uses for its per-state targets; restored on deactivate so
        switching to object_align does not leave the detector blind to buckets.
        """
        self._vision = vision

    # -- lifecycle ----------------------------------------------------------

    def on_activate(self) -> None:
        if self._vision is not None and self.target_label:
            if self._target_restore is None:  # not already borrowed
                self._target_restore = str(getattr(self._vision, "target_label", ""))
            self._vision.target_label = self.target_label
        self.pid.reset()
        self._last_stamp = None
        self._steer = 0.0
        self._state = "idle"
        # _had_ball resets too: entering the mode is a fresh hunt, and a ball
        # seen before the operator switched away is not evidence about now.
        self._had_ball = False
        self._last_seen = 0.0

    def on_deactivate(self) -> None:
        self._set_intake(False)
        if self._vision is not None and self._target_restore is not None:
            self._vision.target_label, self._target_restore = self._target_restore, None

    def on_estop(self) -> None:
        # The manager stops the drivetrain; the intake is ours and must not
        # keep spinning through an e-stop.
        self._set_intake(False)
        self._had_ball = False

    # -- the loop -----------------------------------------------------------

    def update(self, dt: float) -> Optional[DriveCommand]:
        if self.detection_provider is None:
            self._set_intake(False)
            self._state = "no_perception"
            return DriveCommand.stopped()

        now = self._now()
        d = self.detection_provider()
        if d is not None and self.target_label and d.label != self.target_label:
            d = None  # a bucket is not a ball; treat it as nothing seen

        if d is None:
            return self._blind(now)

        self._last_seen = now
        self._had_ball = True

        if d.error_y >= self.collect_line:
            # At the mouth. Intake on, straight and slow — see module docstring.
            self._set_intake(True)
            self.pid.reset()
            self._steer = 0.0
            self._last_stamp = None
            self._state = "collect"
            return DriveCommand.arcade(self.collect_speed, 0.0)

        self._set_intake(False)
        steer = self._steer_for(d, dt)
        self._state = "chase"
        if abs(d.error_x) > self.pivot_threshold:
            # Point-then-go, as object_align and waypoint both do: swing onto
            # the bearing before committing to drive.
            return DriveCommand.arcade(0.0, steer)
        return DriveCommand.arcade(self._approach_speed(d), steer)

    # -- pieces -------------------------------------------------------------

    def _approach_speed(self, d: Detection) -> float:
        """Ease from chase_speed down to collect_speed as the ball descends.

        Linear in error_y between the top of the frame and the collect line, so
        the robot is already at collecting speed when it reaches the line
        instead of braking there. Clamped because error_y above the line is the
        collect branch's business, not this one's.
        """
        if self.collect_line <= -1.0:
            return self.collect_speed
        span = self.collect_line - (-1.0)
        t = (d.error_y - (-1.0)) / span
        t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
        return self.chase_speed + (self.collect_speed - self.chase_speed) * t

    def _steer_for(self, d: Detection, dt: float) -> float:
        """PID on horizontal offset, advanced once per NEW detection."""
        if self._last_stamp is not None and d.stamp <= self._last_stamp:
            return self._steer  # same sample as last tick; hold the output
        dt_det = dt if self._last_stamp is None else (d.stamp - self._last_stamp)
        self._last_stamp = d.stamp
        if dt_det <= 0:
            return self._steer
        self._steer = self.pid.update(d.error_x, dt_det)
        return self._steer

    def _blind(self, now: float) -> Optional[DriveCommand]:
        """Nothing in view. Push, hold, then search."""
        self.pid.reset()
        self._last_stamp = None
        self._steer = 0.0

        if not self._had_ball:
            # Never seen one in this activation: search rather than push, so
            # selecting the mode does not lurch the robot forward.
            self._set_intake(False)
            return self._search(now - self._last_seen if self._last_seen else 1e9)

        lost_for = now - self._last_seen
        # The intake outlives the drive push: a ball in the throat is still
        # being collected after the robot has stopped.
        self._set_intake(lost_for <= self.intake_hold_s)

        if lost_for <= self.collect_push_s:
            self._state = "push"
            return DriveCommand.arcade(self.push_speed, 0.0)
        if lost_for <= self.intake_hold_s:
            self._state = "hold"
            return DriveCommand.stopped()
        return self._search(lost_for - self.intake_hold_s)

    def _search(self, searching_for: float) -> DriveCommand:
        """Sweep in place, step forward, repeat.

        Spinning alone only ever sees one circle of the field; if the nearest
        ball is outside it the robot sweeps forever. Stepping forward between
        sweeps searches new ground.

        Stateless on purpose: the phase is elapsed time modulo the cycle, so
        there is no search state machine to get stuck mid-sweep, and it resets
        by itself the moment a ball is seen and `_last_seen` advances.
        """
        cycle = self.search_spin_s + self.search_advance_s
        if cycle <= 0:
            self._state = "search_spin"
            return DriveCommand.arcade(0.0, self.search_spin_speed)
        phase = searching_for % cycle
        if phase < self.search_spin_s:
            self._state = "search_spin"
            return DriveCommand.arcade(0.0, self.search_spin_speed)
        self._state = "search_advance"
        return DriveCommand.arcade(self.search_advance_speed, 0.0)

    def _set_intake(self, on: bool) -> None:
        """Command the intake every time, not only on change.

        The mechanism latches: a branch that says nothing leaves it running.
        Tracking `_intake_on` here is for telemetry, not to skip the call.
        """
        self._intake_on = on
        if self.intake is None:
            return
        if on:
            self.intake.set_power(self.intake_power)
        else:
            self.intake.stop()

    # -- observability ------------------------------------------------------

    def state(self) -> str:
        return self._state

    def intake_running(self) -> bool:
        return self._intake_on

    def pid_traces(self) -> Dict[str, dict]:
        return {"ball_x": self.pid.trace()}
