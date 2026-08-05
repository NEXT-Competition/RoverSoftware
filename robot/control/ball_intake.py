"""Autonomous ball intake: find a ball, drive onto it, swallow it, look for the next.

Ported from Team Northeast's `auto_chassis.py` v2.4 (MSYaroschuk/TheNorthTeam),
which ran as a standalone script with its own camera loop, its own drivetrain
and an OpenCV window. The BEHAVIOUR here is that script's, tuning constants and
all; what changed is everything around it:

    auto_chassis.py                     this file
    ---------------                     ---------
    opens the IMX500 itself             consumes Detection from sensors/detector.py
    Servo(ch) globals, own accel limit  returns DriveCommand; Drivetrain slews
    intake_motor.angle(INTAKE_SPEED)    drives the `intake` mechanism by preset
    pixels on a 640x480 frame           normalized error_x / error_y
    cv2.imshow + waitKey                telemetry via status()
    module-level `running` flag         the e-stop and mode arbitration

That is not a rewrite for its own sake: the script could only ever be the whole
robot, so it could not be e-stopped from the base station, could not share the
camera with FPV, and stopped the moment the window closed. As a Controller it is
one mode among several, and every safety property the rest of the stack already
has applies to it unchanged.

--- The tracking gate is the load-bearing part ---
Most of this file is the confirm-then-remember gate, and it is the part that was
learned on the robot rather than designed. A real ball persists across frames; a
false positive flickers. Confidence alone cannot separate them, so a ball must
be seen `confirm_frames` times in a row before the robot acts on it, and is then
remembered for `memory_s` through short dropouts so one missed frame does not
brake the robot mid-approach.

The match radius GROWS with the gap since the last sighting (see `match_tol`).
A fixed radius cannot work: the AI Camera attaches inference to only ~25% of
frames, so consecutive detections are ~130 ms apart, and while the robot is
scanning the scene sweeps past far faster than the radius in that time. The ball
then reads as a different object every frame, the streak never completes, and
the robot scans forever — and the lack of a lock is what MAKES it scan, so the
failure feeds itself. Scaling by the real elapsed gap tracks the physical limit
(a ball can only move so fast) instead of assuming a frame rate we do not have.

--- Why the streak advances on stamps, not ticks ---
Same reason object_align's PID does. `detection()` is a cached read, so the loop
sees the same sample for several ticks; counting ticks would confirm a phantom
in a few milliseconds. A changed `stamp` is exactly the script's
`inference_attached`: the only frames the sensor really ran the network on, and
so the only evidence that may advance OR break the streak.
"""

from __future__ import annotations

import time
from typing import Callable, Dict, Optional

from .commands import DriveCommand
from .controller import Controller
from .detection import Detection

# detection_provider() -> latest Detection, or None if nothing is currently seen
DetectionProvider = Callable[[], Optional[Detection]]


def _clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


def _frame_y(error_y: float) -> float:
    """error_y in [-1, 1] -> height fraction down the frame, in [0, 1].

    The script thought in pixels down a 480-line frame and compared against a
    stop line at 70% of it. `stop_line` keeps that number recognisable, so this
    is where the two coordinate systems meet.
    """
    return _clamp((error_y + 1.0) / 2.0, 0.0, 1.0)


class BallIntakeController(Controller):
    name = "ball_intake"

    def __init__(
        self,
        detection_provider: Optional[DetectionProvider] = None,
        mechanisms: Optional[Dict[str, object]] = None,
        intake_mech: str = "intake",
        intake_preset: str = "in",
        # --- approach ---
        # 0.75 falling to 0.40 at the stop line, as the script did: slow down as
        # the ball gets close so the intake can actually grab it.
        cruise_speed: float = 0.75,
        approach_speed: float = 0.40,
        steering_gain: float = 0.4,
        stop_line: float = 0.70,      # frame fraction; below this = "swallow it"
        swallow_speed: float = 0.30,  # creep straight while taking the ball in
        swallow_run_on_s: float = 0.8,
        # --- tracking gate ---
        confirm_frames: int = 2,
        memory_s: float = 0.4,
        match_tol: float = 0.20,
        match_tol_per_s: float = 2.2,
        # --- lost ball ---
        lost_push_s: float = 1.0,
        lost_intake_s: float = 3.0,
        lost_push_speed: float = 0.3,
        # --- search ---
        search_after: float = 5.0,
        scan_spin_s: float = 5.0,
        scan_advance_s: float = 1.0,
        scan_spin_speed: float = 0.25,
        scan_advance_speed: float = 0.3,
    ):
        self.detection_provider = detection_provider
        self.mechanisms: Dict[str, object] = mechanisms if mechanisms is not None else {}
        self.intake_mech = intake_mech
        self.intake_preset = intake_preset
        self.cruise_speed = cruise_speed
        self.approach_speed = approach_speed
        self.steering_gain = steering_gain
        self.stop_line = stop_line
        self.swallow_speed = swallow_speed
        self.swallow_run_on_s = swallow_run_on_s
        self.confirm_frames = confirm_frames
        self.memory_s = memory_s
        self.match_tol = match_tol
        self.match_tol_per_s = match_tol_per_s
        self.lost_push_s = lost_push_s
        self.lost_intake_s = lost_intake_s
        self.lost_push_speed = lost_push_speed
        self.search_after = search_after
        self.scan_spin_s = scan_spin_s
        self.scan_advance_s = scan_advance_s
        self.scan_spin_speed = scan_spin_speed
        self.scan_advance_speed = scan_advance_speed

        # Three variables instead of a tracker class, as the script had it.
        # Upgrade to real data association only if we ever chase more than one
        # ball at a time.
        self._confirmed: Optional[Detection] = None
        self._confirmed_at = 0.0
        self._streak = 0
        self._last_stamp: Optional[float] = None

        self._last_ball_time = 0.0
        self._had_ball = False       # gates the lost-ball push; see _lost()
        self._intake_until = 0.0
        self._intake_on = False
        self._match_tol_now = 0.0    # last tolerance used, for telemetry
        self._phase = "idle"         # what status() reports

    def set_detection_provider(self, provider: DetectionProvider) -> None:
        self.detection_provider = provider

    def set_mechanisms(self, mechanisms: Dict[str, object]) -> None:
        self.mechanisms = mechanisms

    # --- lifecycle ----------------------------------------------------------

    def on_activate(self) -> None:
        now = time.monotonic()
        self._confirmed = None
        self._confirmed_at = 0.0
        self._streak = 0
        self._last_stamp = None
        # "Just saw one", so entry to the mode sits still for search_after
        # rather than immediately spinning.
        self._last_ball_time = now
        # NOT carried across activations: a push is blind motion, and inheriting
        # `had_ball` from a previous run would drive the robot forward on entry
        # for a ball it saw minutes ago in another mode.
        self._had_ball = False
        self._intake_until = 0.0
        self._phase = "idle"
        self._stop_intake()

    def on_deactivate(self) -> None:
        self._stop_intake()
        self._phase = "idle"

    def on_estop(self) -> None:
        # Robot._apply_estop stops every mechanism too, so this is belt and
        # braces — but the flag must be cleared either way, or clearing the stop
        # would leave us believing the intake is still running and never restart
        # it. Cheap insurance against an ordering change up the stack.
        self._stop_intake()
        self._intake_until = 0.0
        self._had_ball = False

    # --- the intake mechanism ----------------------------------------------

    def _start_intake(self) -> None:
        if self._intake_on:
            return  # latched already; re-applying every tick buys nothing
        mech = self.mechanisms.get(self.intake_mech)
        if mech is None or not hasattr(mech, "apply_preset"):
            return
        # hold=False: this is a LATCHED command, not a run-while-held one. The
        # dead-man in PowerMechanism only protects controls something is
        # refreshing, and nothing refreshes an autonomous decision — see
        # PowerMechanism._arm_auto_stop. We are the thing that turns it off.
        if mech.apply_preset(self.intake_preset, hold=False):
            self._intake_on = True

    def _stop_intake(self) -> None:
        mech = self.mechanisms.get(self.intake_mech)
        if mech is not None and hasattr(mech, "stop"):
            mech.stop()
        self._intake_on = False

    # --- the tracking gate --------------------------------------------------

    def _track(self, d: Optional[Detection], now: float) -> Optional[Detection]:
        """Advance the confirm/memory gate; return the ball we trust, or None."""
        if d is not None and d.stamp == self._last_stamp:
            # Same cached sample as last tick: no new evidence either way. Hold
            # the streak exactly as it is — this is the script's `not
            # inference_attached` branch.
            return self._trusted()

        if d is None:
            if self._streak >= self.confirm_frames:
                # Already trusted: coast through a short dropout rather than
                # braking on one bad frame.
                if (now - self._confirmed_at) > self.memory_s:
                    self._confirmed = None
                    self._streak = 0
            else:
                # An unconfirmed candidate dies at once. Otherwise two phantoms
                # memory_s apart confirm each other and "consecutive" means
                # nothing.
                self._confirmed = None
                self._streak = 0
            return self._trusted()

        gap = max(0.0, now - self._confirmed_at) if self._confirmed is not None else 0.0
        self._match_tol_now = self.match_tol + self.match_tol_per_s * gap
        same = (
            self._confirmed is not None
            and abs(d.error_x - self._confirmed.error_x) <= self._match_tol_now
            and abs(d.error_y - self._confirmed.error_y) <= self._match_tol_now
        )
        self._streak = self._streak + 1 if same else 1
        self._confirmed = d
        self._confirmed_at = now
        self._last_stamp = d.stamp
        return self._trusted()

    def _trusted(self) -> Optional[Detection]:
        return self._confirmed if self._streak >= self.confirm_frames else None

    # --- the loop -----------------------------------------------------------

    def update(self, dt: float) -> Optional[DriveCommand]:
        if self.detection_provider is None:
            self._stop_intake()
            self._phase = "no_perception"
            return DriveCommand.stopped()

        now = time.monotonic()
        ball = self._track(self.detection_provider(), now)

        if ball is not None:
            self._last_ball_time = now
            self._had_ball = True
            return self._chase(ball, now)
        return self._lost(now)

    def _chase(self, ball: Detection, now: float) -> DriveCommand:
        y = _frame_y(ball.error_y)

        if y >= self.stop_line:
            # Under the hood in a moment. Creep STRAIGHT while swallowing —
            # steering now would sweep the ball out from under the intake.
            self._intake_until = now + self.swallow_run_on_s
            self._start_intake()
            self._phase = "swallow"
            return DriveCommand.arcade(self.swallow_speed, 0.0)

        # The intake is LATCHED, so this branch has to switch it off: seeing the
        # ball ABOVE the line is positive evidence it was not swallowed. The
        # run-on window is still honoured so jitter across the line does not
        # chatter the motor.
        if now > self._intake_until:
            self._stop_intake()
        self._phase = "track"

        # Slow down as the ball gets closer, so the intake can grab it.
        proximity = min(1.0, y / self.stop_line) if self.stop_line > 0 else 1.0
        speed = self.cruise_speed - (self.cruise_speed - self.approach_speed) * proximity
        steer = ball.error_x * self.steering_gain
        return DriveCommand.arcade(speed, steer)

    def _lost(self, now: float) -> DriveCommand:
        elapsed = now - self._last_ball_time

        # Run-on: the ball vanished under the hood, where the camera cannot see
        # it. Keep going briefly rather than stopping on top of it.
        if now <= self._intake_until:
            self._start_intake()
            self._phase = "run_on"
            return DriveCommand.arcade(self.swallow_speed, 0.0)

        # Two independent timers off the same event. A ball leaves the bottom of
        # the FRAME well before it reaches the intake, so after it vanishes it is
        # still on its way in: drive forward briefly to close that gap, and keep
        # the intake running LONGER so a ball already in the throat finishes
        # going in after the robot has stopped.
        #
        # Both gated on had_ball: at startup "no ball yet" looks identical to
        # "ball just lost", and without the gate the robot drives forward the
        # moment the mode is entered.
        pushing = self._had_ball and elapsed <= self.lost_push_s
        intaking = self._had_ball and elapsed <= self.lost_intake_s

        if intaking:
            self._start_intake()
        else:
            self._stop_intake()

        if pushing:
            # Blind: nothing is steering this. Straight and short.
            self._phase = "push"
            return DriveCommand.arcade(self.lost_push_speed, 0.0)

        if elapsed > self.search_after:
            return self._scan(elapsed - self.search_after)

        self._phase = "wait"
        return DriveCommand.stopped()

    def _scan(self, scan_elapsed: float) -> DriveCommand:
        """Sweep, then step forward, then sweep again.

        Spinning in place only ever sees one circle of the field: if the nearest
        ball is outside it, the robot spins forever. Stepping forward between
        sweeps covers new ground. The phase falls out of elapsed time mod the
        cycle, so there is no scan state machine to get stuck in or reset.
        """
        cycle = self.scan_spin_s + self.scan_advance_s
        if cycle <= 0:
            self._phase = "wait"
            return DriveCommand.stopped()
        if (scan_elapsed % cycle) < self.scan_spin_s:
            self._phase = "scan_spin"
            return DriveCommand.arcade(0.0, self.scan_spin_speed)
        # Blind: no ball in sight and nothing steering.
        self._phase = "scan_advance"
        return DriveCommand.arcade(self.scan_advance_speed, 0.0)

    # --- telemetry ----------------------------------------------------------

    def status(self) -> dict:
        """What the dashboard shows instead of the script's OpenCV overlay.

        `track` is the one number worth watching during a match: a robot that
        never leaves `cand 1/2` is failing the gate, not failing to see balls,
        and the fix is match_tol rather than the confidence threshold.
        """
        locked = self._trusted() is not None
        return {
            "phase": self._phase,
            "track": "locked" if locked else f"cand {self._streak}/{self.confirm_frames}",
            "intake": self._intake_on,
            "match_tol": round(self._match_tol_now, 3),
        }
