"""Object-alignment controller: face a detected object, approach it, stop short.

Turns the robot to face an object the vision stack has detected, creeps toward
it, and stops at a standoff distance — using a PID on the object's horizontal
offset in the camera frame.

This is intentionally decoupled from *how* the object is detected: you inject a
`detection_provider` callable returning an `Optional[Detection]` (None when
nothing is seen). Today that's Edge Impulse (`robot/sensors/detector.py`), but
the controller neither knows nor cares — it runs and unit-tests fine with no
camera, and a different detector could be swapped in without touching this file.

--- The state machine ---
    no provider            -> stop (perception not wired up)
    no detection           -> search: rotate toward where we last saw it
    arrived (latched)      -> stop
    |error_x| > pivot      -> turn in place (point-then-go, as waypoint.py does)
    no size available      -> turn in place only; never advance blind (FOMO)
    otherwise              -> creep forward while the PID trims the heading

--- How near is "arrived" ---
Natively, in box-height units: stop once the box fills `standoff_size` of the
frame. That is what the detector measures and what the arrival latch is tuned in,
but it is not a distance anyone can pace out, so `standoff_m` says the same thing
in metres and a `Rangefinder` converts it (once, on the way in — see
`standoff_threshold`). A routine state sets that per state, which is how "close
in on the bucket, then hang back from the goal" is one document.

Metres are best-effort and never load-bearing: with no rangefinder, no
calibration, or a model that reports no box height at all, the metre standoff is
dropped and `standoff_size` decides. The fallback tightens nothing and loosens
nothing — it is exactly where this controller stopped before distances existed.

--- Why the PID advances on detection stamps, not control ticks ---
The detector and the control loop run at unrelated rates, and `detection()` is a
cached read — so the loop generally sees the SAME sample several ticks in a row
(or, if the loop is slower than the detector, skips samples entirely). Feeding
that to a fixed-dt PID breaks the derivative: the error appears frozen for N
ticks and then jumps, so a `(error - prev)/dt` term reads zero, zero, zero, then
a spike N times too large.

So the PID advances only when `stamp` changes, using the true inter-detection
dt, and the steer is zero-order held in between. That's correct at any ratio of
loop rate to detector rate, in either direction.
"""

from __future__ import annotations

import time
from typing import Callable, Dict, Optional

from .commands import DriveCommand
from .controller import Controller
from .detection import Detection
from .pid import PID
from .rangefinder import Rangefinder

# detection_provider() -> latest Detection, or None if nothing is currently seen
DetectionProvider = Callable[[], Optional[Detection]]
# rate_provider() -> yaw rate in deg/s (CW+), or None
RateProvider = Callable[[], Optional[float]]


class ObjectAlignController(Controller):
    name = "object_align"

    def __init__(
        self,
        detection_provider: Optional[DetectionProvider] = None,
        rate_provider: Optional[RateProvider] = None,
        forward_speed: float = 0.25,
        pivot_threshold: float = 0.25,  # |error_x| above this => turn in place
        aligned_tolerance: float = 0.05,  # |error_x| below this => "aligned"
        approach: bool = True,  # False => face the object but never advance
        standoff_size: float = 0.45,  # stop once bbox height fraction reaches this
        standoff_hysteresis: float = 0.05,  # must shrink by this to un-arrive
        rangefinder: Optional[Rangefinder] = None,  # bbox height <-> metres
        standoff_m: float = 0.0,  # stop this far off instead; 0 = use standoff_size
        search_speed: float = 0.25,  # 0 disables the search sweep
        search_after: float = 0.5,  # ride out dropouts this long before searching
        search_timeout: float = 10.0,  # give up (stop) after searching this long
        hfov_deg: float = 50.0,  # EFFECTIVE post-crop horizontal FOV
        pid: Optional[PID] = None,
    ):
        self.detection_provider = detection_provider
        self.rate_provider = rate_provider
        self.forward_speed = forward_speed
        self.pivot_threshold = pivot_threshold
        self.aligned_tolerance = aligned_tolerance
        self.approach = approach
        self.standoff_size = standoff_size
        self.standoff_hysteresis = standoff_hysteresis
        # Range estimation is optional and stays optional: with no rangefinder,
        # or an uncalibrated one, `standoff_m` cannot be honoured and everything
        # falls back to standoff_size — the behaviour this controller had before
        # distances existed. Arrival is never silently loosened.
        self.rangefinder = rangefinder
        self.standoff_m = standoff_m
        self.search_speed = search_speed
        self.search_after = search_after
        self.search_timeout = search_timeout
        self.hfov_deg = hfov_deg
        # Gains are tuned for ~100-200ms of perception dead time, which is what
        # actually limits stability here. Anything hotter oscillates: the robot
        # steers on an error it measured two frames ago. Start low, not high.
        self.pid = pid or PID(kp=0.5, ki=0.0, kd=0.05, out_limit=0.8)

        self._arrived = False
        self._last_seen = 0.0
        self._last_error_x = 0.0  # which way to sweep when the target vanishes
        self._last_stamp: Optional[float] = None
        self._steer = 0.0
        self._aligned = False
        # The sample this tick acted on (None when nothing is currently seen).
        # Subclasses need it to answer "aligned on *what*" — shooter_align gates
        # firing on whether the detection carries size, which aligned()/arrived()
        # alone can't distinguish from "no target at all".
        self._last_detection: Optional[Detection] = None

    def set_detection_provider(self, provider: DetectionProvider) -> None:
        self.detection_provider = provider

    def set_rate_provider(self, provider: RateProvider) -> None:
        self.rate_provider = provider

    def on_activate(self) -> None:
        self.pid.reset()
        self._arrived = False
        self._last_stamp = None
        self._steer = 0.0
        self._aligned = False
        self._last_detection = None
        # Treat activation as "just saw it" so we sit still for search_after
        # instead of immediately spinning on entry to the mode.
        self._last_seen = time.monotonic()

    def aligned(self) -> bool:
        """True when the target is centered within aligned_tolerance (telemetry)."""
        return self._aligned

    def arrived(self) -> bool:
        """True once stopped at the standoff distance (telemetry)."""
        return self._arrived

    def last_detection(self) -> Optional[Detection]:
        """The sample the last update() acted on, or None if nothing was seen."""
        return self._last_detection

    def distance_m(self) -> Optional[float]:
        """Estimated metres to the current target, or None.

        None covers all three ways this can be unknown and they are worth not
        conflating: nothing is in view, the model reports no box height (FOMO),
        or nobody has calibrated the rangefinder. All three mean "do not print a
        number an operator would steer by".
        """
        if self._last_detection is None or self.rangefinder is None:
            return None
        return self.rangefinder.distance_m(self._last_detection.size)

    def standoff_threshold(self) -> float:
        """The box height arrival is judged against, in size units.

        A metre standoff is converted HERE, once, rather than converting every
        frame's size into metres to compare: the arrival latch and its hysteresis
        were written and tuned in box-height units, and the two tests are
        equivalent anyway — size rises monotonically as distance falls.

        Falls back to `standoff_size` whenever the metres cannot be honoured, so
        an uncalibrated build stops where it always did instead of not stopping.
        """
        if self.standoff_m > 0.0 and self.rangefinder is not None:
            size = self.rangefinder.size_at(self.standoff_m)
            if size is not None:
                return size
        return self.standoff_size

    def pid_traces(self) -> Dict[str, dict]:
        """The steering loop, for the tuning graphs.

        The setpoint is 0 and always will be: "aligned" means the target sits at
        the centre of the frame, so the error IS the normalised horizontal
        offset and there is no separate measurement to report. Reported in the
        loop's own units — [-1, 1] across the lens, not degrees — because those
        are the units the gains are expressed in, and a graph whose y-axis
        doesn't match the numbers you type is a graph that mistunes a robot.
        """
        return {"align.pid": self.pid.trace(setpoint=0.0)}

    def update(self, dt: float) -> Optional[DriveCommand]:
        if self.detection_provider is None:
            return DriveCommand.stopped()  # no perception wired up

        now = time.monotonic()
        d = self.detection_provider()

        if d is None:
            # Lost, stale, or the detector thread died — the provider ages out
            # its own samples, so None covers all three.
            self.pid.reset()
            self._arrived = False
            self._aligned = False
            self._last_stamp = None
            self._steer = 0.0
            self._last_detection = None
            return self._search(now)

        self._last_seen = now
        self._last_detection = d
        self._last_error_x = d.error_x
        self._aligned = abs(d.error_x) <= self.aligned_tolerance

        # Arrived: latched, so we don't chatter forward/stop on the boundary.
        if self.approach and d.size is not None and self._check_arrived(d.size):
            self.pid.reset()
            self._steer = 0.0
            # Drop the stamp too, not just the PID state: we may sit here for
            # minutes, and if the target later recedes and we resume, a dt_det
            # measured from before the whole stop would be enormous. Harmless at
            # the default ki=0, but it would dump a huge integral step the first
            # time someone tunes ki up. Falling back to the tick dt (as on first
            # acquisition) is the honest answer.
            self._last_stamp = None
            return DriveCommand.stopped()

        steer = self._steer_for(d, dt)

        # Point-then-go: swing onto the bearing before committing to drive, the
        # same arbitration waypoint.py uses (there in degrees, here normalized).
        if abs(d.error_x) > self.pivot_threshold:
            return DriveCommand.arcade(0.0, steer)
        if not self.approach or d.size is None:
            # No range information (FOMO): face it, but never drive blind at it.
            return DriveCommand.arcade(0.0, steer)
        return DriveCommand.arcade(self.forward_speed, steer)

    def _check_arrived(self, size: float) -> bool:
        """Latch arrival, releasing only once the target genuinely shrinks.

        Without the latch, `size` dithering around the threshold makes the robot
        lurch forward and stop at the detector's frame rate.
        """
        threshold = self.standoff_threshold()
        if self._arrived:
            # Stay arrived until it's meaningfully smaller (it drove off / we
            # got bumped), not merely a hair under the threshold.
            self._arrived = size > (threshold - self.standoff_hysteresis)
            return self._arrived
        if size >= threshold:
            self._arrived = True
        return self._arrived

    def _steer_for(self, d: Detection, dt: float) -> float:
        """PID on horizontal error, advanced once per detection (see module docstring)."""
        if self._last_stamp is not None and d.stamp == self._last_stamp:
            return self._steer  # same sample as last tick — hold

        dt_det = (d.stamp - self._last_stamp) if self._last_stamp is not None else dt
        if dt_det <= 0:
            return self._steer

        # Derivative-on-measurement from the IMU, as waypoint.py does — but the
        # units differ and that matters. waypoint's error is in DEGREES, so it
        # passes -yaw_rate (deg/s) directly. Our error is NORMALIZED to [-1,1]
        # across hfov_deg, so the same rate must be scaled into those units:
        #   error_x spans 2.0 over hfov_deg  =>  d(error_x)/dt = -rate * 2/hfov
        # Passing raw -rate here would inflate the D term by ~25-60x and the
        # robot would shake itself apart on the first sighting.
        rate = self.rate_provider() if self.rate_provider is not None else None
        derivative = None
        if rate is not None and self.hfov_deg > 0:
            derivative = -rate * (2.0 / self.hfov_deg)

        self._steer = self.pid.update(d.error_x, dt_det, derivative)
        self._last_stamp = d.stamp
        return self._steer

    def _search(self, now: float) -> DriveCommand:
        """Sweep to reacquire a lost target, then give up.

        Rotates back toward the side we last saw the object on, which roughly
        halves reacquisition versus always sweeping one way.
        """
        if self.search_speed <= 0:
            return DriveCommand.stopped()
        since = now - self._last_seen
        if since < self.search_after:
            # Brief dropout (one bad frame, a flicker of confidence) — sit still
            # rather than lurching into a sweep we'll immediately abort.
            return DriveCommand.stopped()
        if since > self.search_after + self.search_timeout:
            # Give up. An unattended robot spinning in place forever because
            # someone walked off with the target is the failure the e-stop
            # exists to catch; don't author it deliberately.
            return DriveCommand.stopped()
        direction = 1.0 if self._last_error_x >= 0 else -1.0
        return DriveCommand.arcade(0.0, direction * self.search_speed)
