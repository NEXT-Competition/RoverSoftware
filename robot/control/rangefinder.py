"""Distance to a detected object: measured when it can be, inferred when it can't.

Two sensors answer "how far away is that", and they fail in opposite directions.

    the CAMERA  reports a bounding box for a named object anywhere in a 50-66
                degree field of view, out to whatever the model can resolve —
                but a box height is not a distance, and turning one into the
                other needs a constant that depends on how tall the object
                really is.
    the SONAR   measures a real distance, but only to whatever is inside a ~15
                degree cone straight ahead, no further than a few metres, and it
                has no idea WHAT it hit — a chair leg in front of the bucket
                answers exactly like the bucket.

So this module does the obvious thing with that pair: when the camera says the
target is centred and the sonar is in range, the sonar's metres are the answer
AND the two together are a free calibration sample. Fitted over a handful of
samples, the camera then reports honest metres well past anything the sonar can
reach. The sonar teaches; the camera extrapolates.

--- The model, and why it is one constant ---
Pinhole optics: an object of real height H at distance D projects to a box of
height h_px = f * H / D, where f is the focal length in pixels. Normalising by
the frame height and folding every fixed term together leaves

    distance_m * size = k          (k in metre-fractions)

The frame height cancels, so this needs no resolution, no focal length in pixels
and no lens data sheet — just ONE measured pair.

There are two ways to get that pair, and they produce the same number:

    by hand    Park the rover a tape-measured distance from the target, run
               `tools/detector_selftest.py`, read the printed `size`. Those two
               numbers ARE the calibration:
                   RS_VISION_RANGE_AT_M=1.5    # where you parked it
                   RS_VISION_RANGE_SIZE=0.30   # what the box measured there
    by sonar   `observe()`, below. Every frame where the target is centred and
               within the ultrasonic's range hands over the same pair without
               anyone holding a tape measure, and it keeps doing it — so the
               calibration tracks the build instead of dating from the one
               afternoon somebody measured it.

--- k folds in the object's height, so it is PER LABEL ---
This is the limitation the hand-calibrated version could not do anything about:
H is inside k, so a cone and a bucket at the same distance give different boxes,
and one constant reports the same distance for both. Learned samples are keyed
on the detector's label, which fixes it for every label the rover has actually
seen from a measurable distance. A label with no fit of its own falls back to
the hand-set pair — never to another label's, because that is precisely the
error the table exists to avoid.

--- What poisons a fit, and what is done about it ---
A bad pair is worse than no pair: it is a wrong number that looks measured. So a
sample has to survive all of:

    centred     The target must be inside the sonar's cone, or the sonar is
                describing something else entirely. `error_x` is normalised over
                the camera's FOV, which makes this arithmetic: a target at
                error_x sits |error_x| * hfov/2 degrees off the axis.
    unclipped   A box touching the top or bottom edge of the frame is a box
                whose height was cut off, and an understated height reads as a
                greater distance. The box spans error_y +/- size in normalised
                units, so an edge is a subtraction away.
    simultaneous  The detector runs at ~10 fps and the sonar at ~15 Hz, on
                unrelated clocks. A reading taken 300 ms after the frame was
                classified describes a different moment, and on a moving rover a
                different distance. Pairs whose stamps disagree are dropped.
    still-ish   Even a matched pair carries the vision pipeline's own latency,
                which is unknown and one-signed (the frame is always older than
                its classification). Learning only while barely moving keeps
                that from becoming a systematic bias in k.
    plausible   Once a fit exists, a sample implying a wildly different k is far
                more likely to be the sonar seeing something nearer than the
                target than a sudden change in the laws of optics.

What survives goes into a short rolling window and the fit is its MEDIAN, so a
single bad sample that clears every gate still cannot move the answer much, and
the window means a fit can still follow a genuine change rather than being
frozen by its first few samples.

--- What this still does not model ---
Lens distortion, pitch (a box seen from an angle is shorter), and the box's own
jitter, which at the frame edge is several percent. And the honest big one: a
label the rover has never seen up close still rides the hand-set constant, with
whatever that constant assumed about the object's height.
"""

from __future__ import annotations

import collections
from typing import Callable, Deque, Dict, Optional, Tuple

from .detection import Detection

# A box taller than this is not a box we will ever see: a target that close is
# clipped by the frame edge, and the detector reports the visible part. Asking to
# stop nearer than this would set an arrival threshold that can never be reached,
# and "never arrives" means "keeps driving forward" — so the ask is clamped and
# said out loud rather than quietly turned into a collision.
_MAX_REACHABLE_SIZE = 0.95

# Half the ultrasonic's beam width. An HC-SR04's usable cone is about 15 degrees
# total; this is the number that decides whether a detection sitting off to one
# side can possibly be the thing the sonar is timing an echo off.
#
# Deliberately a constant rather than a setting. It is a property of the
# transducer, not a preference, and a build that widened it to cover more of the
# frame would be choosing to pair the camera's target with whatever else happens
# to be in the way — which is the one failure this whole file is written around.
SONAR_HALF_ANGLE_DEG = 7.5

# How far apart a detection's stamp and a sonar sample's stamp may be and still
# describe the same moment. One detector frame at ~10 fps.
MAX_PAIR_SKEW_S = 0.15

# Learning gates. A box past this height is close enough that clipping is
# likely; `_MAX_REACHABLE_SIZE` is the harder limit and this is the cautious one,
# because a calibration sample can afford to be picky in a way an arrival test
# cannot.
MAX_LEARN_SIZE = 0.8
# The box spans error_y +/- size in normalised frame units, so this much reach
# toward an edge means the height was probably cut off by it.
EDGE_MARGIN = 0.98
# A new sample may imply a k this many times bigger or smaller than an
# established fit before it is treated as the sonar looking at something else.
MAX_DISAGREEMENT = 2.0
# Samples kept per label. Long enough for the median to be robust, short enough
# that a fit still follows a real change (a different-sized target of the same
# label, a camera that got knocked).
FIT_WINDOW = 15

# Below this commanded throttle the rover counts as still enough to learn from.
# Not zero: the useful samples arrive during a creeping approach, which is
# exactly when a target is centred and inside the sonar's few metres.
MAX_LEARN_THROTTLE = 0.35


class _Fit:
    """One label's calibration: a rolling window of k samples and their median."""

    def __init__(self, window: int = FIT_WINDOW):
        self._samples: Deque[float] = collections.deque(maxlen=window)

    def add(self, k: float) -> None:
        self._samples.append(k)

    @property
    def n(self) -> int:
        return len(self._samples)

    @property
    def k(self) -> float:
        ordered = sorted(self._samples)
        return ordered[len(ordered) // 2]

    def agrees_with(self, k: float) -> bool:
        """Is this k in the same world as the fit? True when there is no fit yet."""
        if not self._samples:
            return True
        established = self.k
        return (established / MAX_DISAGREEMENT) <= k <= (established * MAX_DISAGREEMENT)


class Rangefinder:
    """Converts between bounding-box height fraction and distance in metres.

    Uncalibrated — no hand-set pair, and nothing learned for this label — it
    answers None to everything, and callers fall back to judging distance in
    box-height units. That is the honest failure: a build that has never been
    measured has no business reporting metres to an operator who will believe
    them.
    """

    def __init__(self, ref_distance_m: float = 0.0, ref_size: float = 0.0,
                 hfov_deg: float = 50.0, min_samples: int = 8,
                 learn: bool = True, prefer_sonar: bool = True):
        self.ref_distance_m = float(ref_distance_m)
        self.ref_size = float(ref_size)
        # The camera's field of view, needed to turn a normalised error_x into
        # degrees off the axis. Kept in step with VisionConfig.hfov_deg by
        # `calibrate`, since the two describe the same lens.
        self.hfov_deg = float(hfov_deg)
        # Samples before a learned fit is trusted enough to answer with. Small,
        # because the gates above are strict and the median is robust — the cost
        # of a high number is a rover that drives past the only distances it
        # could have learned from.
        self.min_samples = int(min_samples)
        self.learn = bool(learn)
        self.prefer_sonar = bool(prefer_sonar)

        self._warned_unreachable = False
        # sonar_provider() -> (metres, monotonic stamp), or None
        self._sonar: Optional[Callable[[], Optional[Tuple[float, float]]]] = None
        self._fits: Dict[str, _Fit] = {}
        self._announced: set = set()
        self._last_stamp: Optional[float] = None   # dedupe: one sample per frame
        self._source = ""                          # what answered last: sonar|vision

    # --- calibration --------------------------------------------------------

    @property
    def calibrated(self) -> bool:
        """True if the HAND-SET pair is usable. Unchanged meaning, on purpose:
        several callers ask this to decide whether metres exist at all, and a
        learned fit is per label rather than a property of the whole build."""
        return self.ref_distance_m > 0.0 and self.ref_size > 0.0

    @property
    def k(self) -> float:
        """The hand-set constant: distance_m * size, in metre-fractions."""
        return self.ref_distance_m * self.ref_size

    def k_for(self, label: str = "") -> Optional[float]:
        """The constant to use for this label: learned if there is one, else the
        hand-set pair, else None."""
        fit = self._fits.get(label)
        if fit is not None and fit.n >= self.min_samples:
            return fit.k
        return self.k if self.calibrated else None

    def learned(self, label: str = "") -> Optional[Tuple[float, int]]:
        """(k, samples) for a label the sonar has taught us about, else None."""
        fit = self._fits.get(label)
        return (fit.k, fit.n) if fit is not None and fit.n else None

    def calibrate(self, ref_distance_m: float, ref_size: float,
                  hfov_deg: Optional[float] = None) -> None:
        """Re-measure by hand. Live, so the dashboard's sliders take effect now.

        Learned fits are deliberately NOT dropped: they are measurements, the
        hand-set pair is a fallback for labels that have none, and throwing away
        the former because someone nudged the latter would be backwards.
        """
        if hfov_deg is not None:
            self.hfov_deg = float(hfov_deg)
        if (ref_distance_m, ref_size) == (self.ref_distance_m, self.ref_size):
            return
        self.ref_distance_m = float(ref_distance_m)
        self.ref_size = float(ref_size)
        self._warned_unreachable = False

    def forget(self) -> None:
        """Drop every learned fit. For the tests, and for a rover that has been
        rebuilt around a different target."""
        self._fits.clear()
        self._announced.clear()
        self._last_stamp = None

    # --- the sonar ----------------------------------------------------------

    def set_sonar_provider(
            self, provider: Optional[Callable[[], Optional[Tuple[float, float]]]]) -> None:
        """Wire in a `() -> (metres, stamp) | None` reading.

        A plain callable rather than the sensor itself, because nothing in
        `control/` imports `sensors/` — the same rule `Detection` exists to keep.
        """
        self._sonar = provider

    def sonar_for(self, detection: Optional[Detection]) -> Optional[float]:
        """The sonar's metres, IF it can be looking at this detection.

        None whenever the pairing cannot be justified: no sonar, no echo, the
        reading and the frame describe different moments, or the target is
        outside the beam. Every one of those is "we do not know", and none of
        them is "the target is far away".
        """
        if self._sonar is None or detection is None or self.hfov_deg <= 0:
            return None
        reading = self._sonar()
        if reading is None:
            return None
        distance, stamp = reading
        if abs(detection.stamp - stamp) > MAX_PAIR_SKEW_S:
            return None
        # error_x is normalised so that 1.0 is the frame edge, i.e. half the
        # field of view. Anything outside the sonar's own cone is a target the
        # echo cannot be coming from.
        off_axis_deg = abs(detection.error_x) * (self.hfov_deg / 2.0)
        if off_axis_deg > SONAR_HALF_ANGLE_DEG:
            return None
        return distance

    def observe(self, detection: Optional[Detection],
                throttle: float = 0.0) -> bool:
        """Learn from one (box height, measured distance) pair, if it survives.

        Called once per control tick with whatever the detector currently
        reports; returns True when a sample was actually taken. Cheap enough for
        the loop — one cached read and a handful of comparisons — and it dedupes
        on the detection's stamp, so a sample is taken per FRAME rather than per
        tick however far apart those rates drift.
        """
        if not self.learn or detection is None or detection.size is None:
            return False
        if detection.stamp == self._last_stamp:
            return False        # this frame has already been turned into a sample
        if abs(throttle) > MAX_LEARN_THROTTLE:
            return False        # moving too fast for the pipeline's own latency
        size = float(detection.size)
        if not 0.0 < size <= MAX_LEARN_SIZE:
            return False
        # The box spans error_y +/- size in normalised units, so this is "does
        # it touch the top or the bottom of the frame". A clipped box reports a
        # height it does not have, which reads as a distance it is not at.
        if abs(detection.error_y) + size >= EDGE_MARGIN:
            return False
        distance = self.sonar_for(detection)
        if distance is None or distance <= 0.0:
            # Deliberately WITHOUT marking the frame as considered: the sonar
            # pings on its own clock, and a reading that had not arrived yet on
            # this tick may well have by the next one. The frame is only spent
            # once it has actually produced a pair.
            return False

        self._last_stamp = detection.stamp
        k = distance * size
        fit = self._fits.setdefault(detection.label, _Fit())
        if not fit.agrees_with(k):
            # Far more likely the sonar found something nearer than the target
            # than that the optics changed. Dropped silently: this is a normal
            # event in a cluttered room, not a fault.
            return False
        fit.add(k)
        self._announce(detection.label, fit)
        return True

    def _announce(self, label: str, fit: _Fit) -> None:
        """Say once, per label, when a fit becomes usable.

        Worth a line in the journal because it is the moment a build stops
        reporting a guess: everything downstream — the metre standoff, the
        distance an operator reads, the flywheel speed `spin_up` picks — starts
        answering off a measurement from here on. It also prints the pair to
        write down, since a learned fit lives in memory and a restart begins
        again from whatever was configured by hand.
        """
        if fit.n != self.min_samples or label in self._announced:
            return
        self._announced.add(label)
        k = fit.k
        print(f"[Range] calibrated {label!r} from the ultrasonic: k={k:.3f} "
              f"({fit.n} samples). A box filling half the frame is now "
              f"{k / 0.5:.2f} m. To keep it across a restart, set "
              f"vision.range_at_m={k / 0.45:.2f} and vision.range_size=0.45.")

    # --- reading ------------------------------------------------------------

    def distance_for(self, detection: Optional[Detection]) -> Optional[float]:
        """Metres to THIS detection: measured if possible, inferred otherwise.

        The order is the whole point. A sonar reading that has passed the gates
        is a measurement of the object in front of the rover; the box-height
        estimate is an inference from a constant. Prefer the measurement, fall
        back to the inference, and answer None when there is neither — which is
        also how a FOMO model (no box height at all) gets a distance for the
        first time, since the sonar never needed one.
        """
        if detection is None:
            self._source = ""
            return None
        if self.prefer_sonar:
            distance = self.sonar_for(detection)
            if distance is not None:
                self._source = "sonar"
                return distance
        estimate = self.distance_m(detection.size, detection.label)
        self._source = "vision" if estimate is not None else ""
        return estimate

    @property
    def source(self) -> str:
        """"sonar" | "vision" | "" — where the last `distance_for` came from.

        On the radio as one character, because "0.8 m" from a transducer and
        "0.8 m" from a constant somebody typed are different claims and an
        operator deciding whether to believe the number needs to know which it
        is looking at.
        """
        return self._source

    def distance_m(self, size: Optional[float],
                   label: str = "") -> Optional[float]:
        """Estimated metres to an object whose box measures `size`.

        None when there is no calibration for it or no size — a FOMO model
        reports no box height at all, and inventing a distance for it is how a
        robot drives confidently into something.
        """
        if size is None or size <= 0.0:
            return None
        k = self.k_for(label)
        return None if k is None else k / float(size)

    def size_at(self, distance_m: float, label: str = "") -> Optional[float]:
        """The box height a target would show from `distance_m` away.

        The inverse of `distance_m`, and the direction the controller actually
        uses: an arrival test in box-height units costs one conversion when the
        standoff is set rather than one per frame, and it keeps the arrival latch
        and its hysteresis in the units they were written and tuned in.
        """
        k = self.k_for(label)
        if distance_m <= 0.0 or k is None:
            return None
        size = k / float(distance_m)
        if size > _MAX_REACHABLE_SIZE:
            if not self._warned_unreachable:
                self._warned_unreachable = True
                print(f"[range] {distance_m:.2f} m would need a box filling "
                      f"{size:.2f} of the frame, which is closer than the target "
                      f"stays fully visible — stopping at {_MAX_REACHABLE_SIZE:.2f} "
                      f"(~{k / _MAX_REACHABLE_SIZE:.2f} m) instead")
            return _MAX_REACHABLE_SIZE
        return size

    def status(self, label: str = "") -> dict:
        """What the range estimate is standing on, for a telemetry frame.

        Tiny and only what cannot be inferred from the distance itself: which
        sensor answered, and how many samples the label's fit has (so an
        operator can watch it converge, and can tell "learning" from "learned").
        """
        t: dict = {}
        if self._source:
            t["src"] = self._source[0]      # "s" | "v"
        fit = self._fits.get(label)
        if fit is not None and fit.n:
            t["kn"] = fit.n
        return t
