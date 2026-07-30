"""Distance to a detected object, estimated from its bounding box.

The vision stack reports `size` — the box's HEIGHT as a fraction of the frame
height (see `control/detection.py` for why height and not area). This turns that
into metres, so a routine can say "get within 1.5 m of the bucket" instead of
"drive until the box fills 0.45 of the frame", which is the same instruction
expressed in units nobody can pace out on a field.

--- The model, and why it is one constant ---
Pinhole optics: an object of real height H at distance D projects to a box of
height h_px = f * H / D, where f is the focal length in pixels. Normalising by
the frame height and folding every fixed term together leaves

    distance_m * size = k          (k in metre-fractions)

The frame height cancels, so this needs no resolution, no focal length in pixels
and no lens data sheet — just ONE measured pair. Park the rover a tape-measured
distance from the target, run `tools/detector_selftest.py`, read the printed
`size`, and those two numbers ARE the calibration:

    RS_VISION_RANGE_AT_M=1.5      # where you parked it
    RS_VISION_RANGE_SIZE=0.30     # what the box measured there

--- What this deliberately does not model ---
It assumes every target is the same real height, because k folds H in. That is
true while a build chases one object class and false the moment it chases two:
a cone and a bucket at the same distance give different boxes, and this will
report the same distance for both. It also ignores lens distortion, pitch (a box
seen from an angle is shorter), and the box's own jitter, which at the frame edge
is several percent and lands straight on the estimate.

Which is why this is a placeholder with a seam, not an answer. `Rangefinder` is
the whole interface the controller uses — `distance_m` and `size_at` — so a
regression fitted on real (size, distance) pairs, or a per-label table, drops in
behind those two methods without the controller or the routine schema changing.
Log `size` alongside a tape measure and you have the training set.
"""

from __future__ import annotations

from typing import Optional

# A box taller than this is not a box we will ever see: a target that close is
# clipped by the frame edge, and the detector reports the visible part. Asking to
# stop nearer than this would set an arrival threshold that can never be reached,
# and "never arrives" means "keeps driving forward" — so the ask is clamped and
# said out loud rather than quietly turned into a collision.
_MAX_REACHABLE_SIZE = 0.95


class Rangefinder:
    """Converts between bounding-box height fraction and distance in metres.

    Uncalibrated (either reference at or below zero) it answers None to
    everything, and callers fall back to judging distance in box-height units.
    That is the honest failure: a build that has never been measured has no
    business reporting metres to an operator who will believe them.
    """

    def __init__(self, ref_distance_m: float = 0.0, ref_size: float = 0.0):
        self.ref_distance_m = float(ref_distance_m)
        self.ref_size = float(ref_size)
        self._warned_unreachable = False

    @property
    def calibrated(self) -> bool:
        return self.ref_distance_m > 0.0 and self.ref_size > 0.0

    @property
    def k(self) -> float:
        """The one constant: distance_m * size, in metre-fractions."""
        return self.ref_distance_m * self.ref_size

    def calibrate(self, ref_distance_m: float, ref_size: float) -> None:
        """Re-measure. Live, so the dashboard's sliders take effect on this run."""
        if (ref_distance_m, ref_size) == (self.ref_distance_m, self.ref_size):
            return
        self.ref_distance_m = float(ref_distance_m)
        self.ref_size = float(ref_size)
        self._warned_unreachable = False

    def distance_m(self, size: Optional[float]) -> Optional[float]:
        """Estimated metres to an object whose box measures `size`.

        None when there is no calibration or no size — a FOMO model reports no
        box height at all, and inventing a distance for it is how a robot drives
        confidently into something.
        """
        if size is None or size <= 0.0 or not self.calibrated:
            return None
        return self.k / float(size)

    def size_at(self, distance_m: float) -> Optional[float]:
        """The box height a target would show from `distance_m` away.

        The inverse of `distance_m`, and the direction the controller actually
        uses: an arrival test in box-height units costs one conversion when the
        standoff is set rather than one per frame, and it keeps the arrival latch
        and its hysteresis in the units they were written and tuned in.
        """
        if distance_m <= 0.0 or not self.calibrated:
            return None
        size = self.k / float(distance_m)
        if size > _MAX_REACHABLE_SIZE:
            if not self._warned_unreachable:
                self._warned_unreachable = True
                print(f"[range] {distance_m:.2f} m would need a box filling "
                      f"{size:.2f} of the frame, which is closer than the target "
                      f"stays fully visible — stopping at {_MAX_REACHABLE_SIZE:.2f} "
                      f"(~{self.k / _MAX_REACHABLE_SIZE:.2f} m) instead")
            return _MAX_REACHABLE_SIZE
        return size
