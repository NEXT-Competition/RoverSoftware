"""What the vision stack hands the alignment controller.

This is the perception contract, and it lives in `control/` for the same reason
`DriveCommand` does: it's the vocabulary the controller speaks, not a detail of
any particular sensor. `sensors/detector.py` imports it; nothing in `control/`
imports `sensors/`, so the dependency only ever points one way.

Deliberately pure data with no third-party imports, so the controller (and its
tests) run on a laptop with no camera, no OpenCV, and no Edge Impulse.

--- On `size` being the range proxy ---
`size` is the bounding box's HEIGHT as a fraction of the model input height, not
its area. Height degrades roughly linearly with distance and survives a target
that's clipped by the frame edge or partly occluded; area does neither (it falls
off quadratically and collapses the moment a corner leaves the frame).

It is Optional because FOMO models cannot provide it. FOMO doesn't emit true
bounding boxes — it emits centroids with fixed, cell-sized boxes, so every "box"
is identical in size regardless of how near the object is. When `size` is None
the controller degrades to align-only rather than inventing a distance.

--- On `stamp` ---
The monotonic time the frame was CLASSIFIED, not the time it was read. The
controller uses it two ways: to age out stale detections, and to know when a
genuinely new sample has arrived so its PID advances once per detection rather
than once per control tick (the detector and the control loop run at different,
unrelated rates).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class Detection:
    """One object sighting, expressed in the controller's units."""

    label: str
    confidence: float  # 0..1, as reported by the model
    # Normalized offsets from frame center, both in [-1, 1].
    # error_x: 0 = centered, +1 = target at the right edge.
    # error_y: 0 = centered, +1 = target at the bottom edge.
    error_x: float
    error_y: float
    # Bounding box height / model input height, in (0, 1]. None on FOMO models,
    # which cannot report object size. None => no range information available.
    size: Optional[float]
    stamp: float  # time.monotonic() when this frame was classified
