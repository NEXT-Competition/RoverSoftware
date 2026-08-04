"""Metric range from a bounding box: the 1/d law, its refusals, and the
standoff-in-metres conversion that hangs off it."""

import importlib.util
import pathlib
import time

from robot.config import VisionConfig
from robot.control.detection import Detection, distance_m, size_at_m
from robot.control.object_align import ObjectAlignController

_spec = importlib.util.spec_from_file_location(
    "detector_selftest",
    pathlib.Path(__file__).resolve().parents[1] / "tools" / "detector_selftest.py")
_selftest = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_selftest)

# A bucket 0.29 m tall, seen at half a frame height, on a calibrated build.
FOCAL = 1.07  # frame heights; ~1 / (2 * tan(25 deg))
HEIGHT = 0.29


def test_inverse_law():
    """Half the box height = twice the distance. This is the whole model."""
    near = distance_m(0.40, FOCAL, HEIGHT)
    far = distance_m(0.20, FOCAL, HEIGHT)
    assert near is not None and far is not None
    assert abs(far - 2 * near) < 1e-9


def test_round_trip_from_a_calibration():
    """Calibrate at a known distance, then predict it back."""
    truth_m, size_at_truth = 3.0, 0.11
    focal = size_at_truth * truth_m / HEIGHT
    assert abs(distance_m(size_at_truth, focal, HEIGHT) - truth_m) < 1e-9


def test_none_when_range_is_unavailable():
    """Never invent a distance — a plausible wrong number is worse than None."""
    assert distance_m(None, FOCAL, HEIGHT) is None  # FOMO: no box size
    assert distance_m(0.0, FOCAL, HEIGHT) is None  # degenerate box, no div-by-0
    assert distance_m(0.4, 0.0, HEIGHT) is None  # uncalibrated focal
    assert distance_m(0.4, FOCAL, 0.0) is None  # target height unknown


def test_median_ignores_an_outlier_frame():
    """One frame where the model boxes half the bucket must not move the answer."""
    assert _selftest._median([0.20, 0.21, 0.22]) == 0.21
    assert _selftest._median([0.20, 0.21, 0.22, 0.10]) == 0.21  # outlier, not evidence


def test_selftest_calibration_inverts_distance_m():
    """What the tool prints must be the exact constant distance_m wants back.

    This is the whole contract between the calibration run and the robot: the
    tool computes size*d/h, the rover computes focal*h/size. If those two ever
    stop being inverses, every reported range is silently wrong.
    """
    truth_m, sizes = 2.5, [0.12, 0.13, 0.125]
    med = _selftest._median(sizes)
    focal = med * truth_m / HEIGHT  # what _range_report prints
    assert abs(distance_m(med, focal, HEIGHT) - truth_m) < 1e-9


# --- standoff in metres ------------------------------------------------------
# The shipped bucket calibration (packaging/robot.env), so these tests fail if
# someone recalibrates without revisiting the stop distance.
BUCKET = dict(focal_frac=1.03, target_height_m=0.368)


def _vision(**kw) -> VisionConfig:
    return VisionConfig(**{**BUCKET, **kw})


def test_standoff_m_converts_to_the_size_the_loop_stops_at():
    """1 m must come back out as 1 m through distance_m — same map, inverted."""
    size = _vision(standoff_m=1.0).resolved_standoff_size()
    assert abs(size - 0.379) < 0.001  # 1.03 * 0.368 / 1.0
    assert abs(distance_m(size, **BUCKET) - 1.0) < 1e-9


def test_standoff_m_falls_back_when_it_cannot_be_resolved():
    """Never leave the robot with no stop threshold — fall back, don't refuse.

    Refusing (returning None and letting the loop never latch) would mean
    driving into the bucket. standoff_size needs no calibration, so it is
    always a safe answer, even if it is not the requested one.
    """
    assert _vision(standoff_m=0.0, standoff_size=0.45).resolved_standoff_size() == 0.45
    # Range calibration cleared, metres still requested.
    assert VisionConfig(standoff_m=1.0, standoff_size=0.45).resolved_standoff_size() == 0.45
    assert VisionConfig(standoff_m=1.0, focal_frac=1.03,
                        standoff_size=0.45).resolved_standoff_size() == 0.45


def test_standoff_closer_than_the_frame_allows_is_clamped():
    """A standoff needing a box taller than the frame would never latch."""
    # 1.03 * 0.368 / 0.1 = 3.79 frame heights — unreachable.
    assert _vision(standoff_m=0.1).resolved_standoff_size() == 1.0


def test_controller_stops_at_one_metre():
    """The integration that matters: metres in, wheels stopped at 1 m.

    Drives the real state machine with detections sized as the detector would
    report them at 1.05 m and 0.95 m — straddling the standoff — and asserts it
    creeps at the first and is stopped at the second. Everything upstream of
    this (the conversion, the config plumbing) is only worth having if this
    holds.
    """
    cfg = _vision(standoff_m=1.0)
    box = {"d": None}
    c = ObjectAlignController(
        detection_provider=lambda: box["d"],
        standoff_size=cfg.resolved_standoff_size(),
        forward_speed=0.25,
    )
    c.on_activate()

    def seen_at(metres):
        """The size the detector reports for our bucket at this distance."""
        return Detection(label="bucket", confidence=0.9, error_x=0.0, error_y=0.0,
                         size=size_at_m(metres, **BUCKET), stamp=time.monotonic())

    box["d"] = seen_at(1.05)
    cmd = c.update(0.02)
    assert cmd.left > 0 and cmd.right > 0, "still short of 1 m — keep creeping"
    assert not c.arrived()

    box["d"] = seen_at(0.95)
    cmd = c.update(0.02)
    assert (cmd.left, cmd.right) == (0.0, 0.0), "past 1 m — stop"
    assert c.arrived()
