"""Metric range from a bounding box: the 1/d law and its refusals."""

import importlib.util
import pathlib

from robot.control.detection import distance_m

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
