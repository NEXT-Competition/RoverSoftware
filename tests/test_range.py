"""Metric range from a bounding box: the 1/d law and its refusals."""

from robot.control.detection import distance_m

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
