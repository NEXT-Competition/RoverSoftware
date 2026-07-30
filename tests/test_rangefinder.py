"""Bounding box -> metres, and what the estimate refuses to guess at.

The model itself is one line of arithmetic; what is worth pinning down is the
behaviour around its edges, because every one of them decides whether a robot
stops or keeps driving:

  * uncalibrated must answer None, not a plausible-looking number,
  * a standoff nearer than the target can be seen from must not become an
    arrival threshold that can never be reached,
  * and the two directions must actually be inverses, since the controller
    converts one way and the operator reads the other.

    pytest tests/
"""

from __future__ import annotations

import pytest

from robot.control.detection import Detection
from robot.control.object_align import ObjectAlignController
from robot.control.rangefinder import Rangefinder


def test_the_calibration_pair_reproduces_itself():
    """The reference measurement must come back out unchanged."""
    r = Rangefinder(ref_distance_m=1.5, ref_size=0.30)
    assert r.distance_m(0.30) == pytest.approx(1.5)
    assert r.size_at(1.5) == pytest.approx(0.30)


def test_distance_and_size_are_inverses():
    r = Rangefinder(2.0, 0.25)
    for metres in (0.8, 1.0, 2.0, 4.0, 9.0):
        assert r.distance_m(r.size_at(metres)) == pytest.approx(metres)


def test_a_nearer_target_reads_as_a_bigger_box():
    """Sanity on the direction of the relationship — the sign error that would
    make a robot reverse away from what it was told to approach."""
    r = Rangefinder(1.0, 0.4)
    assert r.distance_m(0.8) < r.distance_m(0.4) < r.distance_m(0.2)


@pytest.mark.parametrize("at_m,size", [(0.0, 0.4), (1.0, 0.0), (-1.0, 0.4)])
def test_uncalibrated_answers_none(at_m, size):
    """No measurement means no metres. Not a default, not a guess: an operator
    who is shown 2.3 m will believe 2.3 m."""
    r = Rangefinder(at_m, size)
    assert not r.calibrated
    assert r.distance_m(0.4) is None
    assert r.size_at(2.0) is None


def test_no_box_height_means_no_distance():
    """FOMO reports no size at all, and inventing one drives into things."""
    r = Rangefinder(1.0, 0.4)
    assert r.distance_m(None) is None
    assert r.distance_m(0.0) is None


def test_an_unreachable_standoff_is_clamped_not_dropped(capsys):
    """A stop distance closer than the target stays visible from.

    Returning the raw >1 size would set an arrival threshold no frame can ever
    satisfy, and a robot that never arrives is a robot that keeps driving
    forward. Clamp to something reachable and say so.
    """
    r = Rangefinder(1.0, 0.4)  # k = 0.4, so 0.2 m would need a box of 2.0
    size = r.size_at(0.2)
    assert size is not None and size <= 1.0
    assert "closer than the target stays fully visible" in capsys.readouterr().out


def test_the_unreachable_warning_does_not_repeat(capsys):
    """It is read on every standoff conversion; one line, not a flood."""
    r = Rangefinder(1.0, 0.4)
    for _ in range(5):
        r.size_at(0.2)
    assert capsys.readouterr().out.count("[range]") == 1


def test_recalibrating_re_arms_the_warning(capsys):
    """New numbers, new chance to be wrong about them."""
    r = Rangefinder(1.0, 0.4)
    r.size_at(0.2)
    capsys.readouterr()
    r.calibrate(2.0, 0.4)
    r.size_at(0.2)
    assert "[range]" in capsys.readouterr().out


# --- what the controller does with it ---------------------------------------

def _det(size):
    return Detection(label="cone", confidence=0.9, error_x=0.0, error_y=0.0,
                     size=size, stamp=1.0)


def test_standoff_in_metres_sets_the_arrival_threshold():
    """1.0 m against a k of 0.45 means arriving at a box height of 0.45."""
    c = ObjectAlignController(rangefinder=Rangefinder(1.0, 0.45),
                              standoff_size=0.9, standoff_m=1.0)
    assert c.standoff_threshold() == pytest.approx(0.45)
    assert not c._check_arrived(0.40)
    assert c._check_arrived(0.46)


def test_metres_are_ignored_without_a_calibration():
    """An unmeasured build stops where it always did, rather than not stopping."""
    c = ObjectAlignController(rangefinder=Rangefinder(0.0, 0.0),
                              standoff_size=0.6, standoff_m=1.0)
    assert c.standoff_threshold() == pytest.approx(0.6)


def test_metres_are_ignored_without_a_rangefinder():
    c = ObjectAlignController(standoff_size=0.6, standoff_m=1.0)
    assert c.standoff_threshold() == pytest.approx(0.6)


def test_zero_metres_means_use_the_size_standoff():
    """0 is 'not specified', which is what every routine predating the field
    says — it must not read as 'stop at zero metres'."""
    c = ObjectAlignController(rangefinder=Rangefinder(1.0, 0.45),
                              standoff_size=0.6, standoff_m=0.0)
    assert c.standoff_threshold() == pytest.approx(0.6)


def test_the_arrival_latch_still_uses_hysteresis_on_a_metre_standoff():
    """The latch is in size units whichever way the threshold was set; without
    it the robot lurches forward and stops at the detector's frame rate."""
    c = ObjectAlignController(rangefinder=Rangefinder(1.0, 0.45),
                              standoff_m=1.0, standoff_hysteresis=0.05)
    assert c._check_arrived(0.46)
    assert c._check_arrived(0.42)   # under the threshold, still latched
    assert not c._check_arrived(0.39)  # genuinely shrunk: released


def test_distance_is_reported_for_the_current_target():
    c = ObjectAlignController(detection_provider=lambda: _det(0.45),
                              rangefinder=Rangefinder(1.0, 0.45))
    c.on_activate()
    c.update(0.02)
    assert c.distance_m() == pytest.approx(1.0)


def test_distance_is_none_with_nothing_in_view():
    c = ObjectAlignController(detection_provider=lambda: None,
                              rangefinder=Rangefinder(1.0, 0.45))
    c.on_activate()
    c.update(0.02)
    assert c.distance_m() is None
