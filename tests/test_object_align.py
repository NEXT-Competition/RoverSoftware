"""Tests for the object_align state machine.

Every branch here is pure logic reachable with a stub provider — no camera, no
model, no motors — which is the whole point of the injected-provider design
(docs/ARCHITECTURE.md, "Injected sensing").

Worth testing rather than eyeballing on a rover: the arrival latch and the search
give-up are exactly the behaviours whose bugs look like "the robot twitched at
the cone" or "the robot span until the battery died" — expensive to reproduce in
a field and trivial to assert here.

    pytest tests/
"""

from __future__ import annotations

import time
from typing import Dict, Optional

import pytest

from robot.control.detection import Detection
from robot.control.object_align import ObjectAlignController


def det(error_x=0.0, size=0.2, stamp=None, label="cone", conf=0.9):
    """A Detection with a fresh stamp by default, so each one looks new."""
    return Detection(
        label=label,
        confidence=conf,
        error_x=error_x,
        error_y=0.0,
        size=size,
        stamp=time.monotonic() if stamp is None else stamp,
    )


def make(**kw):
    """Controller with a settable detection, activated (so _last_seen is now)."""
    box: Dict[str, Optional[Detection]] = {"d": None}
    c = ObjectAlignController(detection_provider=lambda: box["d"], **kw)
    c.on_activate()
    return c, box


# --- no perception -----------------------------------------------------------

def test_no_provider_holds_still():
    c = ObjectAlignController()  # nothing injected
    cmd = c.update(0.02)
    assert (cmd.left, cmd.right) == (0.0, 0.0)


# --- pivot vs approach -------------------------------------------------------

def test_far_off_center_pivots_in_place():
    c, box = make(pivot_threshold=0.25, forward_speed=0.25)
    box["d"] = det(error_x=0.8, size=0.1)
    cmd = c.update(0.02)
    # Pure pivot: tracks equal and opposite, no net forward component.
    assert cmd.left == pytest.approx(-cmd.right)
    assert cmd.left + cmd.right == pytest.approx(0.0)


def test_centered_drives_forward():
    c, box = make(pivot_threshold=0.25, forward_speed=0.25)
    box["d"] = det(error_x=0.0, size=0.1)
    cmd = c.update(0.02)
    assert cmd.left + cmd.right > 0  # net forward


def test_no_size_never_advances():
    """FOMO gives no size — face the target, but never drive blind at it."""
    c, box = make(forward_speed=0.25)
    box["d"] = det(error_x=0.0, size=None)
    cmd = c.update(0.02)
    assert cmd.left + cmd.right == pytest.approx(0.0)


def test_approach_disabled_never_advances():
    c, box = make(approach=False, forward_speed=0.25)
    box["d"] = det(error_x=0.0, size=0.1)
    cmd = c.update(0.02)
    assert cmd.left + cmd.right == pytest.approx(0.0)


# --- the arrival latch -------------------------------------------------------

def test_stops_at_standoff():
    c, box = make(standoff_size=0.45)
    box["d"] = det(error_x=0.0, size=0.5)  # bigger than standoff => close enough
    cmd = c.update(0.02)
    assert (cmd.left, cmd.right) == (0.0, 0.0)
    assert c.arrived()


def test_arrival_latches_through_dither():
    """A size dithering just under standoff must NOT restart the approach.

    Without the latch this is a 10 Hz forward/stop lurch at the target.
    """
    c, box = make(standoff_size=0.45, standoff_hysteresis=0.05)
    box["d"] = det(error_x=0.0, size=0.46)
    c.update(0.02)
    assert c.arrived()
    # Dips a hair below standoff but well inside the hysteresis band: stay put.
    box["d"] = det(error_x=0.0, size=0.44)
    cmd = c.update(0.02)
    assert (cmd.left, cmd.right) == (0.0, 0.0)
    assert c.arrived()


def test_arrival_releases_when_target_really_shrinks():
    """Target genuinely receded (someone moved it) -> approach again."""
    c, box = make(standoff_size=0.45, standoff_hysteresis=0.05, forward_speed=0.25)
    box["d"] = det(error_x=0.0, size=0.5)
    c.update(0.02)
    assert c.arrived()
    box["d"] = det(error_x=0.0, size=0.2)  # far outside the hysteresis band
    cmd = c.update(0.02)
    assert not c.arrived()
    assert cmd.left + cmd.right > 0


# --- target loss: grace, search, give up -------------------------------------

def test_brief_dropout_holds_still():
    """Inside search_after we ride it out rather than lurching into a sweep."""
    c, box = make(search_after=0.5, search_speed=0.25)
    box["d"] = None
    cmd = c.update(0.02)  # on_activate() just set _last_seen = now
    assert (cmd.left, cmd.right) == (0.0, 0.0)


def test_searches_after_grace_period():
    c, box = make(search_after=0.5, search_speed=0.25, search_timeout=10.0)
    box["d"] = None
    c._last_seen = time.monotonic() - 1.0  # past the grace period
    cmd = c.update(0.02)
    assert cmd.left == pytest.approx(-cmd.right)
    assert abs(cmd.left) > 0  # actually rotating


def test_search_turns_toward_last_sighting():
    """Sweeping back the way it went beats always sweeping one way."""
    c, box = make(search_after=0.5, search_speed=0.25)
    box["d"] = det(error_x=0.8)  # last seen to the RIGHT
    c.update(0.02)
    box["d"] = None
    c._last_seen = time.monotonic() - 1.0
    right_cmd = c.update(0.02)

    c2, box2 = make(search_after=0.5, search_speed=0.25)
    box2["d"] = det(error_x=-0.8)  # last seen to the LEFT
    c2.update(0.02)
    box2["d"] = None
    c2._last_seen = time.monotonic() - 1.0
    left_cmd = c2.update(0.02)

    # Opposite sweep directions.
    assert right_cmd.left * left_cmd.left < 0


def test_search_gives_up():
    """Never spin forever: an unattended rover doing that is a safety problem."""
    c, box = make(search_after=0.5, search_speed=0.25, search_timeout=10.0)
    box["d"] = None
    c._last_seen = time.monotonic() - 100.0  # long past search_after + timeout
    cmd = c.update(0.02)
    assert (cmd.left, cmd.right) == (0.0, 0.0)


def test_search_speed_zero_disables_search():
    c, box = make(search_speed=0.0)
    box["d"] = None
    c._last_seen = time.monotonic() - 1.0
    cmd = c.update(0.02)
    assert (cmd.left, cmd.right) == (0.0, 0.0)


def test_loss_clears_arrival():
    c, box = make(standoff_size=0.45)
    box["d"] = det(error_x=0.0, size=0.5)
    c.update(0.02)
    assert c.arrived()
    box["d"] = None
    c.update(0.02)
    assert not c.arrived()


# --- the multi-rate PID ------------------------------------------------------

def test_same_sample_holds_steer():
    """The loop sees the same cached detection many ticks in a row; the PID must
    advance once per DETECTION, not once per tick (else the derivative reads
    zero, zero, zero, then spikes)."""
    c, box = make()
    d = det(error_x=0.5, size=0.1)
    box["d"] = d
    first = c.update(0.02)
    for _ in range(4):
        again = c.update(0.02)
        assert again.left == pytest.approx(first.left)
        assert again.right == pytest.approx(first.right)


def test_new_stamp_advances_steer():
    c, box = make()
    box["d"] = det(error_x=0.1, size=0.1, stamp=time.monotonic())
    c.update(0.02)
    box["d"] = det(error_x=0.9, size=0.1, stamp=time.monotonic() + 0.1)
    cmd = c.update(0.02)
    assert abs(cmd.left) > 0  # responded to the much larger error


def test_yaw_rate_is_scaled_into_normalized_units():
    """error_x is normalized [-1,1] over hfov_deg, NOT degrees like waypoint.py.

    Passing a raw deg/s yaw-rate as the derivative would inflate the D term by
    2/hfov (25-60x) and shake the robot apart on the first sighting. This pins
    the scaling: with kp=0 the entire output IS the D term.

    Driven above pivot_threshold on purpose, so the command is a pure pivot
    (throttle=0) and steer is recoverable as (left-right)/2 without a forward
    component or arcade()'s clamp muddying it.
    """
    from robot.control.pid import PID

    rate = 15.0  # deg/s; small enough that the mixed command never clamps
    hfov = 60.0
    c, box = make(
        hfov_deg=hfov,
        pivot_threshold=0.25,
        rate_provider=lambda: rate,
        pid=PID(kp=0.0, kd=1.0, out_limit=10.0),
    )
    box["d"] = det(error_x=0.8, size=0.1)  # > pivot_threshold => pure pivot
    cmd = c.update(0.02)
    expected = -rate * (2.0 / hfov)  # == -0.5, NOT -15.0
    steer = (cmd.left - cmd.right) / 2.0
    assert steer == pytest.approx(expected, abs=1e-6)


def test_no_integral_kick_after_a_long_stop():
    """Resuming after sitting at standoff must not dump a huge integral step.

    The PID advances on inter-detection dt. If the stamp from before a long stop
    survived, dt_det on resume would span the whole stop (seconds or minutes),
    and integral += error * dt_det would slam the output to its limit on the
    first tick. Invisible at the default ki=0 — which is exactly why it's worth a
    test rather than trusting a field tune to find it.
    """
    from robot.control.pid import PID

    c, box = make(standoff_size=0.45, pid=PID(kp=0.0, ki=1.0, kd=0.0, out_limit=0.8))
    t = time.monotonic()
    # Approach FIRST, so the controller records a stamp. (Arriving on the very
    # first detection would leave _last_stamp unset and hide the bug.)
    box["d"] = det(error_x=0.5, size=0.1, stamp=t)
    c.update(0.02)
    # Now arrive.
    box["d"] = det(error_x=0.0, size=0.5, stamp=t + 0.1)
    c.update(0.02)
    assert c.arrived()
    # Sit here a long time, then the target recedes and we resume.
    box["d"] = det(error_x=0.5, size=0.1, stamp=t + 60.0)
    cmd = c.update(0.02)
    assert not c.arrived()
    steer = (cmd.left - cmd.right) / 2.0
    # ki=1, error=0.5: one 0.02s tick => ~0.01. A 60s dt_det saturates at 0.8.
    assert abs(steer) < 0.1


def test_no_rate_provider_still_works():
    """No IMU (or --no-imu): heading_rate() returns None, so the PID falls back
    to finite-differencing. Must not crash."""
    c, box = make(rate_provider=None)
    box["d"] = det(error_x=0.5, size=0.1)
    assert c.update(0.02) is not None


def test_aligned_flag():
    c, box = make(aligned_tolerance=0.05)
    box["d"] = det(error_x=0.01, size=0.1)
    c.update(0.02)
    assert c.aligned()
    box["d"] = det(error_x=0.5, size=0.1)
    c.update(0.02)
    assert not c.aligned()
