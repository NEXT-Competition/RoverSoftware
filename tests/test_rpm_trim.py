"""The wheel-speed loop: what it corrects, and — mostly — what it refuses to.

This is the one thing in the stack that can ADD throttle nobody asked for, so
the load-bearing tests here are the negative ones. A speed controller that keeps
integrating against a sensor which has stopped reporting does not fail quietly;
it winds that side to full throttle, and the rover lunges.
"""

import pytest

from robot.config import TrimConfig
from robot.control.rpm_trim import RpmTrim

DT = 0.02  # one tick at 50 Hz


def trim(**kw) -> RpmTrim:
    cfg = TrimConfig(**kw)
    return RpmTrim(cfg)


def settle(loop: RpmTrim, left: float, right: float,
           left_rpm: float, right_rpm: float, ticks: int = 50):
    """Run the loop against a FIXED pair of speeds and return its last output.

    Fixed on purpose: the plant is not being simulated here, so this measures
    which way the loop pushes and how far, not whether it converges.
    """
    out = (left, right)
    for i in range(ticks):
        out = loop.apply(left, right, left_rpm, right_rpm, DT, now=i * DT)
    return out


# --- off, and the two ways of having nothing to work with --------------------

def test_off_is_a_passthrough():
    loop = trim(mode="off")
    assert loop.apply(0.5, 0.5, 100.0, 80.0, DT) == (0.5, 0.5)
    assert loop.engaged is False


def test_a_missing_encoder_is_not_a_measurement_of_zero():
    """None means "no reading". If that were treated as 0 rpm, `velocity` mode
    would answer a robot with no encoders by flooring both motors."""
    loop = trim(mode="velocity", max_rpm=200.0)
    assert loop.apply(0.5, 0.5, None, None, DT) == (0.5, 0.5)
    assert loop.apply(0.5, 0.5, 100.0, None, DT) == (0.5, 0.5)
    assert loop.engaged is False


def test_nothing_is_trimmed_below_the_engage_threshold():
    loop = trim(mode="match", min_throttle=0.05)
    assert loop.apply(0.01, 0.01, 5.0, 0.0, DT) == (0.01, 0.01)


# --- match: hold the two sides to each other ---------------------------------

def test_match_speeds_up_the_slow_side_and_eases_the_fast_one():
    """Split, not one-sided: correcting on one motor alone would change the
    average speed every time it corrected, which feels like a throttle that
    goes soft in a straight line."""
    loop = trim(mode="match")
    left, right = settle(loop, 0.5, 0.5, left_rpm=90.0, right_rpm=110.0)
    assert left > 0.5   # the slow side gets more
    assert right < 0.5  # the fast side gets less
    assert (left - 0.5) == pytest.approx(0.5 - right)


def test_match_gets_the_direction_right_in_reverse():
    """The regression this is here for: raw signed RPM makes "right is faster"
    the MORE negative number, so a naive subtraction corrects backwards and the
    rover curves harder and harder while reversing."""
    loop = trim(mode="match")
    # Reversing, right side turning faster (more negative) than the left.
    left, right = settle(loop, -0.5, -0.5, left_rpm=-90.0, right_rpm=-110.0)
    assert left < -0.5   # slow side pushed harder into reverse
    assert right > -0.5  # fast side eased off


def test_match_does_nothing_while_a_turn_is_commanded():
    """A commanded difference is one you asked for. Correcting it would fight
    the steering — the robot would resist every turn it was told to make."""
    loop = trim(mode="match", straight_tolerance=0.05)
    assert loop.apply(0.8, 0.2, 160.0, 40.0, DT) == (0.8, 0.2)
    assert loop.engaged is False


def test_match_holds_its_integral_across_a_turn_rather_than_forgetting():
    """The mismatch between two motors is a physical property that is still
    true after the corner. Resetting on every steering twitch would mean the
    trim never converges on anything."""
    loop = trim(mode="match")
    settle(loop, 0.5, 0.5, 90.0, 110.0)
    learned = loop.apply(0.5, 0.5, 90.0, 110.0, DT, now=99.0)
    loop.apply(0.8, 0.2, 160.0, 40.0, DT, now=99.1)  # a turn
    resumed = loop.apply(0.5, 0.5, 90.0, 110.0, DT, now=99.2)
    assert resumed[0] == pytest.approx(learned[0], abs=0.01)


def test_match_needs_no_calibration_to_work():
    """Its error is a DIFFERENCE, so a wrong counts-per-rev scales both sides
    identically and cancels. That is the whole reason to reach for match first."""
    loop = trim(mode="match")
    a = settle(loop, 0.5, 0.5, 90.0, 110.0)
    # The same mismatch measured through an encoder scaled 10x wrong.
    scaled = trim(mode="match")
    b = settle(scaled, 0.5, 0.5, 900.0, 1100.0)
    assert a[0] > 0.5 and b[0] > 0.5  # both push the same way


# --- velocity: hold each side to throttle x max_rpm --------------------------

def test_velocity_pushes_a_slow_wheel_toward_its_own_setpoint():
    loop = trim(mode="velocity", max_rpm=200.0)
    left, right = settle(loop, 0.5, 0.5, left_rpm=80.0, right_rpm=100.0)
    assert left > 0.5   # 80 is well under the 100 rpm asked for
    assert right == pytest.approx(0.5, abs=1e-6)  # exactly on target


def test_velocity_corrects_during_a_turn_where_match_would_not():
    """The reason to accept the calibration cost: a commanded difference is
    still a pair of setpoints, so each side is held to its own."""
    loop = trim(mode="velocity", max_rpm=200.0)
    left, _ = settle(loop, 0.8, 0.2, left_rpm=100.0, right_rpm=40.0)
    assert left > 0.8


def test_velocity_without_a_max_rpm_declines_rather_than_dividing_by_nothing():
    loop = trim(mode="velocity", max_rpm=0.0)
    assert loop.apply(0.5, 0.5, 80.0, 100.0, DT) == (0.5, 0.5)


# --- authority and safety ----------------------------------------------------

def test_the_correction_cannot_exceed_the_loops_output_limit():
    """The trim is a correction, not a second throttle."""
    loop = trim(mode="velocity", max_rpm=200.0)
    loop.cfg.pid.out_limit = 0.1
    left, _ = settle(loop, 0.5, 0.5, left_rpm=0.0, right_rpm=100.0, ticks=5)
    # Rejected before the stall timer can trip: 5 ticks is 0.1 s.
    assert left <= 0.5 + 0.1 + 1e-9


def test_the_output_stays_inside_the_throttle_range():
    loop = trim(mode="velocity", max_rpm=200.0)
    left, right = settle(loop, 1.0, 1.0, left_rpm=0.1, right_rpm=0.1, ticks=5)
    assert -1.0 <= left <= 1.0 and -1.0 <= right <= 1.0


def test_a_wheel_commanded_but_never_turning_trips_the_stall_fault():
    """The disconnected-encoder case, and the reason this loop is safe to ship.

    Left reads a standstill while being told to drive. Without the trip the
    integrator would grow without bound and pin that side at full throttle.
    """
    loop = trim(mode="velocity", max_rpm=200.0, stall_seconds=0.5)
    for i in range(100):  # 2 s at 50 Hz
        out = loop.apply(0.5, 0.5, 0.0, 100.0, DT, now=i * DT)
    assert loop.fault == "left"
    assert out == (0.5, 0.5)  # open loop from here on


def test_the_stall_fault_stays_latched_until_the_drivetrain_stops():
    """Not until the wheel happens to turn again: an encoder that came loose
    must not be able to re-arm the loop while the rover is still moving."""
    loop = trim(mode="match", stall_seconds=0.5)
    for i in range(100):
        loop.apply(0.5, 0.5, 0.0, 100.0, DT, now=i * DT)
    assert loop.fault == "left"
    assert loop.apply(0.5, 0.5, 100.0, 100.0, DT, now=10.0) == (0.5, 0.5)
    loop.reset()  # what Drivetrain.stop() calls
    assert loop.fault == ""


def test_a_wheel_that_is_not_being_asked_to_move_is_not_stalled():
    loop = trim(mode="match", stall_seconds=0.5)
    for i in range(100):
        loop.apply(0.0, 0.0, 0.0, 0.0, DT, now=i * DT)
    assert loop.fault == ""


def test_the_stall_check_can_be_switched_off():
    loop = trim(mode="velocity", max_rpm=200.0, stall_seconds=0.0)
    for i in range(100):
        loop.apply(0.5, 0.5, 0.0, 100.0, DT, now=i * DT)
    assert loop.fault == ""


# --- observability -----------------------------------------------------------

def test_status_reports_the_corrections_only_while_engaged():
    """"Trimming by nothing" and "not trimming" are the same two numbers and
    different situations, so the absent case is absent rather than zero."""
    loop = trim(mode="match")
    loop.apply(0.5, 0.5, 90.0, 110.0, DT)
    assert loop.status()["mode"] == "match"
    assert "tl" in loop.status()
    loop.apply(0.0, 0.0, 0.0, 0.0, DT)
    assert "tl" not in loop.status()


def test_the_trace_is_in_rpm_so_the_graph_matches_the_gains():
    loop = trim(mode="match")
    loop.apply(0.5, 0.5, 90.0, 110.0, DT)
    trace = loop.trace()
    assert trace["sp"] == 0.0        # match aims at zero mismatch
    assert trace["m"] == pytest.approx(20.0)  # right is 20 rpm faster


def test_no_trace_when_the_loop_is_not_running():
    """A frozen curve left on screen is a graph that lies about the robot."""
    loop = trim(mode="off")
    loop.apply(0.5, 0.5, 90.0, 110.0, DT)
    assert loop.trace() is None
