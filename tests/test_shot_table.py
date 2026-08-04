"""The measured-shot table, and the angle-to-throttle bridge under it.

Two jobs. The table inverts what you measured (angle -> distance) into what you
need (distance -> angle), and refuses rather than guesses outside the data. The
bridge turns a raw servo angle into an ordinary throttle, so a table collected on
a bench rig drives the real drivetrain path — e-stop, jog timeout and all —
instead of reaching past it to the pin.
"""

import pytest

from robot.config import MotorConfig
from robot.control.shot_table import (
    ShotTable,
    reachable_angles,
    throttle_for_angle,
)


def table():
    """A plausible set: wind the wheel up, the ball goes further."""
    return ShotTable([(10, 1.2), (20, 2.4), (30, 3.9), (40, 5.1)])


# --- inverting the measurement ------------------------------------------------

def test_a_measured_distance_gives_back_its_own_angle():
    t = table()
    assert t.angle_for(1.2) == pytest.approx(10)
    assert t.angle_for(3.9) == pytest.approx(30)
    assert t.angle_for(5.1) == pytest.approx(40)


def test_between_two_rows_it_interpolates():
    t = table()
    # Halfway between (20, 2.4) and (30, 3.9) in DISTANCE is 3.15 m.
    assert t.angle_for(3.15) == pytest.approx(25)


def test_the_covered_range_is_reported():
    assert table().range_m() == (1.2, 5.1)


# --- refusing, rather than guessing -------------------------------------------

def test_past_the_last_row_is_none_not_an_extrapolation():
    """A flywheel is least linear exactly where it saturates, which is where an
    extrapolation would be used."""
    t = table()
    assert t.angle_for(5.2) is None
    assert t.angle_for(20.0) is None


def test_nearer_than_the_first_row_is_also_none():
    assert table().angle_for(0.5) is None


def test_an_unmeasured_distance_is_none():
    assert table().angle_for(None) is None


def test_a_table_with_one_row_is_not_calibrated():
    t = ShotTable([(10, 1.2)])
    assert t.calibrated is False
    assert t.angle_for(1.2) is None


def test_an_empty_table_is_not_calibrated():
    t = ShotTable()
    assert t.calibrated is False
    assert t.range_m() is None


# --- bad data is refused loudly -----------------------------------------------

def test_a_bigger_angle_that_threw_shorter_is_refused():
    """It cannot be inverted — there is no single angle for that distance —
    and it means a bad row, not a curve."""
    t = ShotTable([(10, 1.2), (20, 3.0), (30, 2.5)])
    assert t.calibrated is False
    assert any("bad measurement" in p for p in t.problems)


def test_the_same_angle_twice_is_refused():
    t = ShotTable([(10, 1.2), (10, 2.0)])
    assert t.calibrated is False
    assert any("twice" in p for p in t.problems)


def test_a_malformed_row_is_reported_not_crashed():
    t = ShotTable([(10, 1.2), ("banana",), (30, 3.9)])
    assert any("row 2" in p for p in t.problems)


def test_a_zero_distance_row_is_not_a_throw():
    t = ShotTable([(0, 0.0), (20, 2.4)])
    assert any("not a throw" in p for p in t.problems)


# --- explaining itself --------------------------------------------------------

def test_it_says_which_way_it_was_out_of_range():
    t = table()
    assert "further than" in t.explain(9.0)
    assert "nearer than" in t.explain(0.4)
    assert t.explain(3.0) == ""


def test_it_says_when_there_was_no_range_at_all():
    assert table().explain(None) == "no range to the target"


def test_it_names_the_problem_when_the_table_was_rejected():
    t = ShotTable([(10, 3.0), (20, 1.0)])
    assert "rejected" in t.explain(2.0)


# --- the angle -> throttle bridge ---------------------------------------------

def flywheel(**kw):
    base = dict(channel=8, kind="esc", neutral_angle=5.0,
                max_angle=55.0, min_angle=-45.0, deadband=0.03)
    base.update(kw)
    return MotorConfig(**base)


def test_the_throttle_produces_exactly_the_angle_asked_for():
    """The whole point of the bridge: a table in raw angles has to arrive at
    the pin as the angle it recorded."""
    from robot.drive.motor import ESCMotor
    import os
    os.environ["RS_MOCK_MOTORS"] = "1"
    for inverted in (False, True):
        cfg = flywheel(inverted=inverted)
        motor = ESCMotor(cfg)
        for want in (0.0, 10.0, 25.0, 50.0, -20.0):
            motor.set_throttle(throttle_for_angle(cfg, want))
            assert motor.servo._last == pytest.approx(want, abs=0.01)


def test_inverted_cancels_out():
    """`inverted` describes how the motor is WIRED. A table of measurements must
    not change meaning when somebody rewires it."""
    normal = throttle_for_angle(flywheel(inverted=False), 40.0)
    flipped = throttle_for_angle(flywheel(inverted=True), 40.0)
    assert normal == pytest.approx(-flipped)


def test_an_unreachable_angle_is_refused_not_clamped():
    """The throw is symmetric about neutral, so the endpoints alone do not say
    what is reachable — and a clamped angle is a shot nobody chose."""
    cfg = flywheel(max_angle=20.0, min_angle=-20.0)   # throw 15 -> -10..20
    assert throttle_for_angle(cfg, 50.0) is None
    assert throttle_for_angle(cfg, 20.0) == pytest.approx(1.0)


def test_a_direction_cap_is_refused_because_it_would_distort_the_mapping():
    # Angle 45 needs throttle +0.8, so max_forward is the cap that applies.
    assert throttle_for_angle(flywheel(max_forward=0.8), 45.0) is None
    # A cap on the other direction does not touch this angle, so it still works.
    assert throttle_for_angle(flywheel(max_reverse=0.8), 45.0) == pytest.approx(0.8)


def test_the_cap_is_chosen_by_the_throttle_sign_not_the_command_sign():
    """They are opposite on an inverted motor, and consulting the wrong one
    means the cap check passes on exactly the builds it should catch."""
    # Inverted: angle 45 needs throttle -0.8, so max_REVERSE is what applies.
    assert throttle_for_angle(flywheel(inverted=True, max_reverse=0.8), 45.0) is None
    assert throttle_for_angle(
        flywheel(inverted=True, max_forward=0.8), 45.0) == pytest.approx(-0.8)


def test_reachable_angles_reports_the_symmetric_throw():
    assert reachable_angles(flywheel()) == (-45.0, 55.0)
    assert reachable_angles(flywheel(max_angle=20.0, min_angle=-20.0)) == (-10.0, 20.0)
