"""The shot solver: metres in, flywheel throttle out.

The point of these is not that the arithmetic matches a textbook — it is that
every way of NOT knowing the answer produces None rather than a number. A
ballistics model that guesses is a launcher that throws a ball a distance nobody
chose, and at a field that is indistinguishable from a miss.
"""

import math

import pytest

from robot.config import BallisticsConfig
from robot.control.ballistics import G, Ballistics


def fitted(**kwargs) -> Ballistics:
    """A build somebody has actually measured."""
    cfg = BallisticsConfig(max_rpm=6000.0, wheel_diameter_m=0.1, transfer=0.5)
    for key, value in kwargs.items():
        setattr(cfg, key, value)
    return Ballistics(cfg)


# --- the uncalibrated default -------------------------------------------------

def test_a_stock_build_refuses_to_compute_a_shot():
    """max_rpm of 0 means nobody has measured this flywheel, and the whole
    model switches off rather than converting a guess into a launch."""
    stock = Ballistics(BallisticsConfig())
    assert not stock.calibrated
    assert stock.rpm_for(4.0) is None
    assert stock.shot_for(4.0) is None
    assert stock.in_range(4.0) is False


def test_an_unmeasured_range_is_never_a_green_light():
    """None distance is what a FOMO model or an uncalibrated rangefinder gives,
    and it must not read as 'in range'."""
    assert fitted().in_range(None) is False


# --- the physics --------------------------------------------------------------

def test_a_level_shot_at_45_degrees_matches_the_range_equation():
    """With no height to climb, v^2 = g*d at 45 degrees. This is the one case
    with a closed form simple enough to be an independent check."""
    b = fitted(launch_angle_deg=45.0, launch_height_m=0.5, target_height_m=0.5)
    speed = b.speed_for(9.0)
    assert speed == pytest.approx(math.sqrt(G * 9.0), rel=1e-6)


def test_further_targets_need_a_faster_wheel():
    b = fitted()
    rpms = [b.rpm_for(d) for d in (2.0, 3.0, 5.0, 8.0)]
    assert all(r is not None for r in rpms)
    assert rpms == sorted(rpms)


def test_a_target_above_the_launch_angle_has_no_solution():
    """At a fixed angle the ball is still climbing when it passes a target that
    close, however hard it is hit. Not a computation failure — a real answer."""
    # tan(45) * 0.5 m = 0.5 m of rise available, against 0.9 - 0.3 = 0.6 needed.
    b = fitted(launch_angle_deg=45.0, launch_height_m=0.3, target_height_m=0.9)
    assert b.speed_for(0.5) is None
    assert b.in_range(0.5) is False
    # And the same launcher reaches a target far enough away to have the room.
    assert b.in_range(4.0) is True


def test_a_launcher_pointed_at_the_sky_has_no_horizontal_reach():
    """cos(90) is zero and tan(90) overflows; this must answer None, not raise."""
    assert fitted(launch_angle_deg=90.0).speed_for(3.0) is None


def test_zero_and_negative_distances_answer_none():
    b = fitted()
    assert b.speed_for(0.0) is None
    assert b.speed_for(-2.0) is None


# --- the wheel ----------------------------------------------------------------

def test_rpm_follows_the_wheel_circumference():
    """Same shot, half the wheel: twice the RPM. The surface speed is what
    throws the ball, so the diameter is not a cosmetic setting."""
    small = fitted(wheel_diameter_m=0.05).rpm_for(4.0)
    large = fitted(wheel_diameter_m=0.10).rpm_for(4.0)
    assert small == pytest.approx(2.0 * large, rel=1e-9)


def test_a_worse_transfer_needs_a_faster_wheel():
    """`transfer` is the fraction of surface speed the ball leaves at, so
    halving it doubles the wheel speed the same shot needs."""
    slippy = fitted(transfer=0.25).rpm_for(4.0)
    grippy = fitted(transfer=0.50).rpm_for(4.0)
    assert slippy == pytest.approx(2.0 * grippy, rel=1e-9)


def test_a_shot_faster_than_the_wheel_can_turn_is_out_of_range_not_clamped():
    """The important one. Clamping to full throttle would spin up, fire, and
    drop the ball short — which looks exactly like a miss. None lets the routine
    take its other branch and say the target is too far."""
    b = fitted(max_rpm=2500.0)
    assert b.rpm_for(3.0) is not None
    assert b.rpm_for(30.0) is None
    assert b.in_range(30.0) is False


def test_power_is_the_rpm_as_a_fraction_of_the_wheels_top_speed():
    b = fitted(max_rpm=6000.0)
    assert b.power_for(3000.0) == pytest.approx(0.5)
    assert b.power_for(6000.0) == pytest.approx(1.0)


def test_power_never_falls_below_the_esc_floor():
    """A brushless ESC asked for 4% may not commutate at all, so the wheel
    would sit still while the routine believed it was spinning."""
    b = fitted(max_rpm=6000.0, idle_power=0.15)
    assert b.power_for(60.0) == pytest.approx(0.15)


def test_power_above_full_throttle_is_none_rather_than_one():
    assert fitted(max_rpm=6000.0).power_for(9000.0) is None


def test_shot_for_returns_the_pair_from_one_distance():
    """Both numbers from a single call, so a caller cannot pair a fresh RPM
    with a stale throttle."""
    b = fitted()
    rpm, power = b.shot_for(4.0)
    assert rpm == pytest.approx(b.rpm_for(4.0))
    assert power == pytest.approx(b.power_for(rpm))


# --- live tuning --------------------------------------------------------------

def test_editing_the_config_changes_the_next_shot():
    """The model holds the config OBJECT, which is what makes 'fire, watch it
    land, nudge transfer, fire again' a loop with no restart in it."""
    cfg = BallisticsConfig(max_rpm=6000.0, wheel_diameter_m=0.1, transfer=0.5)
    b = Ballistics(cfg)
    before = b.rpm_for(4.0)
    cfg.transfer = 0.25
    assert b.rpm_for(4.0) == pytest.approx(2.0 * before, rel=1e-9)
