"""The two verbs that let a routine work a shot out for itself.

`spin_up` is the only action in the vocabulary that COMPUTES something instead
of relaying a number the operator typed, and `in_range` is the only condition
that asks whether a shot is possible rather than what the robot can see. Both
lean on control/ballistics.py, and both have to fail the same way it does:
silently, without spinning anything, whenever the answer is not known.

Nothing here touches hardware — a fake align controller supplies the range and a
fake mechanism records the throttle, which is the same shape the rest of the
routine tests use.
"""

import pytest

from robot.config import BallisticsConfig
from robot.control.ballistics import Ballistics
from robot.routine.actions import compile_action
from robot.routine.conditions import RoutineContext, compile_condition


class FakeAlign:
    """Just the slice `spin_up` and `in_range` read: how far away it is."""

    def __init__(self, distance=None):
        self.distance = distance

    def distance_m(self):
        return self.distance

    def last_detection(self):
        return None


class FakeFlywheel:
    def __init__(self):
        self.power = None
        self.actuator = "unset"

    def set_power(self, power, actuator=None):
        self.power = power
        self.actuator = actuator
        return True


class FakePulse:
    """A mechanism with no set_power — an intake's servo, or the launcher."""

    def stop(self):
        pass


def fitted() -> Ballistics:
    return Ballistics(BallisticsConfig(
        max_rpm=6000.0, wheel_diameter_m=0.1, transfer=0.5))


def context(distance=None, ballistics=None, mech=None) -> RoutineContext:
    return RoutineContext(
        controllers={"shooter_align": FakeAlign(distance)},
        mechanisms={"flywheel": mech if mech is not None else FakeFlywheel()},
        ballistics=ballistics,
    )


def run_spin_up(ctx, **spec):
    effect, problems, verb = compile_action({"do": "spin_up", "mech": "flywheel",
                                             **spec})
    assert not problems, problems
    assert verb == "spin_up"
    effect(ctx)
    return ctx.mechanisms["flywheel"]


# --- spin_up: the happy path -------------------------------------------------

def test_it_sets_the_throttle_the_measured_range_calls_for():
    ctx = context(distance=4.0, ballistics=fitted())
    flywheel = run_spin_up(ctx)
    expected = fitted().shot_for(4.0)[1]
    assert flywheel.power == pytest.approx(expected)


def test_a_further_target_gets_more_throttle():
    near = run_spin_up(context(distance=2.0, ballistics=fitted()))
    far = run_spin_up(context(distance=8.0, ballistics=fitted()))
    assert far.power > near.power


def test_a_fixed_distance_overrides_the_measurement():
    """The bench case: no camera pointed at anything, but the wheel should still
    spin to a known shot."""
    ctx = context(distance=None, ballistics=fitted())
    flywheel = run_spin_up(ctx, distance_m=5.0)
    assert flywheel.power == pytest.approx(fitted().shot_for(5.0)[1])


def test_an_actuator_can_be_named_for_a_multi_wheel_launcher():
    ctx = context(distance=4.0, ballistics=fitted())
    flywheel = run_spin_up(ctx, actuator="top")
    assert flywheel.actuator == "top"


def test_no_actuator_means_every_wheel_on_the_mechanism():
    ctx = context(distance=4.0, ballistics=fitted())
    assert run_spin_up(ctx).actuator is None


# --- spin_up: every way of not knowing ---------------------------------------

def test_nothing_spins_without_a_ballistics_model():
    ctx = context(distance=4.0, ballistics=None)
    assert run_spin_up(ctx).power is None


def test_nothing_spins_on_an_unmeasured_flywheel():
    """The default config. A robot nobody has measured must not launch."""
    ctx = context(distance=4.0, ballistics=Ballistics(BallisticsConfig()))
    assert run_spin_up(ctx).power is None


def test_nothing_spins_without_a_range_to_the_target():
    """No detection, or an uncalibrated rangefinder. The alternative — some
    fallback power — throws the ball a distance nobody chose."""
    ctx = context(distance=None, ballistics=fitted())
    assert run_spin_up(ctx).power is None


def test_nothing_spins_when_the_shot_is_out_of_reach():
    ctx = context(distance=200.0, ballistics=fitted())
    assert run_spin_up(ctx).power is None


def test_a_missing_mechanism_is_survivable():
    """The layout changed under a stored routine. Report it, do not raise —
    an action that throws mid-run must not take the robot down."""
    ctx = RoutineContext(controllers={"shooter_align": FakeAlign(4.0)},
                         mechanisms={}, ballistics=fitted())
    effect, problems, _ = compile_action({"do": "spin_up", "mech": "flywheel"})
    assert not problems
    effect(ctx)  # no exception


def test_a_mechanism_that_cannot_hold_a_power_is_survivable():
    ctx = context(distance=4.0, ballistics=fitted(), mech=FakePulse())
    effect, _, _ = compile_action({"do": "spin_up", "mech": "flywheel"})
    effect(ctx)  # no exception, and nothing to assert but that


def test_the_mechanism_is_required():
    _, problems, _ = compile_action({"do": "spin_up"})
    assert problems and "mech" in problems[0]


# --- in_range ----------------------------------------------------------------

def predicate():
    pred, problems = compile_condition({"when": "in_range"})
    assert not problems
    return pred


def test_in_range_is_true_for_a_shot_the_launcher_can_take():
    assert predicate()(context(distance=4.0, ballistics=fitted())) is True


def test_in_range_is_false_for_a_target_too_far_to_reach():
    assert predicate()(context(distance=200.0, ballistics=fitted())) is False


def test_in_range_is_false_for_a_target_too_close_to_arc_into():
    """A fixed launch angle cannot throw a short, high shot: at 0.5 m there is
    less climb available than the bucket rim needs. Out of range in the near
    direction, which a plain distance comparison would call fine."""
    b = Ballistics(BallisticsConfig(
        max_rpm=6000.0, wheel_diameter_m=0.1, transfer=0.5,
        launch_angle_deg=45.0, launch_height_m=0.3, target_height_m=0.9))
    assert predicate()(context(distance=0.5, ballistics=b)) is False


def test_in_range_is_false_when_nothing_is_known():
    """No model, no controller, no detection. A state gating its shot on this
    holds fire on an unmeasured build rather than firing blind."""
    assert predicate()(context(distance=4.0, ballistics=None)) is False
    assert predicate()(context(distance=None, ballistics=fitted())) is False
    assert predicate()(RoutineContext(ballistics=fitted())) is False
