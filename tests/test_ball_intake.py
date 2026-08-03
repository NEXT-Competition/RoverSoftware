"""Ball intake: the parts that are safety properties, not features.

Three of these are the bugs this controller was written from, all found on the
robot rather than by reading code, and all the same shape — an actuator that
latches, so a branch which says nothing keeps it running:

  - the intake must stop when the ball is tracked back ABOVE the collect line
  - the intake must stop on e-stop and on deactivate
  - the blind push must not fire before a ball has ever been seen

The fourth is the timer split: the drive stops before the intake does, because
a ball in the throat is still being collected after the robot has halted.
"""

import time

import pytest

from robot.control.ball_intake import BallIntakeController
from robot.control.detection import Detection


class FakeIntake:
    """Records what the controller commanded, in order."""

    def __init__(self):
        self.power = None
        self.calls = []

    def set_power(self, power, actuator=None):
        self.power = power
        self.calls.append(("set_power", power))
        return True

    def stop(self):
        self.power = None
        self.calls.append(("stop",))


def det(error_x=0.0, error_y=0.0, label="ball", stamp=None):
    return Detection(
        label=label,
        confidence=0.9,
        error_x=error_x,
        error_y=error_y,
        size=0.2,
        stamp=time.monotonic() if stamp is None else stamp,
    )


class Clock:
    """A hand-wound clock. Every behaviour in this controller is a function of
    elapsed time, so reaching a state by sleeping makes the test a race - and
    it loses that race exactly at the cycle boundaries worth testing."""

    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make(**kw):
    intake = FakeIntake()
    seen = {"d": None}
    clock = Clock()
    ctl = BallIntakeController(
        detection_provider=lambda: seen["d"], intake=intake, now_fn=clock, **kw
    )
    ctl.on_activate()
    return ctl, intake, seen, clock


# -- the intake latch -------------------------------------------------------


def test_intake_runs_at_the_collect_line():
    ctl, intake, seen, clock = make()
    seen["d"] = det(error_y=0.5)  # below the 0.4 line
    ctl.update(0.02)
    assert ctl.intake_running()
    assert intake.power == 1.0
    assert ctl.state() == "collect"


def test_intake_stops_when_the_ball_goes_back_above_the_line():
    """REGRESSION. On the robot the intake stayed on while a ball was tracked
    above the line, until it left frame entirely — every branch except the
    chase touched the intake, so the chase inherited whatever was set."""
    ctl, intake, seen, clock = make()
    seen["d"] = det(error_y=0.5)
    ctl.update(0.02)
    assert ctl.intake_running()

    seen["d"] = det(error_y=-0.2)  # lifted back above the line, still in frame
    ctl.update(0.02)
    assert not ctl.intake_running(), "intake latched on while chasing"
    assert ctl.state() == "chase"


def test_estop_stops_the_intake():
    ctl, intake, seen, clock = make()
    seen["d"] = det(error_y=0.5)
    ctl.update(0.02)
    assert ctl.intake_running()
    ctl.on_estop()
    assert not ctl.intake_running()
    assert intake.calls[-1] == ("stop",)


def test_deactivate_stops_the_intake():
    ctl, intake, seen, clock = make()
    seen["d"] = det(error_y=0.5)
    ctl.update(0.02)
    ctl.on_deactivate()
    assert not ctl.intake_running()


# -- blind motion -----------------------------------------------------------


def test_no_blind_push_before_any_ball_is_seen():
    """REGRESSION. no_ball_elapsed is ~0 at startup, which reads as 'just lost
    a ball' — without the had_ball gate the robot lurches forward the moment
    the mode is selected."""
    ctl, intake, seen, clock = make()
    seen["d"] = None
    cmd = ctl.update(0.02)
    assert ctl.state().startswith("search"), ctl.state()
    assert not ctl.intake_running()


def test_push_drives_straight_after_losing_the_ball():
    ctl, intake, seen, clock = make(collect_push_s=1.0, push_speed=0.3)
    seen["d"] = det(error_y=0.5)
    ctl.update(0.02)

    seen["d"] = None
    cmd = ctl.update(0.02)
    assert ctl.state() == "push"
    assert cmd.left == cmd.right, "blind push must be straight"
    assert cmd.left > 0
    assert ctl.intake_running()


def test_intake_outlives_the_drive_push():
    """The drive stops at collect_push_s, the intake at intake_hold_s."""
    ctl, intake, seen, clock = make(collect_push_s=0.05, intake_hold_s=0.20)
    seen["d"] = det(error_y=0.5)
    ctl.update(0.02)

    seen["d"] = None
    clock.advance(0.08)  # past the push, inside the hold
    cmd = ctl.update(0.02)
    assert ctl.state() == "hold"
    assert cmd.left == 0.0 and cmd.right == 0.0, "should have stopped driving"
    assert ctl.intake_running(), "intake should still be collecting"

    clock.advance(0.16)  # past the hold too
    ctl.update(0.02)
    assert not ctl.intake_running()


# -- search -----------------------------------------------------------------


def test_search_alternates_spin_and_advance():
    ctl, _, seen, clock = make(
        collect_push_s=0.0, intake_hold_s=0.0,
        search_spin_s=0.10, search_advance_s=0.05,
    )
    seen["d"] = det(error_y=0.5)
    ctl.update(0.02)
    seen["d"] = None

    clock.advance(0.02)
    ctl.update(0.02)
    assert ctl.state() == "search_spin"

    clock.advance(0.12)  # into the advance leg
    ctl.update(0.02)
    assert ctl.state() == "search_advance"


def test_search_advance_is_straight():
    ctl, _, seen, clock = make(
        collect_push_s=0.0, intake_hold_s=0.0,
        search_spin_s=0.05, search_advance_s=0.10,
    )
    seen["d"] = det(error_y=0.5)
    ctl.update(0.02)
    seen["d"] = None
    clock.advance(0.07)
    cmd = ctl.update(0.02)
    assert ctl.state() == "search_advance"
    assert cmd.left == cmd.right and cmd.left > 0


# -- targeting and steering -------------------------------------------------


def test_non_ball_labels_are_ignored():
    """A bucket in frame is not a target — object_align's job, not this one's."""
    ctl, _, seen, clock = make()
    seen["d"] = det(error_y=0.5, label="blue bucket")
    ctl.update(0.02)
    assert not ctl.intake_running()
    assert ctl.state().startswith("search")


def test_collect_drives_straight_even_when_off_centre():
    """At the mouth a steering correction sweeps the intake past the ball."""
    ctl, _, seen, clock = make()
    seen["d"] = det(error_x=0.6, error_y=0.5)
    cmd = ctl.update(0.02)
    assert ctl.state() == "collect"
    assert cmd.left == cmd.right


def test_far_ball_pivots_before_advancing():
    ctl, _, seen, clock = make(pivot_threshold=0.35)
    seen["d"] = det(error_x=0.8, error_y=-0.5)
    cmd = ctl.update(0.02)
    assert ctl.state() == "chase"
    assert cmd.left == -cmd.right, "large error should pivot in place"


def test_approach_eases_off_as_the_ball_nears_the_line():
    ctl, _, seen, clock = make(chase_speed=0.5, collect_speed=0.2, collect_line=0.4)
    far = ctl._approach_speed(det(error_y=-0.9))
    near = ctl._approach_speed(det(error_y=0.35))
    assert far > near
    assert near == pytest.approx(0.2, abs=0.03)


def test_pid_advances_once_per_detection_not_per_tick():
    """The detector is slower than the loop and detection() is a cached read,
    so the same sample arrives several ticks running."""
    ctl, _, seen, clock = make()
    stamp = time.monotonic()
    seen["d"] = det(error_x=0.2, error_y=-0.5, stamp=stamp)
    ctl.update(0.02)
    first = ctl._steer
    ctl.update(0.02)  # identical sample
    assert ctl._steer == first, "PID advanced on a stale sample"


def test_no_provider_stops_everything():
    ctl = BallIntakeController(detection_provider=None, intake=FakeIntake())
    cmd = ctl.update(0.02)
    assert cmd.left == 0.0 and cmd.right == 0.0
    assert not ctl.intake_running()


def test_runs_with_no_intake_wired():
    """Perception and drive must not depend on a mechanism being present."""
    ctl = BallIntakeController(detection_provider=lambda: det(error_y=0.5))
    ctl.on_activate()
    cmd = ctl.update(0.02)
    assert cmd.left > 0
