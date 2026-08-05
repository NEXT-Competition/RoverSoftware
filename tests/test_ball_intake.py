"""The autonomous ball intake, ported from Team Northeast's auto_chassis.py.

What is worth pinning here is not that it drives — it is the behaviour that was
learned on the robot and is invisible from the code: the confirm gate that stops
it chasing phantoms, the growing match radius that stopped it scanning forever,
the two lost-ball timers, and the intake being switched OFF again on every path
that can leave the mode.

Runs with no camera and no HAT: the controller takes a detection provider and a
mechanism registry, and both are trivially faked.
"""

import os

import pytest

os.environ.setdefault("RS_MOCK_MOTORS", "1")

from robot.control.ball_intake import BallIntakeController
from robot.control.detection import Detection


class FakeMech:
    """Enough of PowerMechanism for the controller to drive."""

    def __init__(self):
        self.preset = None
        self.stops = 0
        self.holds = []

    def apply_preset(self, name, hold=False):
        self.preset = name
        self.holds.append(hold)
        return True

    def stop(self):
        self.preset = None
        self.stops += 1


def det(error_x=0.0, error_y=0.0, stamp=1.0, label="ball"):
    return Detection(label=label, confidence=0.9, error_x=error_x,
                     error_y=error_y, size=0.1, stamp=stamp)


def make(**kw):
    mech = FakeMech()
    c = BallIntakeController(mechanisms={"intake": mech}, **kw)
    c.on_activate()
    return c, mech


def feed(c, detections, dt=0.02):
    """Push samples through update(), returning the last drive command."""
    cmd = None
    for d in detections:
        c.detection_provider = lambda d=d: d
        cmd = c.update(dt)
    return cmd


# --- the confirm gate --------------------------------------------------------

def test_one_sighting_is_not_enough_to_act_on():
    """A false positive flickers; a real ball persists. Confidence cannot tell
    them apart, so the robot must see it twice before it moves."""
    c, _ = make(confirm_frames=2)
    cmd = feed(c, [det(stamp=1.0)])
    assert (cmd.left, cmd.right) == (0.0, 0.0)


def test_two_sightings_in_a_row_confirm_the_ball():
    c, _ = make(confirm_frames=2)
    cmd = feed(c, [det(stamp=1.0), det(stamp=1.1)])
    assert cmd.left > 0 and cmd.right > 0


def test_the_same_cached_sample_does_not_advance_the_streak():
    """detection() is a cached read, so the loop sees one sample for several
    ticks. Counting ticks would confirm a phantom in milliseconds — only a
    changed stamp is evidence the sensor really ran the network again."""
    c, _ = make(confirm_frames=2)
    same = det(stamp=1.0)
    cmd = feed(c, [same, same, same, same])
    assert (cmd.left, cmd.right) == (0.0, 0.0)
    assert c.status()["track"] == "cand 1/2"


def test_a_jump_across_the_frame_restarts_the_streak():
    """Two unrelated phantoms are not one ball seen twice."""
    c, _ = make(confirm_frames=2, match_tol=0.1, match_tol_per_s=0.0)
    cmd = feed(c, [det(error_x=-0.8, stamp=1.0), det(error_x=0.8, stamp=1.1)])
    assert (cmd.left, cmd.right) == (0.0, 0.0)


def test_the_match_radius_grows_with_the_gap_since_the_last_sighting():
    """The bug that made the robot scan forever. The sensor attaches inference
    to ~25% of frames, so while scanning the scene sweeps past far faster than
    any fixed radius — the ball read as a new object every frame, the streak
    never completed, and not locking on is what MADE it keep scanning."""
    c, mech = make(confirm_frames=2, match_tol=0.1, match_tol_per_s=5.0)
    import time
    c.detection_provider = lambda: det(error_x=0.0, stamp=1.0)
    c.update(0.02)
    # Same ball, half a second later and well outside the base radius.
    c._confirmed_at = time.monotonic() - 0.5
    c.detection_provider = lambda: det(error_x=0.6, stamp=1.5)
    c.update(0.02)
    assert c.status()["track"] == "locked", "a fixed radius would have reset this"


def test_a_confirmed_ball_survives_a_short_dropout():
    """One missed frame must not brake the robot mid-approach."""
    c, _ = make(confirm_frames=2, memory_s=5.0)
    feed(c, [det(stamp=1.0), det(stamp=1.1)])
    c.detection_provider = lambda: None
    cmd = c.update(0.02)
    assert cmd.left > 0, "should coast through the dropout"


def test_an_unconfirmed_candidate_dies_on_the_first_miss():
    """Otherwise two phantoms memory_s apart confirm each other and
    'consecutive' means nothing."""
    c, _ = make(confirm_frames=3, memory_s=5.0)
    feed(c, [det(stamp=1.0), det(stamp=1.1)])
    c.detection_provider = lambda: None
    c.update(0.02)
    assert c.status()["track"] == "cand 0/3"


# --- approach and swallow ----------------------------------------------------

def test_it_steers_toward_a_ball_off_to_one_side():
    c, _ = make(confirm_frames=1)
    right = feed(c, [det(error_x=0.5, stamp=1.0)])
    assert right.left > right.right, "ball on the right => turn right"


def test_it_slows_down_as_the_ball_gets_closer():
    """So the intake can actually grab it."""
    c, _ = make(confirm_frames=1)
    far = feed(c, [det(error_y=-0.9, stamp=1.0)])
    c2, _ = make(confirm_frames=1)
    near = feed(c2, [det(error_y=0.3, stamp=1.0)])
    assert near.left < far.left


def test_crossing_the_stop_line_runs_the_intake_and_drives_straight():
    """Steering now would sweep the ball out from under the hood."""
    c, mech = make(confirm_frames=1, stop_line=0.7)
    # error_y 0.5 => 75% down the frame, past a 70% stop line.
    cmd = feed(c, [det(error_x=0.9, error_y=0.5, stamp=1.0)])
    assert mech.preset == "in"
    assert cmd.left == cmd.right, "must creep straight, not steer"
    assert cmd.left > 0


def test_the_intake_is_latched_not_held():
    """Nothing refreshes an autonomous decision, so the mechanism's dead-man
    must not be armed — it would stop the intake half a second later."""
    c, mech = make(confirm_frames=1, stop_line=0.7)
    feed(c, [det(error_y=0.5, stamp=1.0)])
    assert mech.holds == [False]


def test_seeing_the_ball_above_the_line_again_switches_the_intake_off():
    """The intake is latched, so this branch has to do it: a ball back above the
    line is positive evidence it was NOT swallowed."""
    c, mech = make(confirm_frames=1, stop_line=0.7, swallow_run_on_s=0.0)
    feed(c, [det(error_y=0.5, stamp=1.0)])
    assert mech.preset == "in"
    feed(c, [det(error_y=-0.5, stamp=1.1)])
    assert mech.preset is None


# --- the ball that went under the hood ---------------------------------------

def test_a_lost_ball_is_pushed_after_but_the_intake_runs_longer():
    """Two timers off one event: the ball leaves the FRAME well before it
    reaches the intake, and one already in the throat has to finish going in
    after the robot has stopped."""
    import time
    c, mech = make(confirm_frames=1, lost_push_s=1.0, lost_intake_s=3.0,
                   swallow_run_on_s=0.0)
    feed(c, [det(stamp=1.0)])          # had_ball = True
    c.detection_provider = lambda: None
    # Expire the memory too, or the controller is still coasting on the last
    # ball rather than treating it as lost — which is its own test above.
    c._confirmed_at = time.monotonic() - 10.0

    c._last_ball_time = time.monotonic() - 0.5     # inside both windows
    cmd = c.update(0.02)
    assert cmd.left > 0 and mech.preset == "in"

    c._last_ball_time = time.monotonic() - 2.0     # past the push, inside intake
    cmd = c.update(0.02)
    assert (cmd.left, cmd.right) == (0.0, 0.0)
    assert mech.preset == "in"

    c._last_ball_time = time.monotonic() - 4.0     # past both
    c.update(0.02)
    assert mech.preset is None


def test_it_does_not_push_forward_at_startup():
    """'No ball yet' looks identical to 'ball just lost'. Without the had_ball
    gate the robot drives forward the moment the mode is entered."""
    import time
    c, _ = make(confirm_frames=1, lost_push_s=1.0, search_after=100.0)
    c.detection_provider = lambda: None
    c._last_ball_time = time.monotonic() - 0.5
    cmd = c.update(0.02)
    assert (cmd.left, cmd.right) == (0.0, 0.0)


# --- search ------------------------------------------------------------------

def test_it_scans_by_spinning_then_stepping_forward():
    """Spinning in place only ever sees one circle of the field — if the nearest
    ball is outside it, the robot spins forever."""
    c, _ = make(scan_spin_s=5.0, scan_advance_s=1.0)
    spin = c._scan(1.0)
    assert spin.left != spin.right, "a sweep turns"
    advance = c._scan(5.5)
    assert advance.left == advance.right > 0, "then it covers new ground"


def test_it_sits_still_before_it_starts_scanning():
    import time
    c, _ = make(search_after=5.0)
    c.detection_provider = lambda: None
    c._last_ball_time = time.monotonic() - 1.0
    cmd = c.update(0.02)
    assert (cmd.left, cmd.right) == (0.0, 0.0)


# --- safety ------------------------------------------------------------------

@pytest.mark.parametrize("hook", ["on_deactivate", "on_estop"])
def test_leaving_the_mode_stops_the_intake(hook):
    """A mechanism left spinning because the operator switched modes is the
    failure the e-stop exists to catch; don't author it deliberately."""
    c, mech = make(confirm_frames=1, stop_line=0.7)
    feed(c, [det(error_y=0.5, stamp=1.0)])
    assert mech.preset == "in"
    getattr(c, hook)()
    assert mech.preset is None
    assert c.status()["intake"] is False


def test_activation_does_not_inherit_a_pending_push():
    """A push is blind motion. Inheriting had_ball across activations would
    drive the robot forward on entry for a ball it saw in another mode."""
    c, _ = make(confirm_frames=1)
    feed(c, [det(stamp=1.0)])
    assert c._had_ball
    c.on_activate()
    assert not c._had_ball


def test_a_rover_with_no_intake_still_drives():
    """The layout decides what exists. A build with no such mechanism enters the
    mode, chases the ball, and simply never swallows — rather than crashing."""
    c = BallIntakeController(mechanisms={}, confirm_frames=1)
    c.on_activate()
    cmd = feed(c, [det(error_y=0.5, stamp=1.0)])
    assert cmd.left > 0


def test_no_perception_holds_the_robot_still():
    c = BallIntakeController(mechanisms={"intake": FakeMech()})
    c.on_activate()
    cmd = c.update(0.02)
    assert (cmd.left, cmd.right) == (0.0, 0.0)
    assert c.status()["phase"] == "no_perception"
