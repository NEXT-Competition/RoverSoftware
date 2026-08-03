"""A queue of actuators, run in order, off the control tick.

The build that motivates it is a shooter with three things on it: a feeder
servo, a flywheel and a belt. They have to happen IN ORDER, and neither
existing mechanism kind can say so — `power` writes every actuator at once and
`pulse` swings them all to the same angle together. So the sequence is the
third kind, and what these pin down is that it stays non-blocking, that a step
can wait on something other than a clock, and above all that every way of
ending early still parks the hardware.

No hardware: RS_MOCK_MOTORS makes ESCMotor write to a mock servo, and the
clock is moved rather than slept through.
"""

import pytest

from robot.config import MechanismConfig, MotorConfig, SequenceStep
from robot.drive.mechanism import SequenceMechanism, build_mechanism


@pytest.fixture(autouse=True)
def mock_motors(monkeypatch):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")


def esc(channel, **kw):
    return MotorConfig(channel=channel, kind="esc", **kw)


def servo(channel, **kw):
    """A servo that actually travels.

    The stock min/max angle are an ESC's endpoints (+/-20 about neutral), and
    `set_angle` clamps to them — so a feeder arm authored in degrees needs its
    own travel declared, exactly as the launcher's does. Getting this wrong is
    silent: the arm just moves less than the layout says.
    """
    kw.setdefault("min_angle", -90.0)
    kw.setdefault("max_angle", 90.0)
    return MotorConfig(channel=channel, kind="servo", **kw)


def shooter(steps, **kw):
    """The motivating build: feeder servo, flywheel, belt."""
    cfg = MechanismConfig(
        name="launcher", kind="sequence", rest_angle=-30.0,
        actuators={"feeder": servo(4, name="feeder"),
                   "flywheel": esc(5, name="flywheel"),
                   "belt": esc(6, name="belt")},
        steps=steps, **kw)
    return SequenceMechanism(cfg)


def step(**kw):
    kw.setdefault("values", {})
    return SequenceStep(**kw)


def age(mech, seconds):
    """Backdate the current step's clock rather than sleeping through it."""
    mech._step_at -= seconds


def throttle(mech, name):
    return mech.motors[name].throttle


def angle(mech, name):
    """The last angle actually written to the channel. Read off the mock servo
    because `set_angle` bypasses the throttle cache, so there is nowhere else
    the commanded position exists."""
    return mech.motors[name].servo._last


# --- the order, which is the whole point -------------------------------------

def test_the_steps_run_in_order_and_not_all_at_once():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.2),
                 step(values={"feeder": 40.0}, seconds=0.3),
                 step(values={"belt": 0.8}, seconds=0.3)])
    assert m.activate() is True

    # Step 1 only. The belt must NOT be running yet — a `power` mechanism with
    # a preset would have started all three here, which is the bug this kind
    # exists to make impossible.
    assert throttle(m, "flywheel") == pytest.approx(1.0)
    assert throttle(m, "belt") == pytest.approx(0.0)
    assert m.status()["step"] == 1

    age(m, 0.25)
    m.update()
    assert m.status()["step"] == 2
    assert throttle(m, "belt") == pytest.approx(0.0)

    age(m, 0.35)
    m.update()
    assert m.status()["step"] == 3
    assert throttle(m, "belt") == pytest.approx(0.8)


def test_an_earlier_step_keeps_running_through_a_later_one():
    """The reason a step is not a preset. The flywheel spun up in step 1 has to
    still be spinning when the feeder pushes a ball into it in step 2 —
    otherwise the sequence is an elaborate way to throw nothing."""
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.1),
                 step(values={"feeder": 40.0}, seconds=0.1)])
    m.activate()
    age(m, 0.2)
    m.update()
    assert m.status()["step"] == 2
    assert throttle(m, "flywheel") == pytest.approx(1.0)


def test_clear_opts_a_step_back_into_preset_behaviour():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.1),
                 step(values={"belt": 0.5}, seconds=0.1, clear=True)])
    m.activate()
    age(m, 0.2)
    m.update()
    assert throttle(m, "belt") == pytest.approx(0.5)
    assert throttle(m, "flywheel") == pytest.approx(0.0)


def test_it_does_not_block():
    """`activate` returns immediately; nothing advances without a tick. A
    sequence that slept between legs would freeze the 50 Hz loop for its whole
    length, trip the slow-tick watchdog and hold the drive outputs where they
    were."""
    m = shooter([step(values={"flywheel": 1.0}, seconds=5.0),
                 step(values={"belt": 1.0}, seconds=5.0)])
    m.activate()
    for _ in range(50):
        m.update()
    assert m.status()["step"] == 1
    assert m.state == "running"


# --- the units, which differ by what the actuator IS --------------------------

def test_a_servo_step_is_degrees_and_an_esc_step_is_throttle():
    m = shooter([step(values={"feeder": 40.0, "flywheel": 0.5}, seconds=0.1)])
    m.activate()
    assert angle(m, "feeder") == pytest.approx(40.0)
    assert throttle(m, "flywheel") == pytest.approx(0.5)


def test_a_servo_parks_at_rest_angle_and_an_esc_parks_at_stop():
    m = shooter([step(values={"feeder": 40.0, "flywheel": 1.0}, seconds=0.1)])
    m.activate()
    age(m, 0.2)
    m.update()
    assert m.state == "rest"
    assert angle(m, "feeder") == pytest.approx(-30.0)   # rest_angle
    assert throttle(m, "flywheel") == pytest.approx(0.0)


# --- time is a MINIMUM, not a duration ---------------------------------------

def test_seconds_is_a_floor_the_step_cannot_end_before():
    m = shooter([step(values={"flywheel": 1.0}, seconds=1.0),
                 step(values={"belt": 1.0}, seconds=1.0)])
    m.activate()
    age(m, 0.9)
    m.update()
    assert m.status()["step"] == 1


def test_a_gate_holds_a_step_open_past_its_dwell():
    """The other half: the dwell has elapsed and the step still does not end,
    because the thing it is waiting for has not happened."""
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.1,
                      wait_for={"kind": "rpm", "actuator": "flywheel",
                                "at_least": 3000}, timeout=10.0),
                 step(values={"belt": 1.0}, seconds=0.1)])
    m.activate()
    age(m, 5.0)
    m.update()
    assert m.status()["step"] == 1      # dwell long gone; no encoder reading


# --- the other factor: gating on a measurement --------------------------------

class FakeEncoder:
    """Stands in for a quadrature encoder. Only `rpm` and the lifecycle calls
    are reached from a mechanism."""

    def __init__(self, value=None):
        self.value = value

    def rpm(self):
        return self.value

    def telemetry(self):
        return None if self.value is None else round(self.value, 1)

    def sample(self, now=None):
        pass

    def start(self):
        return True

    def stop(self):
        pass


def geared(m, actuator, value):
    m.encoders[actuator] = FakeEncoder(value)
    return m.encoders[actuator]


def test_a_step_waits_until_the_flywheel_is_actually_at_speed():
    """The shooter's real sequencing factor, and the reason time alone is not
    enough: how long a wheel takes to reach 3000 rpm depends on the battery,
    which means a time-only sequence is correct at one charge state and early
    at every other."""
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.1,
                      wait_for={"kind": "rpm", "actuator": "flywheel",
                                "at_least": 3000}, timeout=10.0),
                 step(values={"belt": 1.0}, seconds=0.1)])
    enc = geared(m, "flywheel", 500.0)
    m.activate()

    age(m, 1.0)
    m.update()
    assert m.status()["step"] == 1          # 500 rpm is not 3000

    enc.value = 3200.0
    m.update()
    assert m.status()["step"] == 2          # now it may feed


def test_the_direction_of_the_wheel_does_not_matter():
    """Encoder RPM is signed, and a flywheel mounted the other way round turns
    negative. Gating on the raw number would make that build wait forever."""
    m = shooter([step(values={"flywheel": -1.0}, seconds=0.0,
                      wait_for={"kind": "rpm", "actuator": "flywheel",
                                "at_least": 3000}, timeout=10.0),
                 step(values={"belt": 1.0}, seconds=0.0)])
    geared(m, "flywheel", -3200.0)
    m.activate()
    m.update()
    assert m.status()["step"] == 2


def test_an_at_most_gate_waits_for_something_to_slow_down():
    m = shooter([step(values={"flywheel": 0.0}, seconds=0.0,
                      wait_for={"kind": "rpm", "actuator": "flywheel",
                                "at_most": 100}, timeout=10.0),
                 step(values={"belt": 1.0}, seconds=0.0)])
    enc = geared(m, "flywheel", 2000.0)
    m.activate()
    m.update()
    assert m.status()["step"] == 1
    enc.value = 40.0
    m.update()
    assert m.status()["step"] == 2


def test_an_unmeasurable_speed_is_not_treated_as_zero():
    """`Encoder.rpm()` returns None for an absent encoder AND for one whose
    counts cannot be trusted. Reading that as 0 would satisfy an `at_most` gate
    on a wheel that is actually at full speed."""
    m = shooter([step(values={"flywheel": 0.0}, seconds=0.0,
                      wait_for={"kind": "rpm", "actuator": "flywheel",
                                "at_most": 100}, timeout=10.0)])
    geared(m, "flywheel", None)
    m.activate()
    m.update()
    assert m.status()["step"] == 1          # held closed, not waved through


def test_a_step_can_wait_for_another_mechanism():
    class FakeMech:
        def __init__(self):
            self.is_ready = False

        def ready(self):
            return self.is_ready

    other = FakeMech()
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.0,
                      wait_for={"kind": "mech_ready", "mech": "intake"},
                      timeout=10.0),
                 step(values={"belt": 1.0}, seconds=0.0)])
    m.bind({"intake": other})
    m.activate()
    m.update()
    assert m.status()["step"] == 1
    other.is_ready = True
    m.update()
    assert m.status()["step"] == 2


# --- a gate that never opens --------------------------------------------------

def test_a_gate_that_never_opens_aborts_and_parks_everything():
    """The failure this bounds. Without a timeout the mechanism sits with the
    flywheel at full throttle forever, waiting for a speed a stripped belt
    means it will never reach."""
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.0,
                      wait_for={"kind": "rpm", "actuator": "flywheel",
                                "at_least": 3000}, timeout=2.0),
                 step(values={"belt": 1.0}, seconds=0.0)])
    geared(m, "flywheel", 100.0)
    m.activate()
    age(m, 2.5)
    m.update()
    assert m.state == "rest"
    assert throttle(m, "flywheel") == pytest.approx(0.0)
    assert throttle(m, "belt") == pytest.approx(0.0)


def test_aborting_does_not_run_the_rest_of_the_queue():
    """Feeding a ball into a wheel that never got to speed is what jams the
    mechanism — which is the whole reason the gate was written."""
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.0,
                      wait_for={"kind": "rpm", "actuator": "flywheel",
                                "at_least": 3000}, timeout=1.0),
                 step(values={"feeder": 40.0}, seconds=0.0)])
    geared(m, "flywheel", 100.0)
    m.activate()
    age(m, 1.5)
    m.update()
    assert angle(m, "feeder") == pytest.approx(-30.0)   # never fed


def test_the_abort_says_what_it_was_waiting_for_and_what_it_measured(capsys):
    """From outside, a sequence that aborted and one that ran look identical —
    the mechanism is at rest either way. This line is the only thing that tells
    an operator which happened, so it carries both numbers."""
    m = shooter([step(name="spin up", values={"flywheel": 1.0}, seconds=0.0,
                      wait_for={"kind": "rpm", "actuator": "flywheel",
                                "at_least": 3000}, timeout=1.0)])
    geared(m, "flywheel", 850.0)
    m.activate()
    capsys.readouterr()
    age(m, 1.5)
    m.update()
    out = capsys.readouterr().out
    assert "aborted" in out and "spin up" in out
    assert "3000" in out and "850" in out


def test_on_timeout_advance_carries_on_instead():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.0,
                      wait_for={"kind": "rpm", "actuator": "flywheel",
                                "at_least": 3000}, timeout=1.0,
                      on_timeout="advance"),
                 step(values={"belt": 1.0}, seconds=0.0)])
    geared(m, "flywheel", 100.0)
    m.activate()
    age(m, 1.5)
    m.update()
    assert throttle(m, "belt") == pytest.approx(1.0)


def test_a_step_with_no_timeout_of_its_own_uses_the_mechanisms():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.0,
                      wait_for={"kind": "rpm", "actuator": "flywheel",
                                "at_least": 3000})],
                step_timeout=3.0)
    geared(m, "flywheel", 10.0)
    m.activate()
    age(m, 2.0)
    m.update()
    assert m.state == "running"
    age(m, 1.5)
    m.update()
    assert m.state == "rest"


def test_an_unknown_gate_is_held_closed_rather_than_waved_through(capsys):
    """A layout that changed under a running robot. Held closed so the step's
    timeout ends the run: a gate nobody wrote silently passing is how a ball
    reaches a stationary flywheel."""
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.0,
                      wait_for={"kind": "phase_of_the_moon"}, timeout=1.0),
                 step(values={"belt": 1.0}, seconds=0.0)])
    m.activate()
    m.update()
    assert m.status()["step"] == 1
    age(m, 1.5)
    m.update()
    assert m.state == "rest"
    assert throttle(m, "belt") == pytest.approx(0.0)


# --- being interrupted --------------------------------------------------------

def test_stop_mid_sequence_parks_every_actuator():
    """E-stop, mode exit and shutdown all land here. A half-finished sequence
    must not leave a servo against its stop or a flywheel spinning."""
    m = shooter([step(values={"feeder": 40.0, "flywheel": 1.0}, seconds=5.0),
                 step(values={"belt": 1.0}, seconds=5.0)])
    m.activate()
    m.stop()
    assert m.state == "rest"
    assert angle(m, "feeder") == pytest.approx(-30.0)
    assert throttle(m, "flywheel") == pytest.approx(0.0)


def test_a_stopped_sequence_does_not_resume_on_the_next_tick():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.1),
                 step(values={"belt": 1.0}, seconds=0.1)])
    m.activate()
    m.stop()
    for _ in range(10):
        m.update()
    assert throttle(m, "belt") == pytest.approx(0.0)


def test_it_restarts_from_the_top_rather_than_the_middle():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.1),
                 step(values={"belt": 1.0}, seconds=0.1)])
    m.activate()
    age(m, 0.2)
    m.update()
    assert m.status()["step"] == 2
    m.stop()
    m.activate()
    assert m.status()["step"] == 1


# --- the cycle contract, shared with PulseMechanism ---------------------------

def test_activating_a_running_sequence_does_nothing():
    """Something asking every tick gets one run per cycle, rather than needing
    a timer of its own — the same contract `pulse` offers."""
    m = shooter([step(values={"flywheel": 1.0}, seconds=1.0)])
    assert m.activate() is True
    assert m.activate() is False
    assert m.activations == 1


def test_ready_is_false_while_it_runs_so_a_routine_can_wait_for_it():
    """`mech_ready` is already a routine condition; a sequence answering False
    for exactly as long as it runs is what lets a routine wait for the end of
    one without a second vocabulary for it."""
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.1)])
    assert m.ready() is True
    m.activate()
    assert m.ready() is False
    age(m, 0.2)
    m.update()
    assert m.ready() is True


def test_a_cooldown_is_respected_between_runs():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.1)], cooldown=5.0)
    m.activate()
    age(m, 0.2)
    m.update()
    assert m.state == "rest"
    assert m.activate() is False


def test_a_magazine_runs_out():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.0)], max_activations=2)
    for _ in range(2):
        assert m.activate() is True
        m.update()
    assert m.activate() is False


def test_an_empty_queue_activates_nothing():
    """The state a mechanism is in the moment it is added in the editor. It has
    to be inert rather than an exception on the control loop."""
    m = shooter([])
    assert m.activate() is False
    assert m.state == "rest"
    m.update()


def test_fire_is_the_same_thing_so_the_launcher_vocabulary_works():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.1)])
    assert m.fire() is True
    assert m.state == "running"


# --- looping ------------------------------------------------------------------

def test_a_loop_starts_again_instead_of_finishing():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.1),
                 step(values={"belt": 1.0}, seconds=0.1)], loop=True)
    m.activate()
    for _ in range(2):
        age(m, 0.2)
        m.update()
    assert m.state == "running"
    assert m.status()["step"] == 1
    m.stop()
    assert m.state == "rest"


# --- what the dashboard is told -----------------------------------------------

def test_status_names_the_step_it_is_on():
    m = shooter([step(name="spin up", values={"flywheel": 1.0}, seconds=0.1),
                 step(name="feed", values={"feeder": 40.0}, seconds=0.1)])
    m.activate()
    assert m.status()["step_name"] == "spin up"
    age(m, 0.2)
    m.update()
    assert m.status()["step_name"] == "feed"


def test_an_unnamed_step_is_still_identifiable():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.1)])
    m.activate()
    assert m.status()["step_name"] == "1/1"


def test_status_reports_measured_speed_when_there_is_an_encoder():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.1)])
    geared(m, "flywheel", 2750.0)
    assert m.status()["rpm"] == {"flywheel": 2750.0}


def test_a_build_with_no_encoders_says_nothing_about_speed():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.1)])
    assert "rpm" not in m.status()


def test_the_abort_reason_survives_into_telemetry():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.0,
                      wait_for={"kind": "rpm", "actuator": "flywheel",
                                "at_least": 3000}, timeout=1.0)])
    geared(m, "flywheel", 10.0)
    m.activate()
    age(m, 1.5)
    m.update()
    assert "aborted" in m.status()["aborted"] or m.status()["aborted"]


def test_a_fresh_run_clears_the_previous_abort():
    m = shooter([step(values={"flywheel": 1.0}, seconds=0.0,
                      wait_for={"kind": "rpm", "actuator": "flywheel",
                                "at_least": 3000}, timeout=1.0)])
    enc = geared(m, "flywheel", 10.0)
    m.activate()
    age(m, 1.5)
    m.update()
    assert m.status().get("aborted")
    enc.value = 4000.0
    m.activate()
    assert "aborted" not in m.status()


# --- the factory --------------------------------------------------------------

def test_build_mechanism_makes_one_from_a_layout_kind():
    cfg = MechanismConfig(name="launcher", kind="sequence",
                          actuators={"a": esc(3, name="a")},
                          steps=[step(values={"a": 1.0}, seconds=0.1)])
    assert isinstance(build_mechanism(cfg), SequenceMechanism)


def test_the_encoders_are_built_from_the_actuators_that_declare_pins():
    cfg = MechanismConfig(
        name="launcher", kind="sequence",
        actuators={"fly": esc(5, name="fly", encoder_a=17, encoder_b=27,
                              encoder_cpr=20),
                   "belt": esc(6, name="belt")},
        steps=[step(values={"fly": 1.0}, seconds=0.1)])
    m = build_mechanism(cfg)
    assert set(m.encoders) == {"fly"}
