"""The ultrasonic reader: what it believes, and what it refuses to believe.

No GPIO here — a fake stands in for the Fusion HAT's Ultrasonic module, so the
filtering and the two meanings of "no reading" are pinned down on a laptop.

The load-bearing tests are the ones about SILENCE. A cheap ultrasonic answers
"nothing heard" both when the path is clear and when it is unplugged, and every
decision in this module follows from refusing to pretend otherwise: the reading
decays to None rather than going stale, a single wild echo is filtered out, and
a sensor that has never heard anything says so out loud instead of quietly
reporting an open road.
"""

import time

import pytest

from robot.config import UltrasonicConfig
from robot.sensors import ultrasonic as ultra_mod
from robot.sensors.ultrasonic import NO_PIN, Ultrasonic, build_ultrasonic


class FakeModule:
    """The HAT's Ultrasonic, as a list of centimetre readings to hand back.

    -1 is the library's "no echo within the timeout", which is what an open
    field and a dead sensor both produce.
    """

    def __init__(self, readings=(), repeat_last=True):
        self.readings = list(readings)
        self.repeat_last = repeat_last
        self.calls = 0
        self.times = []          # the `times` argument of each read()

    def read(self, times=10):
        self.calls += 1
        self.times.append(times)
        if not self.readings:
            return -1
        if len(self.readings) == 1 and self.repeat_last:
            return self.readings[0]
        return self.readings.pop(0)


def sensor(readings=(), **kw):
    """An Ultrasonic wired to a fake module, without starting its thread.

    `_ping()` is called by hand so the tests step the reader one measurement at
    a time instead of racing a 60 ms timer.
    """
    s = Ultrasonic(trig_pin=27, echo_pin=22, **kw)
    s._sensor = FakeModule(readings)
    s._running = True
    return s


# --- what counts as a reading ------------------------------------------------

def test_centimetres_become_metres():
    s = sensor([42.0])
    s._ping()
    assert s.distance_m() == pytest.approx(0.42)


def test_no_echo_is_not_a_distance():
    """The library's -1. Nothing in range, or nothing wired — from here they
    are the same silence, and inventing a number for either is how a guard
    ends up acting on a sensor that isn't there."""
    s = sensor([-1])
    s._ping()
    assert s.distance_m() is None


def test_readings_outside_the_band_are_dropped_not_clamped():
    """Below min_m the transducer is still ringing from its own burst; above
    max_m the echo is too weak to be what it claims. Clamping either into the
    band would invent an obstacle at exactly the distance that stops the rover."""
    s = sensor([0.5], min_m=0.03, max_m=4.0)   # 0.5 cm = 5 mm
    s._ping()
    assert s.distance_m() is None

    far = sensor([900.0], min_m=0.03, max_m=4.0)   # 9 m
    far._ping()
    assert far.distance_m() is None


def test_one_ping_per_call_so_a_clear_path_does_not_block():
    """The library retries internally and returns the first echo, which turns
    an empty room into a call that blocks for most of a second. Our repeats are
    spaced by `interval` instead, so consecutive pings can't hear each other."""
    s = sensor([-1])
    s._ping()
    assert s._sensor.times == [1]


# --- the median filter -------------------------------------------------------

def test_a_single_wild_echo_is_filtered_out():
    """The characteristic ultrasonic failure: one absurdly short reading between
    good ones. A mean would smear it over the whole window; the median drops it."""
    s = sensor([100.0, 5.0, 100.0], samples=3)
    for _ in range(3):
        s._ping()
    assert s.distance_m() == pytest.approx(1.0)


def test_silence_does_not_pull_the_estimate_outwards():
    """Only real echoes enter the window. A ping that heard nothing is not a
    vote for "further away" — it is not a measurement at all."""
    s = sensor([50.0, -1, -1], samples=3)
    for _ in range(3):
        s._ping()
    assert s.distance_m() == pytest.approx(0.5)


def test_the_raw_reading_is_still_available_unfiltered():
    """What you watch while waving a hand at the sensor: the median hides
    exactly the noise you are trying to judge the size of."""
    s = sensor([100.0, 5.0], samples=3)
    s._ping()
    s._ping()
    assert s.raw_m() == pytest.approx(0.05)
    assert s.distance_m() == pytest.approx(1.0)  # the median is unmoved


def test_samples_of_one_disables_filtering():
    s = sensor([100.0, 5.0], samples=1)
    s._ping()
    s._ping()
    assert s.distance_m() == pytest.approx(0.05)


def test_shrinking_the_window_takes_effect_without_a_restart():
    """`samples` is live from the settings page, so the deque is deliberately
    longer than the window and the width is applied at read time."""
    s = sensor([100.0, 5.0, 100.0], samples=3)
    for _ in range(3):
        s._ping()
    s.samples = 1
    assert s.distance_m() == pytest.approx(1.0)  # the newest sample, unfiltered


# --- staleness ---------------------------------------------------------------

def test_a_reading_decays_to_none_rather_than_going_stale():
    """A sensor that stops answering must stop being believed. Otherwise the
    last distance sits there looking current, and a guard built on it either
    clamps forever or waves the rover through forever."""
    s = sensor([42.0], max_age=0.05)
    s._ping()
    assert s.distance_m() is not None
    time.sleep(0.06)
    assert s.distance_m() is None


# --- saying which silence it is ---------------------------------------------

def test_a_sensor_that_has_never_heard_anything_says_so(capsys):
    """The one thing code CAN tell apart over time: a hundred pings with no echo
    is not a hundred metres of open field, it is a wiring fault."""
    s = sensor([-1])
    for _ in range(ultra_mod._MUTE_AFTER_PINGS):
        s._ping()
    out = capsys.readouterr().out
    assert "not one echo" in out
    assert s.telemetry() == {"mute": True}


def test_the_warning_is_said_once_not_every_ping(capsys):
    s = sensor([-1])
    for _ in range(ultra_mod._MUTE_AFTER_PINGS * 2):
        s._ping()
    assert capsys.readouterr().out.count("not one echo") == 1


def test_one_echo_is_enough_to_clear_the_suspicion():
    """A sensor that has ever worked is wired. After that, silence is just an
    empty room and telemetry says nothing rather than crying wolf."""
    s = sensor([50.0] + [-1] * ultra_mod._MUTE_AFTER_PINGS, max_age=0.05)
    for _ in range(ultra_mod._MUTE_AFTER_PINGS + 1):
        s._ping()
    time.sleep(0.06)               # the one echo ages out; the silence remains
    assert s.telemetry() == {}


def test_telemetry_says_off_when_the_reader_is_not_running():
    s = Ultrasonic(trig_pin=27, echo_pin=22)
    assert s.telemetry() == {"off": True}


def test_telemetry_rounds_to_the_centimetre():
    """It rides a 57600-baud radio shared with driving. Two decimals is the
    resolution of the sensor anyway."""
    s = sensor([42.4242])
    s._ping()
    assert s.telemetry() == {"d": 0.42}


# --- a module that misbehaves ------------------------------------------------

def test_an_exception_from_the_library_does_not_kill_the_reader(monkeypatch):
    """A raising module must cost measurements, not the thread that takes them
    — and not the control loop, which is on the other side of this object."""
    monkeypatch.setattr(ultra_mod, "_ERROR_BACKOFF", 0.0)
    s = sensor([42.0])

    def boom(times=1):
        raise OSError("GPIO went away")

    s._sensor.read = boom
    s._ping()                      # must not raise
    assert s.distance_m() is None
    s._sensor = FakeModule([42.0])
    s._ping()
    assert s.distance_m() == pytest.approx(0.42)


def test_a_module_without_a_times_argument_is_still_read():
    """The retry count is an optimization; the measurement is the point."""
    class OldModule:
        def read(self):
            return 30.0

    s = sensor()
    s._sensor = OldModule()
    s._ping()
    assert s.distance_m() == pytest.approx(0.30)


# --- construction ------------------------------------------------------------

def test_no_pins_means_no_sensor():
    assert not Ultrasonic(trig_pin=NO_PIN, echo_pin=22).configured()
    assert not Ultrasonic(trig_pin=27, echo_pin=NO_PIN).configured()
    # One pin cannot be both the output and the input.
    assert not Ultrasonic(trig_pin=27, echo_pin=27).configured()
    assert Ultrasonic(trig_pin=27, echo_pin=22).configured()


def test_build_ultrasonic_declines_a_build_that_has_none():
    assert build_ultrasonic(UltrasonicConfig()) is None       # enabled=False
    assert build_ultrasonic(UltrasonicConfig(enabled=True, trig_pin=NO_PIN)) is None
    sonar = build_ultrasonic(UltrasonicConfig(enabled=True))
    assert sonar is not None and sonar.trig_pin == 27


def test_starting_without_the_hat_library_is_inert_not_fatal(capsys):
    """Same fallback as the motors and the encoders: on a dev laptop the sensor
    simply never answers, and nothing downstream may care."""
    sonar = Ultrasonic(trig_pin=27, echo_pin=22)
    assert sonar.start() is False
    assert sonar.ok() is False
    assert sonar.distance_m() is None
    assert "fusion_hat is not installed" in capsys.readouterr().out


def test_a_failed_claim_is_reported_and_survivable(monkeypatch, capsys):
    class Angry:
        def __init__(self, *_):
            raise RuntimeError("pin busy")

    monkeypatch.setattr(ultra_mod, "_HatUltrasonic", Angry)
    monkeypatch.setattr(ultra_mod, "_Pin", lambda pin: pin)
    sonar = Ultrasonic(trig_pin=27, echo_pin=22)
    assert sonar.start() is False
    assert "could not claim GPIO 27/22" in capsys.readouterr().out


# --- the thread ---------------------------------------------------------------

def test_the_reader_thread_pings_and_stops(monkeypatch):
    """The one test that exercises start/stop for real: a ping BLOCKS, which is
    the whole reason this happens off the control loop."""
    module = FakeModule([25.0])
    monkeypatch.setattr(ultra_mod, "_HatUltrasonic", lambda trig, echo: module)
    monkeypatch.setattr(ultra_mod, "_Pin", lambda pin: pin)

    sonar = Ultrasonic(trig_pin=27, echo_pin=22, interval=0.005)
    assert sonar.start() is True
    deadline = time.monotonic() + 1.0
    while sonar.distance_m() is None and time.monotonic() < deadline:
        time.sleep(0.005)
    assert sonar.distance_m() == pytest.approx(0.25)

    sonar.stop()
    assert sonar.ok() is False
    assert sonar.distance_m() is None      # no distance from a stopped sensor
    calls = module.calls
    time.sleep(0.02)
    assert module.calls == calls           # the thread really did stop
