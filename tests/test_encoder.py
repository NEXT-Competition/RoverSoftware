"""The quadrature decoder and the speed estimate built on it.

No GPIO here — a fake backend plays the part of pigpio/lgpio, so the decoding
rules are pinned down on a laptop. That matters more than usual for this file:
a decoder that counts backwards at speed still looks perfectly correct when you
turn a wheel by hand, which is exactly how the bug survives bring-up.
"""

import pytest

from robot.config import MotorConfig
from robot.sensors import encoder as enc_mod
from robot.sensors.encoder import Encoder, build_encoder


class FakeBackend:
    """A pair of pins whose levels the test sets, and one callback per pin."""

    name = "fake"

    def __init__(self):
        self.levels = {}
        self.callbacks = {}
        self.closed = False

    def claim(self, pin, on_edge):
        self.levels.setdefault(pin, 1)
        self.callbacks[pin] = on_edge
        return True

    def read(self, pin):
        return self.levels.get(pin, 1)

    def close(self):
        self.closed = True


@pytest.fixture
def gpio(monkeypatch):
    fake = FakeBackend()
    monkeypatch.setattr(enc_mod, "backend", lambda: fake)
    return fake


def make(gpio, cpr=4.0, **kw):
    gpio.levels.update({17: 0, 27: 0})
    e = Encoder(pin_a=17, pin_b=27, counts_per_rev=cpr, **kw)
    assert e.start()
    return e


def turn(gpio, states):
    """Drive the two channels through a sequence of (A, B) levels."""
    for a, b in states:
        gpio.levels[17], gpio.levels[27] = a, b
        gpio.callbacks[17]()  # one edge is enough: the decoder re-reads both


# A leading B: 00 -> 10 -> 11 -> 01 -> 00 is one full quadrature cycle forward.
FORWARD = [(1, 0), (1, 1), (0, 1), (0, 0)]
REVERSE = [(0, 1), (1, 1), (1, 0), (0, 0)]


def test_one_cycle_forward_counts_four(gpio):
    """X4 decoding: every edge on either channel is a count.

    This is the arithmetic `counts_per_rev` is defined against, so if it ever
    became X1 or X2 every RPM the robot reports would be wrong by a factor.
    """
    e = make(gpio)
    turn(gpio, FORWARD)
    assert e.ticks == 4


def test_one_cycle_reverse_counts_four_the_other_way(gpio):
    e = make(gpio)
    turn(gpio, REVERSE)
    assert e.ticks == -4


def test_direction_is_the_phase_not_the_edge_count(gpio):
    """Forward then reverse returns to zero — the whole point of quadrature."""
    e = make(gpio)
    turn(gpio, FORWARD * 3)
    turn(gpio, REVERSE * 3)
    assert e.ticks == 0


def test_invert_flips_the_reported_direction_not_the_decoding(gpio):
    """`encoder_invert` is a sign on the way out, so a mirrored track motor can
    report positive RPM going forward without rewiring the encoder."""
    e = make(gpio, invert=True)
    turn(gpio, FORWARD)
    assert e.ticks == -4


def test_a_missed_transition_is_dropped_rather_than_guessed(gpio):
    """A diagonal move (00 -> 11) means two edges arrived unseen.

    Its direction is genuinely unknowable, and inventing one would inject a
    phantom count in a random direction — worse than losing one, because a speed
    loop reads a rate and a fabricated count biases it.
    """
    e = make(gpio)
    turn(gpio, [(1, 1)])  # straight from 00 to 11
    assert e.ticks == 0
    assert e.missed == 1


def test_rpm_is_counts_per_rev_over_elapsed_time(gpio):
    e = make(gpio, cpr=4.0, tau=0.0)  # one cycle == one revolution
    e.sample(0.0)
    turn(gpio, FORWARD)     # one revolution
    e.sample(0.5)           # ...in half a second => 2 rev/s => 120 rpm
    assert e.rpm() == pytest.approx(120.0)


def test_rpm_holds_until_a_whole_window_has_passed(gpio):
    """Speed is counts over an interval, so a sample inside the window would be
    quantization noise dressed up as a measurement."""
    e = make(gpio, cpr=4.0, window=0.1, tau=0.0)
    e.sample(0.0)
    turn(gpio, FORWARD)
    e.sample(0.05)
    assert e.rpm() == 0.0    # too soon; nothing published yet
    e.sample(0.10)
    assert e.rpm() == pytest.approx(600.0)


def test_a_stopped_wheel_reads_zero_not_the_last_speed(gpio):
    """No counts in a window IS a measurement — of a standstill."""
    e = make(gpio, cpr=4.0, tau=0.0)
    e.sample(0.0)
    turn(gpio, FORWARD)
    e.sample(0.5)
    assert e.rpm() > 0
    e.sample(1.0)  # a whole window with nothing arriving
    assert e.rpm() == 0.0


def test_smoothing_approaches_the_measurement_rather_than_jumping_to_it(gpio):
    e = make(gpio, cpr=4.0, tau=0.5)
    e.sample(0.0)
    turn(gpio, FORWARD)
    e.sample(0.5)
    first = e.rpm()
    assert 0 < first < 120.0  # the filter has not caught up yet
    turn(gpio, FORWARD)
    e.sample(1.0)
    assert first < e.rpm() < 120.0


def test_rpm_is_none_before_start_and_after_stop(gpio):
    """None means "no measurement", 0.0 means "not turning". A speed loop that
    cannot tell them apart winds a dead channel to full throttle."""
    e = Encoder(pin_a=17, pin_b=27, counts_per_rev=4.0)
    assert e.rpm() is None
    e = make(gpio)
    assert e.rpm() == 0.0
    e.stop()
    assert e.rpm() is None


def test_a_zero_counts_per_rev_does_not_divide_by_zero(gpio):
    """It is live-tunable, so somebody can clear it from the dashboard mid-run."""
    e = make(gpio, cpr=4.0)
    e.sample(0.0)
    e.counts_per_rev = 0.0
    turn(gpio, FORWARD)
    e.sample(1.0)  # must not raise
    assert e.rpm() == 0.0


def test_no_backend_leaves_the_encoder_inert_rather_than_raising(monkeypatch):
    """A dev laptop has no GPIO. The robot must still drive."""
    monkeypatch.setattr(enc_mod, "backend", lambda: None)
    e = Encoder(pin_a=17, pin_b=27, counts_per_rev=4.0)
    assert e.start() is False
    assert e.ok() is False
    assert e.rpm() is None


# --- build_encoder: the config -> sensor bridge ------------------------------

def test_build_encoder_returns_nothing_for_an_actuator_without_pins():
    assert build_encoder(MotorConfig(channel=0, name="left")) is None


def test_build_encoder_needs_a_counts_per_rev_as_well_as_pins():
    """Pins with no scale cannot produce an RPM, so there is nothing to build.

    Deliberately not "build it and report zero": zero is a speed, and the loop
    treats a speed it can see as a measurement it can trust.
    """
    motor = MotorConfig(channel=0, name="left", encoder_a=17, encoder_b=27)
    assert build_encoder(motor) is None
    motor.encoder_cpr = 1200
    assert build_encoder(motor) is not None
