"""The quadrature decoder and the speed estimate built on it.

No GPIO here — a fake backend stands in for the Fusion HAT, so the decoding
rules are pinned down on a laptop. That matters more than usual for this file:
a decoder that counts backwards at speed still looks perfectly correct when you
turn a wheel by hand, which is exactly how the bug survives bring-up.

The last section fakes `fusion_hat.pin` itself, to hold down how the pins get
claimed — the debounce question in particular, which no amount of decoder
testing would catch because the dropped edges never reach the decoder.
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


# --- the Fusion HAT backend: how the pins get claimed ------------------------

class FakePin:
    """Stands in for fusion_hat.pin.Pin, recording how it was constructed."""

    IN = "in"
    PULL_UP = "pull_up"

    def __init__(self, pin, mode=None, pull=None, **kw):
        self.pin = pin
        self.mode = mode
        self.pull = pull
        self.kwargs = kw
        self.closed = False
        # Counted, not forbidden: the real irq() works fine, it just quietly
        # debounces the encoder away. A test can only catch that by noticing
        # the backend went around it.
        self.irq_calls = 0

    def irq(self, handler, trigger=None):
        self.irq_calls += 1

    def value(self, value=None):
        return 0

    def close(self):
        self.closed = True


class FakeGPIO:
    """RPi.GPIO, including the part that bites: setup() comes first.

    The real one raises "You must setup() the GPIO channel first" on an input()
    it was never told the direction of. Modelled here because the failure is
    invisible on a laptop and total on the robot — it takes out every encoder,
    at the one moment when the obvious suspect is the wiring.
    """

    BOTH = "both"

    def __init__(self):
        self.detects = []
        self.removed = []
        self.levels = {}
        self.set_up = set()

    def add_event_detect(self, pin, edge, callback=None, **kw):
        if pin not in self.set_up:
            raise RuntimeError("You must setup() the GPIO channel first")
        self.detects.append({"pin": pin, "edge": edge, "callback": callback,
                             "kwargs": kw})

    def remove_event_detect(self, pin):
        self.removed.append(pin)

    def input(self, pin):
        if pin not in self.set_up:
            raise RuntimeError("You must setup() the GPIO channel first")
        return self.levels.get(pin, 0)


class FakeFusionPin:
    """The `fusion_hat.pin` module, as much of it as the backend touches."""

    def __init__(self):
        self.GPIO = FakeGPIO()
        self.built = []
        outer = self

        class _Pin(FakePin):
            def __init__(self, pin, **kw):
                super().__init__(pin, **kw)
                # The real Pin runs GPIO.setup() in its constructor; that is what
                # makes the channel readable at all.
                outer.GPIO.set_up.add(pin)
                outer.built.append(self)

            def close(self):
                super().close()
                outer.GPIO.set_up.discard(self.pin)   # GPIO.cleanup(pin)

        self.Pin = _Pin


@pytest.fixture
def hat(monkeypatch):
    fake = FakeFusionPin()
    monkeypatch.setattr(enc_mod, "fusion_pin", fake)
    monkeypatch.setattr(enc_mod, "_backend", None)
    monkeypatch.setattr(enc_mod, "_backend_error", "")
    return fake


def test_pins_are_claimed_as_inputs_with_a_pull_up(hat):
    """A floating encoder input counts noise as motion."""
    enc_mod._FusionHatBackend().claim(17, lambda: None)
    assert len(hat.built) == 1
    assert hat.built[0].pin == 17
    assert hat.built[0].mode is FakePin.IN
    assert hat.built[0].pull is FakePin.PULL_UP


def test_edges_are_detected_with_no_debounce_whatsoever(hat):
    """The reason this backend does not use Pin.irq().

    Pin.irq() and the when_activated properties both pass RPi.GPIO a bouncetime,
    which DISCARDS every edge inside the window — 20 ms by default, while a wheel
    encoder's whole output is edges far closer together than that. Nothing
    downstream can tell debounced-away counts from a slow wheel, so it has to be
    caught here.
    """
    enc_mod._FusionHatBackend().claim(17, lambda: None)

    assert len(hat.GPIO.detects) == 1
    detect = hat.GPIO.detects[0]
    assert detect["pin"] == 17
    assert detect["edge"] == FakeGPIO.BOTH   # X4 decoding needs both edges
    assert "bouncetime" not in detect["kwargs"]
    assert hat.built[0].kwargs == {}         # nor a bounce_time on the Pin
    assert hat.built[0].irq_calls == 0


def test_the_edge_callback_reaches_the_encoder(hat):
    seen = []
    enc_mod._FusionHatBackend().claim(17, lambda: seen.append(1))
    # RPi.GPIO hands the callback the channel number; the backend drops it.
    hat.GPIO.detects[0]["callback"](17)
    assert seen == [1]


def test_reads_go_straight_to_gpio_input(hat):
    """Not Pin.value(), which is several frames deeper for the same answer."""
    b = enc_mod._FusionHatBackend()
    b.claim(22, lambda: None)
    hat.GPIO.levels[22] = 1
    assert b.read(22) == 1
    hat.GPIO.levels[22] = 0
    assert b.read(22) == 0


def test_close_releases_both_the_interrupt_and_the_pin(hat):
    b = enc_mod._FusionHatBackend()
    b.claim(17, lambda: None)
    b.claim(27, lambda: None)
    b.close()
    assert sorted(hat.GPIO.removed) == [17, 27]
    assert all(p.closed for p in hat.built)


def test_close_survives_a_pin_that_will_not_let_go(hat):
    """Teardown runs while the robot is already on its way out."""
    b = enc_mod._FusionHatBackend()
    b.claim(17, lambda: None)
    b.claim(27, lambda: None)

    def explode():
        raise RuntimeError("gpio busy")

    hat.built[0].close = explode
    b.close()   # must not raise
    assert hat.built[1].closed is True


def test_backend_is_shared_by_every_encoder(hat):
    """One process, one set of GPIO mode/warning globals."""
    assert enc_mod.backend() is enc_mod.backend()


def test_a_missing_fusion_hat_yields_no_backend_rather_than_an_import_error(monkeypatch):
    monkeypatch.setattr(enc_mod, "fusion_pin", None)
    monkeypatch.setattr(enc_mod, "_backend", None)
    monkeypatch.setattr(enc_mod, "_backend_error", "")
    assert enc_mod.backend() is None


def test_a_gpio_that_refuses_to_open_is_only_reported_once(hat, monkeypatch, capsys):
    """The journal gets one line about it, not one per control tick."""
    def explode(self):
        raise RuntimeError("no permission on /dev/gpiochip0")

    monkeypatch.setattr(enc_mod._FusionHatBackend, "__init__", explode)
    assert enc_mod.backend() is None
    assert enc_mod.backend() is None

    printed = capsys.readouterr().out
    assert printed.count("no permission") == 1


def test_an_encoder_starts_and_counts_against_the_real_backend(hat):
    """End to end through _FusionHatBackend, not the FakeBackend seam.

    This is the test that catches ordering: `start()` used to read the pins to
    seed the decoder BEFORE claiming them, which pigpio tolerated and RPi.GPIO
    does not — it raises on an input() it was never given a direction for. The
    result was every encoder failing to start on correctly wired hardware, and
    nothing on a laptop could see it, because the FakeBackend above answers a
    read whether or not anyone claimed the pin.
    """
    e = Encoder(pin_a=17, pin_b=27, counts_per_rev=4.0, tau=0.0)
    assert e.start() is True
    assert e.ok() is True

    fire = {d["pin"]: d["callback"] for d in hat.GPIO.detects}
    for a, b in FORWARD:
        hat.GPIO.levels[17], hat.GPIO.levels[27] = a, b
        fire[17](17)
    assert e.ticks == 4
