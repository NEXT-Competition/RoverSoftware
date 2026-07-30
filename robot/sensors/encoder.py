"""Quadrature wheel encoders: how fast each track is ACTUALLY turning.

Everything upstream of the drivetrain commands a *throttle* — a fraction of full
scale that becomes a servo pulse an ESC interprets however it feels like. Two
identical motors given the same pulse do not turn at the same speed: ESCs differ,
gearboxes differ, one side carries the battery, and one track is on grass while
the other is on dirt. The rover drives a gentle arc while every number in the
system says it is going straight.

An encoder closes that gap by measuring the thing nobody else can see. Two
channels a quarter-cycle apart (hence "quadrature") on plain Pi GPIO pins: count
the edges and you have position, differentiate it and you have speed, and the
phase relationship between the channels says which way the shaft is turning.

    enc = Encoder(pin_a=17, pin_b=27, counts_per_rev=1200, name="left")
    enc.start()
    ...
    enc.sample(time.monotonic())   # once per control tick
    enc.rpm()                      # signed: +forward, -reverse

--- what this is NOT ---
It is not a position sensor for navigation. Wheel odometry on a skid-steer
chassis is dead reckoning through a slipping contact patch, and the error grows
without bound; that is what the GPS and the IMU are for. This exists so the
speed loop in control/rpm_trim.py can hold the two tracks together, which is a
job where a *relative* measurement over a fraction of a second is exactly right
and the accumulated drift never matters.

--- GPIO backends ---
Two, tried in order, because the right answer changed with the Pi 5:

    pigpio  the classic. Needs the `pigpiod` daemon running (`sudo systemctl
            enable --now pigpiod`), does its edge timestamping in the daemon, and
            does NOT support the Pi 5 at all.
    lgpio   the Bookworm/Pi-5 replacement. No daemon; talks to /dev/gpiochipN.

Both are optional imports. With neither installed — a dev laptop — every encoder
is inert: `ok()` is False, `rpm()` returns None, and control/rpm_trim.py sees
that and leaves the throttles exactly as it found them. Nothing here can stop a
robot from driving.

--- counts_per_rev, and why you should measure it rather than compute it ---
This decodes all four edges of each quadrature cycle (an "X4" decoder), so one
cycle of a 300-cycle-per-rev disc is 4 counts and a revolution is 1200. Then
there is the gearbox, and its ratio is frequently not the number printed on the
motor. So `counts_per_rev` is defined as *counts seen per revolution of the
wheel*, and the honest way to get it is tools/encoder_monitor.py: zero the
count, turn the wheel exactly one turn by hand, read the number.

Get it wrong and nothing breaks — both sides are wrong by the same factor, so
`match` mode still holds them together. Only `velocity` mode, which compares
against an absolute RPM, actually cares.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

try:  # pragma: no cover - Pi-only, and only on Pi 4 and older
    import pigpio
except Exception:
    pigpio = None

try:  # pragma: no cover - Pi-only; the Pi 5 / Bookworm path
    import lgpio
except Exception:
    lgpio = None


# Quadrature state transitions, indexed by (previous << 2) | current, where a
# state is (A << 1) | B. Forward is A leading B: 00 -> 10 -> 11 -> 01 -> 00.
# The four entries that stay 0 are the diagonal moves (00 <-> 11, 01 <-> 10),
# which mean a transition was MISSED — at that point the direction is genuinely
# unknowable, and counting it either way would inject a phantom count. Losing one
# is the better error: a speed loop reads a rate, not a position.
_QUAD = (
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0,
)

# Sentinel for "this actuator has no encoder". Not 0, which is a real BCM pin.
NO_PIN = -1

# Speed is counts over an interval, so the interval sets the resolution: a
# 1200-count wheel turning at 30 rpm emits 600 counts a second, and a 20 ms
# control tick sees 12 of them — a granularity of 2.5 rpm. Measuring over a
# longer window than one tick trades latency for resolution, and this default
# (five ticks at 50 Hz) is where a cheap encoder stops looking like noise. It is
# `drive.trim.rpm_window` in the settings page; raise it for a coarse disc.
DEFAULT_WINDOW_S = 0.1

# First-order smoothing on top of the window, as a time constant in seconds.
# 0 disables it. Deliberately small: this measurement is inside a control loop,
# and a filter is dead time, which is the thing that makes a loop oscillate.
DEFAULT_TAU_S = 0.05

_SECONDS_PER_MINUTE = 60.0


class _Backend:
    """Whichever GPIO library we found, behind three methods.

    One instance for the whole robot, shared by every encoder: pigpio holds a
    socket to its daemon and lgpio holds a chip handle, and opening one per
    encoder would be two of each on a stock build for no reason.
    """

    name = "none"

    def claim(self, pin: int, on_edge) -> bool:
        raise NotImplementedError

    def read(self, pin: int) -> int:
        raise NotImplementedError

    def close(self) -> None:
        pass


class _PigpioBackend(_Backend):
    name = "pigpio"

    def __init__(self):
        self._pi = pigpio.pi()
        if not self._pi.connected:
            # pigpio's own failure mode is to print a wall of text about the
            # daemon and hand back a disconnected object, so say the useful
            # sentence ourselves.
            raise RuntimeError(
                "pigpio daemon is not running — sudo systemctl enable --now pigpiod")
        self._callbacks = []

    def claim(self, pin: int, on_edge) -> bool:
        self._pi.set_mode(pin, pigpio.INPUT)
        # Pull-ups, because an open-collector encoder output floats otherwise and
        # a floating input counts electrical noise as motion.
        self._pi.set_pull_up_down(pin, pigpio.PUD_UP)
        self._callbacks.append(
            self._pi.callback(pin, pigpio.EITHER_EDGE, lambda *_: on_edge()))
        return True

    def read(self, pin: int) -> int:
        return int(self._pi.read(pin))

    def close(self) -> None:
        for cb in self._callbacks:
            try:
                cb.cancel()
            except Exception:
                pass
        self._callbacks.clear()
        try:
            self._pi.stop()
        except Exception:
            pass


class _LgpioBackend(_Backend):
    name = "lgpio"

    def __init__(self):
        # gpiochip4 is the Pi 5's header bank; 0 is everything older. Try the
        # newer one first and fall back, rather than sniffing the model.
        self._handle = None
        for chip in (4, 0):
            try:
                self._handle = lgpio.gpiochip_open(chip)
                break
            except Exception:
                continue
        if self._handle is None:
            raise RuntimeError("could not open a GPIO chip (/dev/gpiochip4 or 0)")
        self._claimed = []
        self._notifiers = []

    def claim(self, pin: int, on_edge) -> bool:
        lgpio.gpio_claim_alert(self._handle, pin, lgpio.BOTH_EDGES,
                               lFlags=lgpio.SET_BIAS_PULL_UP)
        self._claimed.append(pin)
        cb = lgpio.callback(self._handle, pin, lgpio.BOTH_EDGES,
                            lambda *_: on_edge())
        self._notifiers.append(cb)
        return True

    def read(self, pin: int) -> int:
        return int(lgpio.gpio_read(self._handle, pin))

    def close(self) -> None:
        for cb in self._notifiers:
            try:
                cb.cancel()
            except Exception:
                pass
        self._notifiers.clear()
        for pin in self._claimed:
            try:
                lgpio.gpio_free(self._handle, pin)
            except Exception:
                pass
        self._claimed.clear()
        try:
            lgpio.gpiochip_close(self._handle)
        except Exception:
            pass


_backend: Optional[_Backend] = None
_backend_error = ""
_backend_lock = threading.Lock()


def backend() -> Optional[_Backend]:
    """The shared GPIO backend, opened on first use. None if there isn't one.

    Never raises: no GPIO library, no daemon, or no permission on /dev/gpiochip
    all end the same way — encoders stay inert and the robot drives open-loop,
    which is exactly how it drove before any of this existed.
    """
    global _backend, _backend_error
    with _backend_lock:
        if _backend is not None or _backend_error:
            return _backend
        attempts = []
        if pigpio is not None:
            attempts.append(_PigpioBackend)
        if lgpio is not None:
            attempts.append(_LgpioBackend)
        if not attempts:
            _backend_error = ("neither pigpio nor lgpio is installed — "
                              "pip install pigpio (Pi 4 and older) or lgpio (Pi 5)")
            print(f"[Encoder] {_backend_error}; wheel speed is not measured")
            return None
        errors = []
        for cls in attempts:
            try:
                _backend = cls()
                return _backend
            except Exception as e:
                errors.append(f"{cls.name}: {e}")
        _backend_error = "; ".join(errors)
        print(f"[Encoder] no usable GPIO backend ({_backend_error}); "
              "wheel speed is not measured")
        return None


def close_backend() -> None:
    """Release the GPIO backend. Called from Drivetrain teardown."""
    global _backend, _backend_error
    with _backend_lock:
        if _backend is not None:
            _backend.close()
        _backend = None
        _backend_error = ""


class Encoder:
    """One quadrature encoder on two GPIO pins.

    Thread-safety: the edge callback runs on the GPIO library's own thread and
    is the ONLY writer of the tick counter, so its `+=` needs no lock. `sample`
    runs on the control loop and only reads, under the lock that also guards the
    published speed.
    """

    def __init__(self, pin_a: int, pin_b: int, counts_per_rev: float,
                 invert: bool = False, name: str = "",
                 window: float = DEFAULT_WINDOW_S, tau: float = DEFAULT_TAU_S):
        self.pin_a = int(pin_a)
        self.pin_b = int(pin_b)
        self.counts_per_rev = float(counts_per_rev)
        self.invert = bool(invert)
        self.name = name or f"gpio{pin_a}/{pin_b}"
        self.window = float(window)
        self.tau = float(tau)

        self._backend: Optional[_Backend] = None
        self._ticks = 0          # written only by the edge callback
        self._state = 0          # last (A << 1) | B seen
        self._missed = 0         # transitions the decoder could not attribute
        self._lock = threading.Lock()
        self._rpm = 0.0
        self._last_ticks = 0
        self._last_at = 0.0
        self._started = False

    # --- lifecycle ----------------------------------------------------------

    def start(self) -> bool:
        """Claim the pins and begin counting. False if there is no GPIO backend.

        Idempotent, and safe to call on a build with no encoder wired: the pins
        are simply not claimed and `ok()` stays False.
        """
        if self._started:
            return True
        if not self.configured():
            return False
        b = backend()
        if b is None:
            return False
        try:
            self._state = (b.read(self.pin_a) << 1) | b.read(self.pin_b)
            b.claim(self.pin_a, self._on_edge)
            b.claim(self.pin_b, self._on_edge)
        except Exception as e:
            print(f"[Encoder] {self.name}: could not claim GPIO "
                  f"{self.pin_a}/{self.pin_b}: {e}")
            return False
        self._backend = b
        self._started = True
        self._last_at = time.monotonic()
        self._last_ticks = 0
        print(f"[Encoder] {self.name}: A=GPIO{self.pin_a} B=GPIO{self.pin_b}, "
              f"{self.counts_per_rev:.0f} counts/rev via {b.name}")
        return True

    def stop(self) -> None:
        """Stop publishing a speed. Pins are released with the shared backend."""
        self._started = False
        with self._lock:
            self._rpm = 0.0

    def configured(self) -> bool:
        """True if this actuator was given a pair of pins to read."""
        return (self.pin_a != NO_PIN and self.pin_b != NO_PIN
                and self.counts_per_rev > 0)

    def ok(self) -> bool:
        """True when the pins are claimed and counts are actually arriving."""
        return self._started

    # --- counting -----------------------------------------------------------

    def _on_edge(self) -> None:
        """One edge on either channel. Runs on the GPIO library's thread.

        Both channels are re-read rather than trusting the edge's own level:
        between the interrupt and this callback the shaft has kept turning, and
        the pair of levels read *now* is the state the decoder must advance to.
        Reading one and assuming the other is what makes a naive decoder count
        backwards at speed.
        """
        b = self._backend
        if b is None:
            return
        try:
            state = (b.read(self.pin_a) << 1) | b.read(self.pin_b)
        except Exception:
            return
        previous, self._state = self._state, state
        delta = _QUAD[(previous << 2) | state]
        if delta:
            self._ticks += delta
        elif previous != state:
            # A diagonal move: two edges arrived faster than we could read them,
            # so the direction is unknowable. Counted, not corrected — a rising
            # `missed` is the symptom of an encoder outrunning the Pi, and
            # tools/encoder_monitor.py prints it.
            self._missed += 1

    @property
    def ticks(self) -> int:
        """Net counts since start, signed. Direction follows `invert`."""
        return -self._ticks if self.invert else self._ticks

    @property
    def missed(self) -> int:
        """Transitions the decoder could not attribute to a direction.

        Zero on healthy wiring. A number that climbs with speed means the edges
        are arriving faster than the callback can service them (a very high
        count-per-rev disc, or a Pi busy with inference); one that climbs at a
        standstill means a floating input or a missing pull-up.
        """
        return self._missed

    def reset(self) -> None:
        """Zero the counter. For bring-up (turn the wheel once and read it)."""
        self._ticks = 0
        self._missed = 0
        with self._lock:
            self._last_ticks = 0
            self._last_at = time.monotonic()
            self._rpm = 0.0

    # --- speed --------------------------------------------------------------

    def sample(self, now: Optional[float] = None) -> None:
        """Recompute the speed, if a whole measurement window has passed.

        Called once per control tick. Cheap and non-blocking: it reads an int
        and does two multiplies, so it belongs in the 50 Hz loop rather than on
        a thread of its own that the loop would then have to synchronize with.
        """
        # counts_per_rev is live-tunable (it is a calibration you get right by
        # turning the wheel), so re-check it here rather than trusting the value
        # start() was happy with — someone can clear it to 0 from the dashboard.
        if not self._started or self.counts_per_rev <= 0:
            return
        now = time.monotonic() if now is None else now
        with self._lock:
            elapsed = now - self._last_at
            if elapsed < 0:
                # The caller keeps a clock whose origin is not ours — the tools
                # and the tests both pass their own `now`. Re-baseline onto it
                # rather than publishing a negative interval as a speed.
                self._last_ticks = self.ticks
                self._last_at = now
                return
            if elapsed < self.window or elapsed <= 0:
                return
            ticks = self.ticks
            revs = (ticks - self._last_ticks) / self.counts_per_rev
            raw = revs * _SECONDS_PER_MINUTE / elapsed
            self._last_ticks = ticks
            self._last_at = now
            if self.tau > 0:
                # Discrete first-order low pass. alpha is derived from the
                # ACTUAL elapsed time, not the nominal window, so a late tick
                # filters correctly instead of over-weighting a long interval.
                alpha = elapsed / (self.tau + elapsed)
                self._rpm += alpha * (raw - self._rpm)
            else:
                self._rpm = raw

    def rpm(self) -> Optional[float]:
        """Signed revolutions per minute of the wheel, or None if not running.

        None rather than 0.0 is load-bearing: a stopped wheel and an absent
        encoder are the same number and opposite situations, and a speed loop
        that cannot tell them apart will happily wind a dead channel to full
        throttle. See control/rpm_trim.py.
        """
        if not self._started:
            return None
        with self._lock:
            return self._rpm

    def telemetry(self) -> Optional[float]:
        """Rounded RPM for a radio frame, or None when there is nothing to say."""
        value = self.rpm()
        return None if value is None else round(value, 1)


def build_encoder(motor, window: float = DEFAULT_WINDOW_S,
                  tau: float = DEFAULT_TAU_S) -> Optional[Encoder]:
    """An `Encoder` for a MotorConfig that declares one, else None.

    Kept here rather than in the drivetrain so the tools can build one from the
    same config the robot does, and so "does this actuator have an encoder" has
    exactly one definition.
    """
    enc = Encoder(pin_a=getattr(motor, "encoder_a", NO_PIN),
                  pin_b=getattr(motor, "encoder_b", NO_PIN),
                  counts_per_rev=getattr(motor, "encoder_cpr", 0.0),
                  invert=bool(getattr(motor, "encoder_invert", False)),
                  name=motor.name or "", window=window, tau=tau)
    return enc if enc.configured() else None
