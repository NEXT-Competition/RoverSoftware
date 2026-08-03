"""Ultrasonic rangefinder: how far away the thing straight ahead is.

An HC-SR04 (or the Fusion HAT's own two-pin ultrasonic port) on a pair of the
HAT's DIGITAL pins. TRIG is pulsed for 10 us, the module emits a burst of 40 kHz
sound, and ECHO comes back high for as long as the round trip took. Distance is
that time times the speed of sound, halved because the sound went there and
back. The Fusion HAT library does all of that; this module is what makes it
usable from a 50 Hz control loop.

    sonar = Ultrasonic(trig_pin=27, echo_pin=22)
    sonar.start()
    ...
    sonar.distance_m()     # metres, or None
    sonar.stop()

--- why it reads on a thread ---
A ping BLOCKS. The library waits for ECHO to go high and then to go low, and
when nothing is in range it waits out its whole timeout — tens of milliseconds,
several times over if it retries. Inside `Robot.run` that is a control tick
spent not commanding the motors, which is exactly the stall the slow-tick
watchdog exists to shout about. So the pings happen here, on their own thread,
at `interval`, and `distance_m()` is a cached lookup that returns instantly.
Same arrangement as the GPS and the IMU, for the same reason.

--- None means two different things, and the difference is the whole module ---
`distance_m()` returns None both when NOTHING IS IN RANGE (the common case: an
open field returns no echo at all) and when the sensor is not answering. That
looks like a design flaw and is the honest shape of the hardware: a disconnected
ECHO wire and a clear path in front of the rover produce byte-identical silence,
and no amount of code here can tell them apart in the moment.

What we can do is notice the difference over time, and say so. A sensor that has
been pinged a hundred times and has never once heard an echo is not looking at a
hundred metres of open field — it is unplugged, or on the wrong pins, or wired
to 5 V through a divider that never made it onto the board. That is what `mute`
in `telemetry()` reports and what the log line at start-up is for.

The consumer (control/collision.py) treats None as "do not intervene", which is
the only safe way round: the alternative is a rover that refuses to drive
whenever its cheapest sensor goes quiet, in a field, with no way to tell the
operator why.

--- filtering ---
Two mechanisms, both cheap, because an ultrasonic's noise is not Gaussian:

    median      A stray echo — off the floor, off a wall at an angle, off
                another robot's sensor pinging at the same moment — arrives as
                ONE wildly short reading between good ones. A mean would smear
                it across the whole window; a median of three throws it away and
                costs one ping of latency. Only real readings enter the window,
                so a ping that heard nothing never pulls the estimate outwards.
    age         Samples older than `max_age` are dropped, so a stopped reader
                thread or an unplugged module decays to None instead of leaving
                the last distance sitting there looking current forever.

--- the pins ---
BCM GPIO numbers, the same numbering as the encoders' `encoder_a`/`encoder_b`
(sensors/encoder.py) and NOT the HAT's PWM channels. They must not collide with
an encoder's pins: both claim the line, and whichever loses reads nothing.

The import is optional, exactly as it is for the encoders and the motors. On a
dev laptop `start()` says why and returns False, `distance_m()` answers None
forever, and the collision guard sees that and clamps nothing. Nothing in this
file can stop a robot from driving.
"""

from __future__ import annotations

import collections
import threading
import time
from typing import Deque, Optional, Tuple

try:  # pragma: no cover - Pi-only; the HAT library isn't installable elsewhere
    from fusion_hat.modules import Ultrasonic as _HatUltrasonic
    from fusion_hat.pin import Pin as _Pin
except Exception:
    _HatUltrasonic = None
    _Pin = None

# Sentinel for "this build has no ultrasonic". Not 0, which is a real BCM pin.
NO_PIN = -1

_CM_PER_M = 100.0

# How many pings may go unanswered from start-up before we say out loud that the
# sensor has never heard anything. Sized as a few seconds at the default 60 ms
# interval: long enough that a rover started facing an open field doesn't get
# accused of bad wiring, short enough that the line appears while somebody is
# still watching the journal during bring-up.
_MUTE_AFTER_PINGS = 50

# Pings per `read()` call. The library retries internally and returns the first
# echo it hears, which turns a clear path (no echo, so every retry times out)
# into a call that blocks for the better part of a second. One ping per call
# instead, and the median window below does the repeating — with the difference
# that OUR repeats are spaced by `interval`, so consecutive pings can't hear
# each other's echoes.
_PINGS_PER_READ = 1

# What the reader sleeps after a failed ping attempt, so a module that raises on
# every call cannot spin a core.
_ERROR_BACKOFF = 0.5


class Ultrasonic:
    """One ultrasonic rangefinder, read on a background thread.

    Thread-safety: the reader thread is the only writer; every reader of the
    cached samples takes the lock. `distance_m()` is safe to call from the
    control loop and never blocks on the hardware.
    """

    def __init__(self, trig_pin: int, echo_pin: int,
                 min_m: float = 0.03, max_m: float = 4.0,
                 interval: float = 0.06, samples: int = 3,
                 max_age: float = 0.5, name: str = "front"):
        self.trig_pin = int(trig_pin)
        self.echo_pin = int(echo_pin)
        # Live-tunable (Robot._push_live_config copies these back on); read by
        # the reader thread on every pass rather than captured at start.
        self.min_m = float(min_m)
        self.max_m = float(max_m)
        self.interval = float(interval)
        self.samples = int(samples)
        self.max_age = float(max_age)
        self.name = name

        self._sensor = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        # (timestamp, metres) for real echoes only. Bounded by both length and
        # age; the length bound is generous so a shrinking `samples` doesn't
        # need the deque rebuilt.
        self._window: Deque[Tuple[float, float]] = collections.deque(maxlen=16)
        self._raw: Optional[float] = None    # last echo, unfiltered, for bring-up
        self._pings = 0
        self._echoes = 0
        self._last_error = ""
        self._warned_mute = False

    # --- lifecycle ----------------------------------------------------------

    def configured(self) -> bool:
        """True if this build was given a pair of pins to ping on."""
        return (self.trig_pin != NO_PIN and self.echo_pin != NO_PIN
                and self.trig_pin != self.echo_pin)

    def start(self) -> bool:
        """Claim the pins and start pinging. False if there is nothing to claim.

        Never raises. No HAT library, pins already taken, wrong permissions on
        the GPIO character device — all end the same way: the sensor stays
        inert, the guard clamps nothing, and the rover drives exactly as it did
        before anyone fitted an ultrasonic.
        """
        if self._running:
            return True
        if not self.configured():
            print(f"[Ultrasonic] {self.name}: no pins configured "
                  f"(trig={self.trig_pin} echo={self.echo_pin}); not started")
            return False
        if _HatUltrasonic is None or _Pin is None:
            print("[Ultrasonic] fusion_hat is not installed — no distance "
                  "measurement, and collision avoidance clamps nothing "
                  "(`just bootstrap` installs it)")
            return False
        try:
            self._sensor = _HatUltrasonic(_Pin(self.trig_pin), _Pin(self.echo_pin))
        except Exception as e:
            print(f"[Ultrasonic] {self.name}: could not claim GPIO "
                  f"{self.trig_pin}/{self.echo_pin}: {e} — is an encoder on the "
                  f"same pins, or the robot service already running?")
            self._sensor = None
            return False

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True,
                                        name=f"ultrasonic-{self.name}")
        self._thread.start()
        print(f"[Ultrasonic] {self.name}: TRIG=GPIO{self.trig_pin} "
              f"ECHO=GPIO{self.echo_pin}, pinging every "
              f"{self.interval * 1e3:.0f} ms, believing "
              f"{self.min_m:.2f}-{self.max_m:.2f} m")
        return True

    def stop(self) -> None:
        """Stop pinging. The pins go back with the process.

        The HAT's Ultrasonic owns its two Pin objects and has no teardown of its
        own; dropping the reference is all there is to do, and the GPIO cleanup
        that matters (the encoders') happens in the drivetrain's shutdown.
        """
        self._running = False
        thread, self._thread = self._thread, None
        if thread is not None:
            # Generous next to a ping: a call already in flight is waiting out
            # the module's own echo timeout, and killing the wait would leave
            # the library mid-measurement.
            thread.join(timeout=1.0)
        self._sensor = None
        with self._lock:
            self._window.clear()
            self._raw = None

    def ok(self) -> bool:
        """True when the reader thread is up and pinging."""
        return self._running

    # --- measurement --------------------------------------------------------

    def _read_loop(self) -> None:
        while self._running:
            started = time.monotonic()
            self._ping()
            # Off the START of the ping, not its end, so the rate is the rate
            # asked for rather than the rate minus however long the module spent
            # timing out. Never below the interval, though — that spacing is
            # what stops one burst's echo landing in the next ping's window.
            time.sleep(max(0.0, self.interval - (time.monotonic() - started)))

    def _ping(self) -> None:
        """One measurement, straight into the cache. Runs on the reader thread."""
        sensor = self._sensor
        if sensor is None:
            return
        try:
            raw_cm = self._read_cm(sensor)
        except Exception as e:
            # A single failed read is not worth a log line every 60 ms; keep the
            # last message for telemetry and back off so a module that raises on
            # every call can't spin.
            self._last_error = str(e)
            time.sleep(_ERROR_BACKOFF)
            return

        now = time.monotonic()
        # Anything at or below zero is the library's "no echo within the
        # timeout" (-1) or its error sentinel; both mean we heard nothing.
        metres = float(raw_cm) / _CM_PER_M if raw_cm is not None and raw_cm > 0 else None
        with self._lock:
            self._pings += 1
            self._raw = metres
            # Out-of-band readings are dropped rather than clamped. Below min_m
            # the transducer is still ringing from its own burst and the number
            # is an artefact; above max_m the echo is too weak to be trusted,
            # and clamping either into the band would invent an obstacle at
            # exactly the distance the guard cares most about.
            if metres is not None and self.min_m <= metres <= self.max_m:
                self._echoes += 1
                self._window.append((now, metres))
            mute = (self._echoes == 0 and self._pings >= _MUTE_AFTER_PINGS
                    and not self._warned_mute)
            if mute:
                self._warned_mute = True
        if mute:
            print(f"[Ultrasonic] {self.name}: {self._pings} pings, not one "
                  f"echo. Either nothing has been within {self.max_m:.1f} m "
                  f"since start-up, or the module is not wired to GPIO "
                  f"{self.trig_pin}(TRIG)/{self.echo_pin}(ECHO) — a silent "
                  f"sensor and a clear path look identical from here, and "
                  f"collision avoidance is clamping nothing either way.")

    def _read_cm(self, sensor) -> Optional[float]:
        """Ask the library for one reading in centimetres.

        `read(times)` is how SunFounder's module spells "retry this many times";
        a version without the argument is called bare rather than treated as a
        failure, since the retry count is an optimization and the measurement is
        the point.
        """
        try:
            return sensor.read(_PINGS_PER_READ)
        except TypeError:
            return sensor.read()

    def distance_m(self) -> Optional[float]:
        """Metres to the nearest thing ahead, or None.

        None is BOTH "nothing within range" and "no working sensor" — see the
        module docstring. A caller that clamps the drivetrain must treat it as
        "do not intervene"; a caller that reports to a human should say which
        one it is, which is what `telemetry()` is for.
        """
        with self._lock:
            return self._median_locked()

    def _median_locked(self) -> Optional[float]:
        cutoff = time.monotonic() - self.max_age
        fresh = [m for stamped, m in self._window if stamped >= cutoff]
        if not fresh:
            return None
        # Only the newest `samples` of them: the deque is longer than the window
        # so that shrinking `samples` from the dashboard takes effect at once.
        width = max(1, self.samples)
        fresh = fresh[-width:]
        fresh.sort()
        return fresh[len(fresh) // 2]

    def stamped_m(self) -> Optional[Tuple[float, float]]:
        """(metres, when it was measured), or None. For pairing with another sensor.

        `distance_m()` answers "how far", which is all a collision guard needs.
        Pairing a distance with a CAMERA frame needs the second half: the two
        sensors run at unrelated rates, and a reading taken 300 ms after the
        frame was classified describes a different moment — while the rover is
        moving, a different distance. The consumer (control/rangefinder.py)
        compares the two stamps and throws the pair away when they are too far
        apart, which it cannot do unless this hands the stamp over.

        The stamp is the NEWEST sample's, while the distance is the median of
        the window — deliberately: the median is the trustworthy value and the
        newest stamp is the honest claim about how old the estimate is.
        """
        with self._lock:
            distance = self._median_locked()
            if distance is None or not self._window:
                return None
            return (distance, self._window[-1][0])

    def raw_m(self) -> Optional[float]:
        """The last echo, unfiltered and unaged. Bring-up only.

        The filtered value is what anything should act on; this is what you
        watch while waving a hand in front of the sensor, because a median hides
        exactly the single-sample noise you are trying to see the size of.
        """
        with self._lock:
            return self._raw

    def counts(self) -> Tuple[int, int]:
        """(pings, echoes) since start. The ratio is the wiring check."""
        with self._lock:
            return self._pings, self._echoes

    def telemetry(self) -> dict:
        """Compact status for a telemetry frame.

        Short keys and few of them; this rides a 57600-baud radio shared with
        driving. `mute` is the one that matters at 2 a.m.: it separates "the
        path is clear" from "this sensor has never said anything", which are the
        same absent distance and completely different situations.
        """
        with self._lock:
            if not self._running:
                return {"off": True}
            distance = self._median_locked()
            t: dict = {}
            if distance is not None:
                t["d"] = round(distance, 2)
            elif self._echoes == 0 and self._pings >= _MUTE_AFTER_PINGS:
                t["mute"] = True
            return t


def build_ultrasonic(config) -> Optional[Ultrasonic]:
    """An `Ultrasonic` for an UltrasonicConfig that declares one, else None.

    Kept here rather than in `Robot` so the tools build theirs from the same
    config the robot does, and so "does this build have an ultrasonic" has
    exactly one definition.
    """
    if not config.enabled:
        return None
    sonar = Ultrasonic(trig_pin=config.trig_pin, echo_pin=config.echo_pin,
                       min_m=config.min_m, max_m=config.max_m,
                       interval=config.interval, samples=config.samples,
                       max_age=config.max_age)
    return sonar if sonar.configured() else None
