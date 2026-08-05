"""Flywheel speed measurement: a quadrature encoder read on a polling thread.

This is the piece `robot/drive/shooter.py` has always been waiting for. That
module's flywheel controller was ported from Team Northeast's bench script with
its gains intact, but with no sensor to close the loop it fell back to
`_estimated_rpm()` — the assumption "the wheel reaches what we commanded", which
makes the error identically zero and leaves the controller running pure
feed-forward. One call to `Shooter.set_measured_rpm()` from here flips
`_pid_has_sensor` for good and the loop becomes genuinely closed.

Ported from encoder_and_pid_msy.py, keeping its measurement choices:

    speed is a TIME SPAN, not a pulse count. Counting pulses in a fixed window
    quantises to 18.75 RPM steps; timing a fixed number of pulses does not. The
    limit is ~1 ms of scheduler jitter, so a longer span dilutes it — 64 pulses
    spans ~120 ms at 2000 RPM and holds noise near 16 RPM.

--- Why this polls instead of using an interrupt ---
`fusion_hat.pin.Pin` does offer `irq()`, and an edge callback would be the
obvious choice. It is a trap here: `Pin.setup()` defaults `bounce_time` to 20 ms
and `irq()` passes that straight to `GPIO.add_event_detect(bouncetime=...)`. At
3400 RPM and 16 pulses per rev the pulses are 1.1 ms apart, so a 20 ms debounce
would swallow about 94% of them.

That matters more than "the reading is wrong". A failing encoder reads SLOW, and
a slow reading makes the controller add throttle — so the wheel runs away while
the display stays calm. The script's own comment says exactly this, and it is
why it burns a thread on polling rather than trusting an edge callback.

--- Why the thread yields ---
The script polls with no sleep at all, which is right when it is the only thing
running. Here it shares a Pi with a 50 Hz control loop and the AI Camera's
decode, so it calls `time.sleep(0)` each pass: no delay, but it releases the GIL
instead of holding it for the interpreter's full 5 ms switch interval. The
achieved rate is published in `telemetry()` for the same reason the script
printed it — a starved poll thread and a stopped wheel look identical from the
RPM alone, and only one of them is a real problem.

At 3400 RPM the wheel needs ~1.8 kHz to catch every edge; this manages two
orders of magnitude more than that, so the yield costs nothing that matters.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Optional

from ..config import EncoderConfig

try:
    from fusion_hat.pin import Pin as _HardwarePin  # real hardware
    HAVE_HW = True
except Exception:  # pragma: no cover - lets the stack run on a dev laptop
    _HardwarePin = None
    HAVE_HW = False


class _MockPin:
    """Stand-in for a real Pin: reads a flat 0, so RPM stays 0.

    Deliberately not a simulated wheel. A fake that span up would make the PID
    look healthy on a laptop, and the one thing worth knowing on a laptop is
    that there is no encoder — which is what a flat zero reports.
    """

    IN = "in"

    def __init__(self, pin, mode=None, **kw):
        self._pin = pin

    def value(self, value=None):
        return 0


def mock_pins() -> bool:
    return not HAVE_HW


class FlywheelEncoder:
    """Pulse counter on two GPIO pins, reporting shaft speed in RPM."""

    def __init__(self, cfg: EncoderConfig):
        self.cfg = cfg
        self._lock = threading.Lock()
        self._pulse_times: "deque[float]" = deque(maxlen=max(2, cfg.window_pulses))
        self._pulses = 0
        self._polls = 0
        self._thread: Optional[threading.Thread] = None
        self._running = False
        # Achieved poll rate, refreshed by telemetry() — see the module
        # docstring. Zero until something asks twice.
        self._poll_hz = 0.0
        self._last_polls = 0
        self._last_poll_time = 0.0
        self._pin_a = None
        self._pin_b = None

    # --- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        if self._running:
            return
        pin_cls = _MockPin if mock_pins() else _HardwarePin
        try:
            # bounce_time=0 even though nothing here uses the IRQ path: it costs
            # nothing, and it means a later reader that DOES reach for irq()
            # does not silently inherit the 20 ms debounce this module exists to
            # avoid.
            self._pin_a = pin_cls(self.cfg.pin_a, mode=pin_cls.IN, bounce_time=0)
            self._pin_b = pin_cls(self.cfg.pin_b, mode=pin_cls.IN, bounce_time=0)
        except TypeError:
            # Older fusion_hat without the keyword; the poll path never debounces.
            self._pin_a = pin_cls(self.cfg.pin_a, mode=pin_cls.IN)
            self._pin_b = pin_cls(self.cfg.pin_b, mode=pin_cls.IN)
        except Exception as e:
            print(f"[encoder] could not open GPIO {self.cfg.pin_a}/{self.cfg.pin_b}: "
                  f"{e} — flywheel speed control falls back to feed-forward")
            return
        if mock_pins():
            print("[encoder] MOCK pins (no fusion_hat) — RPM will read 0")
        else:
            print(f"[encoder] polling A={self.cfg.pin_a} B={self.cfg.pin_b}, "
                  f"{self.cfg.pulses_per_rev} pulses/rev, "
                  f"{self.cfg.window_pulses}-pulse window")
        self._running = True
        self._last_poll_time = time.monotonic()
        self._thread = threading.Thread(target=self._poll_loop, name="encoder",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    # --- the poll loop -----------------------------------------------------

    def _poll_loop(self) -> None:
        a_pin, b_pin = self._pin_a, self._pin_b
        try:
            last_a = a_pin.value()
            last_b = b_pin.value()
        except Exception as e:
            print(f"[encoder] read failed at startup: {e}")
            self._running = False
            return

        yield_every = max(1, int(self.cfg.yield_every))
        i = 0
        while self._running:
            try:
                a = a_pin.value()
                b = b_pin.value()
            except Exception as e:
                print(f"[encoder] read failed: {e} — stopping the poll thread")
                self._running = False
                return
            self._polls += 1

            # One count per A rising edge, gated on either line having moved —
            # the script's condition, kept verbatim. It is not full quadrature
            # decoding (there is no direction here), and it does not need to be:
            # a flywheel only ever turns one way and the controller wants speed.
            if (a != last_a or b != last_b) and a == 1:
                with self._lock:
                    self._pulses += 1
                    self._pulse_times.append(time.monotonic())
            last_a, last_b = a, b

            i += 1
            if i >= yield_every:
                i = 0
                time.sleep(0)  # release the GIL without delaying

    # --- accessors (cheap; safe to call every control tick) ----------------

    def rpm(self) -> float:
        """Shaft speed over the last `window_pulses` pulses, or 0.0.

        Zero for a stopped wheel AND for a dead poll thread, which is the safe
        direction: the shooter's stall guard reads a persistent zero under
        command as "not turning" and cuts power, rather than winding throttle up
        against a sensor that is no longer reporting.
        """
        with self._lock:
            if len(self._pulse_times) < 2:
                return 0.0
            stamps = list(self._pulse_times)
        if time.monotonic() - stamps[-1] > self.cfg.stale_seconds:
            return 0.0
        span = stamps[-1] - stamps[0]
        if span <= 0 or self.cfg.pulses_per_rev <= 0:
            return 0.0
        return ((len(stamps) - 1) / self.cfg.pulses_per_rev) / span * 60.0

    def telemetry(self) -> dict:
        now = time.monotonic()
        elapsed = now - self._last_poll_time
        if elapsed > 0.25:
            self._poll_hz = (self._polls - self._last_polls) / elapsed
            self._last_polls, self._last_poll_time = self._polls, now
        return {
            "ok": bool(self._running),
            "rpm": round(self.rpm(), 1),
            "pulses": self._pulses,
            # In kHz because the interesting question is "orders of magnitude
            # above the ~1.8 kHz the wheel needs", not the exact figure.
            "poll_khz": round(self._poll_hz / 1000.0, 1),
        }
