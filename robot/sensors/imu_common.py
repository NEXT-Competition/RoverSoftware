"""What every IMU reader owes the rest of the robot, whatever wire it is on.

The BNO085 can be read two ways, and this codebase supports both:

    i2c       SHTP over I2C (sensors/bno085.py). The chip's full protocol: a
              fused rotation vector, a calibrated gyro, a calibration accuracy
              level, and commands back to the chip.
    uart_rvc  The "robot vacuum cleaner" mode (sensors/bno085_rvc.py). The chip
              streams 19-byte frames of yaw/pitch/roll and acceleration at
              100 Hz, one direction only, each frame carrying a checksum.

They differ enormously in what they can tell you, but nothing above the sensor
should have to know which one is fitted: `PoseEstimator`, the waypoint
controller and the telemetry frame all want the same four answers. This is that
contract, plus the cache-with-a-clock behind it, in one place — so the rule that
matters most (a stale reading is not a reading) has exactly one implementation
rather than one per transport.

    heading()       degrees, 0 = North, CW+, or None
    yaw_rate()      deg/s, CW+, or None
    calibration()   accuracy level 0-3, or None where the transport has none
    fresh()         is the sensor currently talking to us at all

--- the clock, and why it is the load-bearing part ---
Both readers are deliberately hard to kill: they log, back off and retry rather
than dying, because a rover that loses its IMU should keep driving on the GPS
course rather than fall over. That resilience is exactly what makes staleness
dangerous — a sensor that has stopped answering looks, from outside this object,
identical to one reporting a heading that happens not to be changing, and
`PoseEstimator` prefers any non-None IMU heading to the GPS course. So every
answer here is gated on when it was last actually measured.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

# How long a cached sample stays believable. See the module docstring for why
# this exists at all, and IMUConfig.sample_timeout for why it is 2 seconds and
# not 0.2 — heading source is not a free switch, and flapping between the IMU
# and the GPS course on every bus hiccup would be its own bug.
DEFAULT_SAMPLE_TIMEOUT = 2.0

# At most one read-error line per this many seconds. A bus in a bad mood
# produces the same error many times a second for as long as the cause lasts,
# and a journal full of one repeated line is a journal nobody reads.
ERROR_LOG_INTERVAL = 5.0


class HeadingSource:
    """A cached heading with a clock on it. Subclassed per transport.

    Thread-safety: the reader thread is the only writer, through `_publish`;
    every accessor takes the lock. Subclasses own their thread, their wire, and
    the arithmetic that turns whatever the chip sent into a compass heading —
    and nothing else.
    """

    # What the transport is called in log lines. Overridden by subclasses.
    name = "IMU"

    def __init__(self, heading_offset_deg: float = 0.0, invert: bool = False,
                 min_calib: int = 1,
                 sample_timeout: float = DEFAULT_SAMPLE_TIMEOUT):
        # Frame mapping: where the board is pointing versus where the robot is.
        self.heading_offset_deg = heading_offset_deg
        self.invert = invert
        self.min_calib = min_calib
        # How stale a cached sample may be before it stops being an answer.
        # 0 disables the check, which restores the old behaviour of trusting the
        # last reading forever — an escape hatch for a build where the fallback
        # is worse than the staleness, not a setting anyone should reach for.
        self.sample_timeout = sample_timeout

        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Latest cached sample (guarded by _lock).
        self._heading = 0.0            # degrees, 0 = North, CW positive
        self._yaw_rate = 0.0           # deg/s, CW positive
        self._calib: Optional[int] = None   # accuracy 0-3, None where unavailable
        self._have_reading = False
        # When each quantity was last actually measured. Two stamps, not one: a
        # read that produces one but not the other must not refresh both.
        self._heading_at = 0.0
        self._rate_at = 0.0
        # Read-error bookkeeping, so a bad wire costs one log line per interval
        # rather than many a second, and so the handover to the GPS course is
        # announced once rather than never.
        self._errors = 0
        self._last_error_log = 0.0
        self._stale = False

    # --- what subclasses call -----------------------------------------------

    def _publish(self, heading: Optional[float], yaw_rate: Optional[float],
                 calib: Optional[int] = None) -> None:
        """Fold one sample into the cache. Either quantity may be None.

        `heading` arrives already in the project's convention (0 = North, CW
        positive, offset and inversion applied) — turning whatever the chip
        sent into that is the subclass's job, because it is the one part that
        genuinely differs between a quaternion and an RVC frame.
        """
        now = time.monotonic()
        with self._lock:
            if calib is not None:
                self._calib = int(calib)
            if heading is not None:
                self._heading = heading % 360.0
                self._heading_at = now
                self._have_reading = True
            if yaw_rate is not None:
                self._yaw_rate = yaw_rate
                self._rate_at = now
            # Recovered. Announced because the handover was: an operator told
            # the heading has gone needs telling when it came back, or the next
            # thing they do is go looking for a fault that has fixed itself.
            recovered = self._stale and (heading is not None or yaw_rate is not None)
            errors = self._errors
            if recovered:
                self._stale = False
        if recovered:
            print(f"[{self.name}] reading again after {errors} error(s); "
                  f"the heading is back")

    def _note_read_error(self, error, hint: str = "") -> None:
        """Record one failed read: throttle the log, announce a real handover.

        Two different messages, because they answer two different questions.
        The error line says WHAT the reader choked on; the staleness line says
        what it COST — the point at which the cached heading stopped being an
        answer and navigation went back to the GPS course.
        """
        now = time.monotonic()
        with self._lock:
            self._errors += 1
            errors = self._errors
            last_good = max(self._heading_at, self._rate_at)
            due = (now - self._last_error_log) >= ERROR_LOG_INTERVAL
            if due:
                self._last_error_log = now
            went_stale = (not self._stale and last_good > 0.0
                          and not self._fresh_locked(last_good))
            if went_stale:
                self._stale = True
        if due:
            print(f"[{self.name}] read error: {error} ({errors} since start)"
                  + (f"\n  {hint}" if hint and errors == 1 else ""))
        if went_stale:
            print(f"[{self.name}] no valid sample for {self.sample_timeout:.1f}s "
                  f"— the heading is no longer being reported, so navigation "
                  f"falls back to the GPS course until it recovers")

    # --- the contract -------------------------------------------------------

    def _fresh_locked(self, stamp: float) -> bool:
        """Is a sample taken at `stamp` still an answer? Call under the lock."""
        if stamp <= 0.0:
            return False            # nothing has ever been measured
        if self.sample_timeout <= 0:
            return True             # the check is switched off
        return (time.monotonic() - stamp) <= self.sample_timeout

    def _calibrated(self) -> bool:
        """True once the reported accuracy meets min_calib.

        A transport that reports no accuracy at all (UART-RVC) answers True:
        there is nothing to check, and refusing to report a heading forever
        because a number does not exist would be worse than the gate is good.
        Such a transport says so at start-up rather than leaving it implied.
        """
        return self._calib is None or self._calib >= self.min_calib

    def heading(self) -> Optional[float]:
        """Latest absolute heading in degrees (0 = North, CW+), or None.

        Three ways this is None, and they are one rule: we only answer with a
        heading we currently believe. Nothing has arrived yet; the reported
        accuracy is below min_calib, so the number is not absolute; or the last
        sample is older than `sample_timeout`, so it is not current.
        """
        with self._lock:
            if not self._have_reading or not self._calibrated():
                return None
            if not self._fresh_locked(self._heading_at):
                return None
            return self._heading

    def yaw_rate(self) -> Optional[float]:
        """Latest yaw rate in deg/s (CW+), or None if there isn't a current one.

        Used as the derivative-on-measurement term for the heading PID. Not
        gated on min_calib (a rate needs no magnetometer), but gated on
        freshness for a sharper reason than the heading is: a frozen rate is fed
        to a D term as though it were happening, so the loop keeps damping a
        rotation that stopped seconds ago.
        """
        with self._lock:
            if not self._have_reading or not self._fresh_locked(self._rate_at):
                return None
            return self._yaw_rate

    def calibration(self) -> Optional[int]:
        """Reported accuracy level 0-3, or None where the transport has none.

        Deliberately NOT gated on freshness — it is the raw diagnostic, and a
        caller asking what the chip last said should get what the chip last
        said. Anything reporting it to a human should pair it with `fresh()`.
        """
        with self._lock:
            return self._calib

    def fresh(self) -> bool:
        """True when a sample has arrived recently enough to still be an answer.

        Independent of calibration: this is "is the sensor talking to us", which
        is a different question from "is what it says absolute yet".
        """
        with self._lock:
            return self._fresh_locked(max(self._heading_at, self._rate_at))

    def has_heading(self) -> bool:
        """True once a valid, calibrated, CURRENT absolute heading is available."""
        with self._lock:
            return (self._have_reading and self._calibrated()
                    and self._fresh_locked(self._heading_at))

    def is_running(self) -> bool:
        return self._running

    # --- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        self._running = False
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=1.0)

    def save_calibration(self) -> bool:
        """Persist calibration to the chip. False where the transport can't ask.

        UART-RVC is output-only — there is no command channel — so a build on it
        calibrates over I2C once and relies on the chip's own flash thereafter.
        """
        return False
