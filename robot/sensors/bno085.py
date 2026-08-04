"""CEVA/Bosch BNO085 (BNO08x) 9-DOF IMU reader — the absolute compass the GPS lacks.

The BNO085 runs its own sensor-fusion (the SH-2 firmware) and outputs an ABSOLUTE
orientation quaternion (the "rotation vector") that's valid at a standstill, unlike
the GPS track angle (course over ground). We read it on a background thread and
cache the latest heading / yaw-rate / calibration, so the control loop can poll `heading()`
without ever touching the I2C bus (which would stall the tick — see robot.py's
slow-tick watchdog).

    imu = IMU(i2c_address=0x4a)
    imu.start()
    ...
    h = imu.heading()     # -> degrees [0,360), 0 = North CW+, or None until calibrated
    r = imu.yaw_rate()    # -> deg/s, CW-positive, or None
    imu.stop()

This mirrors the GPS module's contract so PoseEstimator can fuse the two:
    heading() -> heading_deg or None   (0 = North, clockwise-positive)

--- Reports (features) ---
Unlike the BNO055, the BNO08x sends nothing until you subscribe to the specific
reports you want. We enable the ROTATION_VECTOR (the fused, magnetometer-corrected
absolute orientation) for heading and the GYROSCOPE report for yaw rate. `heading`
is derived from the quaternion's yaw; there is no direct Euler output.

--- Heading validity / calibration ---
The rotation vector's absolute heading is only trustworthy once the magnetometer
calibration has converged — that needs a few figure-8 motions on first power-up.
The BNO08x reports a single calibration-accuracy level for that fusion (0-3), and
`heading()` returns None until it reaches `min_calib`, so an uncalibrated IMU
cleanly hands heading back to the GPS course rather than steering the rover with a
wrong bearing. `calibration()` exposes the raw level for the bring-up
tool/telemetry.

--- Freshness ---
Every accessor also has a clock on it. A sample older than `sample_timeout` is
not an answer, and `heading()`/`yaw_rate()`/`has_heading()` go back to None — the
same contract `GPSConfig.fix_timeout` gives a fix that stopped arriving.

That matters more here than it looks, because this reader is deliberately hard
to kill: it survives I2C glitches by logging and retrying (see `_read_loop`), so
a sensor that has stopped answering is indistinguishable, from the outside, from
one answering with an unchanging heading. `PoseEstimator` prefers any non-None
IMU heading to the GPS course, so without a clock on the cache a rover whose IMU
died mid-run would keep navigating on the last bearing it ever read.

Unlike the BNO055 (which forgets calibration on every power cycle and needs its
offsets saved to a JSON file and reloaded by us), the BNO08x persists its own
calibration to on-chip flash. With `persist_calibration=True` the driver runs the
sensor's dynamic calibration and calls save_calibration_data() the first time the
level reaches 3 — so the figure-8 only has to be done once and the chip boots
calibrated on its own, no file to manage.

--- Frame mapping ---
The quaternion yaw depends on how the board is mounted. `heading_offset_deg`
rotates it to align with the robot's forward axis / true North, and `invert` flips
the sign if the board is mounted mirrored, so the output matches the project
convention (0 = North, CW positive) that the waypoint math expects.

Graceful degradation: if the adafruit/Blinka libraries aren't installed, or the
device can't be opened on the I2C bus, `start()` logs why and the reader stays
inert — every accessor returns None and the stack runs unchanged on a dev laptop.

Wiring: unlike the BNO055, the BNO08x speaks the SHTP protocol and does NOT abuse
I2C clock stretching, so it runs at the Pi's normal bus speed — the
dtparam=i2c_arm_baudrate=10000 workaround is not needed and should be removed if it
was set for the old sensor. Strap PS0/PS1 for I2C mode; default address is 0x4A
(0x4B if DI/AD0 is pulled high). Verify with tools/imu_selftest.py.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

from .imu_common import DEFAULT_SAMPLE_TIMEOUT, HeadingSource

try:  # pragma: no cover - hardware/deps optional on a dev laptop
    import board
    import busio
    import adafruit_bno08x
    from adafruit_bno08x import (
        BNO_REPORT_ROTATION_VECTOR,
        BNO_REPORT_GYROSCOPE,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C
except Exception:
    board = None
    busio = None
    adafruit_bno08x = None
    BNO08X_I2C = None
    BNO_REPORT_ROTATION_VECTOR = None
    BNO_REPORT_GYROSCOPE = None

try:  # pragma: no cover - UART backend optional on a dev laptop
    import serial
except Exception:
    serial = None

try:  # pragma: no cover - UART backend optional on a dev laptop
    from adafruit_bno08x_rvc import BNO08x_RVC
except Exception:
    BNO08x_RVC = None

_RAD_TO_DEG = 180.0 / 3.141592653589793

# How often the reader polls the chip's calibration accuracy. Each poll is a
# command round-trip on the I2C bus (not a cached register), and the level moves
# over seconds, so there's nothing to gain from polling it every sample.
CALIB_POLL_S = 1.0

# Pause after a failed read, so a sensor that raises on every attempt cannot
# spin a core or flood the journal at the loop rate.
#
# Applied only to TRANSPORT failures — the bus itself is unhappy, and hammering
# it is neither going to help nor going to tell us anything new. A malformed
# PACKET is a different situation and gets the ordinary loop period instead: the
# bus is working, one packet arrived corrupted, and the next one is very
# probably fine. Backing off 200 ms for that would turn a single bad packet into
# eight lost samples and multiply a 1%-corruption bus into a heading that is
# missing a tenth of the time.
_ERROR_BACKOFF_S = 0.2

# What counts as the transport failing rather than one packet being malformed.
# Everything the parsing path raises (KeyError on an unrecognised report id,
# ValueError/struct errors on a short buffer, the driver's own RuntimeErrors)
# means the bytes were bad; an OSError means the I2C transaction was.
_TRANSPORT_ERRORS = (OSError,)


class IMU(HeadingSource):
    """The BNO085 over SHTP on I2C: the full protocol, in both directions."""

    name = "IMU"

    def __init__(
        self,
        i2c_address: int = 0x4A,
        heading_offset_deg: float = 0.0,
        invert: bool = False,
        min_calib: int = 1,
        persist_calibration: bool = True,
        update_hz: float = 40.0,
        sample_timeout: float = DEFAULT_SAMPLE_TIMEOUT,
        transport: str = "i2c",
        serial_port: str = "/dev/serial0",
        serial_baud: int = 115200,
    ):
        # The cache, its clock and the four accessors every consumer uses live
        # in HeadingSource, shared with the UART-RVC reader — so "a stale
        # reading is not a reading" has one implementation rather than one per
        # transport. What is left here is what genuinely differs: opening an I2C
        # device, subscribing to reports, quaternion maths, and a calibration
        # level that only this transport can report.
        super().__init__(
            heading_offset_deg=heading_offset_deg,
            invert=invert,
            min_calib=min_calib,
            sample_timeout=sample_timeout,
        )
        self.i2c_address = i2c_address
        # Whether to run the sensor's dynamic calibration and save it to the
        # BNO08x's own flash once it converges, so the board boots calibrated. The
        # BNO08x persists this itself — there is no file to manage, unlike the
        # BNO055's saved offsets.
        self.persist_calibration = persist_calibration
        self._period = 1.0 / update_hz if update_hz > 0 else 0.025
        self.transport = (transport or "i2c").strip().lower()
        self.serial_port = serial_port
        self.serial_baud = serial_baud

        self._sensor = None
        self._serial = None
        self._i2c = None
        self._calib_saved = False  # save the chip's calibration once per session
        # When each quantity was last actually measured. Two stamps, not one:
        # the quaternion and the gyro are separate reports, and a read that
        # returns one but not the other must not refresh both.
        # -inf, not 0.0, means "never measured": these hold `time.monotonic()`,
        # which is time since boot, so 0.0 is a REACHABLE reading rather than a
        # sentinel — overloading it makes an early sample indistinguishable from
        # no sample at all.
        self._heading_at = -math.inf
        self._rate_at = -math.inf
        # Read-error bookkeeping, so a bad bus costs one log line per interval
        # rather than five a second, and so the handover to the GPS course is
        # announced once rather than never.
        self._errors = 0
        self._last_error_log = 0.0
        self._stale = False

    def start(self) -> None:
        """Begin reading on a background thread; the sensor is opened there.

        Opening a BNO08x is NOT quick or reliably bounded — the SHTP handshake,
        the feature subscriptions and the calibration command all talk to the
        chip, and the adafruit driver's packet-drain loops have no hard cap (see
        _open). So none of it happens here: start() only spawns the reader and
        returns, and a sick sensor can never wedge the boot sequence. That keeps
        the promise the module docstring makes — a bad IMU degrades to GPS-course
        heading — for a HANG, not just for an exception.
        """
        if self.transport == "uart_rvc":
            if serial is None or BNO08x_RVC is None:
                print(
                    "[IMU] pyserial / adafruit-circuitpython-bno08x-rvc not installed — "
                    "IMU disabled (heading falls back to GPS course)."
                )
                return
            self._running = True
            self._thread = threading.Thread(
                target=self._run, name="imu-rx", daemon=True
            )
            self._thread.start()
            print(
                f"[IMU] opening BNO085 UART-RVC on {self.serial_port} "
                f"@ {self.serial_baud} baud (background; heading uses GPS course until it's up)"
            )
            return

        if board is None or busio is None or adafruit_bno08x is None:
            print(
                "[IMU] adafruit-circuitpython-bno08x / blinka not installed — IMU "
                "disabled (heading falls back to GPS course). "
                "Install on the Pi: pip install adafruit-circuitpython-bno08x"
            )
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="imu-rx", daemon=True)
        self._thread.start()
        print(
            f"[IMU] opening BNO085 @ 0x{self.i2c_address:02x} on I2C "
            f"(background; heading uses GPS course until it's up)"
        )

    def _run(self) -> None:
        """Thread body: open the sensor, then stream from it."""
        if not self._open():
            self._running = False
            return
        self._read_loop()

    def _open(self) -> bool:
        """Open the sensor backend and subscribe to the reports. True on success."""
        if self.transport == "uart_rvc":
            try:
                self._serial = serial.Serial(
                    self.serial_port, baudrate=self.serial_baud, timeout=1
                )
                self._sensor = BNO08x_RVC(self._serial)
            except Exception as e:
                print(
                    f"[IMU] could not open BNO085 UART-RVC on {self.serial_port}: {e} — IMU "
                    "disabled (heading falls back to GPS course)"
                )
                self._serial = None
                self._sensor = None
                return False
            print(
                f"[IMU] reading BNO085 UART-RVC from {self.serial_port} "
                f"({self.serial_baud} baud)"
            )
            return True

        try:
            self._i2c = busio.I2C(board.SCL, board.SDA)
            self._sensor = BNO08X_I2C(self._i2c, address=self.i2c_address)
            # Run dynamic calibration so calibration_status is populated and the
            # magnetometer keeps converging; it loads any calibration already in
            # the chip's flash on power-up regardless.
            if self.persist_calibration:
                try:
                    self._sensor.begin_calibration()
                except Exception as e:
                    print(f"[IMU] begin_calibration failed ({e}); running uncalibrated")
            # Subscribe to the reports we consume: the fused absolute orientation
            # (magnetometer-corrected, standstill-valid) and the raw gyro for yaw
            # rate. The sensor sends nothing until asked.
            self._sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self._sensor.enable_feature(BNO_REPORT_GYROSCOPE)
        except Exception as e:
            print(
                f"[IMU] could not open BNO085 @ 0x{self.i2c_address:02x}: {e} — IMU "
                "disabled (heading falls back to GPS course)"
            )
            self._sensor = None
            return False
        print(
            f"[IMU] reading BNO085 @ 0x{self.i2c_address:02x} (rotation vector), "
            f"offset={self.heading_offset_deg:g} invert={self.invert}"
        )
        return True

    def _read_loop(self) -> None:
        if self.transport == "uart_rvc":
            while self._running:
                t0 = time.monotonic()
                try:
                    heading = self._sensor.heading
                except (
                    Exception
                ) as e:  # keep the reader alive across transient UART glitches
                    self._note_read_error(e)
                    time.sleep(
                        _ERROR_BACKOFF_S
                        if isinstance(e, _TRANSPORT_ERRORS)
                        else self._period
                    )
                    continue
                self._consume_uart_heading(heading)
                sleep_for = self._period - (time.monotonic() - t0)
                if sleep_for > 0:
                    time.sleep(sleep_for)
            return

        next_calib = 0.0
        while self._running:
            t0 = time.monotonic()
            try:
                quat = self._sensor.quaternion  # (i, j, k, real)
                gyro = self._sensor.gyro  # (x, y, z) rad/s
                # calibration_status is not a cached value: every read sends an
                # ME command and waits for the reply, so polling it at the loop
                # rate triples the bus traffic for a number that moves over
                # seconds. Once a second is plenty for telemetry and autosave.
                calib = None
                if t0 >= next_calib:
                    next_calib = t0 + CALIB_POLL_S
                    calib = self._sensor.calibration_status  # single level 0-3
            except (
                Exception
            ) as e:  # keep the reader alive across transient I2C glitches
                self._note_read_error(e)
                # A corrupted packet costs one sample; a broken bus costs a
                # backoff. See _ERROR_BACKOFF_S for why the difference matters.
                time.sleep(
                    _ERROR_BACKOFF_S
                    if isinstance(e, _TRANSPORT_ERRORS)
                    else self._period
                )
                continue
            self._consume(quat, gyro, calib)
            self._maybe_autosave(calib)
            # Pace the loop so we don't hammer the I2C bus.
            sleep_for = self._period - (time.monotonic() - t0)
            if sleep_for > 0:
                time.sleep(sleep_for)

    def _note_read_error(self, error) -> None:
        """Add the hint only this transport can give, then defer to the base.

        A bare number from the adafruit driver is a `KeyError` carrying an SHTP
        report id it does not recognise, and nothing about that message says so.
        The base class owns the throttling and the handover announcement, which
        are the same on any wire; this is the sentence that is not.
        """
        now = time.monotonic()
        with self._lock:
            self._errors += 1
            errors = self._errors
            last_good = max(self._heading_at, self._rate_at)
            due = (now - self._last_error_log) >= _ERROR_LOG_INTERVAL
            if due:
                self._last_error_log = now
            went_stale = (
                not self._stale
                and math.isfinite(last_good)
                and not self._fresh_locked(last_good)
            )
            if went_stale:
                self._stale = True
        if due:
            print(
                f"[IMU] read error: {error} ({errors} since start)"
                + (
                    "\n  A bare number is an SHTP report id the driver does "
                    "not know, i.e. a corrupted or desynchronised stream — not "
                    "a missing sensor. Usual causes: a second process on the "
                    "I2C bus (a monitor tool running against the service), "
                    "wiring or noise, or a stale dtparam=i2c_arm_baudrate."
                    if errors == 1
                    else ""
                )
            )
        if went_stale:
            print(
                f"[IMU] no valid sample for {self.sample_timeout:.1f}s — the "
                f"heading is no longer being reported, so navigation falls "
                f"back to the GPS course until it recovers"
            )

    def _fresh_locked(self, stamp: float) -> bool:
        """Is a sample taken at `stamp` still an answer? Call under the lock."""
        if not math.isfinite(stamp):
            return False  # nothing has ever been measured
        if self.sample_timeout <= 0:
            return True  # the check is switched off
        return (time.monotonic() - stamp) <= self.sample_timeout

    def _consume(self, quat, gyro, calib) -> None:
        """Fold one BNO085 sample into the cached heading / yaw-rate / calibration."""
        # Yaw from the orientation quaternion (i, j, k, real). atan2 gives a
        # math-convention (CCW-positive) yaw; negate to get a compass heading
        # (CW-positive), which is the project convention the waypoint math wants.
        heading = None
        if quat is not None and len(quat) == 4 and None not in quat:
            i, j, k, r = quat
            yaw_ccw = math.atan2(2.0 * (r * k + i * j), 1.0 - 2.0 * (j * j + k * k))
            raw_heading = (-yaw_ccw * _RAD_TO_DEG) % 360.0
            head = -raw_heading if self.invert else raw_heading
            heading = (head + self.heading_offset_deg) % 360.0

        # gyro z is yaw rate in rad/s (CCW-positive); convert to deg/s and negate
        # to match the CW-positive heading, so yaw_rate ~ d(heading)/dt and the
        # heading PID's derivative term has the right sign.
        yaw_rate = None
        if gyro is not None and len(gyro) == 3 and gyro[2] is not None:
            rate_cw = -gyro[2] * _RAD_TO_DEG
            yaw_rate = -rate_cw if self.invert else rate_cw

        now = time.monotonic()
        with self._lock:
            if calib is not None:
                self._calib = int(calib)
            if heading is not None:
                self._heading = heading
                self._heading_at = now
                self._have_reading = True
            if yaw_rate is not None:
                self._yaw_rate = yaw_rate
                self._rate_at = now
            # Recovered. Announced because the handover was: an operator told
            # the heading had gone needs telling when it came back, or the next
            # thing they do is go and look for a fault that has fixed itself.
            recovered = self._stale and (heading is not None or yaw_rate is not None)
            if recovered:
                self._stale = False
        if recovered:
            print(
                f"[IMU] reading again after {self._errors} read error(s); "
                f"the heading is back"
            )

    def _consume_uart_heading(self, heading_data) -> None:
        """Fold one UART-RVC heading tuple into the cached absolute heading."""
        heading = None
        if isinstance(heading_data, (tuple, list)) and len(heading_data) >= 1:
            try:
                yaw = float(heading_data[0])
            except (TypeError, ValueError):
                yaw = None
            if yaw is not None:
                raw_heading = yaw % 360.0
                head = -raw_heading if self.invert else raw_heading
                heading = (head + self.heading_offset_deg) % 360.0

        now = time.monotonic()
        with self._lock:
            self._calib = 3 if heading is not None else self._calib
            if heading is not None:
                self._heading = heading
                self._heading_at = now
                self._have_reading = True
            self._yaw_rate = None
            self._rate_at = now if heading is not None else self._rate_at

    def _calibrated(self) -> bool:
        """True once the fused-orientation calibration level meets min_calib.

        The rotation-vector heading is only absolute once the magnetometer fusion
        has converged; the BNO08x reports that as a single 0-3 accuracy level.
        """
        return self._calib >= self.min_calib

    def heading(self) -> Optional[float]:
        """Latest absolute heading in degrees (0 = North, CW+), or None.

        Three ways this is None, and they are one rule: we only answer with a
        heading we currently believe. Nothing has arrived yet; calibration is
        below min_calib, so the number is not absolute; or the last sample is
        older than `sample_timeout`, so it is not current. Each of them hands
        heading back to the GPS course, which is the honest fallback — see the
        note on DEFAULT_SAMPLE_TIMEOUT for why the last one has to exist.
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
        gated on min_calib (the gyro is good immediately), but gated on
        freshness for a sharper reason than the heading is: a frozen rate is fed
        to a D term as though it were happening, so the loop keeps damping a
        rotation that stopped seconds ago.
        """
        with self._lock:
            if not self._have_reading or not self._fresh_locked(self._rate_at):
                return None
            return self._yaw_rate

    def calibration(self) -> int:
        """Raw fused-orientation calibration accuracy level, 0-3.

        Deliberately NOT gated on freshness — it is the raw diagnostic, and a
        caller asking what the chip last said should get what the chip last
        said. Anything reporting it to a human should pair it with `fresh()`,
        because three calibration pips beside a heading that stopped updating is
        a dashboard telling a comfortable lie. `Robot._telemetry` does.
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
            return (
                self._have_reading
                and self._calibrated()
                and self._fresh_locked(self._heading_at)
            )

    # --- Calibration persistence -------------------------------------------------
    # The BNO08x stores its calibration in on-chip flash, so unlike the BNO055 we
    # don't save/reload offsets ourselves. We just ask the chip to persist its
    # current dynamic calibration once it's good enough for an absolute heading.

    def save_calibration(self) -> bool:
        """Persist the sensor's current calibration to its on-chip flash."""
        if self._sensor is None:
            return False
        try:
            self._sensor.save_calibration_data()
        except Exception as e:
            print(f"[IMU] could not save calibration to the chip: {e}")
            return False
        print(f"[IMU] saved calibration to BNO085 flash (level={self.calibration()})")
        return True

    def _maybe_autosave(self, calib) -> None:
        """Persist the calibration to flash the first time it's fully converged."""
        if self._calib_saved or not self.persist_calibration or calib is None:
            return
        if int(calib) >= 3 and self.save_calibration():
            self._calib_saved = True

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        if self._i2c is not None:
            try:
                self._i2c.deinit()
            except Exception:
                pass
