"""CEVA/Bosch BNO085 (BNO08x) 9-DOF IMU reader — the absolute compass the GPS lacks.

The BNO085 runs its own sensor-fusion (the SH-2 firmware) and outputs an ABSOLUTE
orientation quaternion (the "rotation vector") that's valid at a standstill, unlike
the NEO-6M's course-over-ground. We read it on a background thread and cache the
latest heading / yaw-rate / calibration, so the control loop can poll `heading()`
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

_RAD_TO_DEG = 180.0 / 3.141592653589793


class IMU:
    def __init__(
        self,
        i2c_address: int = 0x4A,
        heading_offset_deg: float = 0.0,
        invert: bool = False,
        min_calib: int = 1,
        persist_calibration: bool = True,
        update_hz: float = 40.0,
    ):
        self.i2c_address = i2c_address
        self.heading_offset_deg = heading_offset_deg
        self.invert = invert
        self.min_calib = min_calib
        # Whether to run the sensor's dynamic calibration and save it to the
        # BNO08x's own flash once it converges, so the board boots calibrated. The
        # BNO08x persists this itself — there is no file to manage, unlike the
        # BNO055's saved offsets.
        self.persist_calibration = persist_calibration
        self._period = 1.0 / update_hz if update_hz > 0 else 0.025

        self._sensor = None
        self._i2c = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()

        # Latest cached sample (guarded by _lock).
        self._heading = 0.0            # degrees, 0 = North, CW positive
        self._yaw_rate = 0.0           # deg/s, CW positive
        self._calib = 0               # single accuracy level 0-3 (magnetometer fusion)
        self._have_reading = False     # True once a valid quaternion heading has arrived
        self._calib_saved = False      # save the chip's calibration once per session

    def start(self) -> None:
        """Open the I2C device and begin reading on a background thread."""
        if board is None or busio is None or adafruit_bno08x is None:
            print("[IMU] adafruit-circuitpython-bno08x / blinka not installed — IMU "
                  "disabled (heading falls back to GPS course). "
                  "Install on the Pi: pip install adafruit-circuitpython-bno08x")
            return
        try:
            self._i2c = busio.I2C(board.SCL, board.SDA)
            self._sensor = BNO08X_I2C(self._i2c, address=self.i2c_address)
            # Subscribe to the reports we consume: the fused absolute orientation
            # (magnetometer-corrected, standstill-valid) and the raw gyro for yaw
            # rate. The sensor sends nothing until asked.
            self._sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self._sensor.enable_feature(BNO_REPORT_GYROSCOPE)
            # Run dynamic calibration so calibration_status is populated and the
            # magnetometer keeps converging; it loads any calibration already in
            # the chip's flash on power-up regardless.
            if self.persist_calibration:
                try:
                    self._sensor.begin_calibration()
                except Exception as e:
                    print(f"[IMU] begin_calibration failed ({e}); running uncalibrated")
        except Exception as e:
            print(f"[IMU] could not open BNO085 @ 0x{self.i2c_address:02x}: {e} — IMU disabled")
            self._sensor = None
            return
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, name="imu-rx", daemon=True)
        self._thread.start()
        print(f"[IMU] reading BNO085 @ 0x{self.i2c_address:02x} (rotation vector), "
              f"offset={self.heading_offset_deg:g} invert={self.invert}")

    def _read_loop(self) -> None:
        while self._running:
            t0 = time.monotonic()
            try:
                quat = self._sensor.quaternion              # (i, j, k, real)
                gyro = self._sensor.gyro                     # (x, y, z) rad/s
                calib = self._sensor.calibration_status      # single level 0-3
            except Exception as e:  # keep the reader alive across transient I2C glitches
                print(f"[IMU] read error: {e}")
                time.sleep(0.2)
                continue
            self._consume(quat, gyro, calib)
            self._maybe_autosave(calib)
            # Pace the loop so we don't hammer the I2C bus.
            sleep_for = self._period - (time.monotonic() - t0)
            if sleep_for > 0:
                time.sleep(sleep_for)

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

        with self._lock:
            if calib is not None:
                self._calib = int(calib)
            if heading is not None:
                self._heading = heading
                self._have_reading = True
            if yaw_rate is not None:
                self._yaw_rate = yaw_rate

    def _calibrated(self) -> bool:
        """True once the fused-orientation calibration level meets min_calib.

        The rotation-vector heading is only absolute once the magnetometer fusion
        has converged; the BNO08x reports that as a single 0-3 accuracy level.
        """
        return self._calib >= self.min_calib

    def heading(self) -> Optional[float]:
        """Latest absolute heading in degrees (0 = North, CW+), or None.

        None until a reading has arrived AND calibration meets min_calib — so an
        uncalibrated IMU falls back to the GPS course instead of steering wrong.
        """
        with self._lock:
            if not self._have_reading or not self._calibrated():
                return None
            return self._heading

    def yaw_rate(self) -> Optional[float]:
        """Latest yaw rate in deg/s (CW+), or None if no reading yet.

        Used as the derivative-on-measurement term for the heading PID. Gated only
        on having a reading (the gyro is good immediately), not on min_calib.
        """
        with self._lock:
            if not self._have_reading:
                return None
            return self._yaw_rate

    def calibration(self) -> int:
        """Raw fused-orientation calibration accuracy level, 0-3."""
        with self._lock:
            return self._calib

    def has_heading(self) -> bool:
        """True once a valid, calibrated absolute heading is available."""
        with self._lock:
            return self._have_reading and self._calibrated()

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
        if self._i2c is not None:
            try:
                self._i2c.deinit()
            except Exception:
                pass
