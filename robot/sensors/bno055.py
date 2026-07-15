"""Bosch BNO055 9-DOF IMU reader — the absolute compass the GPS lacks.

The BNO055 runs its own sensor-fusion DSP (NDOF mode) and outputs an ABSOLUTE
orientation that's valid at a standstill, unlike the NEO-6M's course-over-ground.
We read it on a background thread and cache the latest heading / yaw-rate /
calibration, so the control loop can poll `heading()` without ever touching the
I2C bus (which would stall the tick — see robot.py's slow-tick watchdog).

    imu = IMU(i2c_address=0x28)
    imu.start()
    ...
    h = imu.heading()     # -> degrees [0,360), 0 = North CW+, or None until calibrated
    r = imu.yaw_rate()    # -> deg/s, CW-positive, or None
    imu.stop()

This mirrors the GPS module's contract so PoseEstimator can fuse the two:
    heading() -> heading_deg or None   (0 = North, clockwise-positive)

--- Heading validity / calibration ---
The BNO055's fused heading is only trustworthy once its magnetometer (and system)
calibration has converged — that needs a few figure-8 motions on first power-up.
`heading()` returns None until the reported sys/mag calibration reaches
`min_calib` (0-3), so an uncalibrated IMU cleanly hands heading back to the GPS
course rather than steering the rover with a wrong bearing. `calibration()`
exposes the raw (sys, gyro, accel, mag) levels for the bring-up tool/telemetry.

--- Frame mapping ---
The sensor's raw yaw depends on how the board is mounted. `heading_offset_deg`
rotates it to align with the robot's forward axis / true North, and `invert`
flips the sign if the board is mounted mirrored, so the output matches the
project convention (0 = North, CW positive) that the waypoint math expects.

Graceful degradation: if the adafruit/Blinka libraries aren't installed, or the
device can't be opened on the I2C bus, `start()` logs why and the reader stays
inert — every accessor returns None and the stack runs unchanged on a dev laptop.

Wiring caveat: the BNO055 clock-stretches on the Pi's hardware I2C, corrupting
reads. 100 kHz (the Pi default) is not slow enough — set
dtparam=i2c_arm_baudrate=10000 and reboot, or use a software i2c-gpio bus for
full speed. Verify with tools/imu_selftest.py.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

try:  # pragma: no cover - hardware/deps optional on a dev laptop
    import board
    import busio
    import adafruit_bno055
except Exception:
    board = None
    busio = None
    adafruit_bno055 = None

Calibration = Tuple[int, int, int, int]  # (sys, gyro, accel, mag), each 0-3


class IMU:
    def __init__(
        self,
        i2c_address: int = 0x28,
        heading_offset_deg: float = 0.0,
        invert: bool = False,
        min_calib: int = 1,
        update_hz: float = 40.0,
    ):
        self.i2c_address = i2c_address
        self.heading_offset_deg = heading_offset_deg
        self.invert = invert
        self.min_calib = min_calib
        self._period = 1.0 / update_hz if update_hz > 0 else 0.025

        self._sensor = None
        self._i2c = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()

        # Latest cached sample (guarded by _lock).
        self._heading = 0.0            # degrees, 0 = North, CW positive
        self._yaw_rate = 0.0           # deg/s, CW positive
        self._calib: Calibration = (0, 0, 0, 0)  # (sys, gyro, accel, mag)
        self._have_reading = False     # True once a valid euler heading has arrived

    def start(self) -> None:
        """Open the I2C device and begin reading on a background thread."""
        if board is None or busio is None or adafruit_bno055 is None:
            print("[IMU] adafruit-circuitpython-bno055 / blinka not installed — IMU "
                  "disabled (heading falls back to GPS course). "
                  "Install on the Pi: pip install adafruit-circuitpython-bno055")
            return
        try:
            self._i2c = busio.I2C(board.SCL, board.SDA)
            self._sensor = adafruit_bno055.BNO055_I2C(self._i2c, address=self.i2c_address)
            # NDOF = accel + gyro + mag fused into an absolute, drift-corrected
            # orientation. This is the mode that gives a standstill-valid heading.
            self._sensor.mode = adafruit_bno055.NDOF_MODE
        except Exception as e:
            print(f"[IMU] could not open BNO055 @ 0x{self.i2c_address:02x}: {e} — IMU disabled")
            self._sensor = None
            return
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, name="imu-rx", daemon=True)
        self._thread.start()
        print(f"[IMU] reading BNO055 @ 0x{self.i2c_address:02x} (NDOF), "
              f"offset={self.heading_offset_deg:g} invert={self.invert}")

    def _read_loop(self) -> None:
        while self._running:
            t0 = time.monotonic()
            try:
                euler = self._sensor.euler            # (heading, roll, pitch) degrees
                gyro = self._sensor.gyro              # (x, y, z) rad/s
                calib = self._sensor.calibration_status  # (sys, gyro, accel, mag) 0-3
            except Exception as e:  # keep the reader alive across transient I2C glitches
                print(f"[IMU] read error: {e}")
                time.sleep(0.2)
                continue
            self._consume(euler, gyro, calib)
            # Pace the loop so we don't hammer the I2C bus.
            sleep_for = self._period - (time.monotonic() - t0)
            if sleep_for > 0:
                time.sleep(sleep_for)

    def _consume(self, euler, gyro, calib) -> None:
        """Fold one BNO055 sample into the cached heading / yaw-rate / calibration."""
        raw_yaw = euler[0] if euler is not None else None

        # Map raw sensor yaw -> project convention (0 = North, CW positive).
        heading = None
        if raw_yaw is not None:
            yaw = -raw_yaw if self.invert else raw_yaw
            heading = (yaw + self.heading_offset_deg) % 360.0

        # gyro z is yaw rate in rad/s; convert to deg/s and match the CW-positive
        # heading convention (same sign flip as the heading itself).
        yaw_rate = None
        if gyro is not None and gyro[2] is not None:
            rate = gyro[2] * (180.0 / 3.141592653589793)
            yaw_rate = -rate if self.invert else rate

        with self._lock:
            if calib is not None and len(calib) == 4:
                self._calib = (int(calib[0]), int(calib[1]), int(calib[2]), int(calib[3]))
            if heading is not None:
                self._heading = heading
                self._have_reading = True
            if yaw_rate is not None:
                self._yaw_rate = yaw_rate

    def _calibrated(self) -> bool:
        """True once system AND magnetometer calibration meet min_calib.

        Heading in NDOF mode is only absolute once the magnetometer is calibrated;
        the system level reflects the overall fusion confidence. We gate on both.
        """
        sys_lvl, _gyro, _accel, mag = self._calib
        return sys_lvl >= self.min_calib and mag >= self.min_calib

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
        on having a reading (gyro calibrates almost immediately), not on min_calib.
        """
        with self._lock:
            if not self._have_reading:
                return None
            return self._yaw_rate

    def calibration(self) -> Calibration:
        """Raw (sys, gyro, accel, mag) calibration levels, each 0-3."""
        with self._lock:
            return self._calib

    def has_heading(self) -> bool:
        """True once a valid, calibrated absolute heading is available."""
        with self._lock:
            return self._have_reading and self._calibrated()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._i2c is not None:
            try:
                self._i2c.deinit()
            except Exception:
                pass
