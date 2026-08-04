"""BNO085 in UART-RVC mode: heading over one wire, with a checksum on it.

The "robot vacuum cleaner" mode is the BNO08x's other personality. Strapped for
it, the chip stops speaking SHTP and simply *broadcasts*: a 19-byte frame at
100 Hz carrying yaw, pitch, roll and three accelerations, each frame ending in a
checksum. Nothing is ever sent back to it.

    imu = RVCIMU(port="/dev/ttyAMA1")
    imu.start()
    imu.heading()      # degrees, 0 = North, CW+, or None
    imu.stop()

Same contract as the I2C reader (see sensors/imu_common.py) so `PoseEstimator`,
the waypoint controller and the telemetry frame cannot tell them apart.

--- why you would want this ---
The I2C path's failure mode is a single corrupted bit turning a valid report
into an unrecognised one, with nothing in the protocol to notice: SHTP has no
per-packet checksum, so a flipped bit either raises an obscure error or, worse,
silently becomes a slightly wrong heading. An RVC frame carries a checksum over
its 16 payload bytes, so corruption is DETECTED and the frame is dropped. On a
noisy chassis that is the whole argument.

It is also one wire. RVC never listens, so the sensor's TX into the Pi's RX and
a common ground is the entire connection.

--- what it costs, stated plainly ---
Three things, and none of them is small:

  NO CALIBRATION LEVEL. RVC frames carry no accuracy field, so `min_calib`
  cannot be enforced — there is nothing to compare. A heading is trusted as soon
  as frames arrive. Calibrate the chip over I2C first (it keeps its calibration
  in its own flash across power cycles) and this is a one-time cost; skip that
  and you are navigating on an uncalibrated magnetometer with no warning light.
  `start()` says so out loud, every boot.

  NO GYRO. RVC reports orientation, not angular velocity, so `yaw_rate()` here
  is DIFFERENTIATED from successive yaw samples rather than measured. It feeds
  the heading PID's derivative-on-measurement term. At 100 Hz with 0.01 degree
  resolution the quantisation floor is about 1 deg/s before smoothing, which is
  usable — but it is a derived number and it lags, and a build that leans hard
  on the D term should know that before it wonders why.

  NO COMMANDS. Output only: no calibration save, no feature configuration, no
  product id. `save_calibration()` returns False and means it.

--- the one thing to verify on YOUR unit before trusting it ---
Whether RVC yaw is magnetometer-referenced (an absolute compass heading, which
is the entire reason this robot has an IMU) or comes from the 6-axis fusion
(relative to power-on, and drifting). The rest of the stack assumes absolute:
`PoseEstimator` hands it to the waypoint controller as a heading you may pivot
in place on, which a drifting yaw is not.

Check it before a first autonomous run, and it takes two minutes: point the
rover at a landmark, note the heading, drive it around for a few minutes, return
it to the same spot and read the heading again. A few degrees is fine. Tens of
degrees means it is dead-reckoning, and this build should stay on
`heading_source="gps"` or go back to I2C.

--- the frame ---
19 bytes at 115200 8N1, 100 Hz:

    AA AA  idx  yaw_l yaw_h  pitch_l pitch_h  roll_l roll_h
           ax_l ax_h  ay_l ay_h  az_l az_h  MI  MR  RSVD  csum

Angles are signed hundredths of a degree; accelerations are milli-g. The
checksum is the low byte of the sum of the sixteen bytes between the header and
itself. Bytes are read one frame at a time and resynchronised on the header
pair, so a dropped byte costs one frame rather than every frame after it.

Graceful degradation, exactly as everywhere else: no pyserial, no port, no
permission — `start()` says why, the reader stays inert, every accessor answers
None, and the rover drives on the GPS course.
"""

from __future__ import annotations

import struct
import threading
import time
from typing import NamedTuple, Optional

try:  # pragma: no cover - hardware/deps optional on a dev laptop
    import serial
except Exception:
    serial = None

from .imu_common import DEFAULT_SAMPLE_TIMEOUT, HeadingSource

# The frame, byte for byte.
FRAME_LEN = 19
HEADER = b"\xAA\xAA"
_PAYLOAD = slice(2, 18)      # what the checksum covers
_CHECKSUM = 18

# Fixed by the chip in this mode. Both are on IMUConfig anyway, because a build
# that puts the sensor behind a USB-TTL adapter may see something else, and a
# constant you cannot override is a constant that eventually blocks somebody.
RVC_BAUD = 115200
RVC_HZ = 100.0

_DEG_PER_LSB = 0.01          # angles are hundredths of a degree
_MPS2_PER_MG = 9.80665 / 1000.0

# Serial read timeout. Long enough not to spin on a quiet port, short enough
# that stop() does not have to wait out a wedged read.
_SERIAL_TIMEOUT = 0.2
# Pause after the port itself fails, so an unplugged adapter cannot spin a core.
_ERROR_BACKOFF_S = 0.5

# Smoothing on the differentiated yaw rate, as a time constant in seconds. This
# number is a compromise with no good answer: a differentiated angle is noise
# with a gain on it, and a filter is dead time inside a control loop. 50 ms is
# five frames — enough to bury the 0.01 degree quantisation, short enough that
# the heading PID's D term is still describing the present.
_RATE_TAU_S = 0.05
# A gap longer than this means frames were lost, and a rate differentiated
# across the gap would be an average over a period we did not observe. Publish
# no rate instead, and let it re-establish on the next pair.
_MAX_RATE_GAP_S = 0.2


class RvcFrame(NamedTuple):
    """One decoded frame. Angles in degrees, accelerations in m/s^2."""

    sequence: int   # the frame counter byte; `index` would shadow tuple.index
    yaw: float
    pitch: float
    roll: float
    accel_x: float
    accel_y: float
    accel_z: float


def parse_frame(data: bytes) -> Optional[RvcFrame]:
    """Decode one 19-byte frame, or None if it is not one.

    None covers a wrong length, a missing header and a failed checksum, and the
    caller treats all three the same way: drop it and resynchronise. That is the
    entire advantage this transport has over SHTP — a corrupted frame announces
    itself here instead of becoming a plausible-looking heading downstream.
    """
    if len(data) != FRAME_LEN or data[0:2] != HEADER:
        return None
    if (sum(data[_PAYLOAD]) & 0xFF) != data[_CHECKSUM]:
        return None
    sequence = data[2]
    yaw, pitch, roll, ax, ay, az = struct.unpack_from("<hhhhhh", data, 3)
    return RvcFrame(sequence=sequence,
                    yaw=yaw * _DEG_PER_LSB,
                    pitch=pitch * _DEG_PER_LSB,
                    roll=roll * _DEG_PER_LSB,
                    accel_x=ax * _MPS2_PER_MG,
                    accel_y=ay * _MPS2_PER_MG,
                    accel_z=az * _MPS2_PER_MG)


def wrap_delta(degrees: float) -> float:
    """A change in heading, mapped to [-180, 180).

    Crossing North is a 359 -> 1 step, which is +2 degrees and not -358. Getting
    this wrong shows up as a yaw rate that spikes to several thousand deg/s once
    per revolution, which the D term would answer with a violent correction.
    """
    return (degrees + 180.0) % 360.0 - 180.0


class RVCIMU(HeadingSource):
    """The BNO085 read as a stream of RVC frames.

    Thread-safety: the reader thread is the only writer, through the base
    class's `_publish`.
    """

    name = "IMU-RVC"

    def __init__(self, port: str = "/dev/ttyAMA1", baud: int = RVC_BAUD,
                 heading_offset_deg: float = 0.0, invert: bool = False,
                 sample_timeout: float = DEFAULT_SAMPLE_TIMEOUT):
        # min_calib is accepted by the base and deliberately unused here: RVC
        # reports no accuracy, so `_calibrated()` sees a None level and answers
        # True. See the module docstring — this is a real loosening of a safety
        # gate, and `start()` says so rather than leaving it implied.
        super().__init__(heading_offset_deg=heading_offset_deg, invert=invert,
                         min_calib=0, sample_timeout=sample_timeout)
        self.port = port
        self.baud = baud
        self._serial = None
        # Differentiation state, touched only by the reader thread.
        self._last_yaw: Optional[float] = None
        self._last_yaw_at = 0.0
        self._rate = 0.0
        # Frame health, for the monitor tool and for saying "the port is open
        # but the wiring is wrong" out loud.
        self._frames = 0
        self._bad_frames = 0

    # --- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        """Open the port on a background thread and start reading frames.

        Opened on the thread rather than here for the reason the I2C path does
        it: nothing about a sensor may be able to wedge the boot sequence, and
        a serial open on a device that does not exist is not reliably quick.
        """
        if serial is None:
            print(f"[{self.name}] pyserial not installed — IMU disabled "
                  f"(heading falls back to GPS course). Install: "
                  f"pip install pyserial")
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="imu-rvc",
                                        daemon=True)
        self._thread.start()
        print(f"[{self.name}] opening {self.port} @ {self.baud} "
              f"(background; heading uses GPS course until frames arrive)")
        # Every boot, deliberately. This is the one thing about this transport
        # that quietly weakens a safety gate, and a warning that only appears in
        # a docstring is a warning nobody reads.
        print(f"[{self.name}] note: UART-RVC carries no calibration accuracy, "
              f"so imu.min_calib cannot be enforced — the heading is trusted as "
              f"soon as frames arrive. Calibrate over I2C once (the chip keeps "
              f"it in flash), and confirm the yaw is a compass heading and not "
              f"a drifting one before trusting waypoint autonomy.")

    def _run(self) -> None:
        if not self._open():
            self._running = False
            return
        self._read_loop()

    def _open(self) -> bool:
        try:
            self._serial = serial.Serial(self.port, self.baud,
                                         timeout=_SERIAL_TIMEOUT)
        except Exception as e:
            print(f"[{self.name}] could not open {self.port} @ {self.baud}: {e}"
                  f"\n  The GPS owns /dev/ttyAMA0 on this build, so the IMU "
                  f"needs its own UART: enable a spare one in config.txt (or "
                  f"use a USB-TTL adapter) and check `ls /dev/ttyAMA*`. "
                  f"— IMU disabled, heading falls back to the GPS course")
            return False
        print(f"[{self.name}] reading RVC frames on {self.port} "
              f"({RVC_HZ:.0f} Hz, checksummed)")
        return True

    def stop(self) -> None:
        super().stop()
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    # --- reading ------------------------------------------------------------

    def _read_loop(self) -> None:
        while self._running:
            try:
                frame = self._read_frame()
            except Exception as e:
                # The PORT failed, as opposed to one frame being malformed —
                # the adapter was unplugged, or the fd was closed under us.
                self._note_read_error(
                    e, "The serial port itself failed, so this is the adapter "
                       "or the device node rather than the sensor.")
                time.sleep(_ERROR_BACKOFF_S)
                continue
            if frame is not None:
                self._consume(frame)

    def _read_frame(self) -> Optional[RvcFrame]:
        """One frame off the wire, or None if it was not a good one.

        Resynchronises on the header pair rather than assuming alignment: a
        single dropped byte would otherwise shift every subsequent frame, and
        this transport's whole point is that a fault costs one frame.
        """
        port = self._serial
        if port is None:
            return None
        if port.read(1) != HEADER[:1]:
            return None                     # not a header byte; try the next
        if port.read(1) != HEADER[1:]:
            return None                     # 0xAA followed by something else
        rest = port.read(FRAME_LEN - 2)
        if len(rest) != FRAME_LEN - 2:
            return None                     # timed out mid-frame
        frame = parse_frame(HEADER + rest)
        with self._lock:
            self._frames += 1
            if frame is None:
                self._bad_frames += 1
        return frame

    def _consume(self, frame: RvcFrame) -> None:
        """Turn one frame into a compass heading and a differentiated rate."""
        # RVC yaw is a right-handed rotation about the board's Z axis, i.e.
        # counter-clockwise-positive, and the project convention is a compass
        # heading (clockwise-positive, 0 = North). Same negation the quaternion
        # path applies for the same reason; `invert` is then for a board mounted
        # mirrored, and the offset for where its zero happens to point.
        raw = -frame.yaw
        heading = (-raw if self.invert else raw) + self.heading_offset_deg

        now = time.monotonic()
        rate = self._differentiate(frame.yaw, now)
        self._publish(heading, rate)

    def _differentiate(self, yaw_ccw: float, now: float) -> Optional[float]:
        """A yaw RATE, from successive yaw samples. None when it can't be had.

        The gyro this replaces was a direct measurement; this is a difference of
        two angles, so it is noisier and it lags by the filter. Both are stated
        in the module docstring rather than hidden — a build tuning the heading
        PID's D term deserves to know which kind of number it is holding.
        """
        last, last_at = self._last_yaw, self._last_yaw_at
        self._last_yaw, self._last_yaw_at = yaw_ccw, now
        if last is None:
            return None
        dt = now - last_at
        if dt <= 0 or dt > _MAX_RATE_GAP_S:
            # Frames were lost. A difference across the gap would be an average
            # over a period we did not observe, so publish nothing and let the
            # next pair re-establish it.
            self._rate = 0.0
            return None
        # Same sign convention as the heading: CW positive.
        raw = -wrap_delta(yaw_ccw - last) / dt
        if self.invert:
            raw = -raw
        alpha = dt / (_RATE_TAU_S + dt) if _RATE_TAU_S > 0 else 1.0
        self._rate += alpha * (raw - self._rate)
        return self._rate

    # --- health -------------------------------------------------------------

    def frame_counts(self) -> tuple:
        """(frames seen, frames rejected). The RVC answer to the I2C bus audit.

        A rejected frame is one whose checksum did not match — which is what
        this transport buys: on I2C the same corruption arrives as a plausible
        report the driver either chokes on or silently believes.
        """
        with self._lock:
            return (self._frames, self._bad_frames)
